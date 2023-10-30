/* MIT License
Copyright (c) [2022] [VIP Team RoSE]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include <cr_control/wheel_hardware_interface.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>
// ros messages
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <cr_control/wheel_data.h>
// for debugging
#include <bitset>

WheelHardwareInterface::WheelHardwareInterface(ros::NodeHandle* nh, WheelHwinSettings* wheelSettings)
{
    // data publishing setup
    this->nodeHandle = nh;
    roverDataPub = nh->advertise<cr_control::wheel_data>("wheel/data", 1000);
    test = nh->advertise<std_msgs::Float64>("WheelVelocity", 1000);

    // store and calibrate imu
    // 0x28 is the default address for the imu
    // imu = BNO055(-1, 0x28, 1); 
    imu.begin(imu.OPERATION_MODE_NDOF);

    this->wheelSettings = wheelSettings;
    ROS_INFO("Registering ros_control joint interfaces");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 4; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

WheelHardwareInterface::~WheelHardwareInterface() { }

void WheelHardwareInterface::writeToWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        ROS_INFO_STREAM("========== DEBUG write() =======");
        ROS_INFO_STREAM("Command Values: cmd[0]: " << cmd[0] << " cmd[1]: " << cmd[1] << " cmd[2]: " << cmd[2] << " cmd[3]: " << cmd[3] << "\n");
    }

    // check that roboclaws are on before sending commands

    // need to divide cmd by 10 because diff drive controller 
    // multiplies topic input by 10 for some reason
    for (int i = 0; i < 4; i++)
        cmd[i] /= 10;
    // sendCommandToWheels(rb);
    driveWithSpeed(rb);
}

void WheelHardwareInterface::readFromWheels(Roboclaw *rb)
{

    cr_control::wheel_data msg;
    RoboclawMotorCurrents motorCurrentsFront = rb->ReadMotorCurrents(128);
    RoboclawMotorCurrents motorCurrentsBack = rb->ReadMotorCurrents(129);
    msg.m1FrontAmps = motorCurrentsFront.m1Current;
    msg.m2FrontAmps = motorCurrentsFront.m2Current;
    msg.m1BackAmps = motorCurrentsBack.m1Current;
    msg.m2BackAmps = motorCurrentsBack.m2Current;

    msg.voltageFront = rb->ReadMainBatteryVoltage(128);
    msg.voltageBack = rb->ReadMainBatteryVoltage(129);

    msg.m1FrontVelocity = encoderCountToRevolutions(rb->ReadEncoderSpeedM1(128));
    msg.m2FrontVelocity = encoderCountToRevolutions(rb->ReadEncoderSpeedM2(128));
    msg.m1BackVelocity = encoderCountToRevolutions(rb->ReadEncoderSpeedM1(129));
    msg.m2BackVelocity = encoderCountToRevolutions(rb->ReadEncoderSpeedM2(129));

    // Read from imu and do error checking
    uint8_t system_status, self_test_results, system_error;
    imu.getSystemStatus(&system_status, &self_test_results, &system_error);
    if (system_status == 1) 
    {
        std::cout << "imu error detected... restarting imu..." << std::endl;
        imu.begin(imu.OPERATION_MODE_NDOF);
        std::cout << "imu restarted" << std::endl;
    }
    imu::Quaternion quat = imu.getQuat();
    imu::Vector<3> lin_accel_body_frame = imu.getVector(BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> orientation_body_frame = imu.getVector(BNO055::VECTOR_EULER);
    imu::Vector<3> lin_accel_ned_frame = accel_body_frame_to_ned_frame(quat.w(), quat.x(), quat.y(), quat.z(), 
        lin_accel_body_frame.x(), lin_accel_body_frame.y(), lin_accel_body_frame.z());
    
    msg.xAccelImu0 = lin_accel_ned_frame.x();
    msg.yAccelImu0 = lin_accel_ned_frame.y();
    msg.zAccelImu0 = lin_accel_ned_frame.z();

    msg.wOrientationImu0 = quat.w();
    msg.xOrientationImu0 = quat.x();
    msg.yOrientationImu0 = quat.y();
    msg.zOrientationImu0 = quat.z();

    // if (wheelSettings->debugMode) {
    //     rb->GetVelocityFromWheels(vel);

    //     ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
    // }

    roverDataPub.publish(msg);
}

void WheelHardwareInterface::sendCommandToWheels(Roboclaw* rb)
{
    // convert cmd_vel to a usable command between 0-127
    scaleCommands();

    // prevent zero velocity spamming from ros_control
    if (zeroCmdVelCount <= wheelSettings->maxRetries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            rb->ForwardM2(0x80, cmdToSend[0]);
        else
            rb->BackwardM2(0x80, cmdToSend[0]);

        if (cmd[1] >= 0)  // right_back
            rb->ForwardM2(0x81, cmdToSend[1]);
        else
            rb->BackwardM2(0x81, cmdToSend[1]);

        if (cmd[2] >= 0)  // left_front
            rb->ForwardM1(0x80, cmdToSend[2]);
        else
            rb->BackwardM1(0x80, cmdToSend[2]);

        if (cmd[3] >= 0)  // left_back
            rb->ForwardM1(0x81, cmdToSend[3]);
        else
            rb->BackwardM1(0x81, cmdToSend[3]);

    }
    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
    }
}

void WheelHardwareInterface::driveWithSpeed(Roboclaw *rb)
{
    int32_t cmd_to_send[4];
    for (int i = 0; i < 4; i++)
    {
        cmd_to_send[i] = revolutionsToEncoderCount(cmd[i]);
    }
    // ROS_INFO_STREAM("cmd[0]: " << cmd[0]);
    // ROS_INFO_STREAM("cmd_to_send[0]: " << cmd_to_send[0]);
    // speed = 4 cm/s
    // speed / 2 pi 7.5cm * ticks/rev = ticks/s

    float encoderTicksPerRev = wheelSettings->encoderTicksPerRevolution;
    rb->DriveSpeedAccelM2(128, encoderTicksPerRev / 2, cmd_to_send[0]);
    rb->DriveSpeedAccelM2(129, encoderTicksPerRev / 2, cmd_to_send[1]);
    rb->DriveSpeedAccelM1(128, encoderTicksPerRev / 2, cmd_to_send[2]);
    rb->DriveSpeedAccelM1(129, encoderTicksPerRev / 2, cmd_to_send[3]);

    cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
}

void WheelHardwareInterface::scaleCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
    for (int i = 0; i < 4; i++)
    {
        double res = (fabs(cmd[i]) / 16.667) * wheelSettings->maxEffortValue;

        if (res >= wheelSettings->maxEffortValue)
            cmdToSend[i] = wheelSettings->maxEffortValue;
        else
            cmdToSend[i] = (uint8_t) res;
    }
}

float WheelHardwareInterface::encoderCountToRevolutions(int32_t encoderCount)
{
    return ((float)encoderCount) * wheelSettings->revolutionsPerEncoderTick;
}

// int32 because roboclaws expect an int32 not a float
int32_t WheelHardwareInterface::revolutionsToEncoderCount(float revolutions)
{
    return revolutions * wheelSettings->encoderTicksPerRevolution;
}

ros::Time WheelHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration WheelHardwareInterface::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void WheelHardwareInterface::registerStateHandlers()
{
    ROS_INFO("Registering Joint State Interface");
    hardware_interface::JointStateHandle wheelRightFront(
        wheelSettings->rightWheelNames[0], &pos[0], &vel[0], &eff[0]);
    jointStateInterface.registerHandle(wheelRightFront);

    hardware_interface::JointStateHandle wheelRightBack(
        wheelSettings->rightWheelNames[1], &pos[1], &vel[1], &eff[1]);
    jointStateInterface.registerHandle(wheelRightBack);

    hardware_interface::JointStateHandle wheelLeftFront(
        wheelSettings->leftWheelNames[0], &pos[2], &vel[2], &eff[2]);
    jointStateInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointStateHandle wheelLeftBack(
        wheelSettings->leftWheelNames[1], &pos[3], &vel[3], &eff[3]);
    jointStateInterface.registerHandle(wheelLeftBack);

    registerInterface(&jointStateInterface);
}
void WheelHardwareInterface::registerJointVelocityHandlers()
{
    ROS_INFO("Registering Velocity Joint Interface");
    hardware_interface::JointHandle wheelRightFront(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[0]), &cmd[0]);
    velocityJointInterface.registerHandle(wheelRightFront);

    hardware_interface::JointHandle wheelRightBack(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[1]), &cmd[1]);
    velocityJointInterface.registerHandle(wheelRightBack);

    hardware_interface::JointHandle wheelLeftFront(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[0]), &cmd[2]);
    velocityJointInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointHandle wheelLeftBack(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[1]), &cmd[3]);
    velocityJointInterface.registerHandle(wheelLeftBack);

    registerInterface(&velocityJointInterface);
}

imu::Vector<3> WheelHardwareInterface::accel_body_frame_to_ned_frame(double qW, double qX, double qY, double qZ, double aX, double aY, double aZ)
{
    imu::Vector<3> acc_NEDframe;

    acc_NEDframe[0] = (qW * qW + qX * qX - qY * qY - qZ * qZ) * aX + 2 * (qX * qY - qW * qZ) * aY + 2 * (qW * qY + qX * qZ) * aZ;
    acc_NEDframe[1] = 2 * (qX * qY + qW * qZ) * aX + (qW * qW - qX * qX + qY * qY - qZ * qZ) * aY + 2 * (qY * qZ - qW * qX) * aZ;
    acc_NEDframe[2] = 2 * (qX * qZ + qW * qY) * aX + 2 * (qY * qZ + qW * qX) * aY + (qW * qW - qX * qX - qY * qY + qZ * qZ) * aZ;

    return acc_NEDframe;
}