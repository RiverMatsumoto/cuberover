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
#include <JetsonGPIO.h>
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

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(wheelSettings->in1_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(wheelSettings->in2_pin, GPIO::OUT, GPIO::LOW);

    // store and calibrate imu
    // 0x28 is the default address for the imu
    // imu = BNO055(-1, 0x28, 1); 
    if (wheelSettings->use_imu0)
        imu0.begin(imu0.OPERATION_MODE_NDOF);
    if (wheelSettings->use_imu1)
        imu1.begin(imu1.OPERATION_MODE_NDOF);

    this->wheelSettings = wheelSettings;
    ROS_INFO("Registering ros_control joint interfaces");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 5; i++)
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
    if (cmd > 0)
        rb->ForwardM1(wheelSettings->linearActuatorAddress, 126);
    else if (cmd < 0)
        rb->BackwardM1(wheelSettings->linearActuatorAddress, 126);
    else
        rb->ForwardM1(wheelSettings->linearActuatorAddress, 0);

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
    uint8_t system_status0, self_test_results0, system_error0;
    if (wheelSettings->use_imu0)
    {
        imu0.getSystemStatus(&system_status0, &self_test_results0, &system_error0);
        if (system_status0 == 1) 
        {
            std::cout << "imu0 error detected... restarting imu..." << std::endl;
            imu0.begin(imu0.OPERATION_MODE_NDOF);
            std::cout << "imu0 restarted" << std::endl;
        }
    }
    if (wheelSettings->use_imu1)
    {
        uint8_t system_status1, self_test_results1, system_error1;
        imu1.getSystemStatus(&system_status1, &self_test_results1, &system_error1);
        if (system_status1 == 1) 
        {
            std::cout << "imu1 error detected... restarting imu..." << std::endl;
            imu1.begin(imu1.OPERATION_MODE_NDOF);
            std::cout << "imu1 restarted" << std::endl;
        }
    }

    // collect data from imus and store in ros message
    if (wheelSettings->use_imu0)
    {
        imu::Quaternion quat0 = imu0.getQuat();
        imu::Vector<3> lin_accel_body_frame0 = imu0.getVector(BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> orientation_body_frame0 = imu0.getVector(BNO055::VECTOR_EULER);
        imu::Vector<3> lin_accel_ned_frame0 = accel_body_frame_to_ned_frame(
            quat0.w(), 
            quat0.x(), 
            quat0.y(), 
            quat0.z(), 
            lin_accel_body_frame0.x(), 
            lin_accel_body_frame0.y(), 
            lin_accel_body_frame0.z());
        
        msg.xAccelImu0 = lin_accel_ned_frame0.x();
        msg.yAccelImu0 = lin_accel_ned_frame0.y();
        msg.zAccelImu0 = lin_accel_ned_frame0.z();

        msg.wOrientationImu0 = quat0.w();
        msg.xOrientationImu0 = quat0.x();
        msg.yOrientationImu0 = quat0.y();
        msg.zOrientationImu0 = quat0.z();
    }

    if (wheelSettings->use_imu1)
    {
        imu::Quaternion quat1 = imu1.getQuat();
        imu::Vector<3> lin_accel_body_frame1 = imu1.getVector(BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> orientation_body_frame1 = imu1.getVector(BNO055::VECTOR_EULER);
        imu::Vector<3> lin_accel_ned_frame1 = accel_body_frame_to_ned_frame(
            quat1.w(), 
            quat1.x(), 
            quat1.y(), 
            quat1.z(), 
            lin_accel_body_frame1.x(), 
            lin_accel_body_frame1.y(), 
            lin_accel_body_frame1.z());
        
        msg.xAccelImu1 = lin_accel_ned_frame1.x();
        msg.yAccelImu1 = lin_accel_ned_frame1.y();
        msg.zAccelImu1 = lin_accel_ned_frame1.z();

        msg.wOrientationImu1 = quat1.w();
        msg.xOrientationImu1 = quat1.x();
        msg.yOrientationImu1 = quat1.y();
        msg.zOrientationImu1 = quat1.z();
    }

    roverDataPub.publish(msg);
}

void WheelHardwareInterface::sendCommandToWheels(Roboclaw* rb)
{
    // convert cmd_vel to a usable command between 0-127
    scaleCommands();

    // prevent zero velocity spamming from ros_control
    // if (zeroCmdVelCount <= wheelSettings->maxRetries) {
        ROS_INFO_STREAM("cmd 0: " << cmd[0]);
        ROS_INFO_STREAM("cmd 1: " << cmd[1]);
        ROS_INFO_STREAM("cmd 2: " << cmd[2]);
        ROS_INFO_STREAM("cmd 3: " << cmd[3]);
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] > 0)  // right_front
            rb->ForwardM2(128, cmdToSend[0]);
        else if (cmd[0] < 0)
            rb->BackwardM2(128, cmdToSend[0]);
        else
            rb->ForwardM2(128, 1);

        if (cmd[1] > 0)  // right_back
            rb->ForwardM2(129, cmdToSend[1]);
        else if (cmd[1] < 0)  // right_back
            rb->BackwardM2(129, cmdToSend[1]);
        else
            rb->ForwardM2(129, 1);

        if (cmd[2] > 0)  // left_front
            rb->ForwardM1(128, cmdToSend[2]);
        else if (cmd[2] < 0)  // left_front
            rb->BackwardM1(128, cmdToSend[2]);
        else
            rb->ForwardM1(128, 1);

        if (cmd[3] > 0)  // left_back
        {
            ROS_INFO("Forward command");
            rb->ForwardM1(129, cmdToSend[3]);
        }
        else if (cmd[3] < 0)  // left_back
        {
            ROS_INFO("Backwards command");
            rb->BackwardM1(129, cmdToSend[3]);
        }
        else
        {
            ROS_INFO("Zero command");
            rb->ForwardM1(129, 1);
        }

    // }
    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 &&
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
        cmd_to_send[i] = cmd[i] * wheelSettings->maxEncoderSpeed;
    }
    // ROS_INFO_STREAM("cmd[0]: " << cmd[0]);
    // ROS_INFO_STREAM("cmd_to_send[0]: " << cmd_to_send[0]);
    // speed = 4 cm/s
    // speed / 2 pi 7.5cm * ticks/rev = ticks/s

    float encoderTicksPerRev = wheelSettings->encoderTicksPerRevolution;
    // ROS_INFO_STREAM("cmd 0: " << cmd_to_send[0]);
    // ROS_INFO_STREAM("cmd 1: " << cmd_to_send[1]);
    // ROS_INFO_STREAM("cmd 2: " << cmd_to_send[2]);
    // ROS_INFO_STREAM("cmd 3: " << cmd_to_send[3]);
    
    if (cmd_to_send[0] == 0 &&
        cmd_to_send[1] == 0 &&
        cmd_to_send[2] == 0 &&
        cmd_to_send[3] == 0)
    {
        rb->DriveSpeedAccelM2(128, (int)(encoderTicksPerRev / 2), 0);
        rb->DriveSpeedAccelM2(129, (int)(encoderTicksPerRev / 2), 0);
        rb->DriveSpeedAccelM1(128, (int)(encoderTicksPerRev / 2), 0);
        rb->DriveSpeedAccelM1(129, (int)(encoderTicksPerRev / 2), 0);
    }
    else
    {
        rb->DriveSpeedAccelM2(128, (int)(encoderTicksPerRev / 1.5), cmd_to_send[0]);
        rb->DriveSpeedAccelM2(129, (int)(encoderTicksPerRev / 1.5), cmd_to_send[1]);
        rb->DriveSpeedAccelM1(128, (int)(encoderTicksPerRev / 1.5), cmd_to_send[2]);
        rb->DriveSpeedAccelM1(129, (int)(encoderTicksPerRev / 1.5), cmd_to_send[3]);
    }

    if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 &&
        cmd[3] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
    }
}

void WheelHardwareInterface::linearActuator(int8_t la_cmd)
{
    ROS_INFO_STREAM("msg: " << int(la_cmd));
    if (la_cmd > 0)
    {
        // forward
        if (in1_pin_state != 1)
            GPIO::output(wheelSettings->in1_pin, 1);
        if (in2_pin_state != 0)
            GPIO::output(wheelSettings->in2_pin, 0);
        in1_pin_state = 1;
        in2_pin_state = 0;
    }
    else if (la_cmd < 0)
    {
        // backwards
        if (in1_pin_state != 0)
            GPIO::output(wheelSettings->in1_pin, 0);
        if (in2_pin_state != 1)
            GPIO::output(wheelSettings->in2_pin, 1);
        in1_pin_state = 0;
        in2_pin_state = 1;
    }
    else
    {
        // stop
        if (in1_pin_state != 0)
            GPIO::output(wheelSettings->in1_pin, 0);
        if (in2_pin_state != 0)
            GPIO::output(wheelSettings->in2_pin, 0);
        in1_pin_state = 0;
        in2_pin_state = 0;
    }
}

void WheelHardwareInterface::scaleCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
    for (int i = 0; i < 4; i++)
    {
        double res = fabs(cmd[i]) * wheelSettings->maxEffortValue;

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