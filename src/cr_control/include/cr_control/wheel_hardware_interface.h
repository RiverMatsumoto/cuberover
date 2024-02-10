#ifndef CR_CONTROL_WHEEL_HARDWARE_INTERFACE_H
#define CR_CONTROL_WHEEL_HARDWARE_INTERFACE_H

// essential header files
#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h> // to have a controller in our class
#include <hardware_interface/robot_hw.h> // useful functions
#include <ros/ros.h>
#include <string>

#include <cr_control/roboclaw.h> // motor controller
#include <cr_control/bno055/BNO055.h> // imu

struct WheelHwinSettings
{
    std::string leftWheelNames[2];
    std::string rightWheelNames[2];
    int leftWheelRoboclawAddresses[2]; // Corresponding wheel to roboclaw address
    int rightWheelRoboclawAddresses[2];
    uint8_t maxEffortValue = 126;
    int rosLoopRate = 10;
    int maxRetries = 3;
    bool debugMode = false;
    bool use_imu0 = false;
    bool use_imu1 = false;
    float encoderTicksPerRevolution;
    float revolutionsPerEncoderTick;
};

//might need serial to send commands to the hardware
class WheelHardwareInterface : public hardware_interface::RobotHW
{
public:
    WheelHardwareInterface(ros::NodeHandle *, WheelHwinSettings*);
    ~WheelHardwareInterface();
    void writeToWheels(Roboclaw*);
    void readFromWheels(Roboclaw*);

    ros::Time get_time();
    ros::Duration get_period();

private:
    WheelHwinSettings *wheelSettings;

    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;
    
    void registerStateHandlers();
    void registerJointVelocityHandlers();

    void sendCommandToWheels(Roboclaw *);
    void driveWithSpeed(Roboclaw *);
    void scaleCommands();
    float encoderCountToRevolutions(int32_t encoderCount);
    int32_t revolutionsToEncoderCount(float revolutions);
    imu::Vector<3> accel_body_frame_to_ned_frame(double qW, double qX, double qY, double qZ, double aX, double aY, double aZ);

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    int zeroCmdVelCount;
    // For reading commands sent from the controller
    double cmd[4];
    uint8_t cmdToSend[4];
    // for sending data relating to the joints
    double pos[4];
    double vel[4];
    double eff[4];
    static constexpr double BILLION = 1000000000.0; // nanoseconds to seconds
    ros::NodeHandle* nodeHandle;
    ros::Publisher roverDataPub;
    ros::Publisher test;

    // imu for gathering linear acceleration and absolute orientation
    BNO055 imu0 = BNO055(-1, BNO055_ADDRESS_A, 7);
    BNO055 imu1 = BNO055(0, BNO055_ADDRESS_B, 7);
};

#endif
