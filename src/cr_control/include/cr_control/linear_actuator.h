
#ifndef CR_CONTROL_LINEAR_ACTUATOR_H
#define CR_CONTROL_LINEAR_ACTUATOR_H

#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <hardware_interface/robot_hw.h> 
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <cr_control/roboclaw.h> // motor controller

struct LinearActuatorSettings
{
    int roboclaw_address = 130;
};

//might need serial to send commands to the hardware
class LinearActuatorHwin : public hardware_interface::RobotHW
{
public:
    LinearActuatorHwin(ros::NodeHandle *, LinearActuatorSettings ls);
    ~LinearActuatorHwin();
    void write(Roboclaw*);
    void read(Roboclaw*);

    ros::Time get_time();
    ros::Duration get_period();

private:
    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;

    LinearActuatorSettings settings;

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    // For reading commands sent from the controller
    double cmd;
    // for sending data relating to the joints
    double pos;
    double vel;
    double eff;
    ros::NodeHandle* nodeHandle;
};

#endif
