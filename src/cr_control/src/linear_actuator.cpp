#include <cr_control/linear_actuator.h>

LinearActuatorHwin::LinearActuatorHwin(ros::NodeHandle *nh, LinearActuatorSettings ls)
{
    this->settings = ls;
    // register joint interfaces for ros_control
    hardware_interface::JointStateHandle la(
        "joint_linear_actuator", &cmd, &pos, &vel, &eff);
    jointStateInterface.registerHandle(la);
    registerInterface(&la);

    hardware_interface::JointHandle laJoint(
        jointStateInterface.getHandle("joint_linear_actuator"), &cmd);
    velocityJointInterface.registerHandle(laJoint);
    registerInterface(&laJoint);

    // initialize last_time
    clock_gettime(CLOCK_MONOTONIC, &last_time);
}

LinearActuatorHwin::~LinearActuatorHwin()
{

}

void LinearActuatorHwin::write(Roboclaw *rb)
{
    if (cmd > 0)
    {
        rb->ForwardM1(settings.roboclaw_address, 126);
    }
    else if (cmd < 0)
    {
        rb->BackwardM1(settings.roboclaw_address, -126);
    }
    else
    {
        rb->ForwardM1(settings.roboclaw_address, 0);
    }
}

void LinearActuatorHwin::read(Roboclaw *rb)
{

}

ros::Time LinearActuatorHwin::get_time()
{
    return ros::Time::now();
}

ros::Duration LinearActuatorHwin::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0);
    last_time = current_time;

    return elapsed_time;
}