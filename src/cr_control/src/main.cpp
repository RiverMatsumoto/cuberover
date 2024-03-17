#include <cr_control/wheel_hardware_interface.h>
#include <std_msgs/Int8.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>

void GetYamlParameters(ros::NodeHandle*, WheelHwinSettings*, RoboclawSettings*);
bool validateSettingsAndLogErrors(WheelHwinSettings*);

WheelHardwareInterface *hwin_ptr;

void linearActuator(const std_msgs::Int8& msg)
{
    hwin_ptr->linearActuator(msg.data);
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Loading wheel_hardware_interface_node");
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("Getting yaml parameters for wheel hardware interface settings");
    WheelHwinSettings wheelSettings;
    RoboclawSettings roboclawSettings;
    GetYamlParameters(&nh, &wheelSettings, &roboclawSettings);

    ROS_INFO("Initializing wheel hardware interface");
    WheelHardwareInterface wheelHwin(&nh, &wheelSettings);
    hwin_ptr = &wheelHwin;

    ROS_INFO("Initializing controller manager");
    controller_manager::ControllerManager cmWheel(&wheelHwin);

    ros::Subscriber linear_actuator_sub = nh.subscribe("cuberover/linear_actuator", 10, linearActuator);

    ROS_INFO("Initializing roboclaw interface");
    Roboclaw roboclaw(&roboclawSettings);
    ros::Rate rate(wheelSettings.rosLoopRate);

    while (ros::ok()) 
    {
        wheelHwin.readFromWheels(&roboclaw);
        cmWheel.update(wheelHwin.get_time(), wheelHwin.get_period());
        wheelHwin.writeToWheels(&roboclaw);
        rate.sleep();
    }
    
    roboclaw.ForwardM1(128, 0);
    roboclaw.ForwardM2(128, 0);
    roboclaw.ForwardM1(129, 0);
    roboclaw.ForwardM2(129, 0);
    
    return 0;
}

void GetYamlParameters(ros::NodeHandle* nh, 
    WheelHwinSettings *wheelSettings, 
    RoboclawSettings *roboclawSettings) 
{

    for (int i = 0; i < 2; i++) {
        wheelSettings->rightWheelRoboclawAddresses[i] = 0;
        wheelSettings->leftWheelRoboclawAddresses[i] = 0;
    }

    nh->getParam("/roboclaw_settings/serial_port", roboclawSettings->serialPortAddress);
    nh->getParam("/roboclaw_settings/send_command_retries", roboclawSettings->retries);
    nh->getParam("/roboclaw_settings/encoder_timeout_ms", roboclawSettings->timeoutMs);
    nh->getParam("/roboclaw_settings/loop_frequency", roboclawSettings->loopFrequency);
    nh->getParam("/roboclaw_settings/baud_rate", roboclawSettings->baudRate);

    std::vector<std::string> rightWheelNames, leftWheelNames; // list of wheel joint names
    std::vector<int> leftWheelAddresses, rightWheelAddresses; // each wheel's corresponding roboclaw addresses
    nh->getParam("/wheel_hwin_settings/debug_mode", wheelSettings->debugMode);
    nh->getParam("/roboclaw_settings/send_command_retries", wheelSettings->maxRetries);
    nh->getParam("/wheel_hwin_settings/ros_loop_rate", wheelSettings->rosLoopRate);
    nh->getParam("/wheel_hwin_settings/left_addr", leftWheelAddresses);
    nh->getParam("/wheel_hwin_settings/right_addr", rightWheelAddresses);
    nh->getParam("/wheel_hwin_settings/left_wheel", leftWheelNames);
    nh->getParam("/wheel_hwin_settings/right_wheel", rightWheelNames);
    nh->getParam("/wheel_hwin_settings/use_imu0", wheelSettings->use_imu0);
    nh->getParam("/wheel_hwin_settings/use_imu1", wheelSettings->use_imu1);
    nh->getParam("/wheel_hwin_settings/in1_pin", wheelSettings->in1_pin);
    nh->getParam("/wheel_hwin_settings/in2_pin", wheelSettings->in2_pin);

    for (int i = 0; i < 2; i++) {
        // copy ros_params settings into wheel settings struct
        std::copy(leftWheelNames.begin(), leftWheelNames.end(), wheelSettings->leftWheelNames);
        std::copy(rightWheelNames.begin(), rightWheelNames.end(), wheelSettings->rightWheelNames);
        std::copy(leftWheelAddresses.begin(), leftWheelAddresses.end(), wheelSettings->leftWheelRoboclawAddresses);
        std::copy(rightWheelAddresses.begin(), rightWheelAddresses.end(), wheelSettings->rightWheelRoboclawAddresses);

    }

    nh->getParam("/wheel_hwin_settings/motor_data/encoderTicks_per_revolution", wheelSettings->encoderTicksPerRevolution);
    nh->getParam("/wheel_hwin_settings/motor_data/max_encoder_speed", wheelSettings->maxEncoderSpeed);
    wheelSettings->revolutionsPerEncoderTick = 1.0f / wheelSettings->encoderTicksPerRevolution;

    bool errorFlag = validateSettingsAndLogErrors(wheelSettings);

    if (errorFlag)
        exit(EXIT_FAILURE);
}

bool validateSettingsAndLogErrors(WheelHwinSettings *wheelSettings)
{
    bool errorFlag = false;
    for (int i = 0; i < 2; i++)
    {
        // error checking
        if (wheelSettings->rightWheelNames[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->rightWheelRoboclawAddresses[i] < 128 ||
             wheelSettings->rightWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelNames[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelRoboclawAddresses[i] < 128 || 
            wheelSettings->leftWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }
    }

    return errorFlag;
}