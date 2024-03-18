#include <ros/ros.h>
#include <JetsonGPIO.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <cr_control/LightState.h>

int output_pin = 15;
bool light_state = false;
ros::ServiceServer light_service;

void light_callback(const std_msgs::Bool &on)
{
    GPIO::output(output_pin, on.data);
    light_state = on.data;
}

void toggle_light_callback(const std_msgs::Empty &msg)
{
    light_state = !light_state;
    GPIO::output(output_pin, light_state);
}

bool light_service_cb(cr_control::LightState::Request &req,
                    cr_control::LightState::Response &res)
{
    res.light_state = light_state;
    return true; // service successful
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "light_node");
    ros::NodeHandle nh;

    nh.getParam("cuberover/config/light_pin", output_pin);

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(output_pin, GPIO::OUT, GPIO::LOW);

    light_service = nh.advertiseService("light_state", light_service_cb);
    ros::Subscriber light_sub = nh.subscribe("cuberover/light", 5, light_callback);
    ros::Subscriber toggle_light_sub = nh.subscribe("cuberover/toggle_light", 5, toggle_light_callback);
    ROS_INFO("Started light node");
    ros::spin();

    GPIO::cleanup();

    return 0;
}