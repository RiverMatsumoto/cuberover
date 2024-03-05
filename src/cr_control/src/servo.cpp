#include <ros/ros.h>
#include <JetsonGPIO.h>
#include <std_msgs/Float64.h>
#include <cr_control/ServoState.h>
#include <string>
#include <unistd.h>

int pwm_pin = 15;
double degrees;
ros::ServiceServer servo_service;
GPIO::PWM *p;

void servo(const std_msgs::Float64 &msg)
{
    // range 20-46.6 == 0 degrees-90 degrees
    degrees = msg.data;
    double duty = ((degrees * 26.6) / 90.0) + 20.0;
    p->ChangeDutyCycle(duty);
}

bool servo_state_service_cb(cr_control::ServoState::Request &req,
                        cr_control::ServoState::Response &res)
{
    res.angle = degrees;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle nh;
    nh.getParam("cuberover/config/pwm_pin", pwm_pin);

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(pwm_pin, GPIO::OUT, GPIO::LOW);
    GPIO::PWM temp(pwm_pin, 400);
    p = &temp;
    p->start(20); // 20 maps to -> 0 degree position

    servo_service = nh.advertiseService("servo_state", servo_state_service_cb);
    ros::Subscriber servo_sub = nh.subscribe("cuberover/servo_angle", 5, servo);
    ROS_INFO("Started servo node. Waiting for Float64 messages at topic /servo_node/servo_angle");
    ros::spin();
    p->ChangeDutyCycle(20);
    usleep(50000);
    GPIO::cleanup();
    return 0;
}
