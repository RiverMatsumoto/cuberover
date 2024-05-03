#include <JetsonGPIO.h>
#include <ros/ros.h>

int cw_pin;
int clk_pin;

// calibrate topic
void calibrate_stepper_cb()
{
    
}

void set_target_position()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stepper_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(cw_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(clk_pin, GPIO::OUT, GPIO::LOW);

    spinner.start();
    while (ros::ok())
    {
        // move toward target step

    }

    return 0;
}