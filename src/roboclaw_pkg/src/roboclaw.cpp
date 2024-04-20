#include "roboclaw_pkg/roboclaw.hpp"

Roboclaw::Roboclaw(RoboclawSettings s) : settings(s)
{

}

Roboclaw::~Roboclaw()
{

}

int Roboclaw::init_uart()
{
    return 0;
}


