#include <iostream>
#include "cr_control/roboclaw.h"

using namespace std;

int main(int argc, char**argv)
{
    RoboclawSettings s;
    s.serialPortAddress = "/dev/ttyTHS0";
    Roboclaw rb(&s);

    float voltage = rb.ReadMainBatteryVoltage(128);
    cout << "Voltage: " << voltage << endl;

    return 0;
}