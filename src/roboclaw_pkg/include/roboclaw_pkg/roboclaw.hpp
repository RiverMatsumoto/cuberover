#pragma once
#include <string>
#include "simple_uart.h"

// command enums
#define ROBOCLAW_M1_FORWARD                     0
#define ROBOCLAW_M1_BACKWARD                    1
#define ROBOCLAW_M2_FORWARD                     4
#define ROBOCLAW_M2_BACKWARD                    5
#define ROBOCLAW_M1_READ_ENCODER_POSITION       16
#define ROBOCLAW_M2_READ_ENCODER_POSITION       17
#define ROBOCLAW_M1_READ_ENCODER_SPEED          18
#define ROBOCLAW_M2_READ_ENCODER_SPEED          19
#define ROBOCLAW_READ_MAIN_BATTERY_VOLTAGE      24
#define ROBOCLAW_M1_DRIVE_AT_SPEED              35
#define ROBOCLAW_M2_DRIVE_AT_SPEED              36
#define ROBOCLAW_M1_DRIVE_AT_SPEED_ACCEL        38
#define ROBOCLAW_M2_DRIVE_AT_SPEED_ACCEL        39
#define ROBOCLAW_READ_MOTOR_CURRENTS            49
#define ROBOCLAW_M1_READ_PID_QPPS_SETTINGS      55
#define ROBOCLAW_M2_READ_PID_QPPS_SETTINGS      56
#define ROBOCLAW_M1_MOVE_TO_POSITION            119
#define ROBOCLAW_M2_MOVE_TO_POSITION            120

struct RoboclawSettings
{
    int baudrate = 115200;
    std::string port;
    std::string flags;
};

class Roboclaw {
public:
    Roboclaw(RoboclawSettings settings);
    ~Roboclaw();
    int init_uart();

    void ForwardM1(uint8_t address, uint8_t value);
    void ForwardM2(uint8_t address, uint8_t value);
    void BackwardM1(uint8_t address, uint8_t value);
    void BackwardM2(uint8_t address, uint8_t value);

private:
    void send_command();

    uint8_t writeBuffer[64]; // send data
    uint8_t readBuffer[64]; // receive data
    simple_uart *uart;
    RoboclawSettings settings;
};