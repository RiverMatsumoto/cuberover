/* MIT License
Copyright (c) [2022] [VIP Team RoSE]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef CR_CONTROL_ROBOCLAW_H
#define CR_CONTROL_ROBOCLAW_H

#include <ros/ros.h>
#include <stdint.h>
#include <termios.h>

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
    std::string serialPortAddress;
    int addresses[8]; // 128 - 136
    int timeoutMs                    = 12;
    int retries                     = 3;
    int baudRate                    = 115200;
    const int maxBufferSize               = 100;
    const uint8_t maxEffortValue = 126;
    double loopFrequency            = 10;
    bool debugMode                  = false;
};

struct RoboclawPidQppsSettings
{
    float P;
    float I;
    float D;
    uint32_t Qpps; // quadrature pulses per second
};

struct RoboclawMotorCurrents
{
    float m1Current;
    float m2Current;
};

class Roboclaw 
{
 public:
    explicit Roboclaw(RoboclawSettings*);
    ~Roboclaw();
    void SetupEncoders();
    void CloseEncoders();
    void SendCommandToWheels(double* cmd);
    void GetVelocityFromWheels(double* vel);

    void ForwardM1(uint8_t address, uint8_t value);
    void ForwardM2(uint8_t address, uint8_t value);
    void BackwardM1(uint8_t address, uint8_t value);
    void BackwardM2(uint8_t address, uint8_t value);
    void DriveSpeedM1(uint8_t address, int32_t speed);
    void DriveSpeedM2(uint8_t address, int32_t speed);
    void DriveSpeedAccelM1(uint8_t address, int32_t acceleration, int32_t speed);
    void DriveSpeedAccelM2(uint8_t address, int32_t acceleration, int32_t speed);
    RoboclawPidQppsSettings ReadPidQppsSettingsM1(uint8_t address);
    RoboclawPidQppsSettings ReadPidQppsSettingsM2(uint8_t address);
    float ReadMainBatteryVoltage(uint8_t address);
    RoboclawMotorCurrents ReadMotorCurrents(uint8_t address);
    int32_t ReadEncoderPositionM1(uint8_t address);
    int32_t ReadEncoderPositionM2(uint8_t address);
    int32_t ReadEncoderSpeedM1(uint8_t address);
    int32_t ReadEncoderSpeedM2(uint8_t address);
    void MoveToPositionM1(uint8_t address, uint32_t value);
    void MoveToPositionM2(uint8_t address, int32_t value);

 private:
    void GetBaudRate();
    int ClearIOBuffers();
    int WriteToEncoders(uint8_t* data, int nBytes);
    int WaitReadStatus(int nBytes, int timeout_ms);
    int ReadFromRoboclaw(int nBytes);
    int SendCommands(uint8_t* data, int writeBytes, int readBytes);
    double ConvertPulsesToRadians(double vel);
    uint8_t ScaleCommand(double cmd);
    uint16_t CalculateChecksum(uint8_t* packet, int nBytes);
    uint32_t RecombineBuffer(uint8_t* buf);
    void Copy_uint16_from_bytes(uint16_t& to, uint8_t* from);
    void Copy_uint32_from_bytes(uint32_t& to, uint8_t* from);
    void Copy_int16_from_bytes(int16_t& to, uint8_t* from);
    void Copy_int32_from_bytes(int32_t& to, uint8_t* from);
    void Copy_bytes_from_int32(uint8_t* to, int32_t from);

    termios tty;
    RoboclawSettings* settings;
    int serialPort;
    int zeroCmdVelCount;
    unsigned int baudRate;  // instead of uint32_t for compatability
    uint8_t buf[100];
    char* errorBufPtr;
    char errorBuf[256];     // used by strerror_r, thread safe
};

#endif  // CR_CONTROL_ROBOCLAW_H
