// io
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>

// imu header
#include <cr_control/bno055/BNO055.h>

using namespace std;

imu::Vector<3> Acc_bframe_to_acc_NEDframe(double qW, double qX, double qY, double qZ, double aX, double aY, double aZ)
{

    // static double acc_NEDframe[3]; // X, Y, Z
    imu::Vector<3> acc_NEDframe; // X, Y, Z


    acc_NEDframe[0] = (qW * qW + qX * qX - qY * qY - qZ * qZ) * aX + 2 * (qX * qY - qW * qZ) * aY + 2 * (qW * qY + qX * qZ) * aZ;
    acc_NEDframe[1] = 2 * (qX * qY + qW * qZ) * aX + (qW * qW - qX * qX + qY * qY - qZ * qZ) * aY + 2 * (qY * qZ - qW * qX) * aZ;
    acc_NEDframe[2] = 2 * (qX * qZ + qW * qY) * aX + 2 * (qY * qZ + qW * qX) * aY + (qW * qW - qX * qX - qY * qY + qZ * qZ) * aZ;

    return acc_NEDframe;
}

int main()
{

    imu::Vector<3> acc_NEDframe;
    double *gyr_NEDframe;
    BNO055 bno = BNO055(-1, BNO055_ADDRESS_B, 7);
    bno.begin(bno.OPERATION_MODE_NDOF_FMC_OFF);
    usleep(500000);

    int temp = bno.getTemp();
    std::cout << "Current Temperature: " << temp << " C" << std::endl;
    bno.setExtCrystalUse(true);
    usleep(500000);

    imu::Vector<3> gyr_bframe;

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> acc_bframe = bno.getVector(BNO055::VECTOR_ACCELEROMETER);
    usleep(500000);
    std::cout << "qW: " << quat.w() << " qX: " << quat.x() << " qY: " << quat.y() << " qZ: " << quat.z() << std::endl;
    acc_NEDframe = Acc_bframe_to_acc_NEDframe(quat.w(), quat.x(), quat.y(), quat.z(), acc_bframe.x(), acc_bframe.y(), acc_bframe.z());
    std::cout << "X: " << acc_NEDframe[0] << " Y: " << acc_NEDframe[1] << " Z: " << acc_NEDframe[2] << std::endl;

    while (1)
    {

        quat = bno.getQuat();
        acc_bframe = bno.getVector(BNO055::VECTOR_LINEARACCEL);
        acc_NEDframe = Acc_bframe_to_acc_NEDframe(quat.w(), quat.x(), quat.y(), quat.z(), acc_bframe.x(), acc_bframe.y(), acc_bframe.z());

        // std::cout << "X: " << std::setw(10) << std::fixed << std::setprecision(3) << acc_NEDframe[0] 
        //     << " Y: "  << std::setw(10) << std::fixed << std::setprecision(3) << acc_NEDframe[1] 
        //     << " Z: "  << std::setw(10) << std::fixed << std::setprecision(3) << acc_NEDframe[2]  
        //     << std::endl;

        gyr_bframe = bno.getVector(BNO055::VECTOR_EULER);

        std::cout << "X: " << std::setw(10) << std::fixed << std::setprecision(3) << gyr_bframe[0] 
            << " Y: "  << std::setw(10) << std::fixed << std::setprecision(3) << gyr_bframe[1] 
            << " Z: "  << std::setw(10) << std::fixed << std::setprecision(3) << gyr_bframe[2]  
            << std::endl;


        uint8_t system_status, self_test_results, system_error;
        bno.getSystemStatus(&system_status, &self_test_results, &system_error);
        // std::cout << "System Status: " << static_cast<int>(system_status) << std::endl;
        // std::cout << "Self Test: " << static_cast<int>(self_test_results) << std::endl;
        // std::cout << "System Error: " << static_cast<int>(system_error) << std::endl;
        if (system_status == 1) 
        {
            std::cout << "restarting imu..." << std::endl;
            bno.begin(bno.OPERATION_MODE_NDOF_FMC_OFF);
            usleep(1000000);
        }

        usleep(20000);
    }

    return 0;
}
