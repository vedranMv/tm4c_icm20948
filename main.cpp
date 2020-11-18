#include "HAL/hal.h"
#include "libs/myLib.h"
#include "icm20948/icm20948.h"
#include "serialPort/uartHW.h"
#include <cstdio>

#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

static float timestamp = 0;

void SysTickIntHandler(void)
{
    // Update the Systick interrupt counter.
    timestamp += 100.0;
}

//  DEfinition of these functions for deployment platform must exist
extern "C" {
    /*
    * Sleep implementation for ICM20948
    */
    void inv_icm20948_sleep(int ms) {
        HAL_DelayUS(ms*1000);
    }

    void inv_icm20948_sleep_us(int us){
        HAL_DelayUS(us);
    }

    uint64_t inv_icm20948_get_time_us(void){
        return timestamp;
    }

}

/**
 * main.cpp
 */
int main(void)
{
    int8_t rc = 0;
    //  Mounting matrix describing transformation between IMU reference frame
    //  and a mounting pose
    float mountMatrix[9]= {
        1.f, 0, 0,
        0, 1.f, 0,
        0, 0, 1.f
    };

    ICM20948& imu = ICM20948::GetI();

    //  Initialize board and FPU
    HAL_BOARD_CLOCK_Init();

    SysTickPeriodSet(12000);
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
    SysTickEnable();

    //  Initialize serial port
    SerialPort::GetI().InitHW();
    DEBUG_WRITE("Initialized Uart... \n");

    //  Initialize hardware used by the IMU
    imu.InitHW();

    //  Set instrument settings
    imu.SetAccelerationFSR(AccelFSR2g);
    imu.SetGyroscopeFSR(GyroFSR250dps);
    imu.SetMagnetometerBias(-73.363101, -69.95, -3.0);
    imu.SetMountingMatrix(mountMatrix);

    //  Software initialization of the IMU
    //  (load DMP firmware and enable all the sensors)
    imu.InitSW();

    //
    //  DMP has been loaded, enable the sensors
    //

    rc = imu.EnableSensor(INV_ICM20948_SENSOR_GYROSCOPE, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
#endif
    //  6DOF sensor fusion
    rc = imu.EnableSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Linear acceleration: ");
#endif
    rc = imu.EnableSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    9DOF fusion: ");
#endif
    //  9DOF sensor fusion
    rc = imu.EnableSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Gravity vector: ");
#endif
    rc = imu.EnableSensor(INV_ICM20948_SENSOR_GRAVITY, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Gravity vector: ");
#endif

    float data[3];
    while (1)
    {
        //  Check if IMU toggled interrupt pin
        //  (this example doesn't use interrupts, but polling)
        if (imu.IsDataReady())
        {
            char buffer[160];

            //  Read sensor data
            imu.ReadSensorData(timestamp);

            //  Read and print orientation as reported by the 6DOF and 9DOF fusion
            imu.GetOrientationRPY(Orientation6DOF, data, true);
            snprintf(buffer, 160, "%f,%f,%f",data[0],data[1],data[2]);
            imu.GetOrientationRPY(Orientation9DOF, data, true);
            snprintf(buffer, 160, "%s,%f,%f,%f",buffer, data[0],data[1],data[2]);
            DEBUG_WRITE("%s\n", buffer);

            //  Read acceleration and angular velocity
            imu.GetLinearAcceleration(data);
            snprintf(buffer, 160, "%f,%f,%f",data[0],data[1],data[2]);
            imu.GetGyroscope(data);
            snprintf(buffer, 160, "%s,%f,%f,%f",buffer, data[0],data[1],data[2]);
            DEBUG_WRITE("%s\n\n", buffer);

        }

        // INT pin can be held up for max 50us, so delay here to prevent reading the same data twice
        HAL_DelayUS(50);
    }
}
