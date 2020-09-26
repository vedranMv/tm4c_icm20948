/**
 * ICM20948.h
 *
 *  Created on: 15. 9. 2020.
 *      Author: Vedran Mikov
 *
 *  @version V1.0
 *  V1.0 -15.9.2020
 *  + Basic support for ICM-20948 DMP software
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ICM20948_H_) && defined(__HAL_USE_ICM20948__)
#define ICM20948_H_

#define USE_SPI_NOT_I2C             1       /* Default configuration - I2C */

#define AK0991x_DEFAULT_I2C_ADDR    0x0C    /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E    /* The secondary I2C address for AK0991x Magnetometers */

#define EXTERNAL_SENSOR             0       /* Default configuration is on-board sensor and this flag need not have to be changed */

#if (EXTERNAL_SENSOR == 1)
#define CHIP_SELECT                 0
#else
#define CHIP_SELECT                 1
#endif

#define ICM_I2C_ADDR_REVA           0x68    /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB           0x69    /* I2C slave address for INV device on Rev B board */

/*
* Select communication between Atmel and INV device by setting 0/1 to one of the following defines
*/
#define SERIF_TYPE_SPI (USE_SPI_NOT_I2C)
#define SERIF_TYPE_I2C !(USE_SPI_NOT_I2C)

//  Custom error codes for the library
//#define MPU_SUCCESS             0
#define MPU_ERROR               2

#include "Invn/Devices/Drivers/Icm20948/Icm20948Setup.h"

/**
 * Class object for MPU9250 sensor
 */
class ICM20948
{
    friend void build_sensor_event_data(void * context, inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
    public:
        static ICM20948& GetI();
        static ICM20948* GetP();

        int8_t  InitHW();
        int8_t  InitSW();
        int8_t  Reset();
        int8_t  Enabled(bool en);
        bool    IsDataReady();

        int8_t  ReadSensorData();
        int8_t  RPY(float* RPY, bool inDeg);
        int8_t  Acceleration(float *acc);
        int8_t  RawAcceleration(float *acc);
        int8_t  RawAcceleration2(float *acc);
        void    SetAcceleration(float *acc);
        void    UpdateActivity();
        int     GetActivity();
        void    SetGyroscope(float *gyro);
        int8_t  Gyroscope(float *gyro);
        int8_t  Magnetometer(float *mag);
        int8_t  Gravity(float *gv);

        volatile float  dT;


    protected:
        ICM20948();
        ~ICM20948();
        ICM20948(ICM20948 &arg) {}              //  No definition - forbid this
        void operator=(ICM20948 const &arg) {} //  No definition - forbid this

        //  Yaw-Pitch-Roll orientation[Y,P,R] in radians
        volatile float _ypr[3];
        //  Acceleration [x,y,z]
        volatile float _acc[3], _accRaw[3], _accRaw2[3];
        //  Gyroscope readings [x,y,z]
        volatile float _gyro[3];
        //  Magnetometer readings[x,y,z]
        volatile float _mag[3];
        //  Magnetometer control
        bool _magEn;
        volatile float _quat[4];
        volatile float _gv[3];
        int _act;

};

#endif /* ICM20948_H_ */
