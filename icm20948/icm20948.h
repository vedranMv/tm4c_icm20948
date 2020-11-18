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

enum OrientationDOF
{
    Orientation6DOF,
    Orientation9DOF
};

enum AccelerometerFSR
{
    AccelFSR2g = 2,
    AccelFSR4g = 4,
    AccelFSR8g = 8,
    AccelFSR16g = 16
};

enum GyroscopeFSR
{
    GyroFSR250dps = 250,
    GyroFSR500dps = 500,
    GyroFSR1000dps = 1000,
    GyroFSR2000dps = 2000
};

//  Custom error codes for the library
#define MPU_ERROR               2
#define MPU_NOT_ALLOWED         3

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

        //  Has to be called before InitSW
        int8_t SetAccelerationFSR(AccelerometerFSR aFsr);
        int8_t SetGyroscopeFSR(GyroscopeFSR gFsr);
        int8_t SetMagnetometerBias(float biasX, float biasY, float biasZ);
        int8_t SetMountingMatrix(float *mountMatrix);

        int8_t  InitHW();
        int8_t  InitSW();
        int8_t  Enabled(bool en);

        bool    IsDataReady();
        int8_t  ReadSensorData(const float &timestamp);

        int8_t EnableSensor(inv_icm20948_sensor sensor, uint32_t period);
        int8_t DisableSensor(inv_icm20948_sensor sensor);


        int8_t  GetLinearAcceleration(float *acc);
        int8_t  GetGyroscope(float *gyro);
        int8_t  GetOrientationRPY(OrientationDOF type, float* orientationRPY, bool inDeg);
        int8_t  GetOrientationQuat(OrientationDOF type, float* orientationQuat);
        int8_t  GetMagnetometer(float *mag);
        int8_t  GetGravity(float *gv);

        int8_t  GetVelocity(float *v);
        int8_t  GetDistance(float *s);

    protected:
        ICM20948();
        ~ICM20948();
        ICM20948(ICM20948 &arg) {}              //  No definition - forbid this
        void operator=(ICM20948 const &arg) {} //  No definition - forbid this

        /**
         * Data setters using averaging windows
         */
        void _SetAcceleration(float *acc);
        void _SetGyroscope(float *gyro);



        bool _initialized;

        //  Linear acceleration [x,y,z] in m/s^2
        volatile float _acc[3];
        //  Gyroscope readings in degrees-per-second [x,y,z]
        volatile float _gyro[3];
        //  Magnetometer readings [x,y,z] in
        volatile float _mag[3];
        //  Gravity vector
        volatile float _gv[3];
        //  Quaternions(w,x,y,z) and their accuracy
        volatile float _quat9DOF[4];
        volatile float _quat9DOFaccuracy;
        volatile float _quat6DOF[4];
        volatile float _quat6DOFaccuracy;

};

#endif /* ICM20948_H_ */
