#include "HAL/hal.h"
#include "libs/myLib.h"
#include "icm20948/icm20948.h"
#include "serialPort/uartHW.h"
#include <cstdio>

#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

static uint64_t timestamp;

void SysTickIntHandler(void)
{
    //
    // Update the Systick interrupt counter.
    //
    timestamp += 100;
}

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

class IntegratedData
{
    public:
        static IntegratedData& GetI()
        {
            static IntegratedData singletonInstance;

            return singletonInstance;
        };

        /**
         * Update data
         * @param dT data timestamp in seconds
         * @param acc Array of accelerations(x,y,z) in m/s2
         */
        void Update(const float timestamp,  float *acc)
        {

            if (!_initialized)
            {
                _old_time = timestamp;
                for (uint8_t i = 0; i < 3; i++)
                    _old_a[i] = acc[i];


                _initialized = true;
                return;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                //  Velocity change in this interval
                _v[i] += (_old_a[i] + ((acc[i]-_old_a[i])/2))*(timestamp-_old_time);

                _s[i] += (_old_v[i] + ((_v[i]-_old_v[i]))/2)*(timestamp-_old_time);

                _old_a[i] = acc[i];
                _old_v[i] = _v[i];
            }

            _old_time = timestamp;
        }

        void GetVelocity(float *v)
        {
            for (uint8_t i = 0; i < 3; i++)
                v[i] = _v[i];
        }

        void ResetVelocity()
        {
            _v[0] = _v[1] = _v[2] = 0.0f;
            _old_v[0] = _old_v[1] = _old_v[2] = 0.0f;
            //  Reset old acceleration, otherwise last update step can
            //  integrate the difference into very high displacement
            _old_a[0] = _old_a[1] = _old_a[2] = 0.0f;
        }

        void GetDistance(float *s)
        {
            for (uint8_t i = 0; i < 3; i++)
                s[i] = _s[i];
        }

    protected:
        IntegratedData(): _initialized(false)
        {
            for (uint8_t i = 0; i < 3; i++)
                _v[i] = 0, _s[i] = 0;
        };
        ~IntegratedData() {};
        IntegratedData(IntegratedData const &arg) {};
        void operator=(IntegratedData const &arg) {};

        float _v[3];
        float _s[3];
        bool _initialized;

        float _old_time = 0;
        float _old_a[3] = {0};
        float _old_v[3] = {0};
};

/**
 * main.cpp
 */
int main(void)
{
    ICM20948& mpu = ICM20948::GetI();

    //  Initialize board and FPU
    HAL_BOARD_CLOCK_Init();

    SysTickPeriodSet(12000);
    SysTickIntRegister(SysTickIntHandler);
    //IntMasterEnable();
    SysTickIntEnable();
    SysTickEnable();

    //  Initialize serial port
    SerialPort::GetI().InitHW();
    DEBUG_WRITE("Initialized Uart... \n");

    //  Initialize hardware used by MPU9250
    mpu.InitHW();

    //  Software initialization of MPU9250
    //  Either configure registers for direct sensor readings or load DMP
    //  firmware
    mpu.InitSW();

    float data[3];
    uint32_t counter = 0;
    while (1)
    {
        //  Check if MPU toggled interrupt pin
        //  (this example doesn't use actual interrupts, but polling)
        if (HAL_MPU_DataAvail())
        {

            //  Read sensor data
            mpu.ReadSensorData();
            /**
             *   Integration of sensor data
             */
            if (mpu.GetActivity())
            {
                mpu.Acceleration(data);
                IntegratedData::GetI().Update(timestamp/1000000.0f, data);
            }
            else
            {
                IntegratedData::GetI().ResetVelocity();
                data[0] = data[1] = data[2] = 0.0f;
                IntegratedData::GetI().Update(timestamp/1000000.0f, data);
            }

            if (counter++ >= 0)
            {
                char buffer[80];
                //mpu.Magnetometer(data);
                //mpu.Gravity(data);
//                snprintf(buffer, 80, "RPY: %f,%f,%f\n",data[0],data[1],data[2]);
//                DEBUG_WRITE("%s", buffer);

                /**
                 *   RAW sensor data
                 */
                mpu.Acceleration(data);
                snprintf(buffer, 80, "%f,",data[0]);
                DEBUG_WRITE("%s", buffer);
//                mpu.RawAcceleration(data);
//                snprintf(buffer, 80, "%f,",data[0]);
//                DEBUG_WRITE("%s\n", buffer);

//                mpu.Gyroscope(data);
//                snprintf(buffer, 80, "%f,%f,%f,",data[0],data[1],data[2]);
//                DEBUG_WRITE("%s", buffer);
//                mpu.Magnetometer(data);
//                snprintf(buffer, 80, "%f,%f,%f\n",data[0],data[1],data[2]);
//                DEBUG_WRITE("%s", buffer);

                float v[3], s[3];
                IntegratedData::GetI().GetVelocity(v);
                IntegratedData::GetI().GetDistance(s);

                snprintf(buffer, 80, "%f,%f,%d\n",v[0], s[0], mpu.GetActivity());
                DEBUG_WRITE("%s", buffer);
                counter = 0;
            }
        }

        // INT pin can be held up for max 50us, so delay here to prevent reading the same data twice
        HAL_DelayUS(10);
        timestamp += 10;
    }
}
