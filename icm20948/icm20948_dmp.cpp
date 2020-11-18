/**
 *  icm20948.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "icm20948.h"

#if defined(__HAL_USE_ICM20948_DMP__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"
#include "libs/linkedlist.hpp"
#include <cstdio>

#include "Invn/Devices/Drivers/Icm20948/Icm20948.h"
#include "Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"


//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__


#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

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


///-----------------------------------------------------------------------------
///         DMP related functions --  Start
///-----------------------------------------------------------------------------

//  DMP image
static const uint8_t dmp3_image[] = {
#include "Invn/icm20948_img.dmp3a.h"
};

/*
* Just a handy variable to handle the icm20948 object
*/
inv_icm20948_t icm_device;

static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */

/* FSR configurations */
int32_t cfg_acc_fsr = 2; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 250; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

/*
 * Mounting matrix configuration applied for Accel, Gyro and Mag
 */

static float cfg_mounting_matrix[9]= {
    1.f, 0, 0,
    0, 1.f, 0,
    0, 0, 1.f
};


static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
    INV_SENSOR_TYPE_ACCELEROMETER,
    INV_SENSOR_TYPE_GYROSCOPE,
    INV_SENSOR_TYPE_RAW_ACCELEROMETER,
    INV_SENSOR_TYPE_RAW_GYROSCOPE,
    INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
    INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
    INV_SENSOR_TYPE_BAC,
    INV_SENSOR_TYPE_STEP_DETECTOR,
    INV_SENSOR_TYPE_STEP_COUNTER,
    INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
    INV_SENSOR_TYPE_ROTATION_VECTOR,
    INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
    INV_SENSOR_TYPE_MAGNETOMETER,
    INV_SENSOR_TYPE_SMD,
    INV_SENSOR_TYPE_PICK_UP_GESTURE,
    INV_SENSOR_TYPE_TILT_DETECTOR,
    INV_SENSOR_TYPE_GRAVITY,
    INV_SENSOR_TYPE_LINEAR_ACCELERATION,
    INV_SENSOR_TYPE_ORIENTATION,
    INV_SENSOR_TYPE_B2S
};

static uint8_t icm20948_get_grv_accuracy(void)
{
    uint8_t accel_accuracy;
    uint8_t gyro_accuracy;

    accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
    gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
    return (min(accel_accuracy, gyro_accuracy));
}


inv_bool_t interface_is_SPI(void)
{
#ifdef __HAL_USE_ICM20948_SPI__
    return true;
#else
    return false;
#endif
}


void switch_I2C_to_revA(void)
{
#if SERIF_TYPE_I2C
    I2C_Address = ICM_I2C_ADDR_REVA;
#endif
    return;
}

/* Extra functions from i2cdevlib from github:
 * https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9150
 */
uint8_t dmp_GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
uint8_t dmp_GetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t dmp_GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

/**
 * Process received data from the IMU an save it into appropriate buffers
 * @param context
 * @param sensortype
 * @param timestamp
 * @param data
 * @param arg
 */
void build_sensor_event_data(void * context, inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
    float raw_bias_data[6];
    inv_sensor_event_t event;
    (void)context;
    uint8_t sensor_id = convert_to_generic_ids[sensortype];

    memset((void *)&event, 0, sizeof(event));
    event.sensor = sensor_id;
    event.timestamp = timestamp;
    switch(sensor_id)
    {
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
            memcpy(raw_bias_data, data, sizeof(raw_bias_data));
            memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
            memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
            memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            break;
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
            memcpy(raw_bias_data, data, sizeof(raw_bias_data));
            memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
            memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
            memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            break;
        case INV_SENSOR_TYPE_GYROSCOPE:
            memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
            memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            memcpy(event.data.gyr.vect, (void*)&ICM20948::GetI()._gyro, sizeof(event.data.gyr.vect));
            //  Alternatively, update acceleration data through a median filter
            //  using the call
            //ICM20948::GetI()._SetGyroscope(event.data.gyr.vect);
            break;
        case INV_SENSOR_TYPE_GRAVITY:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
            memcpy((void*)ICM20948::GetI()._gv, event.data.acc.vect, sizeof(event.data.acc.vect));
            break;
        case INV_SENSOR_TYPE_ACCELEROMETER:

        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
            memcpy(event.data.acc.vect, (void*)&ICM20948::GetI()._acc, sizeof(event.data.acc.vect));
            //  Alternatively, update acceleration data through a median filter
            //  using the call
            //ICM20948::GetI()._SetAcceleration(event.data.acc.vect);
            break;
        case INV_SENSOR_TYPE_MAGNETOMETER:
            memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
            memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
            memcpy((void*)ICM20948::GetI()._mag, event.data.mag.vect, sizeof(event.data.mag.vect));
            break;
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
            break;
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            memcpy((void*)ICM20948::GetI()._quat9DOF, event.data.quaternion.quat, sizeof(event.data.quaternion.quat));
            memcpy((void*)&ICM20948::GetI()._quat9DOFaccuracy, (void*)&(event.data.quaternion.accuracy), sizeof(event.data.quaternion.accuracy));
            break;
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
            memcpy((void*)ICM20948::GetI()._quat6DOF, event.data.quaternion.quat, sizeof(event.data.quaternion.quat));
            float tmpAccuracy = (float)event.data.quaternion.accuracy;
            memcpy((void*)&ICM20948::GetI()._quat6DOFaccuracy, (void*)&tmpAccuracy, sizeof(tmpAccuracy));
            break;
        case INV_SENSOR_TYPE_BAC:
            memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
            break;
        case INV_SENSOR_TYPE_PICK_UP_GESTURE:
        case INV_SENSOR_TYPE_TILT_DETECTOR:
        case INV_SENSOR_TYPE_STEP_DETECTOR:
        case INV_SENSOR_TYPE_SMD:
            event.data.event = true;
            break;
        case INV_SENSOR_TYPE_B2S:
            event.data.event = true;
            memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
            break;
        case INV_SENSOR_TYPE_STEP_COUNTER:
            memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
            break;
        case INV_SENSOR_TYPE_ORIENTATION:
            //we just want to copy x,y,z from orientation data
            memcpy(&(event.data.orientation), data, 3*sizeof(float));
            break;
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
            memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
            memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
            memcpy((void*)ICM20948::GetI()._accRaw, event.data.acc.vect, sizeof(event.data.acc.vect));
            break;
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
            memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
            break;
        default:
            return;
    }

}


///-----------------------------------------------------------------------------
///         DMP related functions --  End
///-----------------------------------------------------------------------------


///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
ICM20948& ICM20948::GetI()
{
    static ICM20948 singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
ICM20948* ICM20948::GetP()
{
    return &(ICM20948::GetI());
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring ICM20948               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware used by ICM20948
 * Initializes I2C bus for communication with MPU (SDA - PN4, SCL - PN5), bus
 * frequency 1MHz, connection timeout: 100ms. Initializes pin(PA5)
 * to be toggled by ICM20948 when it has data available for reading (PA5 is
 * push-pull pin with weak pull down and 10mA strength).
 * @return One of MPU_* error codes
 */
int8_t ICM20948::InitHW()
{
    HAL_MPU_Init();
    HAL_MPU_PowerSwitch(true);

    return MPU_SUCCESS;
}

int8_t ICM20948::SetAccelerationFSR(AccelerometerFSR aFsr)
{
    //  Can't be set if the IMU has already been initialized and DMP loaded
    if (_initialized)
        return MPU_NOT_ALLOWED;

    cfg_acc_fsr = aFsr;

    return MPU_SUCCESS;
}
int8_t ICM20948::SetGyroscopeFSR(GyroscopeFSR gFsr)
{
    //  Can't be set if the IMU has already been initialized and DMP loaded
    if (_initialized)
        return MPU_NOT_ALLOWED;

    cfg_gyr_fsr = gFsr;

    return MPU_SUCCESS;
}
int8_t ICM20948::SetMagnetometerBias(float biasX, float biasY, float biasZ)
{
    //  Can't be set if the IMU has already been initialized and DMP loaded
    if (_initialized)
        return MPU_NOT_ALLOWED;

    //  Apply compass bias
    int biasq16[3] = {0};
    biasq16[0] = (int)(biasX*(float)(1L<<16));
    biasq16[1] = (int)(biasY*(float)(1L<<16));
    biasq16[2] = (int)(biasZ*(float)(1L<<16));

    return inv_icm20948_set_bias(&icm_device, INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, biasq16);
}
int8_t ICM20948::SetMountingMatrix(float *mountMatrix)
{
    //  Can't be set if the IMU has already been initialized and DMP loaded
    if (_initialized)
        return MPU_NOT_ALLOWED;

    memcpy((void*)mountMatrix, (void*)cfg_mounting_matrix, sizeof(cfg_mounting_matrix));

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor, load DMP firmware and configure DMP output. Prior to
 * any software initialization, this function power-cycles the board
 * @return One of MPU_* error codes
 */
int8_t ICM20948::InitSW()
{

    //  Power cycle MPU chip on every SW initialization
    HAL_MPU_PowerSwitch(false);
    HAL_DelayUS(20000);
    HAL_MPU_PowerSwitch(true);
    HAL_DelayUS(30000);

    /*
    * Initialize icm20948 serif structure
    */
    struct inv_icm20948_serif icm20948_serif;
    icm20948_serif.context   = 0; /* no need */
    icm20948_serif.read_reg  = HAL_MPU_ReadBytes;
    icm20948_serif.write_reg = HAL_MPU_WriteBytes;
    icm20948_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
    icm20948_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */

    icm20948_serif.is_spi = interface_is_SPI();

    /*
     * Reset icm20948 driver states
     */
    inv_icm20948_reset_states(&icm_device, &icm20948_serif);

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    /*
     * Setup the icm20948 device
     */
    uint8_t i, whoami = 0xff;
    int rc;

    /*
    * Just get the whoami
    */
    rc = inv_icm20948_get_whoami(&icm_device, &whoami);
    if (interface_is_SPI() == 0)
    {       // If we're using I2C
        if (whoami == 0xff)
        {               // if whoami fails try the other I2C Address
            switch_I2C_to_revA();
            rc = inv_icm20948_get_whoami(&icm_device, &whoami);
        }
    }
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("ICM20948 WHOAMI value=0x%02x\n", whoami);
#endif

    /*
    * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
    */
    for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i)
    {
        if(whoami == EXPECTED_WHOAMI[i])
        {
            break;
        }
    }


    if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]))
    {
#ifdef __DEBUG_SESSION__
        DEBUG_WRITE("Bad WHOAMI value. Got 0x%02x.\n", whoami);
#endif
        return MPU_ERROR;
    }

    /* Setup accel and gyro mounting matrix and associated angle for current board */
    inv_icm20948_init_matrix(&icm_device);

    /* set default power mode */
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Putting Icm20948 in sleep mode...\n");
#endif
    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    if (rc != 0)
    {
#ifdef __DEBUG_SESSION__
        DEBUG_WRITE("Initialization failed. Error loading DMP3...\n");
#endif
        return MPU_ERROR;
    }

    /*
    * Configure and initialize the ICM20948 for normal use
    */
#ifdef __DEBUG_SESSION__
        DEBUG_WRITE("Booting up icm20948...\n");
#endif

    /* Initialize auxiliary sensors */
    inv_icm20948_register_aux_compass( &icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
    rc = inv_icm20948_initialize_auxiliary(&icm_device);
#ifdef __DEBUG_SESSION__
    if (rc == -1)
    {
        DEBUG_WRITE("Compass not detected...\n");
    }
#endif

    /*
     * Set full-scale range for sensors
     */
    for (int ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
    {
        inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
    }

    //  Smooth accel output
    icm_device.base_state.accel_averaging = 5;
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);

    /* re-initialize base state structure */
    inv_icm20948_init_structure(&icm_device);

    /* we should be good to go ! */
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("We're good to go! Loading DMP firmware\n");
#endif
    /*
     * Now that Icm20948 device was initialized, we can proceed with DMP image loading
     * This step is mandatory as DMP image are not store in non volatile memory
     */

    rc = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));

    if(rc < 0)
    {
#ifdef __DEBUG_SESSION__
        DEBUG_WRITE(" >Firmware loading failed\n");
#endif
        return MPU_ERROR;
    }
#ifdef __DEBUG_SESSION__
    else
    {
        DEBUG_WRITE(" >Firmware loaded\n");
        DEBUG_WRITE(" >Enabling sensors:\n");
    }
#endif


    // Enable sensors
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("    Accelerometer: ");
#endif
    rc = EnableSensor(INV_ICM20948_SENSOR_ACCELEROMETER, 5);
    rc |= EnableSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Gyroscope: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_GYROSCOPE, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Magnetometer: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Geomag rotation: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    6DOF fusion: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Linear acceleration: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    9DOF fusion: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
    DEBUG_WRITE("    Gravity vector: ");
#endif
    rc |= EnableSensor(INV_ICM20948_SENSOR_GRAVITY, 5);
#ifdef __DEBUG_SESSION__
    if (rc == 0)
        DEBUG_WRITE("OK\n");
    else
        DEBUG_WRITE("ERR\n");
#endif

    _initialized = true;

    return MPU_SUCCESS;
}

/**
 * Control power supply of the ICM20948
 * Enable or disable power supply of the ICM20948 using external MOSFET
 * @param en Power state
 * @return One of MPU_* error codes
 */
int8_t ICM20948::Enabled(bool en)
{
    HAL_MPU_PowerSwitch(en);

    return MPU_SUCCESS;
}

/**
 * Check if new sensor data has been received
 * @return true if new sensor data is available
 *        false otherwise
 */
bool ICM20948::IsDataReady()
{
    return HAL_MPU_DataAvail();
}

/**
 * Enable sensors and set its sampling rate
 * @param sensor Sensor to enable
 * @param period Sampling period in milliseconds
 * @return
 */
int8_t ICM20948::EnableSensor(inv_icm20948_sensor sensor, uint32_t period)
{
    uint8_t retVal = 0;

    retVal = inv_icm20948_enable_sensor(&icm_device, sensor, 1);
    retVal |= inv_icm20948_set_sensor_period(&icm_device, sensor, period);

    return retVal;
}

/**
 * Disable sensor
 * @param sensor Sensor to disable
 * @return
 */
int8_t ICM20948::DisableSensor(inv_icm20948_sensor sensor)
{
    uint8_t retVal = 0;

    retVal = inv_icm20948_enable_sensor(&icm_device, sensor, 0);

    return retVal;
}

/**
 * Trigger reading data from ICM20948
 * Read data from MPU9250s' FIFO and extract quaternions, acceleration, gravity
 * vector & roll-pitch-yaw
 * @param timestamp Reference to the current time in seconds
 * @return One of MPU_* error codes
 */
int8_t ICM20948::ReadSensorData(const float &timestamp)
{
    int8_t retVal = MPU_ERROR;

    retVal = inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);

    //  Gravity is returned in G's
    _acc[0] *= GRAVITY_CONST;
    _acc[1] *= GRAVITY_CONST;
    _acc[2] *= GRAVITY_CONST;

     return retVal;
}


/**
 * Return orientation as RPY angles
 * @param type One of OrientationDOF enums, describing which data to return
 * @param orientationRPY pointer to float buffer of size 3 to hold roll-pitch-yaw
 * @param inDeg if true RPY returned in degrees, if false in radians
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetOrientationRPY(OrientationDOF type, float* orientationRPY, bool inDeg)
{
    Quaternion qt;
    float _ypr[3];

    if (type == Orientation6DOF)
    {
        qt.w = _quat6DOF[0];
        qt.x = _quat6DOF[1];
        qt.y = _quat6DOF[2];
        qt.z = _quat6DOF[3];
    }
    else if (type == Orientation9DOF)
    {
        qt.w = _quat9DOF[0];
        qt.x = _quat9DOF[1];
        qt.y = _quat9DOF[2];
        qt.z = _quat9DOF[3];

    }


    // roll (x-axis rotation)
    float sinr_cosp = 2 * (qt.w * qt.x + qt.y * qt.z);
    float cosr_cosp = 1 - 2 * (qt.x * qt.x + qt.y * qt.y);
    _ypr[2] = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (qt.w * qt.y - qt.z * qt.x);
    if (fabsf(sinp) >= 1)
        _ypr[1] = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _ypr[1] = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (qt.w * qt.z + qt.x * qt.y);
    float cosy_cosp = 1 - 2 * (qt.y * qt.y + qt.z * qt.z);
    _ypr[0] = atan2f(siny_cosp, cosy_cosp);

    for (uint8_t i = 0; i < 3; i++)
        if (inDeg)
            orientationRPY[2-i] = _ypr[i]*180.0/M_PI;
        else
            orientationRPY[2-i] = _ypr[i];

    return MPU_SUCCESS;
}

/**
 * Return orientation as quaternions in format (w,x,y,z)
 * @param type One of OrientationDOF enums, describing which data to return
 * @param orientationQuat pointer to float buffer of size 4 to hold quaternions
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetOrientationQuat(OrientationDOF type, float* orientationQuat)
{
    if (type == Orientation6DOF)
    {
        for (uint8_t i = 0; i < 4; i++)
            orientationQuat[i] = _quat6DOF[i];
    }
    else if (type == Orientation9DOF)
    {
        for (uint8_t i = 0; i < 4; i++)
            orientationQuat[i] = _quat6DOF[i];
    }

    return MPU_SUCCESS;
}

/**
 * Copy acceleration from internal buffer to user-provided one
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *        data
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetAcceleration(float *acc)
{
    memcpy((void*)acc, (void*)_acc, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy raw acceleration from internal buffer to a user-provided one
 * Raw acceleration contains gravity
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *        data
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetAccelerationRaw(uint32_t *acc)
{
    memcpy((void*)acc, (void*)_accRaw, sizeof(_accRaw));

    return MPU_SUCCESS;
}

/**
 * Update accelerometer data by passing it through a median filter (window=23)
 * Accelerometer data for each axis is saved in a doubly linked list, sorted in
 * ascending order.
 * @param acc New accelerometer data sample
 */
void ICM20948::_SetAcceleration(float *acc)
{
    //  Median filter with windows 3 on acceleration
    static const uint8_t windowSize = 23;
    static LinkedList buffer;
    static uint8_t counter = 0;

    //  Insert the element on the list, in a sorted manner
    if (buffer.Size() == windowSize)
        buffer.DeleteWhereIndex(counter);
    buffer.addS(acc[0], counter);

    //  Take a middle element from the sorted list
    if (buffer.Size() == windowSize)
        _acc[0] = buffer.at((windowSize-1)/2);

    //  Increment counter for the next step
    counter = (counter + 1) % windowSize;
}


/**
 * Update gyroscope data
 * Updates raw gyroscope data by passing it through a median filter (window=3)
 * @param gyro New gyro data sample [x,y,z]
 */
void  ICM20948::_SetGyroscope(float *gyro)
{
    //  Median filter with windows 3 on gyro
     static int8_t counter = 0;
     static float buffer[3][3];

     memcpy((void*)&buffer[counter], (void*)gyro, sizeof(float)*3);

     //  For each axis, go through all 3 members and find median then save
     //  median into a variable that will be returned as final gyro value
     for (int8_t axis = 0; axis < 3; axis++)
         for (int8_t i = counter; i < 6; i++)
             if ( ((buffer[i%3][axis] >= buffer[(i-1+3)%3][axis]) &&
                   (buffer[i%3][axis] <= buffer[(i+1)%3][axis])) ||
                  ((buffer[i%3][axis] <= buffer[(i-1+3)%3][axis]) &&
                   (buffer[i%3][axis] >= buffer[(i+1)%3][axis])) )
             {
                 _gyro[axis] = buffer[i%3][axis];
                 break;
             }

     counter = (counter + 1) % 3;
}

/**
 * Copy angular rotation from internal buffer to user-provided one
 * @param gyro Pointer a float array of min. size 3 to store 3-axis rotation
 *        data in degrees-per-second
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetGyroscope(float *gyro)
{
    memcpy((void*)gyro, (void*)_gyro, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy mag. field strength from internal buffer to user-provided one
 * @param mag Pointer a float array of min. size 3 to store 3-axis mag. field
 *        strength data
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetMagnetometer(float *mag)
{
    memcpy((void*)mag, (void*)_mag, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy gravity vector from internal buffer to user-provided one
 * @param gv Pointer a float array of min. size 3 to store 3D vector data
 * @return One of MPU_* error codes
 */
int8_t ICM20948::GetGravity(float *gv)
{
    memcpy((void*)gv, (void*)_gv, sizeof(float)*3);

    return MPU_SUCCESS;
}


///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

ICM20948::ICM20948(): _initialized(false), _quat9DOFaccuracy(0.0), _quat6DOFaccuracy(0.0)
{
    //  Initialize arrays
    memset((void*)_acc, 0, sizeof(_acc));
    memset((void*)_accRaw, 0, sizeof(_accRaw));
    memset((void*)_gyro, 0, sizeof(_gyro));
    memset((void*)_mag, 0, sizeof(_mag));
    memset((void*)_gv, 0, sizeof(_gv));
    memset((void*)_quat9DOF, 0, sizeof(_quat9DOF));
    memset((void*)_quat6DOF, 0, sizeof(_quat6DOF));
}

ICM20948::~ICM20948()
{}

#endif  /* __HAL_USE_MPU9250_DMP__ */
