/**
 *  mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250_DMP__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"

#include "Invn/Devices/Drivers/Icm20948/Icm20948.h"
#include "Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"


//  Enable debug information printed on serial port
#define __DEBUG_SESSION__


#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif


///-----------------------------------------------------------------------------
///         DMP related functions --  Start
///-----------------------------------------------------------------------------


static const uint8_t dmp3_image[] = {
#include "Invn/icm20948_img.dmp3a.h"
};

/*
* Just a handy variable to handle the icm20948 object
*/
inv_icm20948_t icm_device;

static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
static int unscaled_bias[THREE_AXES * 2];

/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 500; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

/*
* Mounting matrix configuration applied for Accel, Gyro and Mag
*/

static const float cfg_mounting_matrix[9]= {
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

/*
* Mask to keep track of enabled sensors
*/
static uint32_t enabled_sensor_mask = 0;

inv_bool_t interface_is_SPI(void)
{
#ifdef __HAL_USE_MPU9250_DMP__
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
            memcpy((void*)MPU9250::GetI()._gyro, event.data.gyr.vect, sizeof(event.data.gyr.vect));
            break;
        case INV_SENSOR_TYPE_GRAVITY:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
            memcpy((void*)MPU9250::GetI()._gv, event.data.acc.vect, sizeof(event.data.acc.vect));
            break;
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        case INV_SENSOR_TYPE_ACCELEROMETER:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
            memcpy((void*)MPU9250::GetI()._acc, event.data.acc.vect, sizeof(event.data.acc.vect));
            break;
        case INV_SENSOR_TYPE_MAGNETOMETER:
            memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
            memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
            memcpy((void*)MPU9250::GetI()._mag, event.data.mag.vect, sizeof(event.data.mag.vect));
            break;
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            //memcpy((void*)MPU9250::GetI()._quat, event.data.quaternion.quat, sizeof(event.data.quaternion.quat));
            break;
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
            memcpy((void*)MPU9250::GetI()._quat, event.data.quaternion.quat, sizeof(event.data.quaternion.quat));
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
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
            memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
            break;
        default:
            return;
    }

}

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
    switch(sensor)
    {
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:       return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:           return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
        case INV_SENSOR_TYPE_ACCELEROMETER:           return INV_ICM20948_SENSOR_ACCELEROMETER;
        case INV_SENSOR_TYPE_GYROSCOPE:               return INV_ICM20948_SENSOR_GYROSCOPE;
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:      return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:         return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
        case INV_SENSOR_TYPE_BAC:                     return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
        case INV_SENSOR_TYPE_STEP_DETECTOR:           return INV_ICM20948_SENSOR_STEP_DETECTOR;
        case INV_SENSOR_TYPE_STEP_COUNTER:            return INV_ICM20948_SENSOR_STEP_COUNTER;
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:    return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
        case INV_SENSOR_TYPE_ROTATION_VECTOR:         return INV_ICM20948_SENSOR_ROTATION_VECTOR;
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:  return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
        case INV_SENSOR_TYPE_MAGNETOMETER:            return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
        case INV_SENSOR_TYPE_SMD:                     return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
        case INV_SENSOR_TYPE_PICK_UP_GESTURE:         return INV_ICM20948_SENSOR_FLIP_PICKUP;
        case INV_SENSOR_TYPE_TILT_DETECTOR:           return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
        case INV_SENSOR_TYPE_GRAVITY:                 return INV_ICM20948_SENSOR_GRAVITY;
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:     return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
        case INV_SENSOR_TYPE_ORIENTATION:             return INV_ICM20948_SENSOR_ORIENTATION;
        case INV_SENSOR_TYPE_B2S:                     return INV_ICM20948_SENSOR_B2S;
        default:                                      return INV_ICM20948_SENSOR_MAX;
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
MPU9250& MPU9250::GetI()
{
    static MPU9250 singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
MPU9250* MPU9250::GetP()
{
    return &(MPU9250::GetI());
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring MPU9250               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware used by MPU9250
 * Initializes I2C bus for communication with MPU (SDA - PN4, SCL - PN5), bus
 * frequency 1MHz, connection timeout: 100ms. Initializes pin(PA5)
 * to be toggled by MPU9250 when it has data available for reading (PA5 is
 * push-pull pin with weak pull down and 10mA strength).
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitHW()
{
    HAL_MPU_Init();
    HAL_MPU_PowerSwitch(true);

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor, load DMP firmware and configure DMP output. Prior to
 * any software initialization, this function power-cycles the board
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitSW()
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

    for (int ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
    {
        inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
    }

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
        DEBUG_WRITE(" >Updating DMP features...");
    }
#endif


    //enable sensors
    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, 20);
    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, 20);

    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 20);
    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 20);


    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_ROTATION_VECTOR, 20);
    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_ORIENTATION, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_ORIENTATION, 20);

    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 20);
    rc = inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_GRAVITY, 1);
    inv_icm20948_set_sensor_period(&icm_device, INV_ICM20948_SENSOR_GRAVITY, 20);



    return MPU_SUCCESS;
}

/**
 * Trigger software reset of the MPU module by writing into corresponding
 * register. Wait for 50ms afterwards for sensor to start up.
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Reset()
{
    //HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 1 << 7);
    HAL_DelayUS(50000);

    return MPU_SUCCESS;
}

/**
 * Control power supply of the MPU9250
 * Enable or disable power supply of the MPU9250 using external MOSFET
 * @param en Power state
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Enabled(bool en)
{
    HAL_MPU_PowerSwitch(en);

    return MPU_SUCCESS;
}

/**
 * Check if new sensor data has been received
 * @return true if new sensor data is available
 *        false otherwise
 */
bool MPU9250::IsDataReady()
{
    return HAL_MPU_DataAvail();
}

/**
 * Get ID from MPU, should always return 0x71
 * @return ID value stored in MPU's register
 */
uint8_t MPU9250::GetID()
{
    uint8_t ID=00;
    //ID = HAL_MPU_ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    return ID;
}

/**
 * Trigger reading data from MPU9250
 * Read data from MPU9250s' FIFO and extract quaternions, acceleration, gravity
 * vector & roll-pitch-yaw
 * @return One of MPU_* error codes
 */
int8_t MPU9250::ReadSensorData()
{
    int8_t retVal = MPU_ERROR;

    inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);

     return retVal;
}

/**
 * Copy orientation from internal buffer to user-provided one
 * @param RPY pointer to float buffer of size 3 to hold roll-pitch-yaw
 * @param inDeg if true RPY returned in degrees, if false in radians
 * @return One of MPU_* error codes
 */
int8_t MPU9250::RPY(float* RPY, bool inDeg)
{
    Quaternion qt;
    qt.x = _quat[1];
    qt.y = _quat[2];
    qt.z = _quat[3];
    qt.w = _quat[0];

    VectorFloat v;
    dmp_GetGravity(&v, &qt);

    dmp_GetYawPitchRoll((float*)(MPU9250::GetI()._ypr), &qt, &v);

    for (uint8_t i = 0; i < 3; i++)
        if (inDeg)
            RPY[2-i] = _ypr[i]*180.0/PI_CONST;
        else
            RPY[2-i] = _ypr[i];

    return MPU_SUCCESS;
}

/**
 * Copy acceleration from internal buffer to user-provided one
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Acceleration(float *acc)
{
    memcpy((void*)acc, (void*)_acc, sizeof(float)*3);
    acc[0] *= GRAVITY_CONST;
    acc[1] *= GRAVITY_CONST;
    acc[2] *= GRAVITY_CONST;

    return MPU_SUCCESS;
}

/**
 * Copy angular rotation from internal buffer to user-provided one
 * @param gyro Pointer a float array of min. size 3 to store 3-axis rotation
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Gyroscope(float *gyro)
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
int8_t MPU9250::Magnetometer(float *mag)
{
    memcpy((void*)mag, (void*)_mag, sizeof(float)*3);

    return MPU_SUCCESS;
}

int8_t MPU9250::Gravity(float *gv)
{
    memcpy((void*)gv, (void*)_gv, sizeof(float)*3);

    return MPU_SUCCESS;
}


///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() :  dT(0), _magEn(true)
{
    //  Initialize arrays
    memset((void*)_ypr, 0, 3);
    memset((void*)_acc, 0, 3);
    memset((void*)_gyro, 0, 3);
    memset((void*)_mag, 0, 3);
}

MPU9250::~MPU9250()
{}

#endif  /* __HAL_USE_MPU9250_DMP__ */
