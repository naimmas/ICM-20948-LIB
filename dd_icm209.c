/*************************************************************************
  Includes
*************************************************************************/

#include "dd_icm209.h"
// InvenSense drivers and utils
#include "ha_iic/ha_iic.h"
#include "ha_timer/ha_timer.h"
#include "ps_app_timer/ps_app_timer.h"
#include "utils/Icm20948.h"
#include "utils/Icm20948MPUFifoControl.h"
#include "utils/Icm20948Setup.h"
#include "utils/SensorTypes.h"

#define MAX_MAGNETOMETER_STARTS (10U)
#define DEFAULT_IIC_TIMEOUT (100U)

IIC_SETUP_PORT_CONNECTION(1, IIC_DEFINE_CONNECTION(IIC_PORT1, 0, 0x68))

/*************************************************************************
  Variables
*************************************************************************/

float  gyro_x, gyro_y, gyro_z;
bool_t gyro_data_ready = FALSE;

float  accel_x, accel_y, accel_z;
bool_t accel_data_ready = FALSE;

float  mag_x, mag_y, mag_z;
bool_t mag_data_ready = FALSE;

float  quat_w, quat_x, quat_y, quat_z;
bool_t quat_data_ready = FALSE;

/**
 * @brief This internal function writes data to a specific register of the
 * QMI8658 device over I2C.
 * @param[in,out] ppt_dev QMI8658 device instance.
 * @param[in] ppt_data Pointer to the data buffer to write.
 * @param[in] p_data_sz Size of the data to write in bytes.
 * @param[in] p_reg_addr The register address to write to in the sensor.
 * @return Result of the execution status.
 */
static int write_register(void* user, uint8_t p_reg_addr, const uint8_t* ppt_data,
                          uint32_t p_data_sz)
{
    response_status_t api_ret_val = RET_OK;
    (void)user;
    api_ret_val = ha_iic_master_mem_write(IIC_GET_DEV_PORT(0),
                                          IIC_GET_DEV_ADDRESS(0),
                                          ppt_data,
                                          p_data_sz,
                                          p_reg_addr,
                                          HW_IIC_MEM_SZ_8BIT,
                                          DEFAULT_IIC_TIMEOUT);

    return api_ret_val == RET_OK ? 0 : -1;
}

/**
 * @brief This internal function reads data from a specific register of the
 * QMI8658 device over I2C.
 * @param[in,out] ppt_dev QMI8658 device instance.
 * @param[out] ppt_data Pointer to the data buffer to read into.
 * @param[in] p_data_sz Size of the data to read in bytes.
 * @param[in] p_reg_addr The register address to read from in the sensor.
 * @return Result of the execution status.
 */
static int read_register(void* user, uint8_t p_reg_addr, uint8_t* ppt_data, uint32_t p_data_sz)
{
    response_status_t api_ret_val = RET_OK;
    (void)user;
    api_ret_val = ha_iic_master_mem_read(IIC_GET_DEV_PORT(0),
                                         IIC_GET_DEV_ADDRESS(0),
                                         ppt_data,
                                         p_data_sz,
                                         p_reg_addr,
                                         HW_IIC_MEM_SZ_8BIT,
                                         DEFAULT_IIC_TIMEOUT);

    return api_ret_val == RET_OK ? 0 : -1;
}

/*************************************************************************
  Invensense Variables
*************************************************************************/

inv_icm20948_t       icm_device;
int                  rc                = 0;
static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
#define AK0991x_DEFAULT_I2C_ADDR  0x0C
#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

static const float cfg_mounting_matrix[9] = { 1.f, 0, 0, 0, 1.f, 0, 0, 0, 1.f };

int32_t cfg_acc_fsr = 4;    // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static const uint8_t dmp3_image[] = {
#include "utils/icm20948_img.dmp3a.h"
};

/*************************************************************************
  Invensense Functions
*************************************************************************/

int load_dmp3(void)
{
    int rc = 0;
    rc     = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
    return rc;
}
void inv_icm20948_sleep_us(int us)
{
    ha_timer_hard_delay_us(us);
}
void inv_icm20948_sleep(int ms)
{
    ha_timer_hard_delay_ms(ms);
}

uint64_t inv_icm20948_get_time_us(void)
{
    return ha_timer_get_cpu_time_us();
}

static void icm20948_apply_mounting_matrix(void)
{
    int ii;

    for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
    {
        inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (enum inv_icm20948_sensor)ii);
    }
}

static void icm20948_set_fsr(void)
{
    inv_icm20948_set_fsr(&icm_device,
                         INV_ICM20948_SENSOR_RAW_ACCELEROMETER,
                         (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device,
                         INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED,
                         (const void*)&cfg_gyr_fsr);
}

int icm20948_sensor_setup(void)
{
    int     rc;
    uint8_t i, whoami = 0xff;

    inv_icm20948_soft_reset(&icm_device);

    // Get whoami number
    rc = inv_icm20948_get_whoami(&icm_device, &whoami);

    // Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
    for (i = 0; i < sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]); ++i)
    {
        if (whoami == EXPECTED_WHOAMI[i])
        {
            break;
        }
    }

    if (i == sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]))
    {
        return rc;
    }

    // Setup accel and gyro mounting matrix and associated angle for current board
    inv_icm20948_init_matrix(&icm_device);

    // set default power mode
    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    if (rc != 0)
    {
        return rc;
    }

    // Configure and initialize the ICM20948 for normal use

    // Initialize auxiliary sensors
    inv_icm20948_register_aux_compass(&icm_device,
                                      INV_ICM20948_COMPASS_ID_AK09916,
                                      AK0991x_DEFAULT_I2C_ADDR);
    rc = inv_icm20948_initialize_auxiliary(&icm_device);

    icm20948_apply_mounting_matrix();

    icm20948_set_fsr();

    // re-initialize base state structure
    inv_icm20948_init_structure(&icm_device);

    return 0;
}

static uint8_t icm20948_get_grv_accuracy(void)
{
    uint8_t accel_accuracy;
    uint8_t gyro_accuracy;

    accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
    gyro_accuracy  = (uint8_t)inv_icm20948_get_gyro_accuracy();
    return (min(accel_accuracy, gyro_accuracy));
}

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

void build_sensor_event_data(void* context, enum inv_icm20948_sensor sensortype, uint64_t timestamp,
                             const void* data, const void* arg)
{
    float              raw_bias_data[6];
    inv_sensor_event_t event;
    (void)context;
    uint8_t sensor_id = convert_to_generic_ids[sensortype];

    memset((void*)&event, 0, sizeof(event));
    event.sensor    = sensor_id;
    event.timestamp = timestamp;
    switch (sensor_id)
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

            // WE WANT THIS
            gyro_x          = event.data.gyr.vect[0];
            gyro_y          = event.data.gyr.vect[1];
            gyro_z          = event.data.gyr.vect[2];
            gyro_data_ready = true;
            break;

        case INV_SENSOR_TYPE_GRAVITY:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
            break;
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        case INV_SENSOR_TYPE_ACCELEROMETER:
            memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));

            // WE WANT THIS
            accel_x          = event.data.acc.vect[0];
            accel_y          = event.data.acc.vect[1];
            accel_z          = event.data.acc.vect[2];
            accel_data_ready = true;
            break;

        case INV_SENSOR_TYPE_MAGNETOMETER:
            memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
            memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));

            // WE WANT THIS
            mag_x          = event.data.mag.vect[0];
            mag_y          = event.data.mag.vect[1];
            mag_z          = event.data.mag.vect[2];
            mag_data_ready = true;
            break;

        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            // WE WANT THIS
            quat_w          = event.data.quaternion.quat[0];
            quat_x          = event.data.quaternion.quat[1];
            quat_y          = event.data.quaternion.quat[2];
            quat_z          = event.data.quaternion.quat[3];
            quat_data_ready = true;
            break;
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
            memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
            break;

        case INV_SENSOR_TYPE_ORIENTATION:
            // we just want to copy x,y,z from orientation data
            memcpy(&(event.data.orientation), data, 3 * sizeof(float));
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
    switch (sensor)
    {
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
            return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
            return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
        case INV_SENSOR_TYPE_ACCELEROMETER:
            return INV_ICM20948_SENSOR_ACCELEROMETER;
        case INV_SENSOR_TYPE_GYROSCOPE:
            return INV_ICM20948_SENSOR_GYROSCOPE;
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
            return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
            return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            return INV_ICM20948_SENSOR_ROTATION_VECTOR;
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
            return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
        case INV_SENSOR_TYPE_MAGNETOMETER:
            return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
        case INV_SENSOR_TYPE_GRAVITY:
            return INV_ICM20948_SENSOR_GRAVITY;
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
            return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
        case INV_SENSOR_TYPE_ORIENTATION:
            return INV_ICM20948_SENSOR_ORIENTATION;
        default:
            return INV_ICM20948_SENSOR_MAX;
    }
}

/*************************************************************************
  Class Functions
*************************************************************************/

void dd_icm209_init(TeensyICM20948Settings settings)
{
    ha_iic_init();
    ha_timer_init();
    // Initialize icm20948 serif structure
    struct inv_icm20948_serif icm20948_serif;
    icm20948_serif.context   = 0; // no need
    icm20948_serif.read_reg  = read_register;
    icm20948_serif.write_reg = write_register;
    icm20948_serif.max_read  = 1024 * 16; // maximum number of bytes allowed per serial read
    icm20948_serif.max_write = 1024 * 16; // maximum number of bytes allowed per serial write
    icm20948_serif.is_spi    = FALSE;     // we are using I2C

    // Reset icm20948 driver states
    inv_icm20948_reset_states(&icm_device, &icm20948_serif);
    inv_icm20948_register_aux_compass(&icm_device,
                                      INV_ICM20948_COMPASS_ID_AK09916,
                                      AK0991x_DEFAULT_I2C_ADDR);

    // Setup the icm20948 device
    rc = icm20948_sensor_setup();

    if (icm_device.selftest_done && !icm_device.offset_done)
    {
        // If we've run self test and not already set the offset.
        inv_icm20948_set_offset(&icm_device, unscaled_bias);
        icm_device.offset_done = 1;
    }

    // Now that Icm20948 device is initialized, we can proceed with DMP image loading
    // This step is mandatory as DMP image is not stored in non volatile memory
    rc += load_dmp3();
    if (rc != 0)
    {
        return; // Handle error
    }

    // Set mode
    inv_icm20948_set_lowpower_or_highperformance(&icm_device, settings.mode);

    // Set frequency
    rc = inv_icm20948_set_sensor_period(&icm_device,
                                        idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR),
                                        1000 / settings.quaternion_frequency);
    rc = inv_icm20948_set_sensor_period(&icm_device,
                                        idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE),
                                        1000 / settings.gyroscope_frequency);
    rc = inv_icm20948_set_sensor_period(&icm_device,
                                        idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER),
                                        1000 / settings.accelerometer_frequency);
    rc = inv_icm20948_set_sensor_period(&icm_device,
                                        idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER),
                                        1000 / settings.magnetometer_frequency);

    // Enable / disable
    rc = inv_icm20948_enable_sensor(&icm_device,
                                    idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE),
                                    settings.enable_gyroscope);
    rc = inv_icm20948_enable_sensor(&icm_device,
                                    idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER),
                                    settings.enable_accelerometer);
    rc = inv_icm20948_enable_sensor(&icm_device,
                                    idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR),
                                    settings.enable_quaternion);
    rc = inv_icm20948_enable_sensor(&icm_device,
                                    idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER),
                                    settings.enable_magnetometer);
}

void dd_icm209_task()
{
    inv_icm20948_poll_sensor(&icm_device, (void*)0, build_sensor_event_data);
}

bool_t dd_icm209_gyroDataIsReady()
{
    return gyro_data_ready;
}

bool_t dd_icm209_accelDataIsReady()
{
    return accel_data_ready;
}

bool_t dd_icm209_magDataIsReady()
{
    return mag_data_ready;
}

bool_t dd_icm209_quatDataIsReady()
{
    return quat_data_ready;
}

void dd_icm209_readGyroData(float* x, float* y, float* z)
{
    *x              = gyro_x;
    *y              = gyro_y;
    *z              = gyro_z;
    gyro_data_ready = FALSE;
}

void dd_icm209_readAccelData(float* x, float* y, float* z)
{
    *x               = accel_x;
    *y               = accel_y;
    *z               = accel_z;
    accel_data_ready = FALSE;
}

void dd_icm209_readMagData(float* x, float* y, float* z)
{
    *x             = mag_x;
    *y             = mag_y;
    *z             = mag_z;
    mag_data_ready = FALSE;
}

void dd_icm209_readQuatData(float* w, float* x, float* y, float* z)
{
    *w              = quat_w;
    *x              = quat_x;
    *y              = quat_y;
    *z              = quat_z;
    quat_data_ready = FALSE;
}
