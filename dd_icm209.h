#ifndef TEENSY_ICM_20948_H
#define TEENSY_ICM_20948_H

#include "su_common.h"

/*************************************************************************
  Defines
*************************************************************************/

typedef struct
{
    int    mode;
    bool_t enable_gyroscope;
    bool_t enable_accelerometer;
    bool_t enable_magnetometer;
    bool_t enable_quaternion;
    int    gyroscope_frequency;
    int    accelerometer_frequency;
    int    magnetometer_frequency;
    int    quaternion_frequency;
} TeensyICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

response_status_t dd_icm209_init(TeensyICM20948Settings p_settings);
void              dd_icm209_task();

bool_t dd_icm209_gyro_data_is_ready();
bool_t dd_icm209_accel_data_is_ready();
bool_t dd_icm209_mag_data_is_ready();
bool_t dd_icm209_quat_data_is_ready();

void dd_icm209_read_gyro_data(float* ppt_x, float* ppt_y, float* ppt_z);
void dd_icm209_read_accel_data(float* ppt_x, float* ppt_y, float* ppt_z);
void dd_icm209_read_mag_data(float* ppt_x, float* ppt_y, float* ppt_z);
void dd_icm209_read_quat_data(float* ppt_w, float* ppt_x, float* ppt_y, float* ppt_z);

#endif // TEENSY_ICM_20948_H
