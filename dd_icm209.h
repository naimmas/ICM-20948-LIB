#ifndef __TEENSY_ICM_20948_H__
#define __TEENSY_ICM_20948_H__

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

response_status_t dd_icm209_init(TeensyICM20948Settings settings);
void              dd_icm209_task();

bool_t dd_icm209_gyroDataIsReady();
bool_t dd_icm209_accelDataIsReady();
bool_t dd_icm209_magDataIsReady();
bool_t dd_icm209_quatDataIsReady();

void dd_icm209_readGyroData(float* x, float* y, float* z);
void dd_icm209_readAccelData(float* x, float* y, float* z);
void dd_icm209_readMagData(float* x, float* y, float* z);
void dd_icm209_readQuatData(float* w, float* x, float* y, float* z);

#endif // __TEENSY_ICM_20948_H__
