#include <stdint.h>
#include "I2C_driver.h"
#define MPU6050_ADDR     0x68
#define WHO_AM_I         0x75
//init config registers
#define PWR_MGMT_1       0x6B
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
//data
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
void readMpuData();
void mpuInit();
void callibrationStuff();

extern float accel_angle_offset;

extern float gyro_z_offset;
