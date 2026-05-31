#include"mpu6050_driver.h"
#include"IMU.h"
#include "math.h"
#include "delay.h"
IMU_Data_t actual_imu_data;
IMU_Data_t *System_IMU = &actual_imu_data;
//uint8_t collabBuf[14];

float accel_angle_offset =0;

float gyro_z_offset =0;
void MPU_WriteReg(uint8_t reg, uint8_t value,uint8_t length) {

    i2cWrite(MPU6050_ADDR, reg, &value, length);
}

void mpuInit()
{
	MPU_WriteReg(PWR_MGMT_1, 0x80, 1);
	delay_ms(100);
	uint8_t whoAmICheck;
    i2cRead(MPU6050_ADDR, WHO_AM_I, &whoAmICheck, 1);

	if(whoAmICheck!=MPU6050_ADDR)
	{
		/*if break point hit that point there is a problem about find the device*/
	}

	//init config

	// Set Gyro full scale range to ±250°/s for maximum resolution in tight tilt control
	MPU_WriteReg(GYRO_CONFIG, 0x00, 1);

	// Wake up MPU6050 and set X-axis gyro as the stable PLL clock reference
	MPU_WriteReg(PWR_MGMT_1, 0x01, 1);

	// Enable Digital Low Pass Filter (DLPF) at ~42Hz to suppress motor vibrations
	MPU_WriteReg(CONFIG, 0x03, 1);

	// Set Accel full scale range to ±2g for highest sensitivity to gravity-based tilt
	MPU_WriteReg(ACCEL_CONFIG, 0x00, 1);
}

void readMpuData()
{
	uint8_t collabBuf[14] = {0};
    i2cRead(MPU6050_ADDR, ACCEL_XOUT_H, collabBuf, 14);

    System_IMU->Accel_X = (int16_t)((collabBuf[0] << 8)  | collabBuf[1]);
    System_IMU->Accel_Y = (int16_t)((collabBuf[2] << 8)  | collabBuf[3]);
    System_IMU->Accel_Z = (int16_t)((collabBuf[4] << 8)  | collabBuf[5]);

    // collabBuf[6] ve [7]  (TEMP_OUT)
    System_IMU->Gyro_Z  = (int16_t)((collabBuf[12] << 8) | collabBuf[13]);
}
void callibrationStuff()
{
    int32_t Ax_sum = 0, Ay_sum = 0, Gz_sum = 0;

    for(int i=0; i<500; i++)
    {
        readMpuData();
        Ax_sum += System_IMU->Accel_X;
        Ay_sum += System_IMU->Accel_Y;
        Gz_sum += System_IMU->Gyro_Z;
        delay_ms(2);
    }

    gyro_z_offset = (float)Gz_sum / 500.0f;

    float avg_X = (float)Ax_sum / 500.0f;
    float avg_Y = (float)Ay_sum / 500.0f;
    accel_angle_offset = atan2(avg_Y, avg_X) * (180.0f / 3.14159265f);
}

