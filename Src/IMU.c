#include"IMU.h"
#include"math.h"
#include"delay.h"
#include"mpu6050_driver.h"
IMU_ScaledData_t scaled_imu_data;
IMU_ScaledData_t *System_IMU_Scalde_Data = &scaled_imu_data;
uint32_t now = 0;
volatile float currentAngle = 0.0f;
uint32_t last_time = 0;
/* Calculates elapsed time in seconds. */
float calculateTime()
{
    uint32_t now = millis();
    if (last_time == 0) last_time = now;

    float diff_ms = (float)(now - last_time);
    float dt_local = diff_ms / 1000.0f;
    last_time = now;
    return dt_local;
}
/*Acceleration = (Accelerometer axis raw data / 65536 * full scale Acceleration range) g*/
void rawToDataAccel()
{
    // convert raw x to g-force (16384 lsb/g for +-2g)
    System_IMU_Scalde_Data->Accel_X_g = ((float)System_IMU->Accel_X / 16384.0f);

    // convert raw y to g-force
    System_IMU_Scalde_Data->Accel_Y_g = ((float)System_IMU->Accel_Y / 16384.0f);

    // convert raw z to g-force
    System_IMU_Scalde_Data->Accel_Z_g = ((float)System_IMU->Accel_Z / 16384.0f);
}

/*Angular velocity = (Gyroscope axis raw data / 65536 * full scale Gyroscope range)*/
void rawToDataAngularVel()
{
	float corrected_gyro_z = System_IMU->Gyro_Z - (float)gyro_z_offset;
    System_IMU_Scalde_Data->Gyro_Z_dps = ((float)corrected_gyro_z / 131.0f);
}
/* Keeps angles inside the -180 to +180 degree range. */
static float wrap180(float a)
{
    if (a > 180.0f)  a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}
/* Combines gyro prediction with accelerometer correction. */
void complementaryFilter()
{
    /* Fixed 10 ms sample time from the control task. */
    float dt_val = 0.01f;

    /* Angle estimated from gravity direction. */
    float accelAngle = atan2(System_IMU_Scalde_Data->Accel_Y_g,
                             System_IMU_Scalde_Data->Accel_X_g) * (180.0f / 3.14159265f);

    /* Remove the initial mechanical offset. */
    float accurate_angle = wrap180(accelAngle - accel_angle_offset);

    /* Gyro gives short-term angular speed. */
    float gyroRate = System_IMU_Scalde_Data->Gyro_Z_dps;

    /* Predict current angle from gyro integration. */
    float predicted = currentAngle + (-gyroRate) * dt_val;

    /* Correct prediction slowly using accelerometer angle. */
    float acc_error = wrap180(accurate_angle - predicted);
    currentAngle = predicted + (1.0f - ALPHA) * acc_error;

    currentAngle = wrap180(currentAngle);
}
void motorRight(uint32_t speed)
{

    if (speed > 3500) speed = 3500;
    TIM3->CCR1 = speed;
    TIM3->CCR2 = 0;
}
void motorLeft(uint32_t speed)
{
    if (speed > 3500) speed = 3500;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = speed;
}
void motorStop()
{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}

void motorBreak()
{
	TIM3->CCR1 = 2200;
	TIM3->CCR2 = 2200;
}


