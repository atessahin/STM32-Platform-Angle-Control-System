#ifndef IMU_H_
#define IMU_H_
#endif /* IMU_H_ */
#include <stdint.h>
#define ALPHA 0.98f
typedef struct {
	int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_Z;
} IMU_Data_t;


typedef struct {
    float Accel_X_g;
    float Accel_Y_g;
    float Accel_Z_g;
    float Gyro_Z_dps;
} IMU_ScaledData_t;



extern IMU_ScaledData_t *System_IMU_Scalde_Data;
extern IMU_Data_t *System_IMU;
extern 	uint8_t collabBuf[14];
extern uint32_t now;
extern volatile float currentAngle;
extern uint32_t last_time;


void rawToDataAngularVel();
void rawToDataAccel();
void complementaryFilter();
float calculateTime();
void motorRight(uint32_t speed);

void motorLeft(uint32_t speed);

void motorStop();
void motorBreak();
void IMU_ResetAngleFromAccel(void);
