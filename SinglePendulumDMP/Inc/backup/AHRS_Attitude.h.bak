#ifndef __AHRS_Attitude_H
#define __AHRS_Attitude_H

#include "stm32f4xx_hal.h"
//#include "nine_axis_module.h"

extern float Pitch, Roll, Yaw;
extern float halfT;
extern float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
extern float MXgain;
extern float MYgain;
extern float MZgain;
extern float MXoffset;
extern float MYoffset;
extern float MZoffset;
extern float heading;

void get_sensor_data(void);
void init_quaternion(void);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);
void Get_Attitude(void);
int get_gyro_bias(void);
void get_compass_bias(void);
void compass_calibration(void);
extern void Read_HMC5883l(void);
void Prepare_Data(void);


#endif
