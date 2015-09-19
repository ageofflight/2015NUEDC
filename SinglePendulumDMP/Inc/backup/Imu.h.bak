#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx.h"
#include "MPU6050.h"
//#include "ms5611.h"


typedef struct{
				float X;
				float Y;
				float Z;}T_FLOAT_XYZ;
typedef struct{
				float ROL;
				float PIT;
				float YAW;}T_FLOAT_ANGEL;
extern T_FLOAT_ANGEL Q_ANGLE;
//extern T_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//


void Prepare_Data(void);
void Get_Attitude(void);

#endif
