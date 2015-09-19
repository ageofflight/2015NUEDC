#ifndef __URANUS_H
#define __URANUS_H


#include "stm32f4xx.h"
#include <stdio.h>

enum input_status
{
	STATUS_IDLE,
	STATUS_SOF,
	STATUS_LEN,
	STATUS_DATA,
};

typedef struct 
{
	int16_t accl[3];
	int16_t gyro[3];
	int16_t mag[3];
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
	int32_t presure;
} imu_data;

void imu_rev_process(char ch);
int imu_rev_get_data(imu_data* data);

void get_control_data(uint8_t* Rx_Data);

#endif

