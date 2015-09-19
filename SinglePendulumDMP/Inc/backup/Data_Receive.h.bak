#ifndef _RC_RC_H_
#define _RC_RC_H_
//#include "Tim_Pwm_In.h"
#include "stm32f4xx_hal.h"

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;}T_RC_DATA;

extern T_RC_DATA Rc_Data;//1000~2000
extern uint8_t ARMED;

void Rc_DataAnl(void);
void Rc_DataCal(void);

#endif
