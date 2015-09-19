#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

typedef enum 
{
  InitState = 0, 
  OnState = !InitState
} WorkStatus;

extern uint16_t PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4;


extern void TIM1_Motor_PWMOutput(uint16_t DR1,uint16_t DR2,uint16_t DR3,uint16_t DR4);
float Get_Nowtime(void);
float Get_Angle_PIDtime(void);
float Get_Rate_PIDtime(void);
float Get_Sampletime(void);



#endif
