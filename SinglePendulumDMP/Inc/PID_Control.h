#ifndef __PID_Control_H
#define __PID_Control_H

#include "stm32f4xx_hal.h"

#define AilMiddle        1501         //副翼中点
#define EleMiddle        1501         //升降舵中点
#define RudMiddle        1501         //方向舵中点
//#define PWM_MOTOR_MIN    1000	      //电调输出最小脉宽0.875ms
#define PWM_MOTOR_MIN    1050	      //电调输出最小脉宽0.875ms
#define PWM_MOTOR_MAX    2000		  //电调输出最大脉宽2ms

#define SpeedStep 1

//float Time_dt;
//float pid_roll;
//float pid_pitch;
//float pid_yaw;
//uint16_t MOTOR1;
//uint16_t MOTOR2;
//uint16_t MOTOR3;	
//uint16_t MOTOR4;
////extern uint16_t Motor_Left;
////extern uint16_t Motor_Right;
////extern uint16_t Motor_Front;	
////extern uint16_t Motor_Back;
//float Motor_Ail=0.0;					   //横滚期望
//float Motor_Ele=0.0;					   //俯仰期望
//float Motor_Thr=0.0;					   //油门
//float Motor_Rud=0.0;					   //航向期望

 
typedef struct {
  float expectation;            //遥控给的期望值
  float Err_k;			     //当前误差值e(k)
  float Err_k_1;		     //k-1时刻误差值e(k-1)
  float Err_k_2;		     //k-2时刻误差值e(k-2)
  float SumErr;              //误差的和
  float Kp;				     //比例系数
  float Ti;				     //积分系数
  float Td;				     //微分系数
  float Ouput_deltaUk;		 //PID计算后的输出量U(k) - U(k-1)，增量式
  float Ouput_deltaUk_Max;		 //限制输出量最大值
  float Ouput_deltaUk_Min;		 //限制输出量最小值
  float PID_Integral_Max;				 //限制积分项最大值
  float PID_Integral_Min;				 //限制积分项最小值
} PID_Struct;

extern PID_Struct PID_Z_Struct;				 //定义Yaw的PID结构体
extern PID_Struct PID_Roll_Struct;			 //定义Roll的PID结构体
extern PID_Struct PID_Pitch_Struct;			 //定义Pitch的PID结构体
extern PID_Struct PID_Para;

extern PID_Struct PID_Pitch_Rate_Struct;     //定义Pitch变化率的PID结构体
extern PID_Struct PID_Roll_Rate_Struct;      //定义Roll变化率的PID结构体

void PID_Init(PID_Struct *PID);
float PID_Calculate(PID_Struct* PID, float measured, float expect, float time_dt);
void MOTOR_CAL(void);
float Limit_PWMOUT(float MOTOR);
void Change_InputPWM_To_Expect(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);

float SpeedInitVal_Pitch_PlusDirec(float pitch);
float SpeedInitVal_Pitch_MinusDirec(float pitch);
float SpeedInitVal_Roll_PlusDirec(float roll);
float SpeedInitVal_Roll_MinusDirec(float pitch);

void Expext_Change_Circle(PID_Struct *PID_Pitch_Struct, PID_Struct *PID_Roll_Struct, float Amplitude, float w, float mil_second);
void Expect_Change(PID_Struct *PID_Pitch_Struct, float Amplitude, float w, float mil_second);
void Expect_Change_TriAngle(PID_Struct *PID_Pitch_Struct, float Amplitude, float step);


void Delay_us(uint16_t nus);
void Delay_ms(uint16_t num);

void CtrlAttiAng(void);
void CtrlAttiRate(void);

#endif
