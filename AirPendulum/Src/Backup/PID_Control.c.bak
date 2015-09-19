//#include "nine_axis_module.h"
//#include "delay.h"
#include "math.h"
//#include "mpu9150.h"
#include "motor.h"
#include "PID_Control.h"

#include "coordinatetrans.h"

PID_Struct PID_Pitch_Struct;				 //定义Pitch的PID结构体,绕Y轴
PID_Struct PID_Roll_Struct;			 //定义Roll的PID结构体,绕X轴
PID_Struct PID_Z_Struct;			 //定义Angle_Z的PID结构体
PID_Struct PID_Para;

PID_Struct PID_Pitch_Rate_Struct;     //定义Pitch变化率的PID结构体
PID_Struct PID_Roll_Rate_Struct;      //定义Roll变化率的PID结构体


//float Time_dt;                 //单环PID时只涉及一个时间，但当双环时两个时间不同
float Angle_Time_dt;             //角度环PID时间,
float Rate_Time_dt;              //速度环PID时间

float pid_pitch;               //绕Z轴角
float pid_roll;                //绕Y轴角
float pid_z;

float pid_pitch_der;
float pid_roll_der;
float pid_z_der;

short ax;
short ay;
short az;

float gx;
float gy;
float gz;

uint16_t MOTOR1=1000;
uint16_t MOTOR2=1000;
uint16_t MOTOR3=1000;	
uint16_t MOTOR4=1000;
uint16_t Motor_Thr=1400;         //设置电机初始速度


float Motor_Angle_X=0.0;					   //横滚期望
float Motor_Angle_Y=0.0;					   //俯仰期望
float Motor_Angle_Z=0.0;					   //油门
//float Motor_Rud=0.0;					   //航向期望

float Pitch_Der_Rate=0.0;              //Pitch变化率期望
float Roll_Der_Rate=0.0;               //Roll变化率期望

float Roll = 0;
float Pitch = 0;
float Yaw;
float Angle_Z;

float Pitch_Der;                   //Pitch变化率
float Roll_Der;                    //Roll变化率

extern float Init_Pitch;

/* 引用自motor.c */
extern const uint16_t motor1[62];
extern const uint16_t motor2[70];
extern const uint16_t motor3[46];
extern const uint16_t motor4[70];

float SpeedInitVal_Pitch_PlusDirec(float pitch);
float SpeedInitVal_Pitch_MinusDirec(float pitch);
float SpeedInitVal_Roll_PlusDirec(float roll);
float SpeedInitVal_Roll_MinusDirec(float pitch);

void Delay_us(uint16_t nus)
{		
	uint16_t a; 
	
	for(a=0;a<nus;a++)
	{
		uint8_t i;
		for(i=6;i>0;i--);
	}
}

void Delay_ms(uint16_t num)
{	 		  	  
	uint16_t  a;

	for(a=0;a<num;a++)
	{
	   Delay_us(1000);			   
	}	  	    
} 

//float Time_dt;
//float pid_roll;
//float pid_pitch;
//float pid_yaw;
//uint16_t MOTOR1;
//uint16_t MOTOR2;
//uint16_t MOTOR3;	
//uint16_t MOTOR4;
//float Motor_Ail=0.0;					   //横滚期望
//float Motor_Ele=0.0;					   //俯仰期望
//float Motor_Thr=0.0;					   //油门
//float Motor_Rud=0.0;					   //航向期望

//PID_Struct PID_Yaw_Struct;				 //定义Yaw的PID结构体
//PID_Struct PID_Roll_Struct;			     //定义Roll的PID结构体
//PID_Struct PID_Pitch_Struct;			 //定义Pitch的PID结构体
//PID_Struct PID_Para;					 //定义PID_Para的PID结构体，用于在线调试PID参数
/*************************
      初始化PID参数
*************************/
void PID_Init(PID_Struct *PID)
{
  PID->expectation         = 0.0;            //遥控给的期望值
  PID->Err_k			   = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;           //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;           //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;			//误差和
  PID->Kp				   = 0.0;           //比例系数，通过串口在线调PID参数再写入Flash
  PID->Ti				   = 0.0;           //积分系数，通过串口在线调PID参数再写入Flash
  PID->Td				   = 0.0;           //微分系数，通过串口在线调PID参数再写入Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 800.0;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -800.0;		    //限制输出量最小值
  PID->PID_Integral_Max    = 200.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -200.0;			//限制积分项最小值
}

/***************************************************************
PID计算-位置式
***************************************************************/
float PID_Calculate(PID_Struct* PID, float measured, float expect, float time_dt)
{
  float Value_Proportion;   //比例项
  float Value_Integral;		//积分项
  float Value_Derivative;	//微分项

  PID->expectation =  expect;
  PID->Err_k = PID->expectation - measured;
  PID->SumErr = PID->SumErr + PID->Err_k;
	
//	printf("\n Roll=%f \n\r", measured);
//	printf("\n Err_K=%f \n\r", PID->Err_k);
//  printf("\n SumErr=%f, ", PID->SumErr);


  //P I D
  Value_Proportion    = PID->Kp * PID->Err_k;
//  Value_Integral      = PID->Kp * PID->SumErr * Time_dt / PID->Ti;	
	Value_Integral      = PID->Kp * PID->SumErr * time_dt * PID->Ti;
  Value_Derivative  = PID->Kp * PID->Td * (PID->Err_k - PID->Err_k_1) / time_dt;

//	printf("\n Value_Proportion=%f, Value_Integral=%f, Value_Derivative=%f \n\r", Value_Proportion, Value_Integral, Value_Derivative);
	
  if(Value_Integral > PID->PID_Integral_Max)
  {
//    PID->SumErr -= PID->Err_k;
		Value_Integral = PID->PID_Integral_Max;
  }
  if(Value_Integral < PID->PID_Integral_Min)
  {
//  	PID->SumErr -= PID->Err_k;
    Value_Integral = PID->PID_Integral_Min;
  }
  
  PID->Ouput_deltaUk = Value_Proportion + Value_Integral + Value_Derivative;

  if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;}
  if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;}

  PID->Err_k_1 = PID->Err_k;	  //保存k-1次误差值
  
  return PID->Ouput_deltaUk;
}

/*************************
输出给电机的PWM计算
*************************/
void MOTOR_CAL(void)		  
{
	
  Angle_Time_dt = Get_Angle_PIDtime();
	pid_pitch = PID_Calculate(&PID_Pitch_Struct, Pitch, Motor_Angle_X, Angle_Time_dt);
	
	pid_roll = PID_Calculate(&PID_Roll_Struct, Roll, Motor_Angle_Y, Angle_Time_dt);
	
	
//	printf("\n PID_ROLL=%f, PID_PITCH=%f, PID_YAW=%f \n\r", pid_pitch, pid_roll, pid_z);
//	printf("\n PID_Pitch=%f, PID_Roll=%f \n\r", pid_pitch, pid_roll);
	
	//+模式
	
	MOTOR1 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Roll_PlusDirec(Motor_Angle_Y)+pid_roll);	//Y轴负向电机
	MOTOR2 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Roll_MinusDirec(Motor_Angle_Y)-pid_roll);	//Y轴正向电机
	MOTOR3 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Pitch_PlusDirec(Motor_Angle_X)+pid_pitch);	//X轴负向电机
	MOTOR4 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Pitch_MinusDirec(Motor_Angle_X)-pid_pitch);	//X轴正向电机


	
//	printf("\n pid_pitch_plusdirec_initval=%f, Motor4=%d \n\r", pid_pitch_plusdirec_initval, MOTOR4);
	
	if(Motor_Thr<=1050)//防止未加油门时，由于四轴倾斜或旋转导致电机转动
	{
		MOTOR1=1000;
		MOTOR2=1000;
		MOTOR3=1000;
		MOTOR4=1000;
	}		  
}

/**********************************************
根据电调需要的PWM占空比来确定，保证高电平时间在
0.875ms~2ms之间
**********************************************/
float Limit_PWMOUT(float MOTOR)
{
   if(MOTOR>PWM_MOTOR_MAX)
   {
     MOTOR=PWM_MOTOR_MAX;
   }
   else if(MOTOR<PWM_MOTOR_MIN)
   {
	   MOTOR=PWM_MOTOR_MIN;
   }
   else 
   {
	   MOTOR=MOTOR;
   }

   return MOTOR;
}

/***************************************************************
计算不同Pitch对应的电机转速基础值 正向对应Motor3,负向对应Motor4
****************************************************************/
float SpeedInitVal_Pitch_PlusDirec(float pitch)
{
//	static float p[2]={17.10863986, 1112.0615911};
  if(pitch>=1)
	{
//		float speedinitval=p[0]*pitch+p[1];
//		return speedinitval;	
		return motor3[(uint16_t)pitch];
	}
	else
	{
		return 1050;
	}
}

float SpeedInitVal_Pitch_MinusDirec(float pitch)
{
//	static float p[2]={-15.8730, 1047.619};
	if(pitch<=-1)
	{
//		float speedinitval=p[0]*pitch+p[1];
//		return speedinitval;
		return motor4[(uint16_t)(fabs(pitch))];
	}
	else
	{
		return 1050;
	}
}

/***************************************************************
计算不同Roll对应的电机转速基础值 正向对应Motor1,负向对应Motor2
****************************************************************/
float SpeedInitVal_Roll_MinusDirec(float roll)
{
//	static float p[2]={-20.9205, 1062.7615};
	if(roll<=-1)
	{
//		float speedinitval=p[0]*roll+p[1];
//		return speedinitval;
		return motor2[(uint16_t)(fabs(roll))];
	}
	else
	{
		return 1050;
	}
}

float SpeedInitVal_Roll_PlusDirec(float roll)
{
//	static float p[2]={17.8571, 1035.7142};
	if(roll>=1)
	{
//		float speedinitval=p[0]*roll+p[1];
//		return speedinitval;
		return motor1[(uint16_t)roll];
	}
	else
	{
		return 1050;
	}		
}


/* X轴期望值改变函数，用于调试单轴摆动,若想作为Y轴调试用，需将Motor_Angle_X改为Motor_Angle_Y */
void Expect_Change(PID_Struct *PID_Pitch_Struct, float Amplitude, float w, float mil_second)
{
//	if((PID_Pitch_Struct->expectation < Amplitude)&&(PID_Pitch_Struct->expectation >=0))
//	{
//		PID_Pitch_Struct->expectation += 2;
//	}
//	else if(PID_Pitch_Struct->expectation >= Amplitude)
//	{
//		PID_Pitch_Struct->expectation -= 2;
//	}
	static float delta = 0;
	static float theta = 0;
//	if(TimeOut == Yes)   //Flag TimeOut用于指示采样频率
//	{
//		delta = w * second;
	  delta = w;
		theta += delta;
		if(theta > 180)
		{
			theta -= 360;
		}
//		Delay_ms(mil_second);
		Motor_Angle_X = Amplitude*sin(theta*pi/180.0f);
//  }
}

/* 三维圆摆期望值改变函数，X轴相位超前Y轴90度 */
void Expext_Change_Circle(PID_Struct *PID_Pitch_Struct, PID_Struct *PID_Roll_Struct, float Amplitude, float w, float mil_second)
{
	static float delta = 0;  
	static float phi = 0;    //Y轴相位
	static float theta = 0;  //X轴相位
//	phi = w * second;
	delta = w;
	phi += delta;
	theta = phi + 90;        //X轴相位超前Y轴相位
	if(theta > 180)
	{
		theta -= 360;
	}
	
	if(phi > 180)
	{
		phi -= 360;
	}
	
//	Delay_ms(mil_second);
	
//	PID_Pitch_Struct->expectation = Amplitude*sin(theta*pi/180.0f);
//	PID_Roll_Struct->expectation = Amplitude*sin(phi*pi/180.0f);
	Motor_Angle_X = Amplitude*sin(theta*pi/180.0f);
	Motor_Angle_Y = Amplitude*sin(phi*pi/180.0f);
}

//PID计算函数，角度环->速度环；角度环只含P，计算值作为速度环的期望值
/* 角度环 */
void CtrlAttiAng(void)
{
	Angle_Time_dt = Get_Angle_PIDtime();
	
	PID_Calculate(&PID_Pitch_Struct, Pitch, Motor_Angle_X, Angle_Time_dt);
	
	PID_Calculate(&PID_Roll_Struct, Roll, Motor_Angle_Y, Angle_Time_dt);
	
//	printf("\n t=%f, PID_Pitch=%f, PID_Roll=%f \n\r", Angle_Time_dt, PID_Pitch_Struct.Ouput_deltaUk, PID_Roll_Struct.Ouput_deltaUk);
	
}

//run in 200Hz or 400Hz loop 
void CtrlAttiRate(void)
{
	Rate_Time_dt = Get_Rate_PIDtime();
	
	pid_pitch = PID_Calculate(&PID_Pitch_Rate_Struct, gy, PID_Pitch_Struct.Ouput_deltaUk, Rate_Time_dt);  //绕x轴为gy,Pitch
	
	pid_roll = PID_Calculate(&PID_Roll_Rate_Struct, gx, PID_Roll_Struct.Ouput_deltaUk, Rate_Time_dt);     //绕Y轴为gx，Roll
	
//	printf("\n t=%f, PID_Pitch=%f, PID_Roll=%f \n\r", Rate_Time_dt, pid_pitch, pid_roll);
	
	MOTOR1 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Roll_PlusDirec(Motor_Angle_Y)+pid_roll);	//Y轴负向电机
	MOTOR2 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Roll_MinusDirec(Motor_Angle_Y)-pid_roll);	//Y轴正向电机
	MOTOR3 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Pitch_PlusDirec(Motor_Angle_X)+pid_pitch);	//X轴负向电机
	MOTOR4 = (uint16_t)Limit_PWMOUT(SpeedInitVal_Pitch_MinusDirec(Motor_Angle_X)-pid_pitch);	//X轴正向电机
}



///**********************************************
//将捕捉到的遥控pwm信号转化成Motor_Thr、Motor_Ele、
//Motor_Ail、Motor_Rud，用于与传感器测量得到的欧拉
//角做差，进行PID计算
//**********************************************/
//void Change_InputPWM_To_Expect(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4)
//{
//  if(ch1<1000){ ch1=1000;}			   //副翼AIL(roll)
//  if(ch1>2000){	ch1=2000;}	

//  if(ch2<1000){	ch2=1000;}			   //升降舵ELE(pitch)
//  if(ch2>2000){	ch2=2000;}

//  if(ch3<1000){	ch3=1000;}			   //油门THR
//  if(ch3>2000){	ch3=2000;}

//  if(ch4<1000){	ch4=1000;}			   //方向舵RUD(yaw)
//  if(ch4>2000){	ch4=2000;}  

//  Motor_Ail=(float)((ch1-AilMiddle) * 0.01);	//roll的期望值-ch1-PA0，对应遥控通道1  值域-5~+5，对应角度并限制范围
////  printf("\n Motor_Ail =%f\n\r", Motor_Ail);
//	Motor_Ele=(float)((ch2-EleMiddle) * 0.01);	//pitch的期望值-ch2-PA1，对应遥控通道2  值域-5~+5，对应角度并限制范围		 
//  Motor_Thr=(float)ch3;					        //油门的期望值-ch3-PA2，对应遥控通道3  值域1000~2000
//  Motor_Rud=(float)((ch4-RudMiddle) * 0.01);	//yaw的期望值-ch4-PA3，对应遥控通道4  值域-5~+5，对应角度并限制范围				
////  printf("\n Motor_Thr=%f \n\r", Motor_Thr);
//} 
///***************************************************************
//PID计算-增量式，需要限制积分项大小和output大小吗？
//***************************************************************/
//float PID_Calculate(PID_Struct* PID, float measured, float expect, float AHRSUpdate_dt)
//{
//  float Value_Proportion;   //比例项
//  float Value_Integral;		//积分项
//  float Value_Derivative;	//微分项
//
//  PID->expectation =  expect;
//  PID->Err_k = PID->expectation - measured;
//
//  //在确定了Kp Ti Td以后就可以将以下三个式子化简为Ae(k)+Be(k-1)+Ce(k-2)
//  Value_Proportion    = PID->Kp * (PID->Err_k - PID->Err_k_1);
//  Value_Integral      = PID->Kp * PID->Err_k * AHRSUpdate_dt / PID->Ti;	  //以后变除法为乘法
////  if(Value_Integral > PID->PID_Integral_Max)
////  {Value_Integral = PID->PID_Integral_Max;}
////  if(Value_Integral < PID->PID_Integral_Max)
////  {Value_Integral = PID->PID_Integral_Min;}
//  Value_Derivative  = PID->Kp * PID->Td * (PID->Err_k - 2.0f*PID->Err_k_1 + PID->Err_k_2) / AHRSUpdate_dt;
//  
//  PID->Ouput_deltaUk = Value_Proportion + Value_Integral + Value_Derivative;
//
////  if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
////    PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;
////  if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
////    PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;
//  
//  PID->Err_k_2 = PID->Err_k_1;    //保存k-2次误差值
//  PID->Err_k_1 = PID->Err_k;	  //保存k-1次误差值
//  
//  return PID->Ouput_deltaUk;
//}



/*------------------------------------------------------------------------------------------*/
