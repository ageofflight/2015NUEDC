#include "pendulumcontrol.h"
#include "PID_Control.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

//来自PID_Control

extern PID_Struct PID_Pitch_Struct;				 //定义Pitch的PID结构体,绕Y轴
extern float Time_dt;
extern float pid_pitch;               //绕Z轴角

extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;

extern short ax;
extern short ay;
extern short az;

extern float gx;
extern float gy;
extern float gz;

extern float Roll;
extern float Pitch;
extern float Pitch_Der;                   //Pitch变化率
extern float Roll_Der;                    //Roll变化率

extern float Motor_Angle_X;					   //横滚期望
extern float Motor_Angle_Y;					   //俯仰期望


extern PID_Struct PID_Pitch_Rate_Struct;     //定义Pitch变化率的PID结构体
extern PID_Struct PID_Roll_Rate_Struct;      //定义Roll变化率的PID结构体

extern PID_Struct PID_Pitch_Struct;				 //定义Pitch的PID结构体,绕Y轴
extern PID_Struct PID_Roll_Struct;			 //定义Roll的PID结构体,绕X轴

//

/* 引用自main.c */
extern uint8_t Hz57;
extern uint8_t Hz167;
/* ------------ */

PID_Struct PID_Pitch_Struct_Plus;
PID_Struct PID_Pitch_Struct_Minus;

S_FLOAT_XYZ DIF_ACC;		//实际去期望相差的加速度
S_FLOAT_XYZ EXP_ANGLE;	//期望角度
S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	

/* 标志 */
uint8_t Circle_Flag=0;
uint8_t Eight_Flag=0;
uint8_t NDirection_Flag=0;
uint8_t eAmplitude_Flag=0;
uint8_t Freefall_Flag=0;

uint8_t Mode = 0;
/*-----------*/

void pendulum_conrol_motor_cal(void);

float Init_Pitch=0;               //Init_Pitch用于等摆幅控制时摆角的参考，当按键按下时记录

SwingDirection Pitch_Direction=PlusDirection;
SwingDirection Roll_Direction=PlusDirection;

float Amplitude=15;   //可控摆幅模式下摆幅参数,默认值为15

float Radius=20;      //可控圆摆默认半径

float Radius_Eight=20;   //八字摆幅度

float nDirection=45;   //任意方向摆角度

float nAmplitude=20;   //任意方向摆幅度



//void Delay_us(uint16_t nus)
//{		
//	uint16_t a; 
//	
//	for(a=0;a<nus;a++)
//	{
//		uint8_t i;
//		for(i=6;i>0;i--);
//	}
//}

//void Delay_ms(uint16_t num)
//{	 		  	  
//	uint16_t  a;

//	for(a=0;a<num;a++)
//	{
//	   Delay_us(1000);			   
//	}	  	    
//} 


//void pendulum_control(float force, float period)
void pendulum_control(void)
{
	//force = m *sin(theta)
	if(Pitch_Direction==PlusDirection)
	{
		TIM1_Motor_PWMOutput(1000,1000,1000,1400);
	}
	else if(Pitch_Direction==MinusDirection)
	{
		TIM1_Motor_PWMOutput(1000,1000,1400,1000);
	}

}

void Get_ChangRate(float* Pitch, float* Roll, float* Pitch_Der, float* Roll_Der)
{
	static float Pitch_Prev=0;
	static float Roll_Prev=0;
	float time_temp=Get_Nowtime();
	*Pitch_Der=(*Pitch-Pitch_Prev)/time_temp;
	*Roll_Der=(*Roll-Roll_Prev)/time_temp;
	Pitch_Prev=*Pitch;
	Roll_Prev=*Roll;
}


/* 一个电机提供动力抵消能量损耗实现自由等幅摆 */
void pendulum_control_EAmplitude(void)
{
////	static float angle=20;
////	Get_ChangRate(&Pitch, &Roll, &Pitch_Der, &Roll_Der);
//	if((fabs(Pitch-Init_Pitch)<5.0) || fabs(Pitch+Init_Pitch<5.0))  //该限制条件只允许PID用于初始方向
	if((fabs(Pitch-Init_Pitch)<5.0f) || fabs(Pitch+Init_Pitch<5.0f))
	{
		MOTOR_CAL();
		TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
		
		printf("\n Pitch=%f, InitVal=%f, motor4=%d, motor3=%d \n\r", Pitch, Init_Pitch, MOTOR4, MOTOR3);
		
//		for(uint16_t i=0; i<1000; i++)
//		{
//			__NOP;
//		}
//		Delay_ms(1000);                           //加速周期为1m
//		TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
		return;
	}
	
	TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
	 return;
}

/* 一个电机提供动力抵消能量损耗实现自由等幅摆 */
void pendulum_control_EAmplitudeV2(void)
{
////	static float angle=20;
////	Get_ChangRate(&Pitch, &Roll, &Pitch_Der, &Roll_Der);
//	if((fabs(Pitch-Init_Pitch)<5.0) || fabs(Pitch+Init_Pitch<5.0))  //该限制条件只允许PID用于初始方向
	if((fabs(Pitch-Init_Pitch)<5.0f) || fabs(Pitch+Init_Pitch<5.0f))
	{
		pendulum_conrol_motor_cal();
		TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
		
		printf("\n Pitch=%f, InitVal=%f, motor4=%d, motor3=%d \n\r", Pitch, Init_Pitch, MOTOR4, MOTOR3);
		
//		for(uint16_t i=0; i<1000; i++)
//		{
//			__NOP;
//		}
//		Delay_ms(1000);                           //加速周期为1m
//		TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
		return;
	}
	
	TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
	 return;
}

void pendulum_control_EPeriod(void)
{
	static float period;
//	static float t1,t2;
	static uint16_t motor=1300;
	uint8_t flag=0;  //用于第一次记录变化率符号与初始PITCH符号相反，当退出if语句需改为默认值
	if(fabs(Pitch-Init_Pitch)<fabs(Init_Pitch))  //判断是否摆到初始摆动方向
//	if(Pitch-Init_Pitch>=0)  //判断是否摆到初始摆动方向
	{
		Get_ChangRate(&Pitch, &Roll, &Pitch_Der, &Roll_Der);
		
//		MPU_Get_Gyroscope(&gx, &gy, &gz);
//		if(gx<0)
//		{
//			
//		}
		
		if((Pitch*Pitch_Der<0)&&(flag!=1))
		{
			flag=1;
			if(fabs(Pitch)>fabs(Init_Pitch))          //判断此时Pitch与初始值的差异已决定加速值加减
			{                                         //
				motor--;                                //
			}                                         //
			else if(fabs(Pitch)<fabs(Init_Pitch))     //
			{                                         //
				motor++;                                //
			}                                         //
			if(Init_Pitch<0)                          //判断加速电机编号
			{
				MOTOR3=motor;
				MOTOR4=1000;
			}
			else if(Init_Pitch>0)
			{
				MOTOR3=1000;
				MOTOR4=motor;
			}
			TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
			Delay_ms(1000);                           //加速周期为1m
			period = Get_Sampletime();
			printf("\n period=%f \n\r", period);
		}
	}
	flag=0;
	TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
}

#define N 11


//完成情况：通过确保摆动方向与初始方向同向来记录周期值,现在或许只能记录
//周期值，还未加入电机控制
void pendulum_control_EPeriod_V2(void)
{
//	 static short ax_temp=0;
	 static float period=0;
	 static uint8_t flag=0;      //用于记录第一次远离初始位置
	 MPU_Get_Accelerometer(&ax, &ay, &az);
	 MPU_Get_Gyroscope(&gx, &gy, &gz);
	 if((gx*Init_Pitch>0)&&(gx*Init_Pitch>0))     //确保摆向初始位置
	 {
		 flag=1;
//		 if(fabs(ax)>=ax_temp)
//		 {
//			 ax_temp=ax;
//		 }
//		 else if(ax<ax_temp)
//		 {
//			 period = Get_Sampletime();
//			 printf("\n period=%f \n\r", period);
//		 }
   }
	 else
	 {
		 if(flag==1)
		 {
			 period = Get_Sampletime();
			 printf("\n period=%f \n\r", period);
			 flag=0;
		 }
		 else
		 {
			 //Did nothing
		 }
	 }
}


/* 用于在自由摆模式下，单轴电机控制 */
void pendulum_conrol_motor_cal(void)
{
	static uint16_t Time_dt=0;
	
	Time_dt = Get_Angle_PIDtime();
	
	if(Pitch < 0)
	{
	  pid_pitch = PID_Calculate(&PID_Pitch_Struct_Minus, Pitch, -fabs(Motor_Angle_X), Time_dt);
	}
	else if(Pitch > 0)
	{
		pid_pitch = PID_Calculate(&PID_Pitch_Struct_Plus, Pitch, fabs(Motor_Angle_X), Time_dt);
	}
  
	
	MOTOR1 = 1000;
	MOTOR2 = 1000;
	MOTOR3 = (uint16_t)Limit_PWMOUT(1000+pid_pitch);	//X轴负向电机
	MOTOR4 = (uint16_t)Limit_PWMOUT(1000-pid_pitch);	//X轴正向电机
	
}

void pendulum_control_expectation_x(PID_Struct *PID_Pitch_Struct)
{
	
}


//先起摆，在自由摆,在TIM7中断中被调用，频率跟TIM7基频一样，注:自由摆周期约为1.67-1.70S
void pendulum_control_freefall(float pitch)
{
	//PID_Pitch_Rate.KP=21,Ti=0,Td=10;PID_Pitch={0,0,0};->真实数据2.1,0,0.1; 0,0,0;
	
	PID_Roll_Struct.Kp=0;
//	
	PID_Pitch_Struct.Kp=0;
//	
	PID_Pitch_Rate_Struct.Kp = 2.1;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.1;
//	
	PID_Roll_Rate_Struct.Kp = 1.0;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.03;
	
//	static uint8_t Freefall_Flag=0;     //用于记录是否第一次进入此函数，若为第一次，则重新初始化定时器TIM8,TIM5
	if(Freefall_Flag == 0)
	{
		HAL_TIM_Base_Stop(&htim8);
//		HAL_TIM_Base_Stop(&htim5);
//		htim5.Init.Period = 1800;
		htim8.Init.Period = 1800;
//		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_Base_Start(&htim8);
//		HAL_TIM_Base_Start(&htim5);
		Freefall_Flag = 1; //初始化后置一，表示已经初始化，但在模式切换时，应注意flag清0
	}

	
	Motor_Angle_X=4*((float)(TIM8->CNT/180) - 4.5f);

}

//可控摆幅等幅摆,在TIM7中断中调用
void pendulum_control_eAmplitude_swing(float amplitude)
{
	
//	
	PID_Pitch_Struct.Kp=0;
	
	PID_Pitch_Rate_Struct.Kp = 1.3;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.03;
//	
	PID_Roll_Struct.Kp=0;
	PID_Roll_Rate_Struct.Kp = 1.1;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.03;
	
//	static uint8_t eAmplitude_Flag=0;     //用于记录是否第一次进入此函数，若为第一次，则重新初始化定时器TIM8,TIM5
	if(eAmplitude_Flag == 0)
	{
		HAL_TIM_Base_Stop(&htim8);
//		HAL_TIM_Base_Stop(&htim5);
//		htim5.Init.Period = 1800;
		htim8.Init.Period = 1800;
//		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_Base_Start(&htim8);
//		HAL_TIM_Base_Start(&htim5);
		eAmplitude_Flag = 1; //初始化后置一，表示已经初始化，但在模式切换时，应注意flag清0
	}
	
	static float amp_last=0;
	static float temp;
	if(amplitude!=amp_last)
	{
		temp = atan(amplitude/88)*11.46;   //更新机制，加快运算速度
		amp_last=amplitude;
//		Motor_Angle_X= atan(amplitude/88)*11.46*((float)(TIM8->CNT/180) - 4.5f); //atan(amplitude/88)*57.3f/5*((float)(TIM8->CNT/180) - 4.5f);
	}
	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
	Motor_Angle_Y=0;
//	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
//	Motor_Angle_Y=temp*((float)(TIM8->CNT/180) - 4.5f);
	
}

//任意方向指定摆幅摆动
void pendulum_control_nDirection_swing(float direction, float amplitude)
{
//	static float temp_x=0;
//	static float temp_y=0;
	
	PID_Roll_Struct.Kp=1.2;
	
	PID_Pitch_Struct.Kp=0.6;
	
	PID_Pitch_Rate_Struct.Kp = 0.9;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.04;
	
	PID_Roll_Rate_Struct.Kp = 0.9;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.03;

	
	static float direc_last=0;
	static float amp_last=0;
//	static uint8_t NDirection_Flag=0;     //用于记录是否第一次进入此函数，若为第一次，则重新初始化定时器TIM8,TIM5
	if(NDirection_Flag == 0)
	{
		HAL_TIM_Base_Stop(&htim8);
		HAL_TIM_Base_Stop(&htim5);
		htim5.Init.Period = 1800;
		htim8.Init.Period = 1800;
		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_Base_Start(&htim8);
		HAL_TIM_Base_Start(&htim5);
		NDirection_Flag = 1; //初始化后置一，表示已经初始化，但在模式切换时，应注意flag清0
	}
	static float Motor_Angle_X_Max=15;
	static float Motor_Angle_Y_Max=15;
	
	if((direction != direc_last) || (amplitude != amp_last))
	{
		Motor_Angle_X_Max=(atan(amplitude*cos(direction/180*3.14159f)/88.0f))*11.46f;
		Motor_Angle_Y_Max=(atan(amplitude*sin(direction/180*3.14159f)/88.0f))*11.46f;
		
//		printf("\n X=%f \n\r", Motor_Angle_X_Max);
		
		direc_last=direction;
		amp_last=amplitude;
		
//		TIM8->ARR = (uint16_t)Motor_Angle_X_Max;
//		TIM5->ARR = (uint16_t)Motor_Angle_Y_Max;
		
	}
	Motor_Angle_X=Motor_Angle_X_Max*((float)(TIM8->CNT/180) - 4.5f);
	Motor_Angle_Y=Motor_Angle_Y_Max*((float)(TIM5->CNT/180) - 4.5f);
	
//	printf("\n Y=%f \n\r", Motor_Angle_Y_Max);
	
}

void pendulum_control_Still(void)
{
	//	PID_Roll_Struct.Kp=0.8;
	PID_Roll_Struct.Kp=0;
	
//	PID_Pitch_Struct.Kp=0;

//	PID_Pitch_Rate_Struct.Kp = 0;
//	PID_Pitch_Rate_Struct.Ti = 0;
//	PID_Pitch_Rate_Struct.Td = 0;
//	
//	PID_Roll_Rate_Struct.Kp = 0;
//	PID_Roll_Rate_Struct.Ti = 0;
//	PID_Roll_Rate_Struct.Td = 0;
	
	PID_Pitch_Struct.Kp=1.8;

	PID_Pitch_Rate_Struct.Kp = 32.6;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.04;
	
	PID_Roll_Rate_Struct.Kp = 28.5;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.02;
	
	Motor_Angle_X=0;
	
//	printf("\n m=%f \n\t", Motor_Angle_X);
	
	Motor_Angle_Y=0;
	


}

//可变半径圆摆动,在TIM7中断中调用
void pendulum_control_circle(float radius)
{
	PID_Roll_Struct.Kp=0;
	
	PID_Pitch_Struct.Kp=0;
	
	PID_Pitch_Rate_Struct.Kp = 1.1;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.05;
	
	PID_Roll_Rate_Struct.Kp = 1.0;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.03;
	
//	static uint8_t Circle_Flag=0;     //用于记录是否第一次进入此函数，若为第一次，则重新初始化定时器TIM8,TIM5
	if(Circle_Flag == 0)
	{
		HAL_TIM_Base_Stop(&htim8);
		HAL_TIM_Base_Stop(&htim5);
		htim5.Init.Period = 1800;
		htim8.Init.Period = 1800;
		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_Base_Start(&htim8);
		HAL_TIM_Base_Start(&htim5);
		Circle_Flag = 1; //初始化后置一，表示已经初始化，但在模式切换时，应注意flag清0
	}
	
	static float amp_last=0;
	static float temp;
	if(radius!=amp_last)
	{
		temp = atan(radius/88)*11.46;   //更新机制，加快运算速度
		amp_last=radius;
//		Motor_Angle_X= atan(amplitude/88)*11.46*((float)(TIM8->CNT/180) - 4.5f); //atan(amplitude/88)*57.3f/5*((float)(TIM8->CNT/180) - 4.5f);
	}
	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
	if((HAL_TIM_Base_GetState(&htim5)==HAL_TIM_STATE_READY) && (TIM8->CNT==1800))
	{
		HAL_TIM_Base_Start(&htim5);
		TIM5->CNT=899;
	}
//	Motor_Angle_X=temp*((float)((TIM8->CNT)/180) - 4.5f);
	Motor_Angle_Y=temp*((float)((TIM5->CNT)/180) - 4.5f);
}

//8字摆动,在TIM7中断中调用
void pendulum_control_eight(float radius)
{
	PID_Roll_Struct.Kp=0;
	
	PID_Pitch_Struct.Kp=1.8;
	
	PID_Pitch_Rate_Struct.Kp = 1.1;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0.05;
	
	PID_Roll_Rate_Struct.Kp = 1.0;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0.03;
	
//	static uint8_t Eight_Flag=0;     //用于记录是否第一次进入此函数，若为第一次，则重新初始化定时器TIM8,TIM5
	if(Eight_Flag == 0)
	{
		HAL_TIM_Base_Stop(&htim8);
		HAL_TIM_Base_Stop(&htim5);
		htim5.Init.Period = 1800;
		htim8.Init.Period = 1800;
		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_Base_Start(&htim8);
		HAL_TIM_Base_Start(&htim5);
		Eight_Flag = 1; //初始化后置一，表示已经初始化，但在模式切换时，应注意flag清0
	}
	
	static float amp_last=0;
	static float temp;
	if(radius!=amp_last)
	{
		temp = atan(radius/88)*11.46;   //更新机制，加快运算速度
		amp_last=radius;
//		Motor_Angle_X= atan(amplitude/88)*11.46*((float)(TIM8->CNT/180) - 4.5f); //atan(amplitude/88)*57.3f/5*((float)(TIM8->CNT/180) - 4.5f);
	}
	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
	if((HAL_TIM_Base_GetState(&htim5)==HAL_TIM_STATE_READY) && (TIM8->CNT==0))
	{
		htim5.Init.Period = 900;
		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Start(&htim5);
	}
	Motor_Angle_Y=temp*((float)((TIM5->CNT)/90) - 4.5f);
}
