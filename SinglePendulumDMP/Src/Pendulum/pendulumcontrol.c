#include "pendulumcontrol.h"
#include "PID_Control.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

//����PID_Control

extern PID_Struct PID_Pitch_Struct;				 //����Pitch��PID�ṹ��,��Y��
extern float Time_dt;
extern float pid_pitch;               //��Z���

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
extern float Pitch_Der;                   //Pitch�仯��
extern float Roll_Der;                    //Roll�仯��

extern float Motor_Angle_X;					   //�������
extern float Motor_Angle_Y;					   //��������


extern PID_Struct PID_Pitch_Rate_Struct;     //����Pitch�仯�ʵ�PID�ṹ��
extern PID_Struct PID_Roll_Rate_Struct;      //����Roll�仯�ʵ�PID�ṹ��

extern PID_Struct PID_Pitch_Struct;				 //����Pitch��PID�ṹ��,��Y��
extern PID_Struct PID_Roll_Struct;			 //����Roll��PID�ṹ��,��X��

//

/* ������main.c */
extern uint8_t Hz57;
extern uint8_t Hz167;
/* ------------ */

PID_Struct PID_Pitch_Struct_Plus;
PID_Struct PID_Pitch_Struct_Minus;

S_FLOAT_XYZ DIF_ACC;		//ʵ��ȥ�������ļ��ٶ�
S_FLOAT_XYZ EXP_ANGLE;	//�����Ƕ�
S_FLOAT_XYZ DIF_ANGLE;	//ʵ�����������ĽǶ�	

/* ��־ */
uint8_t Circle_Flag=0;
uint8_t Eight_Flag=0;
uint8_t NDirection_Flag=0;
uint8_t eAmplitude_Flag=0;
uint8_t Freefall_Flag=0;

uint8_t Mode = 0;
/*-----------*/

void pendulum_conrol_motor_cal(void);

float Init_Pitch=0;               //Init_Pitch���ڵȰڷ�����ʱ�ڽǵĲο�������������ʱ��¼

SwingDirection Pitch_Direction=PlusDirection;
SwingDirection Roll_Direction=PlusDirection;

float Amplitude=15;   //�ɿذڷ�ģʽ�°ڷ�����,Ĭ��ֵΪ15

float Radius=20;      //�ɿ�Բ��Ĭ�ϰ뾶

float Radius_Eight=20;   //���ְڷ���

float nDirection=45;   //���ⷽ��ڽǶ�

float nAmplitude=20;   //���ⷽ��ڷ���



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


/* һ������ṩ���������������ʵ�����ɵȷ��� */
void pendulum_control_EAmplitude(void)
{
////	static float angle=20;
////	Get_ChangRate(&Pitch, &Roll, &Pitch_Der, &Roll_Der);
//	if((fabs(Pitch-Init_Pitch)<5.0) || fabs(Pitch+Init_Pitch<5.0))  //����������ֻ����PID���ڳ�ʼ����
	if((fabs(Pitch-Init_Pitch)<5.0f) || fabs(Pitch+Init_Pitch<5.0f))
	{
		MOTOR_CAL();
		TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
		
		printf("\n Pitch=%f, InitVal=%f, motor4=%d, motor3=%d \n\r", Pitch, Init_Pitch, MOTOR4, MOTOR3);
		
//		for(uint16_t i=0; i<1000; i++)
//		{
//			__NOP;
//		}
//		Delay_ms(1000);                           //��������Ϊ1m
//		TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
		return;
	}
	
	TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
	 return;
}

/* һ������ṩ���������������ʵ�����ɵȷ��� */
void pendulum_control_EAmplitudeV2(void)
{
////	static float angle=20;
////	Get_ChangRate(&Pitch, &Roll, &Pitch_Der, &Roll_Der);
//	if((fabs(Pitch-Init_Pitch)<5.0) || fabs(Pitch+Init_Pitch<5.0))  //����������ֻ����PID���ڳ�ʼ����
	if((fabs(Pitch-Init_Pitch)<5.0f) || fabs(Pitch+Init_Pitch<5.0f))
	{
		pendulum_conrol_motor_cal();
		TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
		
		printf("\n Pitch=%f, InitVal=%f, motor4=%d, motor3=%d \n\r", Pitch, Init_Pitch, MOTOR4, MOTOR3);
		
//		for(uint16_t i=0; i<1000; i++)
//		{
//			__NOP;
//		}
//		Delay_ms(1000);                           //��������Ϊ1m
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
	uint8_t flag=0;  //���ڵ�һ�μ�¼�仯�ʷ������ʼPITCH�����෴�����˳�if������ΪĬ��ֵ
	if(fabs(Pitch-Init_Pitch)<fabs(Init_Pitch))  //�ж��Ƿ�ڵ���ʼ�ڶ�����
//	if(Pitch-Init_Pitch>=0)  //�ж��Ƿ�ڵ���ʼ�ڶ�����
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
			if(fabs(Pitch)>fabs(Init_Pitch))          //�жϴ�ʱPitch���ʼֵ�Ĳ����Ѿ�������ֵ�Ӽ�
			{                                         //
				motor--;                                //
			}                                         //
			else if(fabs(Pitch)<fabs(Init_Pitch))     //
			{                                         //
				motor++;                                //
			}                                         //
			if(Init_Pitch<0)                          //�жϼ��ٵ�����
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
			Delay_ms(1000);                           //��������Ϊ1m
			period = Get_Sampletime();
			printf("\n period=%f \n\r", period);
		}
	}
	flag=0;
	TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
}

#define N 11


//��������ͨ��ȷ���ڶ��������ʼ����ͬ������¼����ֵ,���ڻ���ֻ�ܼ�¼
//����ֵ����δ����������
void pendulum_control_EPeriod_V2(void)
{
//	 static short ax_temp=0;
	 static float period=0;
	 static uint8_t flag=0;      //���ڼ�¼��һ��Զ���ʼλ��
	 MPU_Get_Accelerometer(&ax, &ay, &az);
	 MPU_Get_Gyroscope(&gx, &gy, &gz);
	 if((gx*Init_Pitch>0)&&(gx*Init_Pitch>0))     //ȷ�������ʼλ��
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


/* ���������ɰ�ģʽ�£����������� */
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
	MOTOR3 = (uint16_t)Limit_PWMOUT(1000+pid_pitch);	//X�Ḻ����
	MOTOR4 = (uint16_t)Limit_PWMOUT(1000-pid_pitch);	//X��������
	
}

void pendulum_control_expectation_x(PID_Struct *PID_Pitch_Struct)
{
	
}


//����ڣ������ɰ�,��TIM7�ж��б����ã�Ƶ�ʸ�TIM7��Ƶһ����ע:���ɰ�����ԼΪ1.67-1.70S
void pendulum_control_freefall(float pitch)
{
	//PID_Pitch_Rate.KP=21,Ti=0,Td=10;PID_Pitch={0,0,0};->��ʵ����2.1,0,0.1; 0,0,0;
	
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
	
//	static uint8_t Freefall_Flag=0;     //���ڼ�¼�Ƿ��һ�ν���˺�������Ϊ��һ�Σ������³�ʼ����ʱ��TIM8,TIM5
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
		Freefall_Flag = 1; //��ʼ������һ����ʾ�Ѿ���ʼ��������ģʽ�л�ʱ��Ӧע��flag��0
	}

	
	Motor_Angle_X=4*((float)(TIM8->CNT/180) - 4.5f);

}

//�ɿذڷ��ȷ���,��TIM7�ж��е���
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
	
//	static uint8_t eAmplitude_Flag=0;     //���ڼ�¼�Ƿ��һ�ν���˺�������Ϊ��һ�Σ������³�ʼ����ʱ��TIM8,TIM5
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
		eAmplitude_Flag = 1; //��ʼ������һ����ʾ�Ѿ���ʼ��������ģʽ�л�ʱ��Ӧע��flag��0
	}
	
	static float amp_last=0;
	static float temp;
	if(amplitude!=amp_last)
	{
		temp = atan(amplitude/88)*11.46;   //���»��ƣ��ӿ������ٶ�
		amp_last=amplitude;
//		Motor_Angle_X= atan(amplitude/88)*11.46*((float)(TIM8->CNT/180) - 4.5f); //atan(amplitude/88)*57.3f/5*((float)(TIM8->CNT/180) - 4.5f);
	}
	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
	Motor_Angle_Y=0;
//	Motor_Angle_X=temp*((float)(TIM8->CNT/180) - 4.5f);
//	Motor_Angle_Y=temp*((float)(TIM8->CNT/180) - 4.5f);
	
}

//���ⷽ��ָ���ڷ��ڶ�
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
//	static uint8_t NDirection_Flag=0;     //���ڼ�¼�Ƿ��һ�ν���˺�������Ϊ��һ�Σ������³�ʼ����ʱ��TIM8,TIM5
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
		NDirection_Flag = 1; //��ʼ������һ����ʾ�Ѿ���ʼ��������ģʽ�л�ʱ��Ӧע��flag��0
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

//�ɱ�뾶Բ�ڶ�,��TIM7�ж��е���
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
	
//	static uint8_t Circle_Flag=0;     //���ڼ�¼�Ƿ��һ�ν���˺�������Ϊ��һ�Σ������³�ʼ����ʱ��TIM8,TIM5
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
		Circle_Flag = 1; //��ʼ������һ����ʾ�Ѿ���ʼ��������ģʽ�л�ʱ��Ӧע��flag��0
	}
	
	static float amp_last=0;
	static float temp;
	if(radius!=amp_last)
	{
		temp = atan(radius/88)*11.46;   //���»��ƣ��ӿ������ٶ�
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

//8�ְڶ�,��TIM7�ж��е���
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
	
//	static uint8_t Eight_Flag=0;     //���ڼ�¼�Ƿ��һ�ν���˺�������Ϊ��һ�Σ������³�ʼ����ʱ��TIM8,TIM5
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
		Eight_Flag = 1; //��ʼ������һ����ʾ�Ѿ���ʼ��������ģʽ�л�ʱ��Ӧע��flag��0
	}
	
	static float amp_last=0;
	static float temp;
	if(radius!=amp_last)
	{
		temp = atan(radius/88)*11.46;   //���»��ƣ��ӿ������ٶ�
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
