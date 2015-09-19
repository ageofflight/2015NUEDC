#include "data_transfer.h"
#include "MPU6050.h"
#include "PID_Control.h"
#include "coordinatetrans.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define DATA_TRANSFER_USE_USART

uint8_t Data_Check,Send_Status, Send_Senser, Send_PID;
uint8_t data_to_send[120];

extern FlagStatus Receive;

extern UART_HandleTypeDef huart2;
extern float Pitch, Roll, Angle_Z;
//extern T_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//
//extern T_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;
extern PID_Struct PID_Roll_Struct,  PID_Z_Struct, PID_Pitch_Struct;

extern PID_Struct PID_Pitch_Rate_Struct;     //定义Pitch变化率的PID结构体
extern PID_Struct PID_Roll_Rate_Struct;      //定义Roll变化率的PID结构体

extern float q0, q1, q2, q3;

extern float gx;
extern float gy;
extern float gz;

extern AxisAngle AA;

extern float Motor_Angle_X;					   //横滚期望
extern float Motor_Angle_Y;					   //俯仰期望

/* 引用自uranus.c */
extern int16_t Gyro_X;
extern int16_t Gyro_Y;
/* -------------- */

/* 引用自nine_axis_module.c */
extern short T_X,T_Y,T_Z;
extern float init_gx, init_gy, init_gz;
//

extern uint16_t MOTOR1, MOTOR2, MOTOR3, MOTOR4;

/* 引用自PENDULUMCONTROL.C */
extern float Amplitude;   //可控摆幅模式下摆幅参数
extern float Radius;      //可控圆摆半径
extern float Radius_Eight;   //八字摆幅度
extern float nDirection;   //任意方向摆角度

extern float nAmplitude;   //任意方向摆幅度

/* ----------------------- */



void Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
//	__IO int16_t rc_value_temp;
	uint8_t sum = 0;
	
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X10)								//PID1
	{
//			PID_Pitch_Rate_Struct.Kp = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;  //原KP为/100，为降低灵敏度将KP改为/1
//			PID_Pitch_Rate_Struct.Ti = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;  //原为ki/1000
//			PID_Pitch_Rate_Struct.Td = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_Pitch_Struct.Kp = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
//			PID_Pitch_Struct.Ti = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_Pitch_Struct.Td = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		
//		  PID_Roll_Rate_Struct.Kp = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;  //原KP为/100，为降低灵敏度将KP改为/1
//			PID_Roll_Rate_Struct.Ti = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;  //原为ki/1000
//			PID_Roll_Rate_Struct.Td = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_Roll_Struct.Kp = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
//			PID_Roll_Struct.Ti = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_Roll_Struct.Td = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		
		  PID_Pitch_Rate_Struct.Kp = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;  //原KP为/100，为降低灵敏度将KP改为/1
			PID_Pitch_Struct.Kp = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10;  //原为ki/1000
			PID_Pitch_Rate_Struct.Td = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_Roll_Rate_Struct.Kp = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
			PID_Roll_Struct.Kp = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/10;
			PID_Roll_Rate_Struct.Td = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		
			nDirection = (float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10;
			nAmplitude = (float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10;
			Amplitude = (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10;
//      Radius_Eight = (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10;
			Data_Send_Check(sum);
	}
	
	if(*(data_buf+2)==0X16)								//Expectation,原匿名四轴上位机为接收Roll,Pitch偏移
	{
			PID_Roll_Struct.expectation = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			PID_Pitch_Struct.expectation = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
	}
	
	if(*(data_buf+2)==0X11)								//PID2
	{
		
//			PID_ALT.P = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_ALT.I = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_ALT.D = (float)((__IO int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_POS.P = (float)((__IO int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_POS.I = (float)((__IO int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_POS.D = (float)((__IO int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_1.P = (float)((__IO int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_1.I = (float)((__IO int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_1.D = (float)((__IO int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
//			PID_PID_2.P = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_2.I = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_2.D = (float)((__IO int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_3.P = (float)((__IO int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_3.I = (float)((__IO int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_3.D = (float)((__IO int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_4.P = (float)((__IO int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_4.I = (float)((__IO int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_4.D = (float)((__IO int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
//			PID_PID_5.P = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_5.I = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_5.D = (float)((__IO int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_6.P = (float)((__IO int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_6.I = (float)((__IO int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_6.D = (float)((__IO int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_7.P = (float)((__IO int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_7.I = (float)((__IO int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_7.D = (float)((__IO int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
//			PID_PID_8.P = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_8.I = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_8.D = (float)((__IO int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_9.P = (float)((__IO int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_9.I = (float)((__IO int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_9.D = (float)((__IO int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_10.P = (float)((__IO int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_10.I = (float)((__IO int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_10.D = (float)((__IO int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
//			PID_PID_11.P = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_11.I = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_11.D = (float)((__IO int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_12.P = (float)((__IO int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_12.I = (float)((__IO int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_12.D = (float)((__IO int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			Data_Send_Check(sum);
//			EE_SAVE_PID();
	}
//	if(*(data_buf+2)==0X16)								//OFFSET
//	{
//			AngleOffset_Rol = (float)((__IO int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//			AngleOffset_Pit = (float)((__IO int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//	}
	
	Receive = RESET;  //接收完成

}

void Send_Data(void)
{
	if(Send_Status)
	{
		Send_Status = 0;
		Data_Send_Status();
	}
	
	else if(Send_Senser)
	{
		Send_Senser = 0;
		Data_Send_Senser();
	}
	else if(Send_PID)
	{
		Send_PID = 0;
		Data_Send_PID();
	}
}
void Data_Send_Status(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	__IO int16_t _temp;
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)((TIM8->CNT/180)*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(TIM5->CNT*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(TIM8->CNT)*100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);

	
//	__IO int32_t _temp2 = (int)(Pressure*100);
//	data_to_send[_cnt++]=BYTE3(_temp2);
//	data_to_send[_cnt++]=BYTE2(_temp2);
//	data_to_send[_cnt++]=BYTE1(_temp2);
//	data_to_send[_cnt++]=BYTE0(_temp2);
//		
//	if(ARMED==0)				data_to_send[_cnt++]=0xA0;	//锁定
//	else if(ARMED==1)		data_to_send[_cnt++]=0xA1;
//	
  data_to_send[_cnt++]=0xA1;
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
  HAL_UART_Transmit_DMA(&huart2, data_to_send, _cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
}
void Data_Send_Senser(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	__IO int16_t _temp;
	_temp = (int)(gy);  //gy代表绕Y轴x轴的输出，表示x轴角速度
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(gx);  //gx代表绕X轴y轴的输出，表示y轴角速度
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
//	_temp = (int)(init_gx);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(init_gy);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
	
//	_temp = (int)(q2 * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(q3 * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
	
//	__IO int16_t _temp;
//	_temp = (int)(AA.angle * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(AA.ax * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(AA.ay * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(AA.az * 100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (int)(Motor_Angle_X);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Motor_Angle_Y);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE1(MOTOR4);          //X轴正向电机
	data_to_send[_cnt++]=BYTE0(MOTOR4);          
	data_to_send[_cnt++]=BYTE1(MOTOR3);          //X轴负向电机
	data_to_send[_cnt++]=BYTE0(MOTOR3);
	data_to_send[_cnt++]=BYTE1(MOTOR2);          //Y轴正向电机
	data_to_send[_cnt++]=BYTE0(MOTOR2);
	data_to_send[_cnt++]=BYTE1(MOTOR1);          //Y轴负向电机
	data_to_send[_cnt++]=BYTE0(MOTOR1);
//	data_to_send[_cnt++]=BYTE1(MPU6050_GYRO_LAST.Y);
//	data_to_send[_cnt++]=BYTE0(MPU6050_GYRO_LAST.Y);
//	data_to_send[_cnt++]=BYTE1(MPU6050_GYRO_LAST.Z);
//	data_to_send[_cnt++]=BYTE0(MPU6050_GYRO_LAST.Z);
//	data_to_send[_cnt++]=BYTE1(AK8975_Data[0]);
//	data_to_send[_cnt++]=BYTE0(AK8975_Data[0]);
//	data_to_send[_cnt++]=BYTE1(AK8975_Data[1]);
//	data_to_send[_cnt++]=BYTE0(AK8975_Data[1]);
//	data_to_send[_cnt++]=BYTE1(AK8975_Data[2]);
//	data_to_send[_cnt++]=BYTE0(AK8975_Data[2]);
  data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;

	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
HAL_UART_Transmit_DMA(&huart2, data_to_send, _cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
}

/* -------------发送PWM值---------------------------- */
void Data_Send_MotoPWM(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
//	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(MOTOR4);          //X轴正向电机
	data_to_send[_cnt++]=BYTE0(MOTOR4);          
	data_to_send[_cnt++]=BYTE1(MOTOR3);          //X轴负向电机
	data_to_send[_cnt++]=BYTE0(MOTOR3);
	data_to_send[_cnt++]=BYTE1(MOTOR2);          //Y轴正向电机
	data_to_send[_cnt++]=BYTE0(MOTOR2);
	data_to_send[_cnt++]=BYTE1(MOTOR1);          //Y轴负向电机
	data_to_send[_cnt++]=BYTE0(MOTOR1);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_5);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_5);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_6);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_6);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_7);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_7);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_8);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_8);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit_DMA(&huart2, data_to_send, _cnt);;
}


void Data_Send_PID(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xB0;
	data_to_send[_cnt++]=0;
	
	__IO int16_t _temp;
	_temp = PID_Pitch_Struct.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Pitch_Struct.Ti * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Pitch_Struct.Td * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Roll_Struct.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Roll_Struct.Ti * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Roll_Struct.Td * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Z_Struct.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Z_Struct.Ti * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_Z_Struct.Td * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
HAL_UART_Transmit_DMA(&huart2, data_to_send, _cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
}

/* 发送校验数据 */
void Data_Send_Check(uint16_t check)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

//	Uart1_Put_Buf(data_to_send,8);
	HAL_UART_Transmit_DMA(&huart2, data_to_send, 8);
}


