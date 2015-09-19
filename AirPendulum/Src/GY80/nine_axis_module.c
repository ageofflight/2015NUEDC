/**
  ******************************************************************************
  * @file    nine_axis_mudule.c
  * @date    07/04/2015 
  * @brief   Nine Aixs Module configuration function.
  ******************************************************************************
   */
   
/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"
#include "nine_axis_module.h"
#include "myiic.h"
#include <stdio.h>

//extern I2C_HandleTypeDef hi2c3;

//Register definition of BMP085
short   ac1;
short   ac2;
short 	ac3;
uint8_t ac4;
uint8_t ac5;
uint8_t ac6;
short   b1;
short   b2;
short   mb;
short   mc;
short   md;

//The purpose value of BMP085
long    temperature;
long    pressure;

uint8_t BUF[8];
//char  test=0;
int   M_X=1;
int   M_Y=1;
int   M_Z=1;
int A_X,A_Y,A_Z;
short T_X,T_Y,T_Z;

short data_xyz[3];
extern float Roll;
extern float Pitch;
extern float Yaw;
float Q,T,K;

float init_gx, init_gy, init_gz;
float Gyro_Xout_Offset = 44.95, Gyro_Yout_Offset = 88.59, Gyro_Zout_Offset = 0;

/* 引用自PID_Control.c */
extern float gx;
extern float gy;
/* ------------------- */

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
		
//   __IO uint8_t i=30; //这里可以优化速度	，经测试最低到5还能写入
//  __IO uint8_t i=100;
//	while(i) 
//   { 
//     i--; 
//   }  
	delay_us(40);
}

void delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/**
	*****************************************************************************
	* Function Name  : I2C_Start
	* Description    : Master Start Simulation IIC Communication
	* Input          : None
	* Output         : None
	* Return         : Wheather	 Start
	****************************************************************************** 
	*/
bool I2C_Start(void)
{
	GY80_SDA_H;
	GY80_SCL_H;
	I2C_delay();
	if(!GY80_SDA_read)
	{
		printf("\n Bus busy! \n\r");
		return FALSE;	//SDA线为低电平则总线忙,退出
	}
	GY80_SDA_L;
	I2C_delay();
	if(GY80_SDA_read) 
	{
		printf("\n Bus erro \n\r");
		return FALSE;	//SDA线为高电平则总线出错,退出
	}
	GY80_SDA_L;
	I2C_delay();
	return TRUE;
}
/**
	*****************************************************************************
	* Function Name  : I2C_Stop
	* Description    : Master Stop Simulation IIC Communication
	* Input          : None
	* Output         : None
	* Return         : None
	****************************************************************************** 
	*/
void I2C_Stop(void)
{
	GY80_SCL_L;
	I2C_delay();
	GY80_SDA_L;
	I2C_delay();
	GY80_SCL_H;
	I2C_delay();
	GY80_SDA_H;
	I2C_delay();
} 
/**
	*****************************************************************************
	* Function Name  : I2C_Ack
	* Description    : Master Send Acknowledge Single
	* Input          : None
	* Output         : None
	* Return         : None
	****************************************************************************** 
	*/
void I2C_Ack(void)
{	
	GY80_SCL_L;
	I2C_delay();
	GY80_SDA_L;
	I2C_delay();
	GY80_SCL_H;
	I2C_delay();
	GY80_SCL_L;
	I2C_delay();
}   
/**
	*****************************************************************************
	* Function Name  : I2C_NoAck
	* Description    : Master Send No Acknowledge Single
	* Input          : None
	* Output         : None
	* Return         : None
	****************************************************************************** 
	*/
void I2C_NoAck(void)
{	
	GY80_SCL_L;
	I2C_delay();
	GY80_SDA_H;
	I2C_delay();
	GY80_SCL_H;
	I2C_delay();
	GY80_SCL_L;
	I2C_delay();
} 
/**
	*****************************************************************************
	* Function Name  : I2C_WaitAck
	* Description    : Master Reserive Slave Acknowledge Single
	* Input          : None
	* Output         : None
	* Return         : Wheather	 Reserive Slave Acknowledge Single
	****************************************************************************** 
	*/
bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	GY80_SCL_L;
	I2C_delay();
	GY80_SDA_H;			
	I2C_delay();
	GY80_SCL_H;
	I2C_delay();
	if(GY80_SDA_read)
	{
      GY80_SCL_L;
	  I2C_delay();
      return FALSE;
	}
	GY80_SCL_L;
	I2C_delay();
	return TRUE;
}
/**
	*****************************************************************************
	* Function Name  : I2C_SendByte
	* Description    : Master Send a Byte to Slave
	* Input          : Will Send Date
	* Output         : None
	* Return         : None
	****************************************************************************** 
	*/
void I2C_SendByte(uint8_t SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        GY80_SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        GY80_SDA_H;  
      else 
        GY80_SDA_L;   
        SendByte<<=1;
        I2C_delay();
		GY80_SCL_H;
        I2C_delay();
    }
    GY80_SCL_L;
}  
/**
	*****************************************************************************
	* Function Name  : I2C_RadeByte
	* Description    : Master Reserive a Byte From Slave
	* Input          : None
	* Output         : None
	* Return         : Date From Slave 
	****************************************************************************** 
	*/
unsigned char I2C_ReadByte(void)  //数据从高位到低位//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    GY80_SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      GY80_SCL_L;
      I2C_delay();
	  GY80_SCL_H;
      I2C_delay();	
      if(GY80_SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    GY80_SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())
		{
			printf("\n I2C did not open successfully \n\r");
			return FALSE;
		}
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck())
		{
			I2C_Stop(); 
			printf("\n No ack \n\r");
			return FALSE;
		}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
//    delay5ms();
		delay_us(100);
    return TRUE;
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{  
	unsigned char REG_data;     	
	if(!I2C_Start())
	{
		return FALSE;
	}
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck())
		{ 
			I2C_Stop();
//			test=1; 
			printf("\nADXL345 did not response\n\r");
			return FALSE;
		}
    I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_ReadByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}						      


/*
********************************************************************************
** 函数名称 ： Delay(vu32 nCount)
** 函数功能 ： 延时函数
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
 void Delay(uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/*
********************************************************************************
** 函数名称 ： void Delayms(vu32 m)
** 函数功能 ： 长延时函数	 m=1,延时1ms
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
 void Delayms(uint32_t m)
{
  uint32_t i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

///*
//********************************************************************************
//** 函数名称 ： WWDG_IRQHandler(void)
//** 函数功能 ： 窗口提前唤醒中断
//** 输    入	： 无
//** 输    出	： 无
//** 返    回	： 无
//********************************************************************************
//*/ 

//void WWDG_IRQHandler(void)
//{
//  /* Update WWDG counter */
//  WWDG_SetCounter(0x50);
//	
//  /* Clear EWI flag */
//  WWDG_ClearFlag();
//  
//}
// //************************************************
//void  USART1_SendData(uchar SendData)
//{
//USART_SendData(USART1, SendData);
//Delayms(1);
//}
//	

//********************************************************************
long bmp085ReadTemp(void)
{   short  temp_ut;
	Single_Write(BMP085_Addr,0xF4,0x2E);
	Delayms(5);	// max time is 4.5ms
	temp_ut = Single_Read(BMP085_Addr,0xF6);
	temp_ut = (temp_ut<<8)| Single_Read(BMP085_Addr,0xF7);		
	return (long) temp_ut ;
}
//*************************************************************

long bmp085ReadPressure(void)
{
	long pressure = 0;
	Single_Write(BMP085_Addr,0xF4,0x34);
	Delayms(5);	// max time is 4.5ms
	pressure = Single_Read(BMP085_Addr,0xF6);
	pressure = (pressure<<8)| Single_Read(BMP085_Addr,0xF7);		
	pressure &= 0x0000FFFF;	
	return pressure;	
}

 //******************
void Send_ADXL345_data(int dis_data)
{ 
	float temp ;
	if(dis_data>0x7fff)
		dis_data-=0xffff;
	temp=(float)dis_data*3.9f;  //计算数据和显示,查考ADXL345快速入门第4页
	printf("%f",temp);
}


void Send_HMC5883L()
{
	printf("HMC5883L:%f\n\r",Yaw);
}
//*************************************************
void  Send_BMP085()
{
	printf("BMP085:T=%ld C  P=%ld pa\n\r", temperature, pressure);

}
//*****************************************************

void  Send_ADXL345()
{
	printf("ADXL345:X=");
  Send_ADXL345_data(A_X);
	
	printf(" Y=");
  Send_ADXL345_data(A_Y);
	
	printf(" Z=");
  Send_ADXL345_data(A_Z);

	adxl345_angle();
		 
  printf("\n\r");
}
 //*****************************************************
void Send_L3G4200D()
{
//	float temp;
//	temp=(float)T_X*0.07f;  //计算数据和显示,查考ADXL345快速入门第4页
	printf("L3G4200D:X=%f Y=%f Z=%f\r\n",(float)T_X*0.07f,(float)T_Y*0.07f,(float)T_Z*0.07f);
}

//***************************
void  Init_ADXL345(void)
{
   Single_Write(ADXL345_Addr,0x31,0x0B);   //测量范围,正负16g，13位模式
  // Single_Write(ADXL345_Addr,0x2C,0x0e);   //速率设定为100hz 参考pdf13页
   Single_Write(ADXL345_Addr,0x2D,0x08);   //选择电源模式   参考pdf24页
   Single_Write(ADXL345_Addr,0x2E,0x80);   //使能 DATA_READY 中断
  // Single_Write(ADXL345_Addr,0x1E,0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
  // Single_Write(ADXL345_Addr,0x1F,0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
  // Single_Write(ADXL345_Addr,0x20,0x05);   //Z 偏移量 根据测试传感器的状态写入pdf29页
}


/**
	***********************************************************
	* @param none
  * @brief Read out the 11 calibration coefficient in the EEPROM of BPM085
  *      for calculating of pressure and temperature
  ************************************************************
  */
void  Init_BMP085(void)
{
	ac1 = Single_Read(BMP085_Addr,0xAA);
	ac1 = (ac1<<8)|Single_Read(BMP085_Addr,0xAB);

    ac2 = Single_Read(BMP085_Addr,0xAC);
	ac2 = (ac2<<8)| Single_Read(BMP085_Addr,0xAD);

	ac3 = Single_Read(BMP085_Addr,0xAE);
	ac3 = (ac3<<8)| Single_Read(BMP085_Addr,0xAF);

	ac4 = Single_Read(BMP085_Addr,0xB0);
	ac4 = (ac4<<8)| Single_Read(BMP085_Addr,0xB1);

	ac5 = Single_Read(BMP085_Addr,0xB2);
	ac5 = (ac5<<8)| Single_Read(BMP085_Addr,0xB3);

	ac6 = Single_Read(BMP085_Addr,0xB4);
	ac6 = (ac6<<8)| Single_Read(BMP085_Addr,0xB5);

	b1 = Single_Read(BMP085_Addr,0xB6);
	b1 = (b1<<8)| Single_Read(BMP085_Addr,0xB7);

	b2 = Single_Read(BMP085_Addr,0xB8);
	b2 = (b2<<8)| Single_Read(BMP085_Addr,0xB9);

	mb = Single_Read(BMP085_Addr,0xBA);
	mb = (mb<<8)| Single_Read(BMP085_Addr,0xBB);

	mc = Single_Read(BMP085_Addr,0xBC);
	mc = (mc<<8)| Single_Read(BMP085_Addr,0xBD);

	md = Single_Read(BMP085_Addr,0xBE);
	md = (md<<8)| Single_Read(BMP085_Addr,0xBF);

}
/**
	**************************************************************
	*	@param None
	* @brief Configure the mode and transmit rate
  *         Structue of Configuration Register:
  *           CRA7 CRA6 CRA5 CRA4 CRA3 CRA2 CRA1 CRA0
  *                MA1  MA0  D02  DO1  DO0  MS1   MS0
  *         PURPUSE:
  *           CRA7=0: FOR WORK NORMAL
  *           MA1 MA0=00: AVERAGE SAMPLE NUMBER=1
  *           DO2 DO1 DO0=101: STANDARD DATA OUTPUT RATE=30Hz
  *           MSA MS0=00:NORMAL MEASUREMENT CONFIGURATION(DEFAULT)
	****************************************************************
	*/
void  Init_HMC5883L()
{
   Single_Write(HMC5883L_Addr,0x00,0x14);   //Write Configuration Register A
	Single_Write(HMC5883L_Addr,0x02,0x01);   //Write Mode Register for Single measure
//   Single_Write(HMC5883L_Addr,0x02,0x00);   //Write Mode Register for continious measure
}


 /**
	 **********************************************************************
   * @param  	None
   * @brief 	Configure the Control Register
	 * @notice  For more information,please refer to L3G4200D datasheet
   ************************************************************************
   */
void Init_L3G4200D(void)
{

 /**     CTRL_REG1-- DR1 DR0=00:ODR(OUTPUT DATA RATE SELECTION)=100HZ,
   *              -- BW1 BW0=00:BW(BANDWITH SELECTION)=12.5(CUT-OFF),
   *              -- PD=1:NORMAL MODE
   *							-- ZEN=1:Z AXIS ENABLE
   *							-- YEN=1:Y AXIS ENABLE
   *							-- XEN=1:X AXIS ENABLE
	 */
	Single_Write(L3G4200_Addr,CTRL_REG1, 0x0f);
 /**     CTRL_REG2--HMP1 HMP0=00:Normal mode (reset reading HP_RESET_FILTER)
   *                (HMP:HIGH PASS FILTER MODE)
   *              --HPCF3 HPCF2 HPCF1 HPCF0=0000:CUT OFF FREQ=8Hz(WHEN ODR=100Hz)
   *                (HPCF:High pass filter cut off frecuency)
	 */
	Single_Write(L3G4200_Addr,CTRL_REG2, 0x00);
	Single_Write(L3G4200_Addr,CTRL_REG3, 0x08);
//	Single_Write(L3G4200_Addr,CTRL_REG4, 0x30);	//+-2000dps
	Single_Write(L3G4200_Addr,CTRL_REG4, 0x10);   //+-500dps
	Single_Write(L3G4200_Addr,CTRL_REG5, 0x00);
}	

//******************************************************
void Read_HMC5883l(void)
{
  Single_Write(HMC5883L_Addr,0x00,0x14);   //
  Single_Write(HMC5883L_Addr,0x02,0x00);   //
  Delayms(10);

  BUF[1]=Single_Read(HMC5883L_Addr,0x03);//OUT_X_H_A
  BUF[2]=Single_Read(HMC5883L_Addr,0x04);//OUT_X_L_B

  BUF[3]=Single_Read(HMC5883L_Addr,0x07);//OUT_Y_H_A
  BUF[4]=Single_Read(HMC5883L_Addr,0x08);//OUT_Y_L_B
	
	BUF[5]=Single_Read(HMC5883L_Addr,0x05);//OUT_Z_H_A
	BUF[6]=Single_Read(HMC5883L_Addr,0x06);//OUT_Z_L_H
	
  M_X=(BUF[1] << 8) | BUF[2]; //Combine MSB and LSB of X Data output register
  M_Y=(BUF[3] << 8) | BUF[4]; //Combine MSB and LSB of Y Data output register
	M_Z=(BUF[5] << 8) | BUF[6]; //Combine MSB and LSB of Z Data output register

  if(M_X>0x7fff)
	{
		M_X-=0xffff;	  
	}
  if(M_Y>0x7fff)
	{
		M_Y-=0xffff;	  
	}
	if(M_Z>0x7fff)
	{
		M_Z-=0xffff;	
	}
	
//  Yaw= atan2(y,x) * (180 / 3.14159265) + 180; // angle in degrees
}
//****************************************
void Read_ADXL345(void)
{
       BUF[0]=Single_Read(ADXL345_Addr,0x32);//OUT_X_L_A
       BUF[1]=Single_Read(ADXL345_Addr,0x33);//OUT_X_H_A

	   BUF[2]=Single_Read(ADXL345_Addr,0x34);//OUT_Y_L_A
       BUF[3]=Single_Read(ADXL345_Addr,0x35);//OUT_Y_H_A

	   BUF[4]=Single_Read(ADXL345_Addr,0x36);//OUT_Z_L_A
       BUF[5]=Single_Read(ADXL345_Addr,0x37);//OUT_Z_H_A

	   A_X=(BUF[1]<<8)+BUF[0];  //合成数据  
	   A_Y=(BUF[3]<<8)+BUF[2];  //合成数据
	   A_Z=(BUF[5]<<8)+BUF[4];  //合成数据
}


/**
	***************************************************************
	* @param  None
	* @brief  Calculate the temperature and pressure
  * @notice Please refer to the BMP085 datasheet for more 
	*  information to the calculate of the temperature and pressure
  ****************************************************************
	*/
void Read_BMP085(void)
{
 	long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;

	ut = bmp085ReadTemp();	   //Read uncompensated temperature value
//	ut = bmp085ReadTemp();	   // 读取温度
	
	/********* Calculate true temperature **************/
	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	temperature = (b5 + 8) >> 4;
	
	up = bmp085ReadPressure();  //Read uncompensated pressure value
//	up = bmp085ReadPressure();  // 读取压强
	
	/*********** Calculate true pressure**************/
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((long)ac1 * 4 + x3) + 2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	if( b7 < 0x80000000)
	{	
	  p = (b7 * 2) / b4 ;
	}          
	else  
	{
	  p = (b7 / b4) * 2;
	}
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	 pressure = p + ((x1 + x2 + 3791) >> 4);
}

/**
	**************************************************************
	* @param  None
	* @brief  Read 3 axis data
	* @notice None
	**************************************************************
  */
void Read_L3G4200D(void)
{
   BUF[0]=Single_Read(L3G4200_Addr,OUT_X_L);
   BUF[1]=Single_Read(L3G4200_Addr,OUT_X_H);
   T_X=	(BUF[1]<<8)|BUF[0];
 
   

   BUF[2]=Single_Read(L3G4200_Addr,OUT_Y_L);
   BUF[3]=Single_Read(L3G4200_Addr,OUT_Y_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
  

   BUF[4]=Single_Read(L3G4200_Addr,OUT_Z_L);
   BUF[5]=Single_Read(L3G4200_Addr,OUT_Z_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
	
	 init_gx=(float)(T_X - Gyro_Xout_Offset)*0.0175f;   //陀螺仪+-500dps时灵敏度为17.5mdps/digt
	 init_gy=(float)(T_Y - Gyro_Yout_Offset)*0.0175f;   //陀螺仪+-2000dps时灵敏度为70mdps/digt
//	 init_gz=(float)(T_Z - Gyro_Zout_Offset)*0.0175f;   //陀螺仪+-250dps时灵敏度为8.75mdps/digt
	 gx=init_gx;    
	 gy=-init_gy;   //由实验看出X轴角速度输出方向与标称方向相反
//	printf("\n X=%d, Y=%d, Z=%d \n\r", T_X, T_Y, T_Z);

}

 //******************ADXL345计算倾斜角度************
void adxl345_angle(void)
{

	data_xyz[0]=A_X;  //合成数据   
	data_xyz[1]=A_Y;  //合成数据   
	data_xyz[2]=A_Z;  //合成数据   

	//分别是加速度X,Y,Z的原始数据，10位的
	Q=(float)data_xyz[0]*3.9f;
	T=(float)data_xyz[1]*3.9f;
	K=(float)data_xyz[2]*3.9f;
	Q=-Q;

  Roll=(float)(((atan2(K,Q)*180)/3.14159265)+180);    //X轴角度值
  Pitch=(float)(((atan2(K,T)*180)/3.14159265)+180);  //Y轴角度值
	printf("Roll %f",Roll);
}


