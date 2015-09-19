#include "stm32f4xx_hal.h"
#include "L3G4200D.h"
#include "myiic.h"
#include "usart.h"


#define L3G4200_Addr    0xD2

//Inner register of L3G4200D
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

uint8_t BUF[8];

float T;

short T_X,T_Y,T_Z;

void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Wait_Ack(void);
void I2C_NAck(void);
void I2C_Send_Byte(uint8_t txd);
uint8_t I2C_Read_Byte(unsigned char ack);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
uint8_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     //void


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
	
	printf("\n X=%d, Y=%d, Z=%d \n\r", T_X, T_Y, T_Z);

}

uint8_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	I2C_Start();
    I2C_Send_Byte((SlaveAddress<<1)|0);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(I2C_Wait_Ack())
		{
			I2C_Stop(); 
			return 1;
		}
    I2C_Send_Byte(REG_Address );   //设置低起始地址      
    I2C_Wait_Ack();	
    I2C_Send_Byte(REG_data);
    if(I2C_Wait_Ack())	//等待ACK
		{
			I2C_Stop();	 
			return 1;		 
		}		 
    I2C_Stop();	 
		return 0;
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{  
	unsigned char REG_data;     	
	  I2C_Start(); 
	I2C_Send_Byte((SlaveAddress<<1)|0);//发送器件地址+写命令	
	I2C_Wait_Ack();		//等待应答 
    I2C_Send_Byte(REG_Address);	//写寄存器地址
    I2C_Wait_Ack();		//等待应答
    I2C_Start();
	I2C_Send_Byte((SlaveAddress<<1)|1);//发送器件地址+读命令	
    I2C_Wait_Ack();		//等待应答 
	REG_data=I2C_Read_Byte(0);//读取数据,发送nACK 
    I2C_Stop();			//产生一个停止条件 
	return REG_data;

}						      


//产生IIC起始信号
void I2C_Start(void)
{
	L3G4200D_SDA_OUT();     //sda线输出
	L3G4200D_IIC_SDA=1;	  	  
	L3G4200D_IIC_SCL=1;
	delay_us(4);
 	L3G4200D_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	L3G4200D_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void I2C_Stop(void)
{
	L3G4200D_SDA_OUT();//sda线输出
	L3G4200D_IIC_SCL=0;
	L3G4200D_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	L3G4200D_IIC_SCL=1; 
	L3G4200D_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t I2C_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	L3G4200D_SDA_IN();      //SDA设置为输入  
	L3G4200D_IIC_SDA=1;delay_us(1);	   
	L3G4200D_IIC_SCL=1;delay_us(1);	 
	while(L3G4200D_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}
	L3G4200D_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void I2C_Ack(void)
{
	L3G4200D_IIC_SCL=0;
	L3G4200D_SDA_OUT();
	L3G4200D_IIC_SDA=0;
	delay_us(2);
	L3G4200D_IIC_SCL=1;
	delay_us(2);
	L3G4200D_IIC_SCL=0;
}
//不产生ACK应答		    
void I2C_NAck(void)
{
	L3G4200D_IIC_SCL=0;
	L3G4200D_SDA_OUT();
	L3G4200D_IIC_SDA=1;
	delay_us(2);
	L3G4200D_IIC_SCL=1;
	delay_us(2);
	L3G4200D_IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void I2C_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	L3G4200D_SDA_OUT(); 	    
    L3G4200D_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        L3G4200D_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		L3G4200D_IIC_SCL=1;
		delay_us(2); 
		L3G4200D_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	L3G4200D_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        L3G4200D_IIC_SCL=0; 
        delay_us(2);
		L3G4200D_IIC_SCL=1;
        receive<<=1;
        if(L3G4200D_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        I2C_NAck();//发送nACK
    else
        I2C_Ack(); //发送ACK   
    return receive;
}

