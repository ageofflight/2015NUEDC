/**
  ******************************************************************************
  * @file    nine_axis_mudule.h
  * @author  ageofflight
  * @version V1.0
  * @date    07-02-2015
  * @brief   Header file of ADC ADXL335 for angular copmuting.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NINE_AXIS_MODULE_H
#define __NINE_AXIS_MODULE_H

#include "stm32f4xx_hal.h"
#include <math.h>

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

#define OSS 0   //For usage of BMP085

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

//Configuration for the address of the 4 device
#define HMC5883L_Addr   0x3C
#define ADXL345_Addr    0xA6
#define BMP085_Addr     0xEE
#define L3G4200_Addr    0xD2

extern uint8_t BUF[8];

extern int   x,y;
extern float angle;
extern int A_X,A_Y,A_Z;
extern short T_X,T_Y,T_Z;

extern short data_xyz[3];

//For usage of BMP085
extern short   ac1;
extern short   ac2;
extern short 	ac3;
extern uint8_t ac4;
extern uint8_t ac5;
extern uint8_t ac6;
extern short   b1;
extern short   b2;
extern short   mb;
extern short   mc;
extern short   md;

extern long    temperature;
extern long    pressure;
/*The definiton of the input/output ports of IIC*/
//#define SCL_H         GPIOB->ODR = 1
//#define SCL_L         GPIOB->ODR = 0 
//   
//#define SDA_H         GPIOB->ODR = 1
//#define SDA_L         GPIOB->ODR = 0

//#define SCL_read      GPIOB->IDR  & GPIO_PIN_6
//#define SDA_read      GPIOB->IDR  & GPIO_PIN_7
#define GY80_SDA_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define GY80_SDA_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)

#define GY80_SCL_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define GY80_SCL_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

#define GY80_SCL_read HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) 
#define GY80_SDA_read HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) 

/*Declaration of some function*/
void Delay(uint32_t nTime);
void Delayms(__IO uint32_t m);
//void conversion(long temo_data);
void adxl345_angle(void);
void delay5ms(void);
void I2C_delay(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void); 
void I2C_SendByte(uint8_t SendByte);
unsigned char I2C_ReadByte(void);
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);	
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
long bmp085ReadTemp(void);
long bmp085ReadPressure(void);
void Send_ADXL345_data(int dis_data);
//void Send_L3G420D_data(short dis_data);
void Send_HMC5883L(void);
void Send_BMP085(void);
void Send_ADXL345(void);
void Send_L3G4200D(void);
void Init_ADXL345(void);
void Init_BMP085(void);
void Init_HMC5883L(void);
void Init_L3G4200D(void);
void Read_HMC5883l(void);
void Read_ADXL345(void);
void Read_BMP085(void);
void Read_L3G4200D(void);
void adxl345_angle(void);
extern int SendChar(int ch);
extern int GetKey(void);



#endif

