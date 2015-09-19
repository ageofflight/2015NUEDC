#ifndef __L3G4200D_H
#define __L3G4200D_H

#include "sys.h"

//IO方向设置
#define L3G4200D_SDA_IN()  {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=0<<4*2;}	//PD4输入模式
#define L3G4200D_SDA_OUT() {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=1<<4*2;} //PD4输出模式
//IO操作函数	 
#define L3G4200D_IIC_SCL    PDout(3) //SCL
#define L3G4200D_IIC_SDA    PDout(4) //SDA	 
#define L3G4200D_READ_SDA   PDin(4)  //输入SDA 


void Read_L3G4200D(void);
void Init_L3G4200D(void);





#endif
