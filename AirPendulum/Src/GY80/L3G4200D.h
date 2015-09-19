#ifndef __L3G4200D_H
#define __L3G4200D_H

#include "sys.h"

//IO��������
#define L3G4200D_SDA_IN()  {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=0<<4*2;}	//PD4����ģʽ
#define L3G4200D_SDA_OUT() {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=1<<4*2;} //PD4���ģʽ
//IO��������	 
#define L3G4200D_IIC_SCL    PDout(3) //SCL
#define L3G4200D_IIC_SDA    PDout(4) //SDA	 
#define L3G4200D_READ_SDA   PDin(4)  //����SDA 


void Read_L3G4200D(void);
void Init_L3G4200D(void);





#endif
