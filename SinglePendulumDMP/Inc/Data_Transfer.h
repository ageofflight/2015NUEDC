#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_
//#include "config.h"
#include "stm32f4xx_hal.h"

extern uint8_t Data_Check,Send_Status,Send_Senser,Send_RCData,Send_GpsData,Send_Offset,Send_PID;

void Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void Send_Data(void);
void Data_Send_Status(void);	//AF
void Data_Send_Senser(void);	//AE
void Data_Send_RCData(void);	//AD
void Data_Send_GpsData(void);
void Data_Send_OFFSET(void);	//AC
void Data_Send_PID(void);			//AC

void Data_Send_MotoPWM(void);

void Data_Send_Check(uint16_t check);

void NRF_Send_Test(void);

#endif
