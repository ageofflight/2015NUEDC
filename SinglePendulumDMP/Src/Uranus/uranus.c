#include "uranus.h"

imu_data data;
static char rev_buf[64];

extern uint8_t uart3_RX_data[32];

extern uint8_t Mode;
extern float Amplitude;
extern float Radius;
extern float Radius_Eight;
extern float nDirection;
extern float nAmplitude;

int16_t Accel_X;
int16_t Accel_Y;
int16_t Accel_Z;

int16_t Gyro_X;
int16_t Gyro_Y;
int16_t Gyro_Z;

int16_t Mag_X;
int16_t Mag_Y;
int16_t Mag_Z;

 
 extern float Pitch;
 extern float Roll;
 extern float Yaw;
 

void imu_rev_process(char ch)
{
	static int len=0;
	static int i;
	
	static enum input_status status=STATUS_IDLE;
	
	switch(status)
	{
		case STATUS_IDLE:
			if((uint8_t)ch==0x88)
			{
				status=STATUS_SOF;
			}
			break;
		case STATUS_SOF:
			if((uint8_t)ch==0xAF)
			{
				status=STATUS_LEN;
			}
			break;
		case STATUS_LEN:
			len=ch;
		  status=STATUS_DATA;
		  i=0;
		  break;
		case STATUS_DATA:
			if(i==len)
			{
				status=STATUS_IDLE;
				imu_rev_get_data(&data);
				break;
			}
			rev_buf[i++]=ch;
			break;
		default:
			break;
	}
}

int imu_rev_get_data(imu_data* data)
{
	data->accl[0]=(rev_buf[0]<<8)+rev_buf[1];
	data->accl[1]=(rev_buf[2]<<8)+rev_buf[3];
	data->accl[2]=(rev_buf[4]<<8)+rev_buf[5];
	data->gyro[0]=(rev_buf[6]<<8)+rev_buf[7];
	data->gyro[1]=(rev_buf[8]<<8)+rev_buf[9];
	data->gyro[2]=(rev_buf[10]<<8)+rev_buf[11];
	
	data->mag[0]=(rev_buf[12]<<8)+rev_buf[13];
	data->mag[1]=(rev_buf[14]<<8)+rev_buf[15];
	data->mag[2]=(rev_buf[16]<<8)+rev_buf[17];
	
	data->roll=(rev_buf[18]<<8)+rev_buf[19];
	data->pitch=(rev_buf[2]<<8)+rev_buf[21];
	data->yaw=(rev_buf[22]<<8)+rev_buf[23];
	
	data->presure=(rev_buf[27]<<24)+(rev_buf[26]<<16)+(rev_buf[25]<<8)+(rev_buf[24]<<0);
	
	data->pitch=data->pitch/100;
	data->roll=data->roll/100;
	data->yaw=data->yaw/10;
	
	return 0;
}


void get_control_data(uint8_t* Rx_Data)
{
	Mode=(Rx_Data[0]<<8)+Rx_Data[1];
	
//	Amplitude=Rx_Data[3];
//	nDirection=Rx_Data[4];
//	nAmplitude=Rx_Data[5];
//	
//	Radius=Rx_Data[6];
//	Radius_Eight=Rx_Data[7];
	Amplitude=(Rx_Data[2]<<8)+Rx_Data[3];
	nDirection=(Rx_Data[4]<<8)+Rx_Data[5];
	nAmplitude=(Rx_Data[6]<<8)+Rx_Data[7];
	
	Radius=(Rx_Data[8]<<8)+Rx_Data[9];
	Radius_Eight=(Rx_Data[10]<<8)+Rx_Data[11];
	
//	printf("\n R \n\r");
	
	
//	Pitch=((float)((Rx_Data[21]<<8)+Rx_Data[22]))/100.0f;
//	Roll=((float)((Rx_Data[23]<<21)+Rx_Data[24]))/100.0f;
//	Yaw=((float)((Rx_Data[25]<<21)+Rx_Data[26]))/10.0f;
}

