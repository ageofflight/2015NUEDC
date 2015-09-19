
/********************LAMBO Fly soft********************


star:
13-01-07    edit by edmondchao
*******************************************************************************/

#include "stm32f4xx_hal.h"
#include "MPU6050.h"
#include "IOI2C.h"
#include "sqQueue.h"

#include "math.h"

//struct mpu6050_info S0_6050;
//struct mpu6050_info corr_6050;

extern float Roll;
extern float Pitch;
extern float Angle_Z;

extern sq_Queue Queue_Acc_X;
extern sq_Queue Queue_Acc_Y;
extern sq_Queue Queue_Acc_Z;

extern int SendChar (int ch);
extern int GetKey (void);
extern void delay_ms(uint16_t num);

uint8_t						mpu6050_buffer[14];					//iic��ȡ��������
T_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET={520,-395,-630};			//
uint8_t						GYRO_OFFSET_OK = 1;
uint8_t						ACC_OFFSET_OK = 1;
T_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) 
{
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				enabled =1   ˯��
			    enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
    uint8_t buffer[1];
    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_testConnection(void)
*��������:	    ���MPU6050 �Ƿ��Ѿ�����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   {
		 	printf("\nInit_MPU6050 over\n\r");


	//LED1_Turn;
   return 1;

   }
	 else 
			return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

//void MPU6050_intio_Init(void)
//{
//   	GPIO_InitTypeDef GPIO_InitStructure;

//	
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

//	GPIO_InitStructure.GPIO_Pin =  MPU6050_pin;
//	                              
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  	
//  	GPIO_Init(MPU6050_port, &GPIO_InitStructure);


//}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_initialize(void) 
{
	MPU6050_setSleepEnabled(0); //���빤��״̬
	delay_ms(200);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��  0x6b   0x01
	delay_ms(200);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
	delay_ms(200);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//���ٶȶ�������� +-4G
	delay_ms(200);
	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
	delay_ms(200);
	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	delay_ms(200);
	MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
	delay_ms(200);
////	int16_t temp[6];
////	unsigned char i;

////	MPU6050_intio_Init();


//	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��
//	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
//	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_16);	//���ٶȶ�������� +-2G
//	MPU6050_setSleepEnabled(0); //���빤��״̬
//	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
//	MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
////	//����MPU6050 ���ж�ģʽ ���жϵ�ƽģʽ
////	//����λ����0��INT���ŵ��߼���ƽΪ�ߵ�ƽ������λ����1��INT���ŵ��߼���ƽΪ�͵�ƽ��Ч��////(1)
////	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 1);
////	//����λ����0��INT���ű�����Ϊ���졣����λ����1��INT��������Ϊ©����·��
////	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
////	//����λ����0��INT���ŷ�����50uS�ĳ����塣����λ����1��INT���ű���ǰ��(1)�����ƽ��ֱ���ж������
////	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
////	//����λ����0���ж�״̬λ�����㣬ֻ��ͨ���Ķ�INT_STATUS��ע��58������λ����1���κζ��ж�״̬λ�����������
////	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
////	//������ת������ж�
////    IICwriteBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);

//	//MPU6050_en=1;

}

/************************************************************   
* ������:Init_MPU6050   
* ���� ����ʼ��MPU6050��������Ҫ��ο�pdf�����޸�
* ���� :��   
* ��� :��    
*/
void Init_MPU6050(void)
{

  IICwriteByte(devAddr,MPU6050_RA_PWR_MGMT_1, 0x00);	//�������״̬
	
	
	IICwriteByte(devAddr,MPU6050_RA_SMPLRT_DIV, 0x07);    //�����ǲ�����
	IICwriteByte(devAddr,MPU6050_RA_CONFIG, 0x06);        //5Hz 
	
	IICwriteByte(devAddr,MPU6050_RA_INT_PIN_CFG, 0x42);   //ʹ����·I2C
	IICwriteByte(devAddr,MPU6050_RA_USER_CTRL, 0x40);     //ʹ����·I2C
	
	IICwriteByte(devAddr,MPU6050_RA_GYRO_CONFIG, 0x18);   
	IICwriteByte(devAddr,MPU6050_RA_ACCEL_CONFIG, 0x01);
	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char MPU6050_is_DRY(void)
*��������:	    ��� MPU6050���ж����ţ������Ƿ����ת��
���� 1  ת�����
0 ���ݼĴ�����û�и���
*******************************************************************************/
unsigned char MPU6050_is_DRY(void)
{
    if(HAL_GPIO_ReadPin(MPU6050_port,MPU6050_pin)==GPIO_PIN_RESET)
			return 1;
	  else 
			return 0;
}

uint8_t MPU6050_read_ok=0; 
volatile signed short int  AdWert_Nick = 0, AdWert_Roll = 0, AdWert_Gier = 0;
volatile signed short int  AdWert_AngRoll = 0,AdWert_AngNick = 0,AdWert_AngHoch = 0;
int16_t AngX_mid=0,AngY_mid=0,AngZ_mid= 0;
int16_t AccX_mid=0,AccY_mid=0,AccZ_mid= 0;

//uint8_t mpu6050_star=0;//=0: ��һ������
//void MPU6050_getMotion(void)
//{
//   // LED2_Turn;
////    if(MPU6050_is_DRY()) 
////	{
//		printf("\n MPU6050 IS READY\n\r");

//	 
//	 IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
//	 printf("\n buffer0=%d, buffer1=%d \n\r", buffer[0], buffer[1]);

//	  AdWertAngNick=(((((int16_t)buffer[0]) << 8) | buffer[1])); 
//    AdWertAngRoll=(((((int16_t)buffer[2]) << 8) | buffer[3]));	
//		
//	  printf("\n AdWertAngNick=%d, AdWertAngRoll=%d \n\r", AdWertAngNick, AdWertAngRoll);

//    //AdWertAngHoch=(((((int16_t)buffer[4]) << 8) | buffer[5]));  
//	//�����¶�ADC
//    AdWertRoll=(((((int16_t)buffer[8]) << 8) | buffer[9]));		 
//    AdWertNick=(((((int16_t)buffer[10]) << 8) | buffer[11]));	 
//    AdWertGier=(((((int16_t)buffer[12]) << 8) | buffer[13])); 	 

//#define r_move  5
//	#define acc_num 3
//	if(mpu6050_star)
//	{
//		printf("\n MPU6050_STAR1 \n\r");

//	  AdWert_AngRoll=  (AdWert_AngRoll*acc_num + AdWertAngRoll)/(acc_num+1);
//    AdWert_AngNick=  (AdWert_AngNick*acc_num + AdWertAngNick)/(acc_num+1);
//    //AdWert_AngHoch=  (AdWert_AccHoch*acc_num + AdWertAccHoch)/(acc_num+1);

//	
//	S0_6050.AngRoll=	(((AdWert_AngRoll>>r_move))-AngX_mid-corr_6050.AngRoll); 
//	S0_6050.AngNick= -(((AdWert_AngNick>>r_move))-AngY_mid-corr_6050.AngNick);	
//	//S0_6050.AngHoch=((AdWert_AngHoch>>r_move))-AngZ_mid;
//	S0_6050.AccRoll=((AdWertRoll>>6)-AccX_mid-corr_6050.AccRoll); 					
//	S0_6050.AccNick=((AdWertNick>>6)-AccY_mid-corr_6050.AccNick);	 
//	S0_6050.AccGier=-((AdWertGier>>6)-AccZ_mid-corr_6050.AccGier);
//	}
//	else
//	{		
//		 AngX_mid=((AdWertAngRoll>>r_move));
//		 AngY_mid=((AdWertAngNick>>r_move));
//		 AccX_mid=  (AdWertRoll>>6);
//		 AccY_mid=  (AdWertNick>>6);
//		 AccZ_mid=  (AdWertGier>>6);

//	 	S0_6050.AngNick=	AngX_mid; 
//		S0_6050.AngRoll= AngY_mid;
//		S0_6050.AccNick= AccX_mid; 						
//		S0_6050.AccRoll= AccY_mid;	 
//		S0_6050.AccGier= AccZ_mid;		
//	}

//	
//	
////#define	test_ppm_ad											     
//#ifdef 	test_ppm_ad	

//	//UART1_Put_Chars("Temperature:");
//	//UART1_Put_int((((((int16_t)buffer[6]) <<8) | buffer[7]))>>6);

//	UART1_Put_Chars("AccRoll:");
//	UART1_Put_int(S0_6050.AccRoll);
//	UART1_Put_Chars("AccNick:");
//	UART1_Put_int(S0_6050.AccNick);
//	
//	//UART1_Put_Chars("AngHoch:");
//	//UART1_Put_int(S0_6050.AngHoch); 
//	UART1_Put_Chars("AngRoll:");
//	UART1_Put_int(S0_6050.AngRoll);	
//	UART1_Put_Chars("AngN:");
//	UART1_Put_int(S0_6050.AngNick);	
//	UART1_Put_int(S0_6050.AccNick);	
//	UART1_Put_Chars("AccGier:");	  
//	UART1_Put_int(S0_6050.AccGier);

//	USART_newline;
//#endif	
////	}
//}

uint8_t BUF[10];       //�������ݻ�����		 		
uint16_t GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT,MP6050_Temperature;//X,Y,Z�ᣬ�¶�	

/************************************************************   
* ������:READ_MPU6050_Gyro   
* ���� : ��ȡMPU6050����������
* ����  :��   
* ���  :��    
*/
void READ_MPU6050_Gyro(void)
{
   BUF[0]=I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L); 
   BUF[1]=I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H);
   GYRO_XOUT=	(BUF[1]<<8)|BUF[0];
   GYRO_XOUT/=16.4; 						   //��ȡ����X������

   BUF[2]=I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);
   BUF[3]=I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H);
   GYRO_YOUT=	(BUF[3]<<8)|BUF[2];
   GYRO_YOUT/=16.4; 						   //��ȡ����Y������
   BUF[4]=I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);
   BUF[5]=I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H);
   GYRO_ZOUT=	(BUF[5]<<8)|BUF[4];
   GYRO_ZOUT/=16.4; 					       //��ȡ����Z������

	
  // BUF[6]=I2C_ReadByte(MPU6050_Addr,TEMP_OUT_L); 
  // BUF[7]=I2C_ReadByte(MPU6050_Addr,TEMP_OUT_H); 
  // T_T=(BUF[7]<<8)|BUF[6];
  // T_T = 35+ ((double) (T_T + 13200)) / 280;// ��ȡ������¶�
}
/************************************************************   
* ������:READ_MPU6050_Accel   
* ���� : ��ȡMPU6050���ٶ�����
* ����  :��   
* ���  :��    
*/
void READ_MPU6050_Accel(void)
{
	BUF[0]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); 
	BUF[1]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H);
	ACCEL_XOUT=	(BUF[1]<<8)|BUF[0];
	ACCEL_XOUT=(float)((float)ACCEL_XOUT/16384)*100; 		//����100��	       //��ȡ����X������

	BUF[2]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L);
	BUF[3]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H);
	ACCEL_YOUT=	(BUF[3]<<8)|BUF[2];
	ACCEL_YOUT=(float)((float)ACCEL_YOUT/16384)*100; 						   //��ȡ����Y������
   
	BUF[4]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);
	BUF[5]=I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H);
	ACCEL_ZOUT=	(BUF[5]<<8)|BUF[4];
	ACCEL_ZOUT=(float)((float)ACCEL_ZOUT/16384)*100; 					       //��ȡ����Z������

	
	BUF[6]=I2C_ReadOneByte(devAddr, MPU6050_RA_TEMP_OUT_L); 
	BUF[7]=I2C_ReadOneByte(devAddr, MPU6050_RA_TEMP_OUT_H); 
	MP6050_Temperature=(BUF[7]<<8)|BUF[6];
//	MP6050_Temperature = 35+ ((double) (T_T + 13200)) / 280;// ��ȡ������¶�
	MP6050_Temperature = (((double) MP6050_Temperature )/340+36.53)*10 ;//+36.53;  // ��ȡ������¶�
}

/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.X;
	MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.Y;
	MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - ACC_OFFSET.Z;
	//�����¶�ADC
	MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.X;
	MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.Y;
	MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.Z;
	
	insert(&Queue_Acc_X, MPU6050_ACC_LAST.X);
	
	insert(&Queue_Acc_Y, MPU6050_ACC_LAST.Y);
//	
	insert(&Queue_Acc_Z, MPU6050_ACC_LAST.Z);
	
	MPU6050_ACC_LAST.X = MovAve(&Queue_Acc_X);  //����˲�ֵ
	
	MPU6050_ACC_LAST.Y = MovAve(&Queue_Acc_Y);
//	
	MPU6050_ACC_LAST.Z = MovAve(&Queue_Acc_Z);
//	
	Get_Accel_Angle(MPU6050_ACC_LAST.X, MPU6050_ACC_LAST.Y, MPU6050_ACC_LAST.Z);
	
//	printf("\n%d;\n\r", MPU6050_ACC_LAST.Z);
	printf("\nY=%d;\n\r", MPU6050_ACC_LAST.Z);
	
	//, AccY=%d, AccZ=%d  , MPU6050_ACC_LAST.Y, MPU6050_ACC_LAST.Z
	
	if(!GYRO_OFFSET_OK)
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
// 		LED1_ON;
		if(cnt_g==0)
		{
			GYRO_OFFSET.X=0;
			GYRO_OFFSET.Y=0;
			GYRO_OFFSET.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		tempgx+= MPU6050_GYRO_LAST.X;
		tempgy+= MPU6050_GYRO_LAST.Y;
		tempgz+= MPU6050_GYRO_LAST.Z;
		if(cnt_g==200)
		{
			GYRO_OFFSET.X=tempgx/cnt_g;
			GYRO_OFFSET.Y=tempgy/cnt_g;
			GYRO_OFFSET.Z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
//			Data_Save();
			printf("\n GYRO_OFFSET.X=%d, GYRO_OFFSET.Y=%d, GYRO_OFFSET.Z=%d,  \n\r", GYRO_OFFSET.X, GYRO_OFFSET.Y, GYRO_OFFSET.Z);
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
// 		LED1_ON;
		if(cnt_a==0)
		{
			ACC_OFFSET.X = 0;
			ACC_OFFSET.Y = 0;
			ACC_OFFSET.Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= MPU6050_ACC_LAST.X;
		tempay+= MPU6050_ACC_LAST.Y;
		//tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==200)
		{
			ACC_OFFSET.X=tempax/cnt_a;
			ACC_OFFSET.Y=tempay/cnt_a;
			ACC_OFFSET.Z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
//			Data_Save();
			printf("\n ACC_OFFSET.X=%d, ACC_OFFSET.Y=%d, ACC_OFFSET.Z=%d,  \n\r", ACC_OFFSET.X, ACC_OFFSET.Y, ACC_OFFSET.Z);

			return;
		}
		cnt_a++;		
	}
}

/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���,����MPU6050_Last
*******************************************************************************/
uint8_t MPU6050_Read(void)
{
	return IICreadBytes(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}

void Get_Accel_Angle(int16_t accel_x, int16_t accel_y, int16_t accel_z)
{
	Roll = atan2(accel_y, -accel_z);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.295780ת��Ϊ�Ƕ�
//	init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
  Pitch = atan2(accel_x, -accel_z); 
	
	Roll  = Roll * 57.295780f;	 //����ת�Ƕ�
	Pitch = Pitch * 57.295780f;
}

//------------------End of File----------------------------

