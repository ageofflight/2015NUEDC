


#include "stm32f4xx_hal.h"
#include "IOI2C.h"
//#include "nine_axis_module.h"


  
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
//void IIC_Init(void)
//{			
//	

//	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
// 	//����PB6 PB7 Ϊ��©���  ˢ��Ƶ��Ϊ10Mhz
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	//Ӧ�����õ�GPIOB 
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);   

//}

void get_ms(unsigned long *count)
{}


void delay_us(uint16_t nus)
{		
	uint16_t a; 
	
	for(a=0;a<nus;a++)
	{
		uint8_t i;
		for(i=6;i>0;i--);
	}
}

void delay_ms(uint16_t num)
{	 		  	  
	uint16_t  a;

	for(a=0;a<num;a++)
	{
	   delay_us(1000);			   
	}	  	    
} 

void delay_100ms(uint16_t num)
{	 		  	  
	uint16_t  a;

	for(a=0;a<num;a++)
	{
	   delay_ms(100);			   
	}	  	    
} 

#define I2C_star_delay 2 //4
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void I2C_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     //sda�����
//	SDA_H;	  	  
//	SCL_H;
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(I2C_star_delay);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(I2C_star_delay);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(I2C_star_delay);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(I2C_star_delay);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	  
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 1;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
#define I2C_delay  1	 //2
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(I2C_delay);
	IIC_SCL=1;
	delay_us(I2C_delay);
	IIC_SCL=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(I2C_delay);
	IIC_SCL=1;
	delay_us(I2C_delay);
	IIC_SCL=0;
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(uint8_t txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
	SDA_OUT(); 	    
  IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		delay_us(I2C_delay);   
		IIC_SCL=1;
		delay_us(I2C_delay); 
		IIC_SCL=0;	
		delay_us(I2C_delay);
	}	 
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	if(IICwriteBytes(addr, reg, len, data))
	{
		return 0;
	}
	else
	{
		return -1;
	}
//		int i;
//    if (!IIC_Start())
//        return 1;
//    IIC_Send_Byte(addr << 1 );
//    if (!IIC_Wait_Ack()) {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(reg);
//    IIC_Wait_Ack();
//		for (i = 0; i < len; i++) {
//        IIC_Send_Byte(data[i]);
//        if (!IIC_Wait_Ack()) {
//            IIC_Stop();
//            return 0;
//        }
//    }
//    IIC_Stop();
//    return 0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(IICreadBytes(addr, reg, len, buf))
	{
		return 0;
	}
	else
	{
		return -1;
	}
//    if (!IIC_Start())
//        return 1;
//    IIC_Send_Byte(addr);
//    if (!IIC_Wait_Ack()) {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(reg);
//    IIC_Wait_Ack();
//    IIC_Start();
//    IIC_Send_Byte(addr+1);
//    IIC_Wait_Ack();
//    while (len) {
//        if (len == 1)
//            *buf = IIC_Read_Byte(0);
//        else
//            *buf = IIC_Read_Byte(1);
//        buf++;
//        len--;
//    }
//    IIC_Stop();
//    return 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
  for(i=0;i<8;i++ )
	{
		IIC_SCL=0; 
		delay_us(I2C_delay);
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;   
		delay_us(I2C_delay); 
  }					 
	if (ack)
			IIC_Ack(); //����ACK 
	else
			IIC_NAck();//����nACK  
	return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
  uint8_t count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++)
	{
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

   return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
	*data=I2C_ReadOneByte(dev, reg);
//	  *data = Single_Read(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0) 
		{
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } 
		else 
		{
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data){
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

/************************************************************   
* ������:I2C_NoAddr_WriteByte   
* ���� : ���ض��豸id��д���ֽ� 
* ����  :�豸id������   
* ���  :��    
*/
void IIC_NoAddr_WriteByte(unsigned char DeviceAddr,unsigned char info)
{

   IIC_Start();
   IIC_Send_Byte(DeviceAddr);
   IIC_Wait_Ack();
   IIC_Send_Byte(info);
   IIC_Wait_Ack();
   IIC_Stop();
   //delay2(50);
   delay_us(250);

}

//------------------End of File----------------------------

