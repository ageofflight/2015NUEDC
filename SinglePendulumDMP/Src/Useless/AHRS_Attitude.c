#include "AHRS_Attitude.h"
//#include "delay.h"
//#include "adxl335.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "STM32_I2C.h"
#include "math.h"
#include "motor.h"
#include <stdio.h>
#include "MPU6050.h"
//#include "Time.h"

#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.0610351				
#define Gyro_Gr	0.0010653			
#define FILTER_NUM 20

//定义不同测量范围的刻度因子
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

#define Accel_Xout_Offset		-130
#define Accel_Yout_Offset		 96
#define Accel_Zout_Offset		 460
//#define FILTER_NUM	            20

#define Kp 2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.00125f  //half the sample period,halfT需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度
//float Yaw;
//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
float Gyro_Xout_Offset = 6.727800, Gyro_Yout_Offset = 20.726801, Gyro_Zout_Offset = 16.154400;
//Gyro_Xout_Offset=6.727800, Gyro_Yout_Offset=20.726801, Gyro_Zout_Offset=16.154400
extern float Pitch, Roll, Yaw;
float halfT;
float maxMagX = 0;
float minMagX = 0;
float maxMagY = 0;
float minMagY = 0;
float maxMagZ = 0;
float minMagZ = 0;
float MXgain = 1.069597;
float MYgain = 1.074847;
float MZgain = 1.000000;
float MXoffset = -237.985367;
float MYoffset = 169.288345;
float MZoffset = -193.000000;
float heading;

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
T_INT16_XYZ ACC_AVG;

extern short T_X,T_Y,T_Z; //Data of L3G4200D without any dealing
extern int M_X,M_Y,M_Z;
extern uint32_t AccValue[3];
T_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;
//extern float Acc_X;
//extern float Acc_Y;
//extern float Acc_Z;


extern void Read_L3G4200D(void);
extern void GetAccelValue(void);


void Prepare_Data(void)
{
	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;
	ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
	ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / FILTER_NUM;
	ACC_AVG.Y = temp2 / FILTER_NUM;
	ACC_AVG.Z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	
//	GYRO_I.Z += MPU6050_GYRO_LAST.Z*Gyro_G*0.002;
}

/*******************************************************************************
* Function Name  : get_mpu9150_data
* Description    : 读取mpu9150的加速度计 陀螺仪 磁力计数据并做校准和滤波.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void get_sensor_data(void)
{
	int32_t	Tempgx=0, Tempgy=0, Tempgz=0;
	int32_t	Tempax=0, Tempay=0, Tempaz=0;
	uint8_t cnt=1;
	while(cnt<=200)
	{
		MPU6050_Read();
		MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.X;
		MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.Y;
		MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - ACC_OFFSET.Z;
		//跳过温度ADC
		MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.X;
		MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.Y;
		MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.Z;
		
		if(!GYRO_OFFSET_OK)
		{
	// 		LED1_ON;
			if(cnt==1)
			{
				GYRO_OFFSET.X=0;
				GYRO_OFFSET.Y=0;
				GYRO_OFFSET.Z=0;
				Tempgx = 0;
				Tempgy = 0;
				Tempgz = 0;
			}
			Tempgx+= MPU6050_GYRO_LAST.X;
			Tempgy+= MPU6050_GYRO_LAST.Y;
			Tempgz+= MPU6050_GYRO_LAST.Z;
			if(cnt==200)
			{
				GYRO_OFFSET.X=Tempgx/cnt;
				GYRO_OFFSET.Y=Tempgy/cnt;
				GYRO_OFFSET.Z=Tempgz/cnt;
				cnt = 0;
				GYRO_OFFSET_OK = 1;
	//			Data_Save();
				printf("\n GYRO_OFFSET.X=%d, GYRO_OFFSET.Y=%d, GYRO_OFFSET.Z=%d,  \n\r", GYRO_OFFSET.X, GYRO_OFFSET.Y, GYRO_OFFSET.Z);
			}
		}
		if(!ACC_OFFSET_OK)
		{
			if(cnt==1)
			{
				ACC_OFFSET.X = 0;
				ACC_OFFSET.Y = 0;
				ACC_OFFSET.Z = 0;
				Tempax = 0;
				Tempay = 0;
				Tempaz = 0;
			}
			Tempax+= MPU6050_ACC_LAST.X;
			Tempay+= MPU6050_ACC_LAST.Y;
			//tempaz+= MPU6050_ACC_LAST.Z;
			if(cnt==200)
			{
				ACC_OFFSET.X=Tempax/cnt;
				ACC_OFFSET.Y=Tempay/cnt;
				ACC_OFFSET.Z=Tempaz/cnt;
				cnt = 0;
				ACC_OFFSET_OK = 1;
	//			Data_Save();
				printf("\n ACC_OFFSET.X=%d, ACC_OFFSET.Y=%d, ACC_OFFSET.Z=%d,  \n\r", ACC_OFFSET.X, ACC_OFFSET.Y, ACC_OFFSET.Z);
			}
			cnt++;	
		}	
	}		
	  GetAccelValue();
//	  printf("\ngetaccvalue: AccValue[0]=%d, Acc_X=%f", AccValue[0], Acc_X);
	  init_ax=(float)MPU6050_ACC_LAST.X;
	  init_ay=(float)MPU6050_ACC_LAST.X;
	  init_az=(float)MPU6050_ACC_LAST.X;
	
	  Read_L3G4200D();
//		printf("\nRead_L3G4200D: T_X=%d", T_X);

		init_gx=(float)(MPU6050_GYRO_LAST.X)*0.07f;
		init_gy=(float)(MPU6050_GYRO_LAST.X)*0.07f;
		init_gz=(float)(MPU6050_GYRO_LAST.X)*0.07f;
		
	  Read_HMC5883l();
//		printf("\nRead_HMC58831:M_X=%d", M_X);
	
//	  printf("\n mx=%d, my=%d, mz=%d \n\r", M_X, M_Y, M_Z);

		init_mx=(float)M_X * MXgain + MXoffset;
		init_my=(float)M_Y * MYgain + MYoffset;
		init_mz=(float)M_Z * MZgain + MZoffset;

//    printf("\n inti_mx=%f, init_my=%f, init_mz=%f \n\r", init_mx, init_my, init_mz);
}

/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
  float init_Yaw, init_Pitch, init_Roll;
  get_sensor_data();

	//陀螺仪y轴为前进方向    
	init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.295780转化为角度
//	init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
  init_Pitch = atan2(init_ay, init_az); 
	init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
										 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度				            
	//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
	q0 = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
	q1 = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   绕x轴旋转是pitch
	q2 = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   绕y轴旋转是roll
	q3 = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   绕z轴旋转是Yaw
  
		printf("\ninit_Yaw=%f, init_Pitch=%f, init_Roll=%f, q0=%f, q1=%f,q2=%f, q3=%f \n\r", init_Yaw, init_Pitch, init_Roll, q0, q1, q1, q3);

	//陀螺仪x轴为前进方向
	//  init_Roll  = atan2(init_ay, init_az);
	//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
	//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
	//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
	//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
	//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
	//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
	//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

	init_Roll  = init_Roll * 57.295780f;	 //弧度转角度
	init_Pitch = init_Pitch * 57.295780f;
	init_Yaw   = init_Yaw * 57.295780f;
	if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //将Yaw的范围转成0-360
	if(init_Yaw > 360){init_Yaw = init_Yaw - 360;}
	heading    = init_Yaw; 	    
	printf("由初始化四元数得到：Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw, init_Pitch, init_Roll);
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*方便之后的程序使用，减少计算时间*/
   //auxiliary variables to reduce number of repeated operations，
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
//   norm = invSqrt(ax*ax + ay*ay + az*az);       
//   ax = ax * norm;
//   ay = ay * norm;
//   az = az * norm;
	 norm = sqrt(ax*ax + ay*ay + az*az);
   ax = ax / norm;
   ay = ay / norm;
   az = az / norm;
	 
//	 printf("\n ax=%f, ay=%f, az=%f \n\r", ax, ay, az);
	 
//   norm = invSqrt(mx*mx + my*my + mz*mz); 	 
//   mx = mx * norm;
//   my = my * norm;
//   mz = mz * norm;

//   printf("\n mx=%f, my=%f, mz=%f \n\r", mx, my, mz);
	 
   norm = sqrt(mx*mx + my*my + mz*mz);
   mx = mx / norm;
   my = my / norm;
   mz = mz / norm;	 

//   printf("\n mx=%f, my=%f, mz=%f \n\r", mx, my, mz);
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
	 
//	 printf("\n hx=%f, hy=%f, hz=%f \n\r", hx, hy, hz);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
	 
//	 printf("\n by=%f, bz=%f \n\r", by, bz);
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;
	 
//	 printf("\n vx=%f, vy=%f, vz=%f \n\r", vx, vy, vz);

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
	 
//	 printf("\n wx=%f, wy=%f, wz=%f \n\r", wx, wy, wz);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	 
//	 printf("\n ex=%f, ey=%f, ez=%f \n\r", ex, ey, ez);
   
//   // integral error scaled integral gain
//   exInt = exInt + ex*Ki;		
//   eyInt = eyInt + ey*Ki;
//   ezInt = ezInt + ez*Ki;
//   // adjusted gyroscope measurements
//   gx = gx + Kp*ex + exInt;
//   gy = gy + Kp*ey + eyInt;
//   gz = gz + Kp*ez + ezInt;

   halfT=Get_Nowtime();		//得到每次姿态更新的周期的一半

//   printf("\n halft=%f \n\r", halfT);
	 
//	 printf("\n gx=%f, gy=%f, gz=%f \n\r", gx, gy, gz);
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
	
		  printf("\n gx=%f, gy=%f, gz=%f \n\r", gx, gy, gz);
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         
	 
//	 printf("\n q0=%f, q1=%f, q2=%f, q3=%f \n\r", q0, q1, q2, q3);
//	 printf("\n gx=%f, gy=%f, gz=%f \n\r", gx, gy, gz);

   // integrate quaternion rate and normalise，四元数更新算法
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	 
//	 printf("\n q0=%f, q1=%f, q2=%f, q3=%f \n\r", q0, q1, q2, q3);
        
   // normalise quaternion
//   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//   q0 = q0 * norm;       //w
//   q1 = q1 * norm;       //x
//   q2 = q2 * norm;       //y
//   q3 = q3 * norm;       //z
	 norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 / norm;       //w
   q1 = q1 / norm;       //x
   q2 = q2 / norm;       //y
   q3 = q3 / norm;       //z
	 
//	 printf("\n q0=%f, q1=%f, q2=%f, q3=%f \n\r", q0, q1, q2, q3);
        
/*Y轴指向正北，由四元数计算出Pitch  Roll  Yaw，只需在需要PID控制时才将四元数转化为欧拉角
乘以57.295780是为了将弧度转化为角度*/
	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
  if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch =  asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
  Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动
  
	printf("\n Roll=%f, Pitch=%f \n\r", Roll, Pitch);

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.295780是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //偏航角，绕z轴转动

//	printf("halfT=%f  \n\r", halfT);
//    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}

/*******************************************************************************
姿态解算-更新四元数	
*******************************************************************************/
void Get_Attitude(void)
{
   AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
}

/*******************************************************************************
校准陀螺仪零偏	
*******************************************************************************/
int get_gyro_bias(void)
{
  unsigned short int i;
  signed int gyro_x=0, gyro_y=0, gyro_z=0;
  static unsigned short count=0;
  for(i=0;i<5000;i++)
  {
		Read_L3G4200D();
		gyro_x += T_X;
	  gyro_y += T_Y;
		gyro_z += T_Z;
		count++;
		printf("\n Count=%d \n\r", count);
	}
  Gyro_Xout_Offset = (float)gyro_x / count;
  Gyro_Yout_Offset = (float)gyro_y / count;
  Gyro_Zout_Offset = (float)gyro_z / count;
  printf("gyro_x=%d, gyro_y=%d, gyro_z=%d, Gyro_Xout_Offset=%f, Gyro_Yout_Offset=%f, Gyro_Zout_Offset=%f, count=%d \n\r",
          gyro_x, gyro_y, gyro_z, Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset, count);

  return 0;
}
/*******************************************************************************
得到mag的Xmax、Xmin、Ymax、Ymin、Zmax、Zmin	
*******************************************************************************/
void get_compass_bias(void)
{
  Read_HMC5883l();
	
	init_mx=M_X;
	init_my=M_Y;
	init_mz=M_Z;
	
//	printf("\n mx=%d, my=%d, mz=%d \n\r", M_X, M_Y, M_Z);

//	init_mx =(float)M_X * MXgain + MXoffset;		//转换坐标轴				
//	init_my =(float)M_Y * MYgain + MYoffset;
//	init_mz =(float)M_Z * MZgain + MZoffset;

  if(init_mx > maxMagX)
  maxMagX = init_mx;
  if(init_mx < minMagX)
  minMagX = init_mx;

  if(init_my > maxMagY)
  maxMagY = init_my;
  if(init_my < minMagY)
  minMagY = init_my;

  if(init_mz > maxMagZ)
  maxMagZ = init_mz;
  if(init_mz < minMagZ)
  minMagZ = init_mz;
//  printf("maxMagX=%f, minMagX=%f, maxMagY=%f, minMagY=%f, maxMagZ=%f, minMagZ=%f \n\r", 
//          maxMagX, minMagX, maxMagY, minMagY, maxMagZ, minMagZ);  
}
/*******************************************************************************
空间校准compass	
*******************************************************************************/
void compass_calibration(void)
{ //将有最大响应的轴的增益设为1
  if(((maxMagX - minMagX) >= (maxMagY - minMagY)) && ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
  {
    MXgain = 1.0;
		MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
		MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);
		MXoffset = -0.5 * (maxMagX + minMagX);
		MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
		MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);	 
  }
  if(((maxMagY - minMagY) > (maxMagX - minMagX)) && ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
  {
    MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
		MYgain = 1.0;
		MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
		MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
		MYoffset = -0.5 * (maxMagY + minMagY);
		MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);    
  }
  if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) && ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
  {
    MXgain = (maxMagZ - minMagZ) / (maxMagX - minMagX);
		MYgain = (maxMagZ - minMagZ) / (maxMagY - minMagY);
		MZgain = 1.0;
		MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
		MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
		MZoffset = -0.5 * (maxMagZ + minMagZ);    
  }
//  printf("MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f \n\r", 
//          MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);         
}

/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
