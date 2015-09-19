/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 15/08/2015 22:16:48
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

//#include "nine_axis_module.h"
#include "PID_Control.h"
#include "motor.h"
//#include "adxl335.h"
//#include "Imu.h"
//#include "dmpctl.h"
#include "Data_Transfer.h"
//#include "IOI2C.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "coordinatetrans.h"
#include "pendulumcontrol.h"
#include "nine_axis_module.h"
//#include "L3G4200D.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern float Motor_Angle_X;

extern float Pitch,Roll,Yaw; 		//欧拉角

extern float Pitch_Der;              //Pitch变化率期望
extern float Roll_Der;               //Roll变化率期望

extern WorkStatus workstatus;
extern FlagStatus press;
extern FunctionalState state;

extern FlagStatus Receive;
extern uint16_t Mode;
extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;

extern float q0,q1,q2,q3;

extern float gx;
extern float gy;
extern float gz;

/* 引用自Uranus.c */
extern int16_t Gyro_X;
extern int16_t Gyro_Y;
extern int16_t Gyro_Z;
/* ------------- */

extern uint8_t uart3_RX_data[32];

extern uint8_t uart2_RX_data[100];

Quaternion q;
AxisAngle AA;

uint8_t Hz57 = 0;
uint8_t Hz167 = 0;

//extern PID_Struct PID_Roll_Struct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_DMA(&huart2, uart3_RX_data, 12);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	
	
	/* 初始化电调 */
//	while(1)
//	{
////		TIM1_Motor_PWMOutput(PWMInCh3, PWMInCh3, PWMInCh3, PWMInCh3);
//		if(press == SET)
//		{
//			TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
//			for(uint16_t i=0; i<=500; i++)
//    	{
//				__NOP();
//			}
//			break;
//		}
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   //Red light on

//  }
	
	printf("\n Init \n\r");
	
	Init_L3G4200D();
	MPU_Init();					//初始化MPU6050
	
	while(mpu_dmp_init())
	{
		printf("\n MPU6050 Error \n\r");
	}
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);   //Red light off
	
//	mpu6050_dmp_init();
  while(mpu_dmp_get_data(&Pitch,&Roll,&Yaw))
  {
//		if(mpu_dmp_get_data(&Pitch,&Roll,&Yaw)==0)
//		{
			printf("\nRead Error\n\r");
//			printf("\n%f\n\r", Pitch);
//		}
//	 DMP_update();
//	 printf("%f   ,   %f   ,   %f    \r\n",Pitch,Roll,Yaw);
	 delay_ms(10);	 
  
  }
	
	

//	MPU6050_initialize();
//	
//	if(MPU6050_testConnection() == 0)
//	{
//		printf("\n error \n\r");
//		while(1);//
//	}
	
	printf("\n OK \n\r");
	
//	while(1)   //加速度计矫正
//	{
//		MPU6050_Read();	
//		MPU6050_Dataanl();
//	}
	
	PID_Init(&PID_Para); //用于将(&PID_Para)->Kp、(&PID_Para)->Ti、(&PID_Para)->Td清零，与PID控制程序无关
  PID_Init(&PID_Roll_Struct);	 //初始化PID参数
  PID_Init(&PID_Pitch_Struct);
  PID_Init(&PID_Z_Struct);	
	
	PID_Init(&PID_Pitch_Rate_Struct);     
  PID_Init(&PID_Roll_Rate_Struct);      
	
//	HAL_TIM_Base_Start_IT(&htim6);      //Time6可开启中断，开启中断表明用于电机响应记录，不开中断用于计算采样时间
	HAL_TIM_Base_Start(&htim10);          //Time10用于记录PID时间
	HAL_TIM_Base_Start_IT(&htim7);        //Time7用于数据发送管理,及程序时间管理
//	HAL_TIM_Base_Start_IT(&htim11);     //Time11用于产生周期性力
  HAL_TIM_Base_Start(&htim13);
	
	HAL_TIM_Base_Start(&htim10);          //Time10用于记录角度PID时间
	HAL_TIM_Base_Start(&htim14);          //Time14用于记录角速度PID时间
	
	HAL_TIM_Base_Start(&htim8);          //Time8用于期望值改变
//	HAL_TIM_Base_Start(&htim5);
	
	workstatus = OnState;
	
////	PID_Roll_Struct.Kp=0.8;
	PID_Roll_Struct.Kp=0;
//	
	PID_Pitch_Struct.Kp=0;
	
	PID_Pitch_Rate_Struct.Kp = 0;
	PID_Pitch_Rate_Struct.Ti = 0;
	PID_Pitch_Rate_Struct.Td = 0;
	
	PID_Roll_Rate_Struct.Kp = 0;
	PID_Roll_Rate_Struct.Ti = 0;
	PID_Roll_Rate_Struct.Td = 0;

//	PID_Roll_Struct.Kp=0.8;
//	PID_Roll_Struct.Kp=0;
//	
//	PID_Pitch_Struct.Kp=0;
//	
//	PID_Pitch_Rate_Struct.Kp = 1.3;
//	PID_Pitch_Rate_Struct.Ti = 0;
//	PID_Pitch_Rate_Struct.Td = 0.03;
//	
//	PID_Roll_Rate_Struct.Kp = 1.1;
//	PID_Roll_Rate_Struct.Ti = 0;
//	PID_Roll_Rate_Struct.Td = 0.03;
	
//	float temp=0;

   Mode = 1;

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		MPU_Get_Gyroscope(&gx, &gy, &gz);
//		Read_L3G4200D();
		
//		printf("\n %f \n\r", Get_Sampletime());

		Send_Data(); 
//		printf("\n m=%f \n\t", Motor_Angle_X);
		
		
//		printf("\n %d \n\r", TIM8->CNT);
		
		if(Hz57 == 1)
		{
			Hz57 = 0;
			CtrlAttiAng();
		}
		
		if(Hz167 == 1)
		{
			Hz167 = 0;
			CtrlAttiRate();
		}
//		

 
		
//		MOTOR_CAL();
		
		
		if((workstatus == OnState) && (state == ENABLE))
		{
			TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
			
			
//			pendulum_control_EPeriod();
//			  pendulum_control_EAmplitude();
			
//			Expect_Change(&PID_Pitch_Struct, 20, 2, 200);

			
//			TIM1_Motor_PWMOutput(1000, 1000, 1000, 1500);

		}
		else
		{
//			__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
			
			TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
//			HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
		}

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(void)
{
	
	
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
