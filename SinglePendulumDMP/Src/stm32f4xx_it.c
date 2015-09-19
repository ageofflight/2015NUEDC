/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    15/08/2015 22:16:36
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* USER CODE BEGIN 0 */

#include "data_transfer.h"

uint16_t UART_RX_MAXLEN = 100;
uint8_t uart2_RX_data[100] = {0};

FlagStatus Receive=RESET;   //设立标志用于判断串口是否处于接收状态,Reset=发送，Set=接收

uint32_t rx_len;

uint8_t RxBuffer[50];
uint8_t RxState = 0;
uint8_t RxBufferNum = 0;
uint8_t RxBufferCnt = 0;
uint8_t RxLen = 0;

/* 引用自PID_Control.c */
extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;
/*--------------------*/

/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles EXTI Line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/**
* @brief This function handles DMA1 Stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles TIM1 Trigger and Commutation interrupts and TIM11 global interrupt.
*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
* @brief This function handles TIM8 Update interrupt and TIM13 global interrupt.
*/
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	
//		uint32_t tmp1 = 0, tmp2 = 0,i,rx_len;
	uint32_t tmp1 = 0, tmp2 = 0,i;
//	
////	printf("\n IRQ \n\r");

	/* UART RXNE interrupt occurred ------------------------------------*/
//	tmp1 = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE);
//	tmp2 = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE);  

//  
//	
//  if((tmp1 != RESET) && (tmp2 != RESET))
//	{
//		uint8_t com_data = huart2.Instance->DR;
//		static uint8_t _data_len = 0, _data_cnt = 0;
//		if(RxState==0&&com_data==0xAA)
//		{
//			RxState=1;
//			RxBuffer[0]=com_data;
//		}
//		else if(RxState==1&&com_data==0xAF)
//		{
//			RxState=2;
//			RxBuffer[1]=com_data;
//		}
//		else if(RxState==2&&com_data>0&&com_data<0XF1)
//		{
//			RxState=3;
//			RxBuffer[2]=com_data;
//		}
//		else if(RxState==3&&com_data<50)
//		{
//			RxState = 4;
//			RxBuffer[3]=com_data;
//			_data_len = com_data;
//			_data_cnt = 0;
//		}
//		else if(RxState==4&&_data_len>0)
//		{
//			_data_len--;
//			RxBuffer[4+_data_cnt++]=com_data;
//			if(_data_len==0)
//				RxState = 5;
//		}
//		else if(RxState==5)
//		{
//			RxState = 0;
//			RxBuffer[4+_data_cnt]=com_data;
//			Data_Receive_Anl(RxBuffer,_data_cnt+5);
//		}
//		else
//			RxState = 0;
//	}

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles DMA1 Stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles EXTI Line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI Line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	
	static uint16_t temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	uint16_t Rc_Channel1, Rc_Channel2;
//	uint16_t Rc_Channel3, Rc_Channel4;
	
	if(TIM3->SR & TIM_IT_CC1) 
	{
		TIM3->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->SR = ~TIM_FLAG_CC1OF;
		if(GPIOB->IDR & GPIO_PIN_4)
		{
			temp_cnt1 = TIM3->CCR1;
		}
		else
		{
			temp_cnt1_2 = TIM3->CCR1;
			if(temp_cnt1_2>=temp_cnt1)
				Rc_Channel1= temp_cnt1_2-temp_cnt1;
			else
				Rc_Channel1 = 0xffff-temp_cnt1+temp_cnt1_2;
			
			if(Rc_Channel1>1500)
			{
				MOTOR3=2*(Rc_Channel1+20)-2000;
				MOTOR4=1000;
			}
			else
			{
				MOTOR4=4000-2*(Rc_Channel1+20);
				MOTOR3=1000;
			}
//			printf("\n 4=%d\n\r", Rc_Channel1+20);
		}
	}
	if(TIM3->SR & TIM_IT_CC2) 
	{
		TIM3->SR = ~TIM_IT_CC2;
		TIM3->SR = ~TIM_FLAG_CC2OF;
		if(GPIOB->IDR & GPIO_PIN_5)
		{
			temp_cnt2 = TIM3->CCR2;
		}
		else
		{
			temp_cnt2_2 = TIM3->CCR2;
			if(temp_cnt2_2>=temp_cnt2)
				Rc_Channel2 = temp_cnt2_2-temp_cnt2;
			else
				Rc_Channel2 = 0xffff-temp_cnt2+temp_cnt2_2;
			
			if(Rc_Channel2>1500)
			{
				MOTOR1=2*(Rc_Channel2+55)-2000;
				MOTOR2=1000;
			}
			else
			{
				MOTOR2=4000-2*(Rc_Channel2+55);
				MOTOR1=1000;
			}
			
//			printf("\n 2=%d\n\r", Rc_Channel2+55);
		}
	}
	if(TIM3->SR & TIM_IT_CC3) 
	{
		TIM3->SR = ~TIM_IT_CC3;
		TIM3->SR = ~TIM_FLAG_CC3OF;
		if(GPIOB->IDR & GPIO_PIN_0)
		{
			temp_cnt3 = TIM3->CCR3;
		}
		else
		{
			temp_cnt3_2 = TIM3->CCR3;
			if(temp_cnt3_2>=temp_cnt3)
				MOTOR3 = temp_cnt3_2-temp_cnt3;
			else
				MOTOR3 = 0xffff-temp_cnt3+temp_cnt3_2;
		}
	}
	if(TIM3->SR & TIM_IT_CC4) 
	{
		TIM3->SR = ~TIM_IT_CC4;
		TIM3->SR = ~TIM_FLAG_CC4OF;
		if(GPIOB->IDR & GPIO_PIN_1)
		{
			temp_cnt4 = TIM3->CCR4;
		}
		else
		{
			temp_cnt4_2 = TIM3->CCR4;
			if(temp_cnt4_2>=temp_cnt4)
				MOTOR4 = temp_cnt4_2-temp_cnt4;
			else
				MOTOR4 = 0xffff-temp_cnt4+temp_cnt4_2;
		}
	}

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles DMA1 Stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles EXTI Line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
