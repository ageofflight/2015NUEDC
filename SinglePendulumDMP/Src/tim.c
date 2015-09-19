/**
  ******************************************************************************
  * File Name          : TIM.c
  * Date               : 15/08/2015 22:16:26
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

#include "MPU6050.h"
#include "data_transfer.h"
#include "motor.h"
#include "PID_Control.h"
#include "pendulumcontrol.h"

uint8_t SystemInitReady = 0;

/* 引用自PENDULUMCONTROL.C */
extern float Amplitude;   //可控摆幅模式下摆幅参数
extern float Radius;      //可控圆摆半径
extern float Radius_Eight;   //八字摆幅度

extern float nDirection;   //任意方向摆角度

extern float nAmplitude;   //任意方向摆幅度

/* ----------------------- */


/* 标志 */
extern  uint8_t Circle_Flag;
extern uint8_t Eight_Flag;
extern uint8_t NDirection_Flag;
extern uint8_t eAmplitude_Flag;
extern uint8_t Freefall_Flag;

extern uint8_t Mode;
/*-----------*/

/* 引用自PID_Control.c */
extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;

extern float Motor_Angle_X;
extern float Motor_Angle_Y;

extern float Pitch;
extern float Roll;
/*--------------------*/

/* 引用自main.c */
extern uint8_t Hz57;
extern uint8_t Hz167;
/* ------------ */

/* 引用自Data_Transfer.c */
extern uint8_t Send_Status,Send_Senser;
/* --------------------- */

extern WorkStatus workstatus;
extern FunctionalState state;

extern SwingDirection Pitch_Direction;
extern SwingDirection Roll_Direction;

void TIM_ICxPolarityConfig(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t TIM_ICPolarity);

unsigned char TIM3CH1_CAPTURE_STA=1;  //边沿捕捉标志位，=1表示捕捉到了上升沿，=0表示捕捉到了下降沿
unsigned char TIM3CH2_CAPTURE_STA=1;
unsigned char TIM3CH3_CAPTURE_STA=1;
unsigned char TIM3CH4_CAPTURE_STA=1;
uint16_t TIM3CH1_Rise, TIM3CH1_Fall,
         TIM3CH2_Rise, TIM3CH2_Fall,
         TIM3CH3_Rise, TIM3CH3_Fall,
         TIM3CH4_Rise, TIM3CH4_Fall,
		     TIM3_T1, TIM3_T2;

//extern WorkStatus workstatus;
//extern FunctionalState state;

//Time1用于电机控制
//Time6可开启中断，开启中断表明用于电机响应记录，不开中断用于计算采样时间
//Time7用于数据发送管理
//Time8用于X轴周期性改变期望
//Time5用于Y轴周期性改变期望
//Time10用于记录角度环PID时间
//Time11用于产生周期性力
//Time14用于记录速度环PID时间
//Time13暂定


/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 21000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 36000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim5.Init.Period = 1800;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

}
/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 12000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 72;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}
/* TIM8 init function */
void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 36000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 1800;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim8.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

}
/* TIM9 init function */
void MX_TIM9_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 72;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim9);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2);

}
/* TIM10 init function */
void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 36000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim10);

}
/* TIM11 init function */
void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 36000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim11);

}
/* TIM13 init function */
void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 72;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim13);

}
/* TIM14 init function */
void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 36000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __TIM1_CLK_ENABLE();
  
    /**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __TIM5_CLK_ENABLE();
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* Peripheral clock enable */
    __TIM6_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __TIM7_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __TIM8_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }
  else if(htim_base->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspInit 0 */

  /* USER CODE END TIM9_MspInit 0 */
    /* Peripheral clock enable */
    __TIM9_CLK_ENABLE();
  
    /**TIM9 GPIO Configuration    
    PE5     ------> TIM9_CH1
    PE6     ------> TIM9_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM9_MspInit 1 */

  /* USER CODE END TIM9_MspInit 1 */
  }
  else if(htim_base->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspInit 0 */

  /* USER CODE END TIM10_MspInit 0 */
    /* Peripheral clock enable */
    __TIM10_CLK_ENABLE();
  /* USER CODE BEGIN TIM10_MspInit 1 */

  /* USER CODE END TIM10_MspInit 1 */
  }
  else if(htim_base->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspInit 0 */

  /* USER CODE END TIM11_MspInit 0 */
    /* Peripheral clock enable */
    __TIM11_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* USER CODE BEGIN TIM11_MspInit 1 */

  /* USER CODE END TIM11_MspInit 1 */
  }
  else if(htim_base->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspInit 0 */

  /* USER CODE END TIM13_MspInit 0 */
    /* Peripheral clock enable */
    __TIM13_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM13_MspInit 1 */

  /* USER CODE END TIM13_MspInit 1 */
  }
  else if(htim_base->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspInit 0 */

  /* USER CODE END TIM14_MspInit 0 */
    /* Peripheral clock enable */
    __TIM14_CLK_ENABLE();
  /* USER CODE BEGIN TIM14_MspInit 1 */

  /* USER CODE END TIM14_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM1_CLK_DISABLE();
  
    /**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14);

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN TIM1:TIM1_TRG_COM_TIM11_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_TRG_COM_TIM11_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn); */
  /* USER CODE END TIM1:TIM1_TRG_COM_TIM11_IRQn disable */

  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM3_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM5_CLK_DISABLE();
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM6_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM7_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM8_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN TIM8:TIM8_UP_TIM13_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM8_UP_TIM13_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn); */
  /* USER CODE END TIM8:TIM8_UP_TIM13_IRQn disable */

  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspDeInit 0 */

  /* USER CODE END TIM9_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM9_CLK_DISABLE();
  
    /**TIM9 GPIO Configuration    
    PE5     ------> TIM9_CH1
    PE6     ------> TIM9_CH2 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN TIM9_MspDeInit 1 */

  /* USER CODE END TIM9_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspDeInit 0 */

  /* USER CODE END TIM10_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM10_CLK_DISABLE();
  /* USER CODE BEGIN TIM10_MspDeInit 1 */

  /* USER CODE END TIM10_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspDeInit 0 */

  /* USER CODE END TIM11_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM11_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN TIM11:TIM1_TRG_COM_TIM11_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_TRG_COM_TIM11_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn); */
  /* USER CODE END TIM11:TIM1_TRG_COM_TIM11_IRQn disable */

  /* USER CODE BEGIN TIM11_MspDeInit 1 */

  /* USER CODE END TIM11_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspDeInit 0 */

  /* USER CODE END TIM13_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM13_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN TIM13:TIM8_UP_TIM13_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM8_UP_TIM13_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn); */
  /* USER CODE END TIM13:TIM8_UP_TIM13_IRQn disable */

  /* USER CODE BEGIN TIM13_MspDeInit 1 */

  /* USER CODE END TIM13_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspDeInit 0 */

  /* USER CODE END TIM14_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM14_CLK_DISABLE();
  /* USER CODE BEGIN TIM14_MspDeInit 1 */

  /* USER CODE END TIM14_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	 if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00)//ch1发生捕获事件
//  {	
////    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);   //清除中断标志位

//	if(TIM3CH1_CAPTURE_STA == 1)	//捕获到上升沿
//	{ 
//	  TIM3CH1_Rise = htim->Instance->CCR1;		           //获取上升沿的数据	  
//	  TIM3CH1_CAPTURE_STA = 0;		                        //转换标志位为下降沿
//	  TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);		//设置为下降沿捕获	  			    			  
//	}
//	else  						    //捕获到下降沿
//	{
//	  TIM3CH1_Fall = htim->Instance->CCR1;      //获取下降沿的数据	  
//	  TIM3CH1_CAPTURE_STA = 1;		//转换标志位为上升沿
//	  if(TIM3CH1_Fall < TIM3CH1_Rise)     //情况1：表示高电平跨过了65535这个特殊值，此时状态为上升沿接近65535，并且下降沿超过了65535从0开始计算，Tim2有溢出
//	  {
//	    TIM3_T1 = 65535;
//	  }
//	  else  //情况2：表示正常情况，上升沿和下降沿都在0-65535之间，且下降沿数值>上升沿数值。
//	  {
//	    TIM3_T1 = 0;
//	  }	
//		PWMInCh1 = TIM3CH1_Fall - TIM3CH1_Rise + TIM3_T1 + 62;  //得到总的高电平时间，值域1000~2000
//		Motor_Angle_X = (PWMInCh1 - 1500.0) /10.0;
//    if(PWMInCh1!=0)
//		{
//			MOTOR4 = PWMInCh1;
//			printf("\n 4=%d\n\r", MOTOR4);
//		}
//		TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);		//设置为下降沿捕获	  			    			  
////		if(SystemInitReady == 1)
////		{
//			HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);			
////		}
//  }		    
//  }
//  			     	    					   	   	   
//  /*---------------------
//    如果捕获到ch2升降舵
//  ---------------------*/		  
//  if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00)//ch2发生捕获事件
//  {	
////    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);   //清除中断标志位

//		if(TIM3CH2_CAPTURE_STA == 1)	//捕获到上升沿
//		{ 
//			TIM3CH2_Rise = htim->Instance->CCR2;		           //获取上升沿的数据	  
//			TIM3CH2_CAPTURE_STA = 0;		                        //转换标志位为下降沿
//			TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);		//设置为下降沿捕获	  			    			  
//		}
//		else  						    //捕获到下降沿
//		{
//			TIM3CH2_Fall = htim->Instance->CCR2;      //获取下降沿的数据	  
//			TIM3CH2_CAPTURE_STA = 1;		//转换标志位为上升沿
//			if(TIM3CH2_Fall < TIM3CH2_Rise)     //情况1：表示高电平跨过了65535这个特殊值，此时状态为上升沿接近65535，并且下降沿超过了65535从0开始计算，Tim2有溢出
//			{
//				TIM3_T2 = 65535;
//			}
//			else  //情况2：表示正常情况，上升沿和下降沿都在0-65535之间，且下降沿数值>上升沿数值。
//			{
//				TIM3_T2 = 0;
//			}	
//			PWMInCh2 = TIM3CH2_Fall - TIM3CH2_Rise + TIM3_T2;  //得到总的高电平时间，值域1000~2000
//			if(PWMInCh2!=0)
//			{
//				printf("\nChannel2_Eli=%d\n\r", PWMInCh2);
//			}
//			TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);		//设置为下降沿捕获	  			    			  
//			HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);	
//		}		    
// }
	  
//  /*-------------------
//    如果捕获到ch3油门
//  -------------------*/		  
//  if ((htim->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00)//ch1发生捕获事件
//  {	
////    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);   //清除中断标志位

//	if(TIM3CH3_CAPTURE_STA == 1)	//捕获到上升沿
//	{ 
//	  TIM3CH3_Rise = htim->Instance->CCR3;		           //获取上升沿的数据	  
//	  TIM3CH3_CAPTURE_STA = 0;		                        //转换标志位为下降沿
//	  TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);		//设置为下降沿捕获	  			    			  
//	}
//	else  						    //捕获到下降沿
//	{
//	  TIM3CH3_Fall = htim->Instance->CCR3;      //获取下降沿的数据	  
//	  TIM3CH3_CAPTURE_STA = 1;		//转换标志位为上升沿
//	  if(TIM3CH3_Fall < TIM3CH3_Rise)     //情况1：表示高电平跨过了65535这个特殊值，此时状态为上升沿接近65535，并且下降沿超过了65535从0开始计算，Tim2有溢出
//	  {
//	    TIM3_T = 65535;
//	  }
//	  else  //情况2：表示正常情况，上升沿和下降沿都在0-65535之间，且下降沿数值>上升沿数值。
//	  {
//	    TIM3_T = 0;
//	  }	
//      PWMInCh3 = TIM3CH3_Fall - TIM3CH3_Rise + TIM3_T;  //得到总的高电平时间，值域1000~2000
//	    TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);		//设置为下降沿捕获	  			    			  
//	}		    
//  }

//  /*---------------------
//    如果捕获到ch4方向舵
//  ---------------------*/		  
//  if ((htim->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00)//ch4发生捕获事件
//  {	
////    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);   //清除中断标志位

//	if(TIM3CH4_CAPTURE_STA == 1)	//捕获到上升沿
//	{ 
//	  TIM3CH4_Rise = htim->Instance->CCR4;		           //获取上升沿的数据	  
//	  TIM3CH4_CAPTURE_STA = 0;		                        //转换标志位为下降沿
//	  TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);		//设置为下降沿捕获	  			    			  
//	}
//	else  						    //捕获到下降沿
//	{
//	  TIM3CH4_Fall = htim->Instance->CCR4;      //获取下降沿的数据	  
//	  TIM3CH4_CAPTURE_STA = 1;		//转换标志位为上升沿
//	  if(TIM3CH4_Fall < TIM3CH4_Rise)     //情况1：表示高电平跨过了65535这个特殊值，此时状态为上升沿接近65535，并且下降沿超过了65535从0开始计算，Tim2有溢出
//	  {
//	    TIM3_T = 65535;
//	  }
//	  else  //情况2：表示正常情况，上升沿和下降沿都在0-65535之间，且下降沿数值>上升沿数值。
//	  {
//	    TIM3_T = 0;
//	  }	
//      PWMInCh4 = TIM3CH4_Fall - TIM3CH4_Rise + TIM3_T;  //得到总的高电平时间，值域1000~2000
//	    TIM_ICxPolarityConfig( &htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);		//设置为下降沿捕获	  			    			  
//	}		    
//  }
}

void TIM_ICxPolarityConfig(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t TIM_ICPolarity)
{
	if(Channel == TIM_CHANNEL_1)
	{
		uint32_t tmpccer = 0;
		tmpccer = htim->Instance->CCER;
		
		/* Set or Reset the CC1P Bit */
		tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC1P);
		tmpccer |= TIM_ICPolarity;
		
		/* Write to TIMx CCMR1 and CCER registers */
		htim->Instance->CCER = tmpccer;
	}
	if(Channel == TIM_CHANNEL_2)
	{
		uint32_t tmpccer = 0;
		tmpccer = htim->Instance->CCER;
		
		/* Set or Reset the CC1P Bit */
		tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC2P);
		tmpccer |= TIM_ICPolarity;
		
		/* Write to TIMx CCMR1 and CCER registers */
		htim->Instance->CCER = tmpccer;
	}
	if(Channel == TIM_CHANNEL_3)
	{
		uint32_t tmpccer = 0;
		tmpccer = htim->Instance->CCER;
		
		/* Set or Reset the CC1P Bit */
		tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC3P);
		tmpccer |= TIM_ICPolarity;
		
		/* Write to TIMx CCMR1 and CCER registers */
		htim->Instance->CCER = tmpccer;
	}
	if(Channel == TIM_CHANNEL_4)
	{
		uint32_t tmpccer = 0;
		tmpccer = htim->Instance->CCER;
		
		/* Set or Reset the CC1P Bit */
		tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC4P);
		tmpccer |= TIM_ICPolarity;
		
		/* Write to TIMx CCMR1 and CCER registers */
		htim->Instance->CCER = tmpccer;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t ms1=0, ms2=0, ms3=0;
	static uint16_t temp=0;
	
//	static uint16_t Motor_Step=3;
	
	if(htim->Instance == TIM7)
	{
//		Mode = 2;
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);  //2ms更新升降舵值
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);  //2ms更新油门值
		Hz167 = 1;
	 switch(Mode)
		{
			case Still:
			{
				pendulum_control_Still();
				eAmplitude_Flag=0;
				NDirection_Flag=0;
				Circle_Flag=0;
				Eight_Flag=0;
				Freefall_Flag=0;
			}
		case FreeFall:
			 {
				 pendulum_control_freefall(25);
				 eAmplitude_Flag=0;
				 NDirection_Flag=0;
				 Circle_Flag=0;
				 Eight_Flag=0;
			 }
			 break;
		case eAmplitude:
			{
				 pendulum_control_eAmplitude_swing(Amplitude);
				 Freefall_Flag=0;
				 NDirection_Flag=0;
				 Circle_Flag=0;
				 Eight_Flag=0;
			 }
			 break;
		 case NDirection:
			{
				 pendulum_control_nDirection_swing(nDirection, nAmplitude);
				 Freefall_Flag=0;
				 eAmplitude_Flag=0;
				 Circle_Flag=0;
				 Eight_Flag=0;
			 }
			 break;
		 case Circle:
			{
				 pendulum_control_circle(Radius);
				 Freefall_Flag=0;
				 eAmplitude_Flag=0;
				 NDirection_Flag=0;
				 Eight_Flag=0;
				 
			 }
			 break;
		 case Eight:
			{
				 pendulum_control_eight(Radius_Eight);
				 Freefall_Flag=0;
				 eAmplitude_Flag=0;
				 NDirection_Flag=0;
				 Circle_Flag=0;
			 }
			 break;
			 default:
			 
			 break;
		 }
			 
//		pendulum_control_nDirection_swing(nDirection, nAmplitude);

//		pendulum_control_eight(Radius_Eight);
//		pendulum_control_circle(Radius);
//		pendulum_control_eAmplitude_swing(Amplitude);
////		//自由摆
////		Motor_Angle_X=4*((float)(TIM8->CNT/180) - 4.5f);
		
		temp++;
		if(temp >= 4)
		{
			
//			Expect_Change_TriAngle(&PID_Pitch_Struct, 15, 1);

//			Expect_Change(&PID_Pitch_Struct, 20, 1.0, 1);
//			Expext_Change_Circle(&PID_Pitch_Struct, &PID_Roll_Struct, 40, 1.0, 1);
			temp=0;
		}
			
		if(htim == (&htim7))
		{
			ms1++;
			ms2++;
			ms3++;
		}
		
		if(ms1==1&&ms2!=2)
		{
			ms1=0;
			Send_Senser=1;
		}
		else
		{
			ms1=0;
			ms2=0;
		  Send_Status=1;
		}
		if(ms3 == 4)
		{
			Hz57 = 1;
			ms3 = 0;
		}
  }
//	if(htim->Instance == TIM6)
//	{
//		MOTOR3+=Motor_Step;
////		TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
//	}
	
//	if((htim->Instance == TIM11) && (workstatus == OnState) && (state == ENABLE))
//	{
//		Pitch_Direction=~Pitch_Direction;
//		pendulum_control();
//	}
	
//	if(htim->Instance ==TIM11)
//	{
//		if(Pitch_Direction==PlusDirection)
//		{
//			MOTOR3=1000;
//			MOTOR4=1200;
//		}
//		else
//		{
//			MOTOR3=1200;
//			MOTOR4=1000;
//		}
//		Pitch_Direction=~Pitch_Direction;
//	}

//	
//	if(ms1==3)
//	{
//		ms1 = 0;
//		MPU6050_Read();	
//		MPU6050_Dataanl();
//		Data_Send_Senser();
//		Data_Send_Status();
//		MOTOR_CAL();
//	}
//	
//	if(ms2==4)
//	{
//		ms2 = 0;
//		if((workstatus == OnState) && (state == ENABLE))
//		{
//			TIM1_Motor_PWMOutput(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
////			printf("\n Control \n\r");
//		}
//		else
//		{
//			TIM1_Motor_PWMOutput(1000, 1000, 1000, 1000);
//		}
//	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
