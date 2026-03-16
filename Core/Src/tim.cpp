/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "stdio.h"

/* USER CODE BEGIN 0 */
extern void tickTimer(void);
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

/* TIM2 init function */
void MX_TIM_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};

   htim2.Instance = TIM2;
   htim2.Init.Prescaler = 479;   // (48 MHz / (479 + 1) = 100 kHz)
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.Period = 9;        // (100 kHz / (9 + 1) = 10 kHz => 0.1 msec)
   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
   {
      Error_Handler();
   };
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
   {
      Error_Handler();
   };
   if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
   {
      Error_Handler();
   };
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
   {
      Error_Handler();
   };
   //HAL_TIM_Base_Stop(&htim2);
   //HAL_TIM_Base_Start_IT(&htim2);
   //HAL_NVIC_EnableIRQ(TIM2_IRQn);
   //HAL_TIM_Base_Start(&htim2);
}

void onStartTimer2(uint32_t time)
{
   HAL_TIM_Base_Stop(&htim2);
   TIM2->ARR = time;
   HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim2);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
   if(tim_baseHandle->Instance==TIM2)
   {
      __HAL_RCC_TIM2_CLK_ENABLE();

      /* TIM2 interrupt Init */
      HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
      HAL_NVIC_EnableIRQ(TIM2_IRQn);
   };
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
   if(tim_baseHandle->Instance==TIM2)
   {
      /* Peripheral clock disable */
      __HAL_RCC_TIM3_CLK_DISABLE();

      /* TIM3 interrupt Deinit */
      HAL_NVIC_DisableIRQ(TIM2_IRQn);
   }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim == &htim2) tickTimer2();
   //if(htim == &htim2) printf("t2\n");tickTimer2();
   //if(htim == &htim3) tickTimer();
}

void HAL_SYSTICK_Callback(void)
{
   tickTimer();
}

/*********************************************************/
CTimeOut::CTimeOut() :
   m_cntT(0),
   m_maxT(1000),
   m_ID(0),
   m_flag(false),
   m_timeOutFlag(false)
{

}

CTimeOut::~CTimeOut()
{

}

void CTimeOut::onTick(void)
{
   if(m_flag)
   {
      if(++m_cntT >= m_maxT)
      {
         m_timeOutFlag = true;
         m_flag = false;
         m_ID = 0;
      };
   };
}

void CTimeOut::onStart(uint32_t tim, uint32_t ID)
{
   //SWO_PrintString("TIMEOUT START\n");
   m_timeOutFlag = false;
   m_maxT = tim;
   m_cntT = 0;
   m_ID = ID;
   m_flag = true;
}

void CTimeOut::onStop(void)
{
   m_flag = false;
   m_timeOutFlag = false;
   //SWO_PrintString("TIMEOUT STOP\n");
}

bool CTimeOut::onIsTimeOut(void)
{
   if(m_timeOutFlag == true) return true;
   else return false;
}

bool CTimeOut::onIsActive(uint32_t ID)
{
   if(m_ID == ID) return m_flag;
   return false;
//   bool res = false;
//   if((m_flag == true) && (m_ID == ID))
//   {
//      res = true;
//   };
//   return res;
}

bool CTimeOut::onIsTimerOFF(void)
{
   if(m_flag == false) return true;
   return false;
}


