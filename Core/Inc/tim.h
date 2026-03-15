/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;

//void onStartTimer2(uint32_t time);
void tickTimer2(void);
void MX_TIM_Init(void);

class CTimeOut
{
public:
   CTimeOut();
   ~CTimeOut();
   void onTick(void);
   void onStart(uint32_t tim, uint32_t ID);
   void onStop(void);
   bool onIsActive(uint32_t ID);
   bool onIsTimeOut(void);
   bool onIsTimerOFF(void);

protected:
private:
   uint32_t m_cntT;
   uint32_t m_maxT;
   uint32_t m_ID;
   bool m_flag;
   bool m_timeOutFlag;

};

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

