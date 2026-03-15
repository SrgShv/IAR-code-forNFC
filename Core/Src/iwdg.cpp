/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
#include "iwdg.h"


IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;  /* (32000 Hz / 32 = 1000 Hz) */
  hiwdg.Init.Reload = 200;                  /* (200 / 1000 Hz = 0.2 sec) */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

void resetWDT(void)
{
   HAL_IWDG_Refresh(&hiwdg);
}
