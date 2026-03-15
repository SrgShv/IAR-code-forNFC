/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EnableTx485_GPIO_Port, EnableTx485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay_Pin|GreenLED_Pin|RedLED_Pin|EnableWP_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EnableTx485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EnableTx485_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = Relay_Pin|GreenLED_Pin|RedLED_Pin|EnableWP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void onSetChipSel(bool flg)
{
   if(flg == true) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void onEnablePinTxRS485(bool flg)      // GPIOC, PIN_2
{
   if(flg == true) HAL_GPIO_WritePin(EnableTx485_GPIO_Port, EnableTx485_Pin, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(EnableTx485_GPIO_Port, EnableTx485_Pin, GPIO_PIN_RESET);
}

void onSetRELAY(bool flg)              // GPIOB, PIN_1
{
   if(flg == true) HAL_GPIO_WritePin(GPIOB, Relay_Pin, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(GPIOB, Relay_Pin, GPIO_PIN_RESET);
}

void onSetLedRED(bool flg)             // GPIOB, PIN_12
{
   if(flg == true) HAL_GPIO_WritePin(GPIOB, RedLED_Pin, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(GPIOB, RedLED_Pin, GPIO_PIN_RESET);
}

void onSetLedGREEN(bool flg)           // GPIOB, PIN_13
{
   if(flg == true) HAL_GPIO_WritePin(GPIOB, GreenLED_Pin, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(GPIOB, GreenLED_Pin, GPIO_PIN_RESET);
}

bool isIRQ(void)
{
	// IRQ is low when data is ready to be received
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5))
	{
		return false;
	}
	return true;
}
