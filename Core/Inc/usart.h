/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#pragma once
#include "main.h"

#define USART_RX_BUFFER_SIZE     64

extern UART_HandleTypeDef huart2;
void MX_USART2_UART_Init(void);

#pragma pack(push,1)
struct dPTR
{
   uint8_t *data;
   uint16_t *byteCount;
   uint8_t status;
};
#pragma pack(pop)

class CPortM
{
public:
   CPortM();
   ~CPortM();

   void onInit(void);
   void onDeInit(void);
   void onClearFlgRX(void);
   void onSend(uint8_t *data, uint16_t len);
   bool onRead(uint8_t *data, uint16_t &len);
   void onSetRX(uint16_t RxPackLen);
   uint16_t onGetRxLen(void);
   void onSetRxBusyFlg(bool flg);
protected:
    uint8_t *m_buffTX;
   uint8_t *m_buffRX;
private:
   uint8_t *m_CrcTX;
   uint8_t *m_CrcRX;
   uint16_t m_RxPackLen;
   uint32_t m_delayTX;

   DMA_Stream_TypeDef* dma;
   uint8_t* buf;
   const uint16_t buffSize;
   uint16_t lastPos;
   uint16_t nextPos;
};

class CByteBuff
{
public:
   explicit CByteBuff(uint16_t len);
   ~CByteBuff();

   void onAddData(uint8_t *data, uint16_t len);
   void onGetData(uint8_t *data, uint16_t len);
   bool onCheck(void);
   void onClear(void);
   void onCopyRX(uint8_t *data, uint16_t &len);

protected:

private:
   uint16_t m_len;
   const uint16_t m_maxSz;
   uint8_t *m_buffRX;
};

class CBuffUART
{
public:
   explicit CBuffUART(uint16_t packLen, uint8_t packCnt);
   ~CBuffUART();
   void onWrite(uint8_t *data, uint16_t len);
   bool onDirectWrite(dPTR &ptr);
   bool onDirectRead(dPTR &ptr);
   bool onRead(uint8_t *data, uint16_t &len);
   bool onCheck(void);
   void onClear(void);

protected:
private:
   uint8_t **m_Buff;
   uint16_t *m_Len;
   const uint16_t m_lenPack;
   const uint8_t m_cntPack;
   uint8_t m_cntr;
   uint8_t m_wrtr;
   uint8_t m_read;
   uint8_t *m_BuffWrite;
};

void onEnableTxRS485(uint32_t delay);
void onShowREG32(uint32_t reg);
void printFlagsUSART(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

