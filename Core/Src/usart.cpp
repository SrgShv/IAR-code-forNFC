/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include "stdio.h"


UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/**
settings:
         USART:
            Mode: DMA Circular
            UART global interrupt: DISABLE
         DMA:
            Circular
            Memory Increment ENABLE
            Peripheral Increment DISABLE

buffer:  __attribute__((aligned(32)))
         uint8_t uart1_dma[1024];

start:   HAL_UART_Receive_DMA(&huart2, buffer, 1024);
         __IO uint32_t NDTR;
         DMA1_Stream5->NDTR
*/

//extern CByteBuff *pBuffMB;
extern CPortM *pPortMB;
extern CBuffUART *pBuffUART;
//extern bool checkBusyRS485(void);
//extern CTimeOut *pTimeActiveRX;
//extern CTimeOut *pTimeEnRS485;
extern void onStartTimer2(uint32_t time);

#define USART_DMA_BFF_LEN     64
uint8_t *pBuffRX_MA = 0;
uint8_t *pBuffRX_MB = 0;
volatile bool USART_RXA = false;
volatile bool USART_RXB = false;

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = onGetUART_BPS();
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   if(uartHandle->Instance==USART2)
   {
      /* DMA controller clock enable */
      __HAL_RCC_DMA1_CLK_ENABLE();

//      /* DMA interrupt init */
//      /* DMA1_Stream5_IRQn interrupt configuration */
//      HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
//      HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//      /* DMA1_Stream6_IRQn interrupt configuration */
//      HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
//      HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

      /* USART2 clock enable */
      __HAL_RCC_USART2_CLK_ENABLE();

      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**USART2 GPIO Configuration
      PA2     ------> USART2_TX
      PA3     ------> USART2_RX
      */
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* USART2 DMA Init */
      /* USART2_TX Init */
      hdma_usart2_tx.Instance = DMA1_Stream6;
      hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
      hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
      hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
      hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      hdma_usart2_tx.Init.Mode = DMA_NORMAL;
      hdma_usart2_tx.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
      {
         Error_Handler();
      }

      __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);
      //__HAL_LINKDMA(&huart2,hdmatx,hdma_usart2_tx);

      /* USART2_RX Init */
      hdma_usart2_rx.Instance = DMA1_Stream5;
      hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
      hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
      hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
      hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
      hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
      {
         Error_Handler();
      }

      __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

      /* DMA interrupt init */
      /* DMA1_Stream5_IRQn interrupt configuration */
      ///HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 1);
      ///HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
      /* DMA1_Stream6_IRQn interrupt configuration */
      HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 2);
      HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

      /* USART2 interrupt Init */
      HAL_NVIC_SetPriority(USART2_IRQn, 1, 3);
      HAL_NVIC_EnableIRQ(USART2_IRQn);
      
      //__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);

      ///__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);        // Активує переривання для помилок
      ///__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);       // Для отримання даних

      ///__HAL_DMA_ENABLE_IT(&hdma_usart2_rx, DMA_IT_TE);   // Помилка передачі
      ///__HAL_DMA_ENABLE_IT(&hdma_usart2_rx, DMA_IT_HT);   // Половинне завершення
      ///__HAL_DMA_ENABLE_IT(&hdma_usart2_rx, DMA_IT_TC);   // Завершення передачі
   }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

/*******************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   //__disable_irq();
   if(huart->Instance == USART2)
   {  /** Modbus RX B */
      USART_RXB = true;
   };
   //__enable_irq();

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {  /** Modbus RX A */
      USART_RXA = true;
      //assert_failed((uint8_t *)__FILE__, __LINE__);
   };
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART1)
   {
   }
   else if(huart->Instance == USART2)
   {
      printf("==TXC\n\r");
      //assert_failed((uint8_t *)__FILE__, __LINE__);
   };
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART1)
   {
   }
   else if(huart->Instance == USART2)
   {
      printf("--TXH\n\r");
      //assert_failed((uint8_t *)__FILE__, __LINE__);
   };
}

// ================ classes =================
//#define USART_DMA_BFF_LEN     256
//uint8_t *pBuffRX_MA = 0;
//uint8_t *pBuffRX_MB = 0;

/** ======= main USART COM PORT for RS485 =======*/
CPortM::CPortM() :
   m_buffTX(0),
   m_buffRX(0),
   m_RxPackLen(USART_DMA_BFF_LEN),               // 8 BYTE
   m_delayTX(0),
   lastPos(0),
   buffSize(USART_DMA_BFF_LEN)
{
   m_buffTX = new uint8_t[USART_DMA_BFF_LEN];      // 64 BYTE
   m_buffRX = new uint8_t[USART_DMA_BFF_LEN];      // 64 BYTE
   pBuffRX_MA = (uint8_t *)&(m_buffRX[0]);
   pBuffRX_MB = (uint8_t *)&(m_buffRX[m_RxPackLen]);
   //sz = USART_DMA_BFF_LEN;
}

CPortM::~CPortM()
{
    delete [] m_buffTX;
    delete [] m_buffRX;
}

void CPortM::onInit(void)
{
   dma = DMA1_Stream5;
   huart2.Instance = USART2;
   huart2.Init.BaudRate = onGetUART_BPS();
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      //assert_failed((uint8_t *)__FILE__, __LINE__);
   };
}
//#define MIFAREDEBUG
void CPortM::onSend(uint8_t *data, uint16_t len)
{
   if (huart2.gState != HAL_UART_STATE_READY)
   {
       // НЕ запускати DMA
      printf("++TX DMA NOT READY!!!\r\n");
   }
   else
   {
      m_delayTX = ((uint32_t)len * 1200000) / onGetUART_BPS();
      onEnableTxRS485(m_delayTX);
      for(uint16_t i=0; i<len; i++)
      {
         m_buffTX[i] = data[i];
      };
      //huart2.gState = HAL_UART_STATE_READY;
      if(HAL_UART_Transmit_DMA(&huart2, m_buffTX, len))
      {
         printf("++TX DMA ERROR\r\n");
      };

#ifdef MIFAREDEBUG
      printf("ModbusTX: ");
      for (int i = 0; i < len; i++)
      {
         printf("0x%02X, ", (int)data[i]);
      };
      printf("\r\n");
#endif
   };
}

bool CPortM::onRead(uint8_t *data, uint16_t &len)
{
   ///if(HAL_UART_Receive(&huart2, m_buffRX, len, 100) == HAL_OK) return true;
   ///return false;
   uint16_t cnt = 0;
   uint16_t nextPos = buffSize - DMA1_Stream5->NDTR;
   //printf("NDTR:%d\n", (int)DMA1_Stream5->NDTR);
   if(nextPos == lastPos) 
   {
      len = 0;
      return false;
   };

   if(nextPos > lastPos)
   {
      cnt = 0;
      len = nextPos - lastPos;
      for(int16_t i=lastPos; i<(lastPos+len); i++)
      {
         data[cnt++] = m_buffRX[i];
      };
      lastPos = nextPos;    
      return true;
   };
   
   if(nextPos < lastPos)
   {
      cnt = 0;
      len = buffSize + nextPos - lastPos;
      for(int16_t i=lastPos; i<(lastPos+len); i++)
      {
         if(i >= buffSize) i = 0;
         data[cnt++] = m_buffRX[i];
      };
      lastPos = nextPos;
      return true;
   };
}

void CPortM::onSetRX(uint16_t RxPackLen)
{
   USART2->SR &= ~(0x0001<<5);
   HAL_UART_Receive_DMA(&huart2, m_buffRX, RxPackLen);
}

uint8_t tempR = 0;
void CPortM::onClearFlgRX(void)
{

   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart2);
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)) __HAL_UART_CLEAR_FEFLAG(&huart2);
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE)) __HAL_UART_CLEAR_PEFLAG(&huart2);
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
   {
      while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) tempR = huart2.Instance->DR;
   };

//   HAL_UART_DMAStop(&huart2);
//   __HAL_DMA_DISABLE(&hdma_usart2_rx);
   __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TCIF0_4);
   __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_HTIF0_4);
   
   this->onSetRX(USART_RX_BUFFER_SIZE);
}

void printFlagsUSART(void)
{
   uint32_t tempR1 = 0;
   tempR1 = huart2.Instance->SR;
   printf("SR : ");
   onShowREG32(tempR1);
   tempR1 = huart2.Instance->CR1;
   printf("CR1: ");
   onShowREG32(tempR1);
   tempR1 = huart2.Instance->CR2;
   printf("CR2: ");
   onShowREG32(tempR1);
   tempR1 = huart2.Instance->CR3;
   printf("CR3: ");
   onShowREG32(tempR1);

   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))             printf("Clear flag ORE\n\r");
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))              printf("Clear flag FE\n\r");
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE))              printf("Clear flag PE\n\r");
   if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))            printf("Clear flag RXNE\n\r");
   if(__HAL_DMA_GET_FLAG(&hdma_usart2_rx, DMA_FLAG_TCIF0_4))   printf("flag DMA TCIF0_4\n\r");
   if(__HAL_DMA_GET_FLAG(&hdma_usart2_rx, DMA_FLAG_HTIF0_4))   printf("flag DMA HTIF0_4\n\r");
                                                               printf("------------------\r\n");
}

void CPortM::onDeInit(void)
{
   /* USART2 interrupt Deinit */
   HAL_NVIC_DisableIRQ(USART2_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);

   /* Peripheral clock disable */
   __HAL_RCC_USART2_CLK_DISABLE();
   HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

   /* USART2 DMA DeInit */
   HAL_DMA_DeInit(huart2.hdmatx);
   HAL_DMA_DeInit(huart2.hdmarx);
}

uint16_t CPortM::onGetRxLen(void)
{
   return m_RxPackLen;
}

//==============================

CByteBuff::CByteBuff(uint16_t len) :
   m_len(0),
   m_maxSz(len),
   m_buffRX(0)
{
   m_buffRX = new uint8_t[m_maxSz];
}

CByteBuff::~CByteBuff()
{
   if(m_buffRX != 0) delete [] m_buffRX;
}

void CByteBuff::onAddData(uint8_t *data, uint16_t len)
{
   for(uint16_t i=0; i<len; i++)
   {
      m_buffRX[m_len++] = data[i];
      if(m_len >= m_maxSz) break;
   };
}

void CByteBuff::onGetData(uint8_t *data, uint16_t len)
{
   if(len <= m_maxSz)
   {
      m_len = len;
      for(uint16_t i=0; i<m_len; i++)
      {
         m_buffRX[i] = data[i];
      };
   };
}

void CByteBuff::onClear()
{
   if(m_len > 0)
   {
      for(uint16_t i=0; i<m_len; i++)
      {
         m_buffRX[i] = 0;
      };
      m_len = 0;
   };
}

bool CByteBuff::onCheck()
{
   if(m_len > 0) return true;
   else return false;
}

void CByteBuff::onCopyRX(uint8_t *data, uint16_t &len)
{
   len = m_len;
   for(uint16_t i=0; i<m_len; i++)
   {
      data[i] = m_buffRX[i];
   };
   m_len = 0;
}

//==========================
/*****************************************************************************/
CBuffUART::CBuffUART(uint16_t packLen, uint8_t packCnt) :
   m_Buff(0),
   m_Len(0),
   m_lenPack(packLen),
   m_cntPack(packCnt),
   m_cntr(0),
   m_wrtr(0),
   m_read(0),
   m_BuffWrite(0)
{
   m_Buff = new uint8_t* [m_cntPack];
   for(uint8_t i=0; i<m_cntPack; i++)
   {
      m_Buff[i] = new uint8_t[m_lenPack];
   };
   m_Len = new uint16_t [m_cntPack];
}

CBuffUART::~CBuffUART()
{
   delete [] m_Buff;
   delete m_Len;
}

bool CBuffUART::onCheck(void)
{
   bool res = false;
   if(m_cntr > 0) res = true;
   return res;
}

void CBuffUART::onClear(void)
{
   m_cntr = 0;
   m_wrtr = 0;
   m_read = 0;
}

void CBuffUART::onWrite(uint8_t *data, uint16_t len)
{
   if(m_cntr < m_cntPack)
   {
      if(len > m_lenPack) len = m_lenPack;
      for(uint16_t i=0; i<len; i++)
      {
         m_Buff[m_wrtr][i] = data[i];
      };
      m_Len[m_wrtr] = len;
      if(++m_wrtr >= m_cntPack) m_wrtr = 0;
      ++m_cntr;
   };
}

bool CBuffUART::onDirectWrite(dPTR &ptr)
{
   if(m_cntr < m_cntPack)
   {
      ptr.data = &(m_Buff[m_wrtr][0]);
      ptr.byteCount = &(m_Len[m_wrtr]);
      if(++m_wrtr >= m_cntPack) m_wrtr = 0;
      ++m_cntr;
      return true;
   };
   return 0;
}

bool CBuffUART::onDirectRead(dPTR &ptr)
{
   if(m_cntr > 0)
   {
      ptr.data = &(m_Buff[m_read][0]);
      ptr.byteCount = &(m_Len[m_read]);
      if(++m_read >= m_cntPack) m_read = 0;
      --m_cntr;
      return true;
   };
   return false;
}

bool CBuffUART::onRead(uint8_t *data, uint16_t &len)
{
   bool res = false;
   if(m_cntr > 0)
   {
      len = m_Len[m_read];
      for(uint16_t i=0; i<len; i++)
      {
          data[i] = m_Buff[m_read][i];
      };
      if(++m_read >= m_cntPack) m_read = 0;
      --m_cntr;
      if(len > 0) res = true;
   }
   else
   {
      len = 0;
   };
   return res;
}

/**----------------------------------------------------**/
//#define USART_ERROR_PRINT
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {
#ifdef USART_ERROR_PRINT
      printf("USART Error!!!\r\n");
      printFlagsUSART();
#endif // USART_ERROR_PRINT
      pBuffUART->onClear();
      pPortMB->onClearFlgRX();
   };
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {
#ifdef USART_ERROR_PRINT
      printf("USART AbortCplt!!!\r\n");
      printFlagsUSART();
#endif // USART_ERROR_PRINT
   };
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {
#ifdef USART_ERROR_PRINT
      printf("USART AbortTX!!!\r\n");
      printFlagsUSART();
#endif // USART_ERROR_PRINT
   };
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {
#ifdef USART_ERROR_PRINT
      printf("USART AbortRX!!!\r\n");
      printFlagsUSART();
#endif // USART_ERROR_PRINT
   };
}
