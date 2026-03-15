/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);
   
#define START_SEND_RQ_WRITE     (0x01)
#define START_SEND_RQ_READ      (0x02)
#define START_SEND_RESPONSE     (0x03)

#define FLASH_CODE_WORD    0x55CC82AA
#define FLASH_TEXT_LEN     46

#pragma pack(push,1)
struct sFLASH                // sAddressFLASH: 128 Bytes R/W
{
   uint32_t codeWord;            // 0x55CC82AA              4B
   uint8_t version;              // device version          1B
   uint8_t devNumb;              // device number           1B
   uint8_t subNet;               // sub net number          1B
   uint8_t locNet;               // local net number        1B
   uint8_t devType;              //                         1B
   uint8_t uartSET;              // UART settings           1B
   uint8_t relayTime;            // relay time ON (sec)     1B
   uint8_t respDelay;            // response delay (msec)   1B
   uint16_t crc16;               // crc16                   2B
   char text[FLASH_TEXT_LEN];    //                         100B
};    // 124 Byte
#pragma pack(pop)

class CFlash
{
public:
   CFlash();
   ~CFlash();
   void onInit(void);
   void onWriteByte(uint16_t regAddr, uint8_t Data);
   uint8_t onReceiveByte(uint16_t regAddr);
   uint8_t onGetDevVers(void);
   uint8_t onGetDevNumb(void);
   uint8_t onGetSubNet(void);
   uint8_t onGetLocNet(void);
   uint8_t onGetDevType(void);
   uint8_t onGetSetUART(void);
   uint8_t onGetRelayTime(void);
   uint8_t onGetRespDelay(void);
   uint16_t onGetCRC16(void);
   char *onGetText(void);
//   uint32_t onGetSrvIPA(void);
   char onCheckWritten(void);
   void onWriteProtect(char flg);
   

protected:
private:
   sFLASH m_flash;
   uint8_t *pData;
   uint8_t m_len;
   char m_flag;
};

class CBuffer
{
public:
   explicit CBuffer(uint16_t packLen, uint8_t packCnt);
   ~CBuffer();
   void onWrite(uint8_t *data, uint16_t len);
   bool onRead(uint8_t *data, uint16_t &len);
   bool onCheck(void);

protected:
private:
   uint8_t **m_Buff;
   uint16_t *m_Len;
   const uint16_t m_lenPack;
   const uint8_t m_cntPack;
   uint8_t m_cntr;
   uint8_t m_wrtr;
   uint8_t m_read;
};

//#define EEPROM_INIT
#ifdef EEPROM_INIT
void WriteEEPROM(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

