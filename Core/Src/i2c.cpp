/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

#define EEPROM_ADDRESS  0xA0
#define EEPROM_HW_ADDRESS  0xA0
#define TRUE         1
#define FALSE        0

#define EEPROM_MAXPKT         32              //(page size)
#define EEPROM_WRITE          10              //time to wait in ms
#define EEPROM_TIMEOUT        5*EEPROM_WRITE  //timeout while writing
#define EEPROM_SECTIONSIZE	   64

I2C_HandleTypeDef hi2c1;

static const uint16_t crc16Table [] =
{
    0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
    0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
    0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
    0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
    0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
    0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
    0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
    0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
    0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
    0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
    0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
    0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
    0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
    0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
    0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
    0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
    0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
    0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
    0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
    0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
    0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
    0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
    0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
    0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
    0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
    0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
    0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
    0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
    0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
    0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
    0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
    0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
};

uint16_t onCRC16(const uint8_t *nData, uint16_t len)
{
   uint16_t crc = 0xFFFF;
   for (uint16_t i = 0; i < len; i++)
   {
      crc = (uint16_t)((crc << 8) ^ crc16Table[(crc >> 8) ^ nData[i]]);
   };
   return crc;
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/**************************** CFlash ****************************/
char txtr[100];
CFlash::CFlash() :
   m_len(0),
   m_flag(0)
{
   pData = (uint8_t *)&(m_flash);
   m_len = sizeof(m_flash);
}

CFlash::~CFlash()
{
}

void CFlash::onInit(void)
{
   MX_I2C1_Init();
   HAL_Delay(20);
   for(uint8_t i=0; i<m_len; i++)
   {
      pData[i] = this->onReceiveByte(i);
   };
   if(this->onCheckWritten())
   {
      m_flag = 1;
   };
}

void CFlash::onWriteProtect(char flg)
{
   if(flg) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void CFlash::onWriteByte(uint16_t regAddr, uint8_t Data)
{
   HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &Data, 1, 5);
   HAL_Delay(EEPROM_WRITE);
}

uint8_t CFlash::onReceiveByte(uint16_t regAddr)
{
    uint8_t tmp = 0;
    HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 5);
    HAL_Delay(EEPROM_WRITE/2);
    return tmp;
}

char CFlash::onCheckWritten(void)
{
   if(m_flash.codeWord == FLASH_CODE_WORD) return 1;
   else return 0;
}

uint8_t CFlash::onGetDevVers(void)
{
   return m_flash.version;
}

uint8_t CFlash::onGetDevNumb(void)
{
   return m_flash.devNumb;
}

uint8_t CFlash::onGetDevType(void)
{
   return m_flash.devType;
}

uint8_t CFlash::onGetSubNet(void)
{
   return m_flash.subNet;
}

uint8_t CFlash::onGetLocNet(void)
{
   return m_flash.locNet;
}

uint8_t CFlash::onGetSetUART(void)
{
   return m_flash.uartSET;
}

uint8_t CFlash::onGetRelayTime(void)
{
   return m_flash.relayTime;
}

uint8_t CFlash::onGetRespDelay(void)
{
   return m_flash.respDelay;
}

uint16_t CFlash::onGetCRC16(void)
{
   return m_flash.crc16;
}

char * CFlash::onGetText(void)
{
   return m_flash.text;
}

