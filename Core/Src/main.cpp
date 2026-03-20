/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/**
   uint32_t codeWord;            // 0x55AA82CC               4B
   uint8_t version;              // device version           1B
   uint8_t devNumb;              // device number            1B
   uint8_t subNet;               // sub net number           1B
   uint8_t locNet;               // local net number         1B
   uint8_t devType;              // MAC address              1B
   uint8_t uartSET;              // UART settings            1B
   uint8_t relayTime;            // relay time ON (sec)      1B
   uint8_t respDelay;            // response delay (msec)    1B
   uint16_t crc16;               // crc16                    2B
   char text[FLASH_TEXT_LEN];    //                         100B
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "modbus.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "stdio.h"

//extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern uint8_t *pBuffRX_MA;
extern uint8_t *pBuffRX_MB;
extern volatile bool USART_RXA;
extern volatile bool USART_RXB;

static volatile uint16_t step_NFC = 0;
static volatile uint16_t step_2 = 0;
static volatile uint16_t step_MBR03 = 0;
static volatile uint16_t step_MBR06 = 0;
static volatile uint32_t timeRT1 = 0;
static volatile uint32_t timeRT2 = 0;
static volatile uint32_t timeRT3 = 0;
static volatile uint32_t timeRT4 = 0;

static volatile uint32_t SysResetCounter = 0;
static volatile uint32_t MB_CRC_ERR_cnt = 0;


#define  WATCH_DOG_ENABLED
//#define  USE_PRINT_ASSERT
//#define  PN532DEBUG
#define  MIFAREDEBUG

txRegPTR mTxRegPTR;
txRegPTRW mTxRegPTRW;
//   pTimeOutNFC = new CTimeOut();
//   pTimeEnRS485 = new CTimeOut();
//   pTimeOutLEDR = new CTimeOut();
//   pTimeOutLEDG = new CTimeOut();
//   pTimeOutRELAY = new CTimeOut();
//   pTimeCardT = new CTimeOut();
//   pTimeRespMBR = new CTimeOut();
CTimeOut mTimeOutLEDR;
CTimeOut mTimeOutLEDG;
CTimeOut mTimeOutRELAY;
CTimeOut mTimeOutNFC;
CTimeOut mTimeEnRS485;
CTimeOut mTimeCardT;
//CTimeOut mTimeRespMBR;
//CTimeOut mTimeActiveRX;
CTimeOut mTimeCtrlRX;
CTimeOut mTimeResetUSART;

CTimeOut *pTimeOutLEDR = &mTimeOutLEDR;
CTimeOut *pTimeOutLEDG = &mTimeOutLEDG;
CTimeOut *pTimeOutRELAY = &mTimeOutRELAY;
CTimeOut *pTimeOutNFC = &mTimeOutNFC;
CTimeOut *pTimeEnRS485 = &mTimeEnRS485;
CTimeOut *pTimeCardT = &mTimeCardT;
//CTimeOut *pTimeRespMBR = &mTimeRespMBR;
//CTimeOut *pTimeActiveRX = &mTimeActiveRX;

CBuffUART mBuffUART(8, 25);
CBuffUART *pBuffUART = &mBuffUART;

CBuffer mBuffUSB(100, 2);

CBuffer *pBuffUSB = &mBuffUSB;

CByteBuff mBuffMB(USART_RX_BUFFER_SIZE);
CByteBuff *pBuffMB = &mBuffMB;

CFlash mFlashM;
CFlash *pFlashM = &mFlashM;
CPortM mPortMB;
CPortM *pPortMB = &mPortMB;
CModbus mModbus;
CModbus *pModbus = &mModbus;
sFLASH defaultInitStruct;
sFLASH mainInitStruct;
sFLASH tempStruct;
sFLASH *pSettStruct = 0;
uint8_t delayResponseMBR = 1;
bool DefaultSettFlag = true;
uint8_t devNumber = 1;

const uint8_t MB_LEN = 128;
uint16_t MB_REGISTERS[MB_LEN];

#define PN532_PACKBUFFSIZ 64
char pn532_packetbuffer[PN532_PACKBUFFSIZ];

char pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
/**************************************************************************/

/**************************************************************************/
char pn532response_firmwarevers[] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5};

void SystemClock_Config(void);
/**************************************************************************/

/**************************************************************************/
CBuffer::CBuffer(uint16_t packLen, uint8_t packCnt) :
   m_Buff(0),
   m_Len(0),
   m_lenPack(packLen),
   m_cntPack(packCnt),
   m_cntr(0),
   m_wrtr(0),
   m_read(0)
{
   m_Buff = new uint8_t* [m_cntPack];
   for(uint8_t i=0; i<m_cntPack; i++)
   {
      m_Buff[i] = new uint8_t[m_lenPack];
   };
   m_Len = new uint16_t [m_cntPack];
}
/**************************************************************************/

/**************************************************************************/
CBuffer::~CBuffer()
{

}
/**************************************************************************/

/**************************************************************************/
bool CBuffer::onCheck(void)
{
   bool res = false;
   if(m_cntr > 0) res = true;
   return res;
}
/**************************************************************************/

/**************************************************************************/
void CBuffer::onWrite(uint8_t *data, uint16_t len)
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
/**************************************************************************/

/**************************************************************************/
bool CBuffer::onRead(uint8_t *data, uint16_t &len)
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
/**************************************************************************/

/**************************************************************************/
void delay_us(uint32_t us)
{
   uint32_t tcnt = 0;
   while(++tcnt < us*3) ;
}
/**************************************************************************/

/**************************************************************************/
static volatile bool timeFlg3 = false;
volatile uint32_t timeCnt3 = 0;
void tickTimer2(void)
{
   onEnablePinTxRS485(false);
//   printf("T2!!!\r\n");
//   if(timeCnt3 >= 10000)
//   {
//      if(timeFlg3) timeFlg3 = false;
//      else timeFlg3 = true;
//      timeCnt3 = 0;
//      printf("T2\r\n");
//   };
//   ++timeCnt3;
}
/**************************************************************************/

/**************************************************************************/
uint32_t tstCnt1 = 0;
bool sysFlg = false;
bool sysFlgW = true;
void tickTimer(void)
{
   if(sysFlg == false) sysFlg = true;
   else sysFlg = false;

   //pTimeActiveRX->onTick();
   pTimeOutNFC->onTick();
   pTimeEnRS485->onTick();
   pTimeOutLEDR->onTick();
   pTimeOutLEDG->onTick();
   pTimeOutRELAY->onTick();
   pTimeCardT->onTick();
   mTimeCtrlRX.onTick();
   mTimeResetUSART.onTick();
   //if(++SysTimerCounter > 3600000) SystemReset();   // Time reset - 10 min
   //ticChronometr();
}
/**************************************************************************/

/**************************************************************************/
void onEnableTxRS485(uint32_t delay)
{
   onEnablePinTxRS485(true);
   //onStartTimer2(delay);
//   onEnablePinTxRS485(true);
//   pTimeEnRS485->onStart(delay, 2);
}
/**************************************************************************/

/**************************************************************************/
//void onEnableTxRS485(uint32_t delay)
//{
//   onEnablePinTxRS485(true);
//   onStartTimer2(delay);
////   onEnablePinTxRS485(true);
////   pTimeEnRS485->onStart(delay, 2);
//}
/**************************************************************************/

/**************************************************************************/
void handleRxUSB(uint8_t *data, uint16_t len)
{
#ifdef PN532DEBUG
   printf("IRQ USB <=>\nlen = 0x%02X\n", (int)len);
#endif
   pBuffUSB->onWrite(data, len);
}
/**************************************************************************/

/**************************************************************************/
uint8_t txbuff[100];
uint8_t rxbuff[100];
void PN532_Init(void)
{
   if(true != SAMConfig())
   {
#ifdef PN532DEBUG
      printf("Init SAMConfig ERROR!!!\n");
#endif
      return;
   };
   HAL_Delay(500);
#ifdef PN532DEBUG
   printf("Init SAMConfig OK!!!\n");
#endif
}
/**************************************************************************/

/**************************************************************************/
uint8_t trxb[100];
void PN532_Send(uint8_t *data, uint16_t length)
{
    HAL_SPI_Transmit(&hspi1, data, length, 500);
}
/**************************************************************************/

/**************************************************************************/
void PN532_Receive(uint8_t *buffer, uint16_t length)
{
    HAL_SPI_Receive(&hspi1, buffer, length, 500);
}
/**************************************************************************/

/**************************************************************************/
//#define READ_DATA_MIFARE
char ttstr[100];
void readdata(uint8_t *buff, uint8_t n)
{
   onSetChipSel(true);
   uint8_t cmd = PN532_SPI_DATAREAD;   // 0x03
   PN532_Send(&cmd, 1);
   PN532_Receive((uint8_t *)buff, n);
   onSetChipSel(false);

#ifdef READ_DATA_MIFARE
   printf("RX data: ");
   for (uint8_t i = 0; i < n; i++)
   {
      printf("0x%02X, ", (int)buff[i]);
   };
   printf("\r\n");
#endif

}


void readdataSH(uint8_t *buff, uint8_t n)
{
   uint8_t cmd = PN532_SPI_DATAREAD;   // 0x03
   PN532_Send(&cmd, 1);
   PN532_Receive((uint8_t *)buff, n);

#ifdef READ_DATA_MIFARE
   printf("RX data: ");
   for (uint8_t i = 0; i < n; i++)
   {
      printf("0x%02X, ", (int)buff[i]);
   };
   printf("\r\n");
#endif

}
/**************************************************************************/

/**************************************************************************/
#define USB_PARSE_PRINT
uint8_t RxUSB[100];
uint8_t ReadFlash[100];
void ParseRxUSB(void)                        /****<== PARSE - PARSE ==>****/
{
   uint16_t cntr = 0;
   uint16_t len = 0;
   uint8_t flen = 0;
   //mTimeResetUSART.onStart(10000, 10);
   pBuffUSB->onRead(RxUSB, len);
   sFLASH *pFL = (sFLASH *)&(RxUSB[1]);
   if(pFL->codeWord == FLASH_CODE_WORD)
   {
#ifdef USB_PARSE_PRINT
      printf("RxUSB - OK!!!\n");
#endif
      if(RxUSB[0] == START_SEND_RQ_WRITE)       //  write to EEPROM request
      {
         if(pFL->crc16 == onCRC16((uint8_t*)&(pFL->version), 8))
         {
            if(pFlashM != 0)
            {
               pFlashM->onWriteProtect(0);
               for(uint8_t i=1; i<len; i++)
               {
                  pFlashM->onWriteByte(i-1, RxUSB[i]);
               };
               HAL_Delay(100);
               pFlashM->onWriteProtect(1);
               pFlashM->onInit();
               //SystemReset();
               SystemResetD((char *)__FILE__, __LINE__);
            };
         };
      }
      else if(RxUSB[0] == START_SEND_RQ_READ)  // read from EEPROM request
      {
         if(pFlashM != 0)
         {
            ReadFlash[cntr++] = START_SEND_RESPONSE;
            flen = (uint8_t)sizeof(struct sFLASH);
            uint8_t *pTX = (uint8_t *)pSettStruct;
            for(uint8_t i=0; i<flen; i++)
            {
               ReadFlash[cntr++] = pTX[i];
            };
            CDC_Transmit_FS(ReadFlash, flen+1);
         };
      }
      else
      {
      };
   };
   //mTimeResetUSART.onStop();
}
/**************************************************************************/

/**************************************************************************/
uint8_t packet[100];
void writecommand(uint8_t *cmd, uint8_t cmdlen)
{
   onSetChipSel(true);
   HAL_Delay(1);
   packet[0] = 0x01;
	uint8_t checksum = PN532_HOSTTOPN532;
   uint16_t cnt = 1;
	//uint8_t *p = packet;
   packet[cnt++] = PN532_PREAMBLE;			   // 0x00
   packet[cnt++] = PN532_STARTCODE1;		   // 0x00
   packet[cnt++] = PN532_STARTCODE2;		   // 0xFF
   packet[cnt++] = cmdlen + 1;
   packet[cnt++] = ~(cmdlen + 1) + 1;
   packet[cnt++] = PN532_HOSTTOPN532;		   // 0xD4
   for (uint8_t i = 0; i < cmdlen; i++)
   {
      packet[cnt++] = cmd[i];
      checksum += cmd[i];
   };
   packet[cnt++] = ~checksum + 1;
   packet[cnt++] = PN532_POSTAMBLE;			   // 0x00
   PN532_Send(packet, cnt + 1);
   HAL_Delay(1);
   onSetChipSel(false);

#ifdef PN532DEBUG
   printf("write command: ");
   for (int i = 0; i < 8 + cmdlen + 1; i++)
   {
      sprintf(tstr, "0x%02X, ", (int)packet[i]);
      printf(tstr);
   };
   printf("\n");
#endif

}

void writecommandSH(uint8_t *cmd, uint8_t cmdlen)
{
   packet[0] = 0x01;
	uint8_t checksum = PN532_HOSTTOPN532;
   uint16_t cnt = 1;
	//uint8_t *p = packet;
   packet[cnt++] = PN532_PREAMBLE;			   // 0x00
   packet[cnt++] = PN532_STARTCODE1;		   // 0x00
   packet[cnt++] = PN532_STARTCODE2;		   // 0xFF
   packet[cnt++] = cmdlen + 1;
   packet[cnt++] = ~(cmdlen + 1) + 1;
   packet[cnt++] = PN532_HOSTTOPN532;		   // 0xD4
   for (uint8_t i = 0; i < cmdlen; i++)
   {
      packet[cnt++] = cmd[i];
      checksum += cmd[i];
   };
   packet[cnt++] = ~checksum + 1;
   packet[cnt++] = PN532_POSTAMBLE;			   // 0x00
   PN532_Send(packet, cnt + 1);

#ifdef PN532DEBUG
   printf("write command: ");
   for (int i = 0; i < 8 + cmdlen + 1; i++)
   {
      sprintf(tstr, "0x%02X, ", (int)packet[i]);
      printf(tstr);
   };
   printf("\n");
#endif

}
/**************************************************************************/

/**************************************************************************/
bool sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout)
{
   writecommand(cmd, cmdlen);

   if (!waitready(timeout))
   {
#ifdef PN532DEBUG
      printf("Wait for chip - ERR\n");
#endif
      return false;
   };

   if (!readack())
   {
#ifdef PN532DEBUG
      printf("No ACK frame received!\n");
#endif
      return false;
   }
   else
   {
#ifdef PN532DEBUG
      printf("ACK frame received OK!!!\n");
#endif
   };

   if (!waitready(timeout))
   {
#ifdef PN532DEBUG
      printf("No ACK frame received!\n");
#endif
      return false;
   };
   return true; // ack'd command
}
/**************************************************************************/

/**************************************************************************/
bool waitready(uint16_t timeout)
{
   uint16_t timer = 0;
   while (!isready())
   {
      if (timeout != 0)
      {
         timer += 1;
         if (timer > timeout)
         {
#ifdef PN532DEBUG
            printf("TIMEOUT!\n");
#endif
            return false;
         };
         HAL_Delay(1);
      };
   };
   return true;
}
/**************************************************************************/

/**************************************************************************/
bool readack()
{
   uint8_t ackbuff[6];
   readdata(ackbuff, 6);
   return (0 == memcmp((char *)ackbuff, (char *)pn532ack, 6));
}
/**************************************************************************/

/**************************************************************************/
uint8_t reply[5];
bool isready(void)
{
	if (!isIRQ())
	{
		return false;
	};
	return true;
}
/**************************************************************************/

/**************************************************************************/
uint8_t _pn532_packetbuffer[10];
bool SAMConfig(void)
{
	// Compose message
   _pn532_packetbuffer[0] = 0x14; //PN532_COMMAND_SAMCONFIGURATION;
   _pn532_packetbuffer[1] = 0x01; // normal mode;
   _pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
   _pn532_packetbuffer[3] = 0x01; // use IRQ pin!

   if(!sendCommandCheckAck((uint8_t *)_pn532_packetbuffer, 4, 100))
   {
#ifdef PN532DEBUG
      printf("CommandCheckAck - ERR\n");
#endif
      return false;
   };

   HAL_Delay(20);
   readdata((uint8_t *)_pn532_packetbuffer, 8);
   int offset = 6;
   return (_pn532_packetbuffer[offset] == 0x15);
}
/**************************************************************************/

/**************************************************************************/
void onInitDeviceSet(void)
{
   uint8_t cnt = 0;
   defaultInitStruct.codeWord = 0x55555555;
   /** crc16 pack start */
   defaultInitStruct.version = 1;
   defaultInitStruct.devNumb = 1;
   defaultInitStruct.subNet = 1;
   defaultInitStruct.locNet = 1;
   defaultInitStruct.devType = 1;
   defaultInitStruct.uartSET = 0x02;      // 8 bit, 9600 bps
   defaultInitStruct.relayTime = 10;      // 10 sec
   defaultInitStruct.respDelay = 1;       // 1 msec
   /** crc16 pack stop */
   defaultInitStruct.text[cnt++] = 'A';
   defaultInitStruct.text[cnt++] = 'p';
   defaultInitStruct.text[cnt++] = 'o';
   defaultInitStruct.text[cnt++] = '-';
   defaultInitStruct.text[cnt++] = 'N';
   defaultInitStruct.text[cnt++] = 'F';
   defaultInitStruct.text[cnt++] = 'C';
   defaultInitStruct.text[cnt++] = ' ';
   defaultInitStruct.text[cnt++] = 'r';
   defaultInitStruct.text[cnt++] = 'e';
   defaultInitStruct.text[cnt++] = 'a';
   defaultInitStruct.text[cnt++] = 'd';
   defaultInitStruct.text[cnt++] = 'e';
   defaultInitStruct.text[cnt++] = 'r';
   defaultInitStruct.text[cnt++] = 0;
   defaultInitStruct.crc16 = onCRC16((uint8_t*)&(defaultInitStruct.version), 8);
}
/**************************************************************************/

/**************************************************************************/
void onInitRegModbus(void)
{
   for(uint8_t i=0; i<MB_LEN; i++)
   {
      MB_REGISTERS[i] = 0;
   };
}
/**************************************************************************/

/**************************************************************************/
void onReadFlashMem(void)
{
   if(pFlashM->onCheckWritten())
   {
      mainInitStruct.codeWord = FLASH_CODE_WORD;
      mainInitStruct.version = pFlashM->onGetDevVers();
      mainInitStruct.devNumb = pFlashM->onGetDevNumb();
      mainInitStruct.subNet = pFlashM->onGetSubNet();
      mainInitStruct.locNet = pFlashM->onGetLocNet();
      mainInitStruct.devType = pFlashM->onGetDevType();
      mainInitStruct.uartSET = pFlashM->onGetSetUART();
      mainInitStruct.relayTime = pFlashM->onGetRelayTime();
      mainInitStruct.respDelay = pFlashM->onGetRespDelay();
      mainInitStruct.crc16 = pFlashM->onGetCRC16();
      char *t = pFlashM->onGetText();
      for(uint8_t n=0; n<FLASH_TEXT_LEN; n++)
      {
         mainInitStruct.text[n] = t[n];
      };

      if(mainInitStruct.crc16 == onCRC16((uint8_t*)&(mainInitStruct.version), 8))
      {
         DefaultSettFlag = false;
         pSettStruct = &mainInitStruct;
         delayResponseMBR = pSettStruct->respDelay;
         devNumber = pSettStruct->devNumb;
#ifdef PN532DEBUG
         printf("Init from EEPROM\n");
#endif
         return;
      };
   }
   else
   {
#ifdef PN532DEBUG
      printf("Init Default Settings\n");
#endif
   };
   DefaultSettFlag = true;
   pSettStruct = &defaultInitStruct;
   return;
}
/**************************************************************************/

/**************************************************************************/
uint32_t m_bps = 0;
uint32_t onGetUART_BPS(void)
{
   switch(pSettStruct->uartSET & 0x0F)
   {
   case 0:
     m_bps = 2400;
     break;
   case 1:
     m_bps = 4800;
     break;
   case 2:
     m_bps = 9600;
     break;
   case 3:
     m_bps = 19200;
     break;
   case 4:
     m_bps = 38400;
     break;
   case 5:
     m_bps = 57600;
     break;
   case 6:
     m_bps = 115200;
     break;
   default:
     m_bps = 9600;
   };
   return m_bps;
}
/**************************************************************************/

/**************************************************************************/
//#define UID_MIFARE_PRINTF
uint32_t M_UID = 0;
uint16_t M_LenUID = 0;
bool parseRxUID(uint8_t *data, uint8_t len)  /****<== PARSE - PARSE ==>****/
{
//   assert_failed((uint8_t *)__FILE__, __LINE__);
   bool res = true;
   uint8_t pos = 0;
   uint8_t temp = 0;
   if(data[5] != 0xD5) res = false;
   if(data[6] != 0x4B) res = false;
   if(data[8] == 0x00) res = false;
   if(res)
   {
      temp = data[3];
      temp = ~temp + 1;
      if(temp == data[4])
      {
         pos = data[3] + 5;
         if(data[pos] == onCheckNFC_CRC((uint8_t *)&(data[5]), data[3]))
         {
            uint8_t *pRG = (uint8_t*)&(MB_REGISTERS[1]);
            if(data[12] == 4)
            {
               pRG[0] = 4;
               pRG[1] = 0;
               pRG[2] = data[14];
               pRG[3] = data[13];
               pRG[4] = data[16];
               pRG[5] = data[15];
#ifdef UID_MIFARE_PRINTF
               printf("UID: %02X%02X%02X%02X\r\n", pRG[3], pRG[2], pRG[5], pRG[4]);
#endif
            }
            else if(data[12] == 7)
            {
               pRG[0] = 7;
               pRG[1] = 0;
               pRG[2] = data[14];
               pRG[3] = data[13];
               pRG[4] = data[16];
               pRG[5] = data[15];
               pRG[6] = data[18];
               pRG[7] = data[17];
               pRG[8] = 0;
               pRG[9] = data[19];
#ifdef UID_MIFARE_PRINTF
               printf("UID: %02X%02X%02X%02X%02X%02X%02X\r\n", pRG[3], pRG[2], pRG[5], pRG[4], pRG[7], pRG[6], pRG[9]);
#endif
            };
         };
      };
   };
   return res;
}
/**************************************************************************/

/**************************************************************************/
uint8_t onCheckNFC_CRC(uint8_t *datax, uint8_t dlen)
{
   uint8_t crc = 0;
   for(uint8_t i=0; i<dlen; i++)
   {
      crc += datax[i];
   };
   crc = ~crc + 1;
   return crc;
}
/**************************************************************************/

/**************************************************************************/
void onClearRegisterUID(void)
{
   uint8_t *pRG = (uint8_t*)&(MB_REGISTERS[0x01]);
   pRG[0] = 0;
   pRG[1] = 0;
   pRG[2] = 0;
   pRG[3] = 0;
   pRG[4] = 0;
   pRG[5] = 0;
   pRG[6] = 0;
   pRG[7] = 0;
   pRG[8] = 0;
   pRG[9] = 0;
}
/**************************************************************************/

/**************************************************************************/
//#define MODBUS_PRINTF
//#define MODBUS_PRINTF1
bool ParseModbusRX(uint8_t *data, uint16_t len)/****<== PARSE - PARSE ==>****/
{
   uint8_t pos = 0;
   uint8_t tval = 0;
   uint16_t taddr = 0;
   uint16_t tdata = 0;
   bool res = false;

#ifdef MODBUS_PRINTF
   printf("ModbusRX\r\n");
#endif

   /** Check Modbus packet */
   if(data[0] == devNumber)
   {
      //Chronometr(true);
      if(onCheckRqCRC16(data, len))
      {
         res = true;
         MB_CRC_ERR_cnt = 0;
         if(data[1] == 0x03)
         {
            //Chronometr(true);
#ifdef MODBUS_PRINTF
            printf("CRC16 OK!!! => READ REG (0x03)\n");
            printf("0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X\r\n",
                  (int)MB_REGISTERS[1], (int)MB_REGISTERS[2],
                  (int)MB_REGISTERS[3], (int)MB_REGISTERS[4],
                  (int)MB_REGISTERS[5]);
#endif
            pos = data[3];
            mTxRegPTR.devAddr = devNumber;
            mTxRegPTR.dataReg = (uint16_t*)&(MB_REGISTERS[pos]);
            mTxRegPTR.len = 0x00FF & (uint16_t)data[5];
            pTimeOutLEDR->onStart(100, 3);
            pTimeCardT->onStart(15, 5);
            onSetLedRED(true);
            step_MBR03 = 1;          /** start response Mjdbus TX, n-delay */
         }
         else if(data[1] == 0x06)
         {
            /**
            If RX command 0x06 (write reg), if address 0x0006, if data value bit0 == 1,
            THEN SET order for green led & relay - start ON with timeout.
            */
            //Chronometr(false);   /**      DEBUG  DEBUG  DEBUG  DEBUG  DEBUG  DEBUG  DEBUG  DEBUG  DEBUG  DEBUG   */
            taddr = (((uint16_t)data[2])<<8 & 0xFF00);
            taddr |= (((uint16_t)data[3]) & 0x00FF);
            tdata = (((uint16_t)data[4])<<8 & 0xFF00);
            tdata |= (((uint16_t)data[5]) & 0x00FF);
            if((taddr == 0x0006) && (tdata & 0x0001))
            {
               tval = (uint8_t)((tdata>>1) & 0x007F);
               if(tdata & 0x00FE)
               if(taddr < MB_LEN)
               {
                  MB_REGISTERS[taddr] = tdata;
                  mTxRegPTRW.devAddr = devNumber;
                  mTxRegPTRW.regAddr = taddr;
                  mTxRegPTRW.regData = tdata;
                  //step_MBR06 = 1;
               };
               onOrderUID(tval);
#ifdef MODBUS_PRINTF1
               printf("relay time: %d, dev No. %d, reg addr: %d, data: 0x%04X\r\n",
                  (int)tval, (int)devNumber, (int)taddr, (int)tdata);
#endif
            };
         };
      }
      else if(++MB_CRC_ERR_cnt > 5) SystemResetD((char *)__FILE__, __LINE__);
   };
   return res;
}
/**************************************************************************/

/**************************************************************************/
bool onCheckRqCRC16(uint8_t *data, uint16_t len)
{
   uint16_t crcL = (uint16_t)(0x00FF & data[len-2]);
   uint16_t crcH = (uint16_t)(0x00FF & data[len-1]);
   if((crcL == 0) && (crcH == 0)) return false;
   uint16_t val = CRC16(data, len-2);
   if(val == ((crcH<<8) | (crcL))) return true;
   return false;
}
/**************************************************************************/

/**************************************************************************/
static volatile uint32_t timeCounter = 0;
static volatile bool ttFlg = false;
void Chronometr(bool start)
{
   if(start == true)
   {
      ttFlg = true;
      timeCounter = 0;
   };
   if((start == false) && (ttFlg == true))
   {
      ttFlg = false;
      printf("Chrono: %d msec\r\n", timeCounter);
   };
}
/**************************************************************************/
/**************************************************************************/
void ticChronometr(void)
{
   if(ttFlg == true) ++timeCounter;
}
/**************************************************************************/

/**************************************************************************/
void onOrderUID(uint8_t time)
{
   if(time > 0)
   {
      pTimeOutLEDG->onStart(1000*time, 3);
      onSetLedGREEN(true);
      pTimeOutRELAY->onStart(1000*time, 4);
      onSetRELAY(true);
   }
   else
   {
      pTimeOutLEDG->onStart(1000*pSettStruct->relayTime, 3);
      onSetLedGREEN(true);
      pTimeOutRELAY->onStart(1000*pSettStruct->relayTime, 4);
      onSetRELAY(true);
   };
};
/**************************************************************************/

/**************************************************************************/
/** MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN */
//#define DEFINE_OBJECT_SIZE
#define MIFARE_RQ_TIMEOUT     100
static volatile bool RXF1 = false;
static volatile bool oRXF1 = false;
static volatile bool rxPIN = false;
static volatile bool rxPIN1 = false;
uint32_t timeOut_1 = 0;
uint8_t tdat[10];
uint8_t rdat[100];
uint8_t rxBuffMBR[USART_RX_BUFFER_SIZE];
uint16_t rxLenBuffMBR = 0;
uint16_t rxLenMBR = 0;
dPTR pD;

uint8_t MBRrx[USART_RX_BUFFER_SIZE];
//uint8_t* MBRrx = NULL;
uint16_t MBRlen = 0;
int main(void)
{
   onMainInit();

/********************************************************************************/
/**   START MAIN LOOP   START MAIN LOOP   START MAIN LOOP   START MAIN LOOP     */
   while(1)
   {
      /*********************************************************************/
      /** RX PIN USART2 STATUS CONTROL                                     */
//      if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2))
//      {
//         if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)) // if USART PIN:PA3 RX DATA
//         {
//            if(RXF1 == false) RXF1 = true;
//         };
//      };

      /**
         runtime control of the DMA USART RX status
      */
      while(pPortMB->onRead(MBRrx, MBRlen))
      {
         if(MBRlen > 0)
         {
            pBuffMB->onAddData(MBRrx, MBRlen);
            pTimeEnRS485->onStart(2, 2);
         };
      };
      
      /**
         timeout pause of the end serial RX
      */
      if(pTimeEnRS485->onIsTimeOut())
      {
         if(pBuffMB->onCheck())
         {
            pBuffMB->onCopyRX(rxBuffMBR, rxLenBuffMBR);
         };
         pTimeEnRS485->onStop();
         printf("ModbusRX: len=%d\n", rxLenBuffMBR);
         for (int i = 0; i < rxLenBuffMBR; i++)
         {
            printf("0x%02X, \n\r", (int)rxBuffMBR[i]);
         };
         
         /** 
            parse RX from RS485, 0 delay 
         */
         if(ParseModbusRX(rxBuffMBR, rxLenBuffMBR))
         {
            SysResetCounter = 0;
         };
         pBuffMB->onClear();
      };

//      if(USART_RXA)                    /** <= HAL_UART_RxHalfCpltCallback **/
//      {
//         USART_RXA = false;
//         mBuffUART.onWrite(pBuffRX_MA, 8);   // copy from pBuffRX_MA -> to mBuffUART.m_Buff[][]
//         if(RXF1 == true) RXF1 = false;
//      };

//      if(USART_RXB)                    /** <= HAL_UART_RxCpltCallback **/
//      {
//         USART_RXB = false;
//         mBuffUART.onWrite(pBuffRX_MB, 8);   // copy from pBuffRX_MA -> to mBuffUART.m_Buff[][]
//         pPortMB->onSetRX(16);
//         if(RXF1 == true) RXF1 = false;
//      };
      
//      if(oRXF1 != RXF1)
//      {
//         oRXF1 = RXF1;
//         if(RXF1 == true) mTimeCtrlRX.onStart(5, 9);
//         else mTimeCtrlRX.onStop();
//         printf("USART_RX+\r\n");
//      };

//      if(mBuffUART.onCheck())
//      {
//         mBuffUART.onDirectRead(pD);
//         /** parse RX from RS485, 0 delay **/
//         if(ParseModbusRX(pD.data, *pD.byteCount))
//         {
//            SysResetCounter = 0;
//         };
//      };

//      if(mTimeCtrlRX.onIsTimeOut())
//      {
//         printf("URX-ERR\r\n");
//         if(RXF1 == true) RXF1 = false;
//         mTimeCtrlRX.onStop();
//         if(SysResetCounter > 2)
//         {
//            mBuffUART.onClear();
//            pPortMB->onClearFlgRX();
//         };
//         //if(++SysResetCounter > 5) SystemReset();
//         if(++SysResetCounter > 5) SystemResetD((char *)__FILE__, __LINE__);
//      };

/********************************************************************************/
/**   START MAIN LOOP PERIOD 1 msec 1 msec 1 msec 1 msec 1 msec 1 msec 1 msec   */
      if(sysFlgW != sysFlg)
      {
         if(sysFlgW == true) sysFlgW = false;
         else sysFlgW = true;

#ifdef  WATCH_DOG_ENABLED
         resetWDT();
#endif

         /********************************************/
         /**     for drive pin enable RED LED OFF    */
         if(pTimeOutLEDR->onIsTimeOut())
         {
            pTimeOutLEDR->onStop();
            onSetLedRED(false);
            //SysResetCounter = 0;
         };
         /********************************************/
         /**    for drive pin enable GREEN LED OFF   */
         if(pTimeOutLEDG->onIsTimeOut())
         {
            pTimeOutLEDG->onStop();
            onSetLedGREEN(false);
         };
         /********************************************/
         /**     for drive pin enable RELAY OFF      */
         if(pTimeOutRELAY->onIsTimeOut())
         {
            pTimeOutRELAY->onStop();
            onSetRELAY(false);
         };
         /********************************************/
         /**        for Clear Register UID           */
         if(pTimeOutNFC->onIsTimeOut())
         {
            pTimeOutNFC->onStop();
            onClearRegisterUID();
         };
         /********************************************/
         /**    Start scan PN532 for Mifare Card     */
         if(pTimeCardT->onIsTimeOut())
         {
            step_NFC = 1;
            pTimeCardT->onStop();
         };
         /********************************************/

/********************************************************************************/
/**      START READ MIFARE UID from PN532                                       */
         onReadMifareNFC();
/********************************************************************************/

/********************************************************************************/
/**      MAKE RESPONSE FOR MBR func:0x03                                        */
         onMakeResponseMBR();
/********************************************************************************/

         if(true == pBuffUSB->onCheck()) ParseRxUSB();
      };
   };
}
/** MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN */
/** MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN */
/*******************************************************************************/
/*******************************************************************************/
//assert_failed((uint8_t *)__FILE__, __LINE__);
void onMainInit(void)
{
   onInitDeviceSet();
   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* Configure the system clock */
   SystemClock_Config();

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   //MX_DMA_Init();
   MX_I2C1_Init();
   MX_USB_DEVICE_Init();

   SysResetCounter = 0;

   MX_TIM_Init();

   /** init command pack for scan Mifare UID */
   tdat[0] = 0x4A;
   tdat[1] = 0x01;
   tdat[2] = 0x00;

   pFlashM->onInit();
   onReadFlashMem();
   onInitRegModbus();

#ifdef EEPROM_INIT
   WriteEEPROM();
#endif

   pPortMB->onInit();

   HAL_Delay(100);
   MX_SPI1_Init();
   HAL_Delay(100);
   PN532_Init();
   HAL_Delay(1000);
   pBuffMB->onClear();

   pPortMB->onSetRX(USART_RX_BUFFER_SIZE);

   printf("MAIN INIT & START!\n");

#ifdef  WATCH_DOG_ENABLED
   MX_IWDG_Init();
#endif

#ifdef  USE_PRINT_ASSERT
assert_failed((uint8_t*)__FILE__, __LINE__);
#endif /* USE_FULL_ASSERT */

#ifdef DEFINE_OBJECT_SIZE
   uint32_t objSz = sizeof(mTimeOutLEDR);
   uint32_t objSzUm = objSz*7;
   printf("CTimeOut sz: %d\r\n", objSz*7);//mTxRegPTR
   objSz = sizeof(mTxRegPTR);
   objSzUm += objSz;
   printf("mTxRegPTR sz: %d\r\n", objSz);
   objSz = sizeof(mTxRegPTRW);
   objSzUm += objSz;
   printf("mTxRegPTRW sz: %d\r\n", objSz);//mBuffUART
   objSz = sizeof(mBuffUART);
   objSzUm += objSz;
   printf("mBuffUART sz: %d\r\n", objSz);//mBuffUSB
   objSz = sizeof(mBuffUSB);
   objSzUm += objSz;
   printf("mBuffUSB sz: %d\r\n", objSz);//mFlashM
   objSz = sizeof(mFlashM);
   objSzUm += objSz;
   printf("mFlashM sz: %d\r\n", objSz);//mPortMB
   objSz = sizeof(mPortMB);
   objSzUm += objSz;
   printf("mPortMB sz: %d\r\n", objSz);//mModbus
   objSz = sizeof(mModbus);
   objSzUm += objSz;
   printf("mModbus sz: %d\r\n", objSz);//sFLASH

   objSz = sizeof(struct sFLASH);
   objSzUm += objSz*3;
   printf("sFLASH sz: %d\r\n", objSz*3);//sFLASH

   printf("All objects sz: %d\r\n", objSzUm+342);
   volatile uint32_t stack_addr = (uint32_t)&stack_addr;
   printf("Stack address: 0x%08X\n", stack_addr);
   while(1) ;
#endif
}

void onReadMifareNFC(void)
{
   switch(step_NFC)
   {
   case 1:
      onSetChipSel(true);
      ++step_NFC;
      break;
   case 2:
      writecommandSH(tdat, 3); // 0x4A; 0x01; 0x00;
      ++step_NFC;
      break;
   case 3:
      onSetChipSel(false);
      timeOut_1 = 0;
      ++step_NFC;
      break;
   case 4:
      if(isready()) ++step_NFC;
      else if(++timeOut_1 > MIFARE_RQ_TIMEOUT) step_NFC = 0;
      break;
   case 5:
      if(readack()) ++step_NFC;
      else if(++timeOut_1 > MIFARE_RQ_TIMEOUT) step_NFC = 0;
      break;
   case 6:
      if(isready()) ++step_NFC;
      else if(++timeOut_1 > MIFARE_RQ_TIMEOUT) step_NFC = 0;
      break;
   case 7:
      onSetChipSel(true);
      ++step_NFC;
      break;
   case 8:
      readdataSH(rdat, 24);
      if(parseRxUID(rdat, 24)) pTimeOutNFC->onStart(2500, 1);
      ++step_NFC;
      break;
   case 9:
      onSetChipSel(false);
      step_NFC = 0;
      break;
   default:
      break;
   };
}

void onMakeResponseMBR(void)
{
/********************************************************************************/
/**      MAKE RESPONSE FOR MBR func:0x03                                        */
   if(step_MBR03 == 1)
   {
      timeRT3 = delayResponseMBR;
      if(timeRT3 > 0) --timeRT3;
      ++step_MBR03;
   }
   else if(step_MBR03 == 2)
   {
      if(timeRT3 > 0) --timeRT3;
      else
      {
         pModbus->onRespREG(mTxRegPTR.devAddr, mTxRegPTR.dataReg, mTxRegPTR.len);
         step_MBR03 = 0;
      };
   };
/********************************************************************************/
/**      MAKE RESPONSE FOR MBR func:0x06                                        */
   if(step_MBR06 == 1)
   {
      timeRT4 = delayResponseMBR;
      if(timeRT4 > 0) --timeRT4;
      ++step_MBR06;
   }
   else if(step_MBR06 == 2)
   {
      if(timeRT4 > 0) --timeRT4;
      else
      {
         pModbus->onRespREGWR(mTxRegPTRW.devAddr, mTxRegPTRW.regAddr, mTxRegPTRW.regData);
         step_MBR06 = 0;
      };
   };
/*******************************************************************************/
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(SysTick_IRQn, 2, 1);
}

/*****************************************************************************/
void SystemResetD(char * file, int line)
{
   //initFlag = false;
   //pEthernet->m_pLanA->enc28j60_soft_reset();
   //pPortMB->onClearFlgRX();
   //pPortMB->onDeInit();
   printf("RESET SYSTEM from: \n%s,\nline %d\r\n", file, line);
   delay_us(10000);
   NVIC_SystemReset();
}
/*****************************************************************************/

void SystemReset(void)
{
//   pPortMB->onDeInit();
//   SPI1_MspDeInit();
   NVIC_SystemReset();
}

void onShowREG32(uint32_t reg)
{
   //printf("a:0x%02X v:0x%04X, bin: ", adr, res);
   uint32_t d = 0;
   for(uint32_t i=0; i<32; i++)
   {
      if((i == 4) || (i == 8) || (i == 12) || (i == 16) || (i == 20) || (i == 24) || (i == 28)) printf(" ");
      d = 0x00000001 & (reg>>(31-i));
      printf("%d", d);
   };
   printf("\r\n");
}

#ifdef EEPROM_INIT
const char* textInit = "Apogey smart-card reader v1.0\n";
void WriteEEPROM(void)
{
   uint8_t *pDataOut = (uint8_t *)&defaultInitStruct;
   uint8_t *pDataIn = (uint8_t *)&tempStruct;
   uint16_t len = sizeof(defaultInitStruct);

   for(uint16_t i=0; i<len; i++)
   {
      pDataIn[i] = pDataOut[i];
   };
   tempStruct.codeWord = FLASH_CODE_WORD;
   tempStruct.crc16 = onCRC16((uint8_t*)&(tempStruct.version), 8);

   if(pFlashM != 0)
   {
      printf("Start write EEPROM\n");
      pFlashM->onWriteProtect(0);
      for(uint16_t i=0; i<len; i++)
      {
         pFlashM->onWriteByte(i, pDataIn[i]);
      };
      HAL_Delay(100);
      pFlashM->onWriteProtect(1);
      pFlashM->onInit();
      printf("End of write EEPROM\n");
   };
}
#endif /* EEPROM_WRITE */



void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
   printf("Error Handler\n");
  __disable_irq();
  while (1)
  {
  };
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
   printf("File %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */

/**************************************************************************/

/************************************************************************************/
//#ifdef  USE_PRINT_ASSERT
//assert_failed((uint8_t*)__FILE__, __LINE__);
//#endif /* USE_FULL_ASSERT */
/** START NFC CARD ACTIVED - NFC CARD ACTIVED - NFC CARD ACTIVED - NFC CARD ACTIVED */
/*
//         if(++tstCnt > 200)
//         {
//            tstCnt = 0;
//            if(pTimeOutNFC->onIsActive(1))
//            {
//               //printf("NFC CARD ACTIVED\n");
//               CardFlg = true;
//            }
//            else
//            {
//               //printf("NFC CARD ACTIVED\n");
//               CardFlg = false;
//            };
//         };
//
//         if(CardFlg != CardFlgW)
//         {
//            CardFlgW = CardFlg;
//            if(CardFlgW == false)
//            {
//               uint8_t *pRG = (uint8_t*)&(MB_REGISTERS[0x02]);
//               pRG[0] = 0;
//               pRG[1] = 0;
//               pRG[2] = 0;
//               pRG[3] = 0;
//               printf("NFC CARD NOT ACTIVED\n");
//            };
//         };
*/
/** STOP NFC CARD ACTIVED - NFC CARD ACTIVED - NFC CARD ACTIVED - NFC CARD ACTIVED */
/************************************************************************************/

            /**
            printf("USART-RX-A { ");
            for(uint8_t n=0; n<8; n++)
            {
               printf("0x%02X, ", (int)pBuffRX_MA[n]);
            };
            printf("}\r\n");

            printf("USART-RX-B { ");
            for(uint8_t n=0; n<8; n++)
            {
               printf("0x%02X, ", (int)pBuffRX_MB[n]);
            };
            printf("}\r\n");
            **/
/*
//uint32_t getFirmwareVersion(void)
//{
//   uint32_t response;
//	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;		// Set buffer position 0 with command

//	if (! sendCommandCheckAck((uint8_t *)pn532_packetbuffer, 1, 1))		// send command with length of 1 byte
//	{														// when result is false byte is not send.
//		return 0;											// return when fails
//	};
//	// data was successfully send now retrieve data
//	// read data packet
//	readdata((uint8_t *)pn532_packetbuffer, 12);						// Receive 12 byte data and put in packet buffer provided by pointer.

//	// When buffer is not equal firmware version return
////	if (0 != strncmp((char *)pn532_packetbuffer, (char *)NFC_CONST::pn532response_firmwarevers, 6))
////	{
////		return 0;
////	}

//	// Shift relevant data from response in to uint32
//	// while SPI is used though conditional operator select a shift of 6 positions
//	int offset = 6;  // Skip a response byte when using I2C to ignore extra data.
//	response = pn532_packetbuffer[offset++];	// position 6 of pn532_packetbuffer into response
//	response <<= 8;								// shift response 8 bits left
//	response |= pn532_packetbuffer[offset++];	// or position 7 of pn532_packetbuffer with response
//	response <<= 8;								// shift response 8 bits left
//	response |= pn532_packetbuffer[offset++]; 	// or position 8 of pn532_packetbuffer into response
//	response <<= 8;								// shift response 8 bits left
//	response |= pn532_packetbuffer[offset++]; 	// or position 9 of pn532_packetbuffer into response

//#ifdef PN532DEBUG
//   sprintf(sstr, "FirmwareVersion: 0x%08X\n", (int)response);
//   printf(sstr);
//#endif

//   return response;
//}
*/

