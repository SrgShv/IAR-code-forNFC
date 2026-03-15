#ifndef MODBUS_H_INCLUDED
#define MODBUS_H_INCLUDED

#include "stdint.h"






#pragma pack(push,1)
struct txRegPTR
{
   uint8_t devAddr;
   uint16_t* dataReg;
   uint16_t len;
};
#pragma pack(pop)

#pragma pack(push,1)
struct txRegPTRW
{
   uint8_t devAddr;
   uint16_t regAddr;
   uint16_t regData;
};
#pragma pack(pop)

uint16_t CRC16(const uint8_t *nData, uint16_t wLength);

class CModbus
{
public:
   CModbus();
   ~CModbus();

   void onFunc0(void);
   void onReadREG(uint8_t devAddr, uint16_t pos, uint16_t len);
   void onRespREG(uint8_t devAddr, uint16_t *data, uint16_t len);
   void onRespREGWR(uint8_t devAddr, uint16_t addr, uint16_t data);
   void onWriteREG(uint8_t devAddr, uint16_t pos, uint16_t val);
   void onWriteREGW(uint8_t devAddr, uint16_t pos, uint16_t len, uint16_t *date);


protected:
private:
   uint8_t *m_buffTX;
   const uint16_t m_txBuffSz;
};

#endif /* MODBUS_H_INCLUDED */
