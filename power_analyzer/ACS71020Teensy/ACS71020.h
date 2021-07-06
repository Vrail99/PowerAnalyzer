/*
 * Library for interaction with the ACS71020 Power Monitoring IC
 */

#ifndef ACS71020_h
#define ACS71020_h

#define ACS_VCODE ACSchip.readReg(0x2A) & 0x1FFFF
#define ACS_ICODE ACSchip.readReg(0x2B) & 0x1FFFF
#define ACS_VRMS ACSchip.readReg(0x20) & 0x7FFF
#define ACS_IRMS (ACSchip.readReg(0x20) >> 16) & 0x7FFF
#define ACS_PINST ACSchip.readReg(0x2C)
#define ACS_PF ACSchip.readReg(0x24) & 0x7FF
#define ACS_PACTIVE ACSchip.readReg(0x21) & 0x1FFFF
#define ACS_PAPPARENT ACSchip.readReg(0x22) & 0xFFFF;
#define ACS_PIMAG ACSchip.readReg(0x23) & 0xFFFF;

#include "Arduino.h"
#include "SPI.h"

class ACS71020
{
private:
  //Private datafields
  const uint32_t _ADDRESS_MASK = 0x7F;
  const uint32_t _READ = 0x80;
  const uint32_t _WRITE = 0x00;
  uint8_t _CS;
  SPISettings _settings;
  //Private functions
  int32_t _SignExtendBitfield(uint32_t data, uint16_t width);
  void _writeReg(uint16_t address, uint32_t value);

public:
  ACS71020(uint32_t spi_speed, uint8_t CS, uint32_t customer_code);
  uint32_t readReg(uint16_t address);
  uint32_t readEEPROM(uint32_t address, uint32_t valuemask, uint8_t pos);
  void writeEEPROM(uint16_t address, int32_t value, uint32_t valuemask, uint8_t pos);
  float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
  float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
};
#endif
