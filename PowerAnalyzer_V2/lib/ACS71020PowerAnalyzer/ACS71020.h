/*
 * Library for interaction with the ACS71020 Power Monitoring IC
 */

#ifndef ACS71020_h
#define ACS71020_h

#include "Arduino.h"
#include "SPI.h"

union REG_0x0B
{
  struct {
    uint32_t QVO_FINE : 9;
    uint32_t SNS_FINE : 9;
    uint32_t CRS_SNS : 3;
    uint32_t IAVGSELEN : 1;
    uint32_t skip : 4;
    uint32_t CRC : 6;
  }REG_FIELDS;
  uint32_t completeReg;
};

union REG_0x0C
{
  struct {
    uint32_t RMS_AVG1 : 7;
    uint32_t RMS_AVG2 : 10;
    uint32_t CRC : 15;
  }fields;
  uint32_t completeReg;
};

union REG_0x0D
{
  struct {
    uint32_t PACC_TRIM : 7;
    uint32_t ICHAN_DEL_EN : 1;
    uint32_t skip2 : 1;
    uint32_t CHAN_DEL_SEL : 3;
    uint32_t skip1 : 1;
    uint32_t FAULT : 8;
    uint32_t FLTDLY : 3;
    uint32_t HALFCYCLE_EN : 1;
    uint32_t SQUAREWAVE_EN : 1;
    uint32_t CRC : 6;
  }fields;
  uint32_t completeReg;
};

union REG_0x0E
{
  struct {
    uint32_t VEVENT_CYCS : 6;
    uint32_t VADC_RATE_SET : 1;
    uint32_t skip1 : 1;
    uint32_t OVERVREG : 6;
    uint32_t UNDERVREG : 6;
    uint32_t DELAYCNT_SEL : 1;
    uint32_t CRC : 11;
  }fields;
  uint32_t completeReg;
};

class ACS71020
{
private:
  //Private datafields
  const uint32_t _ADDRESS_MASK = 0x7F;
  const uint32_t _READ = 0x80;
  const uint32_t _WRITE = 0x00;
  uint8_t _CS;
  uint8_t _EN;
  uint8_t _EN_delay = 10; //ns
  SPISettings _settings;

  //EEPROM_Save for edit
  REG_0x0B regB;
  REG_0x0C regC;
  REG_0x0D regD;
  REG_0x0E regE;

  float _conv_factor_VRMS = 8.384F * pow(10, -6);
  float _conv_factor_IRMS = 1.015F * pow(10, -3);

  //Private functions
  int32_t _SignExtendBitfield(uint32_t data, uint16_t width);
  float _calcVRMS(uint8_t avg);
  float _calcIRMS(uint8_t avg);

  uint32_t tmp = 0;
  uint32_t vcodes = 0;
  uint32_t icodes = 0;
  uint32_t flags = 0;
  uint32_t pinst = 0;
  uint32_t numptsout = 0;
  uint32_t pfact = 0;
  uint32_t pImag = 0;
  uint32_t pApp = 0;
  uint32_t pActive = 0;
  uint32_t v_rms = 0;
  uint32_t i_rms = 0;
  uint32_t pact_sec = 0;
  uint32_t irms_sec = 0;
  uint32_t vrms_sec = 0;
  uint32_t pact_min = 0;
  uint32_t irms_min = 0;
  uint32_t vrms_min = 0;

  float energylog = 0;

public:
  ACS71020(uint32_t spi_speed, uint8_t CS, uint8_t EN_PIN);
  boolean init(uint32_t customer_code = 0x4F70656E);
  uint32_t readReg(uint16_t address);
  void writeReg(uint16_t address, uint32_t value);
  uint32_t readEEPROM(uint32_t address, uint32_t valuemask, uint8_t pos);
  void writeEEPROM(uint32_t address, int32_t value, uint32_t valuemask, uint8_t pos);
  float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
  float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
  void setConversionFactors(float v, float i);
  void activateCurrentAveraging();
  void activateVoltageAveraging();
  void printEEPROMContent(usb_serial2_class *sBus, uint16_t address);

  float readVRMS(uint8_t avg);
  float readIRMS(uint8_t avg);
  float readVCODE();
  float readICODE();
  float readPINST();
  float readPOWERFACTOR();
  float readPACTIVE(uint8_t avg);
  float readPIMAG();
  float readPAPP();
  float addEnergy(float value, float time);

  uint32_t readVRMS_RAW(uint8_t avg);
  uint32_t readIRMS_RAW(uint8_t avg);
  uint32_t readVCODE_RAW();
  uint32_t readICODE_RAW();
  uint32_t readPINST_RAW();
  uint32_t readPOWERFACTOR_RAW();
  uint32_t readPACTIVE_RAW(uint8_t avg);
  uint32_t readPIMAG_RAW();
  uint32_t readPAPP_RAW();
  uint32_t readFlags();
  uint32_t readNumOfPoints();

};
#endif
