/*
 * ACS71020.cpp - Library for communication with an ACS71020 Power Monitoring IC
 *
 */

#include "Arduino.h"
#include "ACS71020.h"

ACS71020::ACS71020(uint32_t spi_speed, uint8_t CS, uint8_t EN_PIN) {
  pinMode(CS, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(CS, HIGH);
  digitalWrite(EN_PIN, LOW); //Low-Active
  _settings = SPISettings(spi_speed, LSBFIRST, SPI_MODE3);
  _EN = EN_PIN;
  _CS = CS;

  SPI.begin();
}
boolean ACS71020::init(uint32_t customer_code) {
  writeReg(0x2F, customer_code);
  regB.completeReg = readReg(0x0B);
  regC.completeReg = readReg(0x0C);
  regD.completeReg = readReg(0x0D);
  regE.completeReg = readReg(0x0E);
  return true;
}
/*!
  @brief Read a 32-bit Value from an EEPROM-Address
  @param address EEPROM-Address
  @return Unsigned 32-bit value from the Register
*/
uint32_t ACS71020::readReg(uint16_t address) {
  uint32_t val;

  digitalWrite(_EN, HIGH);
  SPI.beginTransaction(_settings);
  uint8_t comm = (address & _ADDRESS_MASK) | _READ;
  digitalWrite(_CS, LOW); //Start communication

  SPI.transfer(comm); //Send Adress to read
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);

  digitalWrite(_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(4);
  SPI.beginTransaction(_settings);
  digitalWrite(_CS, LOW);

  //Get SPI Data (LSBFirst)
  SPI.transfer(comm);
  val = (uint32_t)SPI.transfer(0);
  val |= (uint32_t)SPI.transfer(0) << 8;
  val |= (uint32_t)SPI.transfer(0) << 16;
  val |= (uint32_t)SPI.transfer(0) << 24;
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();

  digitalWrite(_EN, LOW);
  return val;
}

/*!
  @brief write a 32-bit value to an EEPROM-Address
  @param address EEPROM-Address
  @param value unsigned 32-bit value
*/
void ACS71020::writeReg(uint16_t address, uint32_t value) {
  digitalWrite(_EN, HIGH);
  SPI.beginTransaction(_settings);
  uint16_t comm = ((address & _ADDRESS_MASK) | _WRITE);
  digitalWrite(_CS, LOW);
  SPI.transfer(comm);
  SPI.transfer((uint8_t)value);
  SPI.transfer((uint8_t)(value >> 8));
  SPI.transfer((uint8_t)(value >> 16));
  SPI.transfer((uint8_t)(value >> 24));
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();
  //Writing to EEPROM delaytime
  if (address < 0x10) {
    delay(30);
  }
  digitalWrite(_EN, LOW);
}

void ACS71020::printEEPROMContent(usb_serial2_class *sBus, uint16_t address) {
  REG_0x0B settings;
  settings.completeReg = readReg(address);

  sBus->printf("Settings Register 0x%0x: \nCRC: %u\n%u\n%u\n%u\n%u\n", address,
  settings.REG_FIELDS.CRC,
  settings.REG_FIELDS.IAVGSELEN, settings.REG_FIELDS.CRS_SNS,
  settings.REG_FIELDS.SNS_FINE, settings.REG_FIELDS.QVO_FINE);
  sBus->printf("Complete: \n %u\n", settings.completeReg);
}

/*!
  @brief Write an unsigned 32-bit value to an EEPROM-Address, e.g. 0x20, 255, 0xFFDFFFFF, 21
  @param address EEPROM-Address (e.g. 0x20)
  @param value signed 32-bit value to write
  @param valuemask unsigned 32-bit mask
  @param pos offset of the written value in the register.
*/
void ACS71020::writeEEPROM(uint32_t address, int32_t value, uint32_t valuemask, uint8_t pos) {
  //Read the value at the desired address
  uint32_t whole = readReg(address);
  //Apply the mask for the value and shift the value in position
  uint32_t newValue = (whole & valuemask) | (value << pos);

  //Write new value to register
  //Serial.printf("Writing %d to %u, with %u shifted by %u\n",value,address,valuemask,pos);
  writeReg(address, newValue);
}

uint32_t ACS71020::readEEPROM(uint32_t address, uint32_t valuemask, uint8_t pos) {
  //Read the Addresses value and mask out the value
  //Then shift it to the right
  uint32_t data = (readReg(address) & ~valuemask) >> pos;
  return data;
}

////////////////////////////////////////////////////////////////////////////////
// Conversion functions
////////////////////////////////////////////////////////////////////////////////
/*!
   Convert an unsigned bitfield which is right justified, into a floating point number

      @param inputValue        - the bitfield to be converted
      @param binaryPoint - the binary point (the bit to the left of the binary point)
      @param width       - the width of the bitfield
      @param returns     - the floating point number
*/
float ACS71020::ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width) {
  uint32_t mask;

  if (width == 32) {
    mask = 0xFFFFFFFF;
  } else {
    mask = (1UL << width) - 1UL;
  }

  return (float)(inputValue & mask) / (float)(1L << binaryPoint);
}

/*!
   Convert a signed bitfield which is right justified, into a floating point number

      @param inputValue        - the bitfield to be sign extended then converted
      @param binaryPoint - the binary point (the bit to the left of the binary point)
      @param width       - the width of the bitfield
      @param returns     - the floating point number
*/
float ACS71020::ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width) {
  int32_t signedValue = _SignExtendBitfield(inputValue, width);
  return (float)signedValue / (float)(1L << binaryPoint);
}

/*
   Sign extend a bitfield which if right justified

      data        - the bitfield to be sign extended
      width       - the width of the bitfield
      returns     - the sign extended bitfield
*/
int32_t ACS71020::_SignExtendBitfield(uint32_t data, uint16_t width) {
  // If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
  if (width == 32) {
    return (int32_t)data;
  }

  int32_t x = (int32_t)data;
  int32_t mask = 1L << (width - 1);

  x = x & ((1 << width) - 1); // make sure the upper bits are zero
  int32_t res = ((x ^ mask) - mask);

  return res;
}

/*!
  @brief Sets Current Averaging true
*/

void ACS71020::activateCurrentAveraging() {
  if (regB.REG_FIELDS.IAVGSELEN == 0) {
    regB.REG_FIELDS.IAVGSELEN = 1;
    writeReg(0x1B, regB.completeReg);
  }
}

/*!
  @brief Sets Voltage Averaging true
*/

void ACS71020::activateVoltageAveraging() {
  if (regB.REG_FIELDS.IAVGSELEN == 1) {
    regB.REG_FIELDS.IAVGSELEN = 0;
    writeReg(0x1B, regB.completeReg);
  }
}

/*!
  @brief Calculates the RMS voltage according to avg
  @param avg 0: No average, 1: Average over one second, 2: average over one minute
  @return returns a RMS voltage measurement
*/

float ACS71020::readVRMS(uint8_t avg) {
  return _calcVRMS(avg);
}

/*!
  @brief Calculates the RMS voltage according to avg
  @param avg 0: No average, 1: Average over one second, 2: average over one minute
  @return returns a RMS current measurement
*/

float ACS71020::readIRMS(uint8_t avg) {
  return _calcIRMS(avg);
}

/*!
  @brief returns a raw current measurement
*/

float ACS71020::readICODE() {
  icodes = readReg(0x2B) & 0x1FFFF;

  return ConvertSignedFixedPoint(icodes, 15, 17);
}

/*!
  @brief returns a raw voltage measurement
*/

float ACS71020::readVCODE() {
  vcodes = readReg(0x2A) & 0x1FFFF;

  return ConvertSignedFixedPoint(vcodes, 16, 17);
}

/*!
  @brief returns a raw Power measurement
*/

float ACS71020::readPINST() {
  pinst = readReg(0x2C);

  return ConvertSignedFixedPoint(pinst, 29, 32);
}
/*!
  @brief returns a power factor measurement
*/

float ACS71020::readPOWERFACTOR() {
  pfact = readReg(0x24) & 0x7FF;

  return ConvertSignedFixedPoint(pfact, 9, 11);
}
/*!
  @brief Calculates the active power according to avg
  @param avg 0: No average, 1: Average over one second, 2: average over one minute
  @brief returns an active power measurement
*/

float ACS71020::readPACTIVE(uint8_t avg) {
  uint32_t acsrms;
  if (avg == 0) {
    pActive = readReg(0x21) & 0x1FFFF;
    acsrms = pActive;
  } else if (avg == 1) {
    pact_sec = readReg(0x28) & 0x1FFFF;
    acsrms = pact_sec;
  } else {
    pact_min = readReg(0x29) & 0x1FFFF;
    acsrms = pact_min;
  }

  return ConvertSignedFixedPoint(acsrms, 15, 17);
}
/*!
  @brief returns a imaginary power measurement
*/

float ACS71020::readPIMAG() {
  pImag = readReg(0x23) & 0xFFFF;

  return ConvertSignedFixedPoint(pImag, 15, 16);
}
/*!
  @brief returns an apparent power measurement
*/

float ACS71020::readPAPP() {
  pApp = readReg(0x22) & 0xFFFF;

  return ConvertSignedFixedPoint(pApp, 15, 16);
}

float ACS71020::_calcVRMS(uint8_t avg) {
  uint32_t acsrms;
  if (avg == 0) {
    acsrms = readReg(0x20) & 0x7FFF;
    v_rms = acsrms;
  } else if (avg == 1) {
    acsrms = readReg(0x26) & 0x7FFF;
    vrms_sec = acsrms;
  } else {
    acsrms = readReg(0x27) & 0x7FFF;
    vrms_min = acsrms;
  }
  //uint32_t calib_Code = 21280;              //Calibration Factor
  //conv_factor_VRMS = exp_VRMS / calFactor_VRMS; //Actual Sensitivity
  float vrms_rsense = acsrms * _conv_factor_VRMS;      //Voltage over RS1
  float vrms_input = (vrms_rsense * (4003000.0F / 3000.0F));
  return vrms_input;
}

float ACS71020::_calcIRMS(uint8_t avg) {
  uint32_t acsrms;
  if (avg == 0) {
    acsrms = (readReg(0x20) >> 16) & 0x7FFF;
    i_rms = acsrms;
  } else if (avg == 1) {
    acsrms = (readReg(0x26) >> 16) & 0x7FFF;
    irms_sec = acsrms;
  } else {
    acsrms = (readReg(0x27) >> 16) & 0x7FFF;
    irms_min = acsrms;
  }
  //uint32_t i_calib_Code = 257;
  //float conv_factor = exp_IRMS / calFactor_IRMS; //Calibration from Lightbulb
  //float irms = acsrms * _conv_factor_IRMS; //conv_factor;
  return acsrms * _conv_factor_IRMS;
}

void ACS71020::setConversionFactors(float v_conv, float i_conv) {
  if (i_conv > 0.0)
    _conv_factor_IRMS = i_conv;
  else if (v_conv > 0.0)
    _conv_factor_VRMS = v_conv;
}
/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readVCODE_RAW() {
  vcodes = readReg(0x2A) & 0x1FFFF;
  return vcodes;
}
/*!
  @brief Read a raw icode
  @return returns a raw icode
*/
uint32_t ACS71020::readICODE_RAW() {
  icodes = readReg(0x2B) & 0x1FFFF;
  return icodes;
}
/*!
  @brief Read raw vrms
  @param avg Average mode: 0=instant, 1=second, >= 2: minute
  @return returns raw volt RMS
*/
uint32_t ACS71020::readVRMS_RAW(uint8_t avg) {
  if (avg == 0) {
    v_rms = readReg(0x20) & 0x7FFF;
    return v_rms;
  } else if (avg == 1) {
    vrms_sec = readReg(0x26) & 0x7FFF;
    return vrms_sec;
  } else {
    vrms_min = readReg(0x27) & 0x7FFF;
    return vrms_min;
  }
}
/*!
  @brief Read raw vrms
  @param avg Average mode: 0=instant, 1=second, >= 2: minute
  @return returns raw volt RMS
*/
uint32_t ACS71020::readIRMS_RAW(uint8_t avg) {
  if (avg == 0) {
    i_rms = (readReg(0x20) >> 16) & 0x7FFF;
    return i_rms;
  } else if (avg == 1) {
    irms_sec = (readReg(0x26) >> 16) & 0x7FFF;
    return irms_sec;
  } else {
    irms_min = (readReg(0x27) >> 16) & 0x7FFF;
    return irms_min;
  }
}
/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readPINST_RAW() {
  vcodes = readReg(0x2C);
  return vcodes;
}
/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readPOWERFACTOR_RAW() {
  pfact = readReg(0x24) & 0x7FF;
  return pfact;
}
/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readPACTIVE_RAW(uint8_t avg) {
  if (avg == 0) {
    pActive = readReg(0x21) & 0x1FFFF;
    return pActive;
  } else if (avg == 1) {
    pact_sec = readReg(0x28) & 0x1FFFF;
    return pact_sec;
  } else {
    pact_min = readReg(0x29) & 0x1FFFF;
    return pact_min;
  }
}
/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readPIMAG_RAW() {
  pImag = readReg(0x23) & 0xFFFF;
  return pImag;
}

/*!
  @brief Read a raw vcode
  @return returns a raw vcode
*/
uint32_t ACS71020::readPAPP_RAW() {
  pApp = readReg(0x22) & 0xFFFF;
  return pApp;
}

uint32_t ACS71020::readFlags() {
  flags = readReg(0x2D);
  return flags;
}

uint32_t ACS71020::readNumOfPoints() {
  numptsout = readReg(0x25) & 0x1FF;
  return numptsout;
}

float ACS71020::addEnergy(float value, float time_in_seconds) {
  energylog += value * 3600.0 / (time_in_seconds);
  return energylog;
}