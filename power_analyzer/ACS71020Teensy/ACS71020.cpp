/*
 * ACS71020.cpp - Library for communication with an ACS71020 Power Monitoring IC
 * 
 */

#include "Arduino.h"
#include "ACS71020.h"

ACS71020::ACS71020(uint32_t spi_speed, uint8_t CS, uint32_t customer_code)
{
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  _settings = SPISettings(spi_speed, LSBFIRST, SPI_MODE3);
  SPI.begin();
  _CS = CS;
  writeReg(0x2F, customer_code);
}

/*!
	@brief Read a 32-bit Value from an EEPROM-Address
	@param address EEPROM-Address
	@return Unsigned 32-bit value from the Register
*/

uint32_t ACS71020::readReg(uint16_t address)
{
  //uint32_t startTime = micros();
  uint32_t val;

  SPI.beginTransaction(_settings);
  uint8_t comm = (address & _ADDRESS_MASK) | _READ;

  digitalWrite(_CS, LOW); //Start communication

  SPI.transfer(comm); //Send Adress to read
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);

  digitalWrite(_CS, HIGH);
  delayMicroseconds(3);
  digitalWrite(_CS, LOW);

  //Get SPI Data (LSBFirst)
  SPI.transfer(comm);
  val = (uint32_t)SPI.transfer(0);
  val |= (uint32_t)SPI.transfer(0) << 8;
  val |= (uint32_t)SPI.transfer(0) << 16;
  val |= (uint32_t)SPI.transfer(0) << 24;
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();
  //uint32_t endTime = micros();
  //Serial.printf("Time for read: %u\n", (endTime-startTime));
  return val;
}

/*!
	@brief write a 32-bit value to an EEPROM-Address
	@param address EEPROM-Address
	@param value unsigned 32-bit value
*/
void ACS71020::writeReg(uint16_t address, uint32_t value)
{
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
  if (address < 0x10)
  {
    delay(30);
  }
}

/*!
	@brief Write an unsigned 32-bit value to an EEPROM-Address, e.g. 0x20, 255, 0xFFDFFFFF, 21
	@param address EEPROM-Address (e.g. 0x20)
	@param value signed 32-bit value to write
	@param valuemask unsigned 32-bit mask
	@param pos offset of the written value in the register.
*/
void ACS71020::writeEEPROM(uint16_t address, int32_t value, uint32_t valuemask, uint8_t pos)
{
  //Read the value at the desired address
  uint32_t whole = readReg(address);
  //Apply the mask for the value and shift the value in position
  uint32_t newValue = (whole & valuemask) | (value << pos);

  //Write new value to register
  //Serial.printf("Writing %d to %u, with %u shifted by %u\n",value,address,valuemask,pos);
  writeReg(address, newValue);
}

uint32_t ACS71020::readEEPROM(uint16_t address, uint32_t valuemask, uint8_t pos)
{
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
float ACS71020::ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
{
  uint32_t mask;

  if (width == 32)
  {
    mask = 0xFFFFFFFF;
  }
  else
  {
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
float ACS71020::ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
{
  int32_t signedValue = _SignExtendBitfield(inputValue, width);
  return (float)signedValue / (float)(1L << binaryPoint);
}

/*
   Sign extend a bitfield which if right justified

      data        - the bitfield to be sign extended
      width       - the width of the bitfield
      returns     - the sign extended bitfield
*/
int32_t ACS71020::_SignExtendBitfield(uint32_t data, uint16_t width)
{
  // If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
  if (width == 32)
  {
    return (int32_t)data;
  }

  int32_t x = (int32_t)data;
  int32_t mask = 1L << (width - 1);

  x = x & ((1 << width) - 1); // make sure the upper bits are zero
  int32_t res = ((x ^ mask) - mask);

  return res;
}
