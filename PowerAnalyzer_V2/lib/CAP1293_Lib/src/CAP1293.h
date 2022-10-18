/**
 * @file CAP1293.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Microchip CAP1293 Touch Sense IC library. CAP1293 Types and Enumerations
 * @version 0.1
 * @date 2021-02-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __CAP1293_H__
#define __CAP1293_H__

#include <Arduino.h>
#include <Wire.h>
#include "CAP1293_Registers.h"

 // Declare I2C Address
#define CAP1293_I2C_ADDR 0x28

// Register values

// Product ID - always the same
#define PROD_ID_VALUE 0x6F // 0x6D for CAP1293

// Capacitive sensor input
#define OFF 0x00 // No touch detecetd
#define ON 0x01	 // Check capacitive sensor input

// Pads to be power buttion
#define PWR_CS1 0x00 // Pad 1 (Left)
#define PWR_CS2 0x01 // Pad 2 (Middle)
#define PWR_CS3 0x02 // Pad 3 (Right)

#define PAD_LEFT PWR_CS1
#define PAD_MIDDLE PWR_CS2
#define PAD_RIGHT PWR_CS3

// Power button hold time to generate interrupt
#define PWR_TIME_280_MS 0x00  // 280 ms
#define PWR_TIME_560_MS 0x01  // 560 ms
#define PWR_TIME_1120_MS 0x02 // 1.12 sec
#define PWR_TIME_2240_MS 0x03 // 2.24 sec

// Sensitivity for touch detection
#define SENSITIVITY_128X 0x00 // Most sensitive
#define SENSITIVITY_64X 0x01
#define SENSITIVITY_32X 0x02
#define SENSITIVITY_16X 0x03
#define SENSITIVITY_8X 0x04
#define SENSITIVITY_4X 0x05
#define SENSITIVITY_2X 0x06
#define SENSITIVITY_1X 0x07 // Least sensitive

// Sensitivity Control Reg 0x1F
typedef union
{
	struct
	{
		uint8_t BASE_SHIFT : 4;
		uint8_t DELTA_SENSE : 3;
		uint8_t EMPTY_1 : 1;
	} SENSITIVITY_CONTROL_FIELDS;
	uint8_t SENSITIVITY_CONTROL_COMBINED;
} SENSITIVITY_CONTROL_REG;

// General Status Reg 0x02
typedef union
{
	struct
	{
		uint8_t TOUCH : 1;
		uint8_t MTP : 1;
		uint8_t MULT : 1;
		uint8_t EMPTY_1 : 1;
		uint8_t PWR : 1;
		uint8_t ACAL_FAIL : 1;
		uint8_t BC_OUT : 1;
		uint8_t EMPTY_2 : 1;
	} GENERAL_STATUS_FIELDS;
	uint8_t GENERAL_STATUS_COMBINED;
} GENERAL_STATUS_REG;

// Sensor Input Status Reg 0x03
typedef union
{
	struct
	{
		uint8_t CS1 : 1;
		uint8_t CS2 : 1;
		uint8_t CS3 : 1;
		uint8_t EMPTY_1 : 5;
	} SENSOR_INPUT_STATUS_FIELDS;
	uint8_t SENSOR_INPUT_STATUS_COMBINED;
} SENSOR_INPUT_STATUS_REG;

// Main Control Register 0x00
typedef union
{
	struct
	{
		uint8_t INT : 1;
		uint8_t EMPTY_1 : 3;
		uint8_t DSLEEP : 1;
		uint8_t STBY : 1;
		uint8_t EMPTY_2 : 2;
	} MAIN_CONTROL_FIELDS;
	uint8_t MAIN_CONTROL_COMBINED;
} MAIN_CONTROL_REG;

// Power Button Register 0x60
typedef union
{
	struct
	{
		uint8_t PWR_BTN : 3;
		uint8_t EMPTY_1 : 5;
	} POWER_BUTTON_FIELDS;
	uint8_t POWER_BUTTON_COMBINED;
} POWER_BUTTON_REG;

// Power Button Configuration Register 0x61
typedef union
{
	struct
	{
		uint8_t PWR_TIME : 2;
		uint8_t PWR_EN : 1;
		uint8_t EMPTY_1 : 1;
		uint8_t STBY_PWR_TIME : 2;
		uint8_t STBY_PWR_EN : 1;
		uint8_t EMPTY_2 : 1;
	} POWER_BUTTON_CONFIG_FIELDS;
	uint8_t POWER_BUTTON_CONFIG_COMBINED;
} POWER_BUTTON_CONFIG_REG;

// Interrupt Enable Register 0x27
typedef union
{
	struct
	{
		uint8_t CS1_INT_EN : 1;
		uint8_t CS2_INT_EN : 1;
		uint8_t CS3_INT_EN : 1;
		uint8_t EMPTY_1 : 5;
	} INTERRUPT_ENABLE_FIELDS;
	uint8_t INTERRUPT_ENABLE_COMBINED;
} INTERRUPT_ENABLE_REG;

// Calibration Activate and Status Register 0x26, default value is 0x00
typedef union
{
	struct
	{
		uint8_t CS1_CAL : 1;
		uint8_t CS2_CAL : 1;
		uint8_t CS3_CAL : 1;
		uint8_t EMPTY_1 : 5;
	} CALIBRATION_ACTIVATE_AND_STATUS_FIELDS;
	uint8_t CALIBRATION_ACTIVATE_AND_STATUS_COMBINED;
} CALIBRATION_ACTIVATE_AND_STATUS_REG;

// Repeat Rate ENABLE Register 0x28, default value is 0x07
typedef union
{
	struct
	{
		uint8_t CS1_RPT_EN : 1;
		uint8_t CS2_RPT_EN : 1;
		uint8_t CS3_RPT_EN : 1;
		uint8_t EMPTY_1 : 5;
	} REPEAT_RATE_ENABLE_FIELDS;
	uint8_t REPEAT_RATE_ENABLE_COMBINED;
} REPEAT_RATE_ENABLE_REG;

//  Multiple Touch Configuration Register 0x2a
typedef union
{
	struct
	{
		uint8_t EMPTY_1 : 2;
		uint8_t B_MULT_T : 2;
		uint8_t EMPTY_2 : 3;
		uint8_t MULT_BLK_EN : 1;
	} MTP_CONFIG_FIELDS;
	uint8_t MTP_CONFIG_COMBINED;
} MTP_CONFIG_REG;

// MTP Enable Register 0x2b
typedef union
{
	struct
	{
		uint8_t MTP_ALERT : 1;
		uint8_t COMP_PTRN : 1;
		uint8_t MTP_TH : 2;
		uint8_t EMPTY_1 : 3;
		uint8_t MTP_EN : 1;
	} MTP_ENABLE_FIELDS;
	uint8_t MTP_ENABLE_COMBINED;
} MTP_ENABLE_REG;

// MTP Enable Register 0x2d
typedef union
{
	struct
	{
		uint8_t CS1_PTRN : 1;
		uint8_t CS2_PTRN : 1;
		uint8_t CS3_PTRN : 1;
		uint8_t EMPTY_1 : 5;
	} MULTIPLE_TOUCH_PATTERN_FIELDS;
	uint8_t MULTIPLE_TOUCH_PATTERN_COMBINED;
} MULTIPLE_TOUCH_PATTERN_REG;

////////////////////////////////
// CAP1293 Class Declaration //
////////////////////////////////
class CAP1293
{
public:
	// Constructor
	CAP1293(byte addr = CAP1293_I2C_ADDR);
	//bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = CAP1293_I2C_ADDR);
	bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = CAP1293_I2C_ADDR);
	bool isConnected();
	void setSensitivity(uint8_t sensitivity);
	uint8_t getSensitivity();

	// Check if a capacitive touch panel has been touched or released(if enabled release interrupt)
	bool isLeftTouched();
	bool isMiddleTouched();
	bool isRightTouched();
	bool isTouched();
	uint8_t isLeftOrRightTouched();

	// get which key is pressed or released(if enabled release interrupt)
	uint8_t getTouchKeyStatus(bool keyStatus[], uint8_t num = 3);

	// Check if a swipe has occured
	bool isRightSwipePulled();
	bool isLeftSwipePulled();

	// Clears INT bit
	void clearInterrupt();

	// Configures if interrupt triggers on touch
	void setInterruptDisabled();
	void setInterruptEnabled();
	bool isInterruptEnabled();

	// Entering and exiting sleep mode
	void setDeepSleepDisabled();
	void setDeepSleepEnabled();
	bool isDeepSleepEnabled();

	// Get main control regesiter's value
	uint8_t checkMainControl();

	// Get general status register's value
	uint8_t checkStatus();
	void clearStatus();

	// Power button configuration and check settings
	bool setPowerButtonPad(uint8_t pad);
	bool setPowerButtonTime(uint8_t time);
	uint8_t getPowerButtonPad();
	uint16_t getPowerButtonTime();
	void setPowerButtonEnabled();
	void setPowerButtonDisabled();
	bool isPowerButtonEnabled();
	bool isPowerButtonTouched();

	// Configures if sensor can detect more than one key input
	void setMultiTouchEnabled();
	void setMultiTouchDisabled();

	// calibration active configuration
	void setForceCalibrateEnabled();
	void setForceCalibrateDisabled();

	// repeat rate enable configuration
	void setRepeatRateEnabled();
	void setRepeatRateDisabled();

	// MTP
	void setMTPEnabled(bool left = true, bool middle = true, bool right = true);
	bool isMTPStatus();

	// Configures if interrupt triggers on Release touch key
	void setReleaseInterruptDisabled();
	void setReleaseInterruptEnabled();
	bool isReleaseInterruptEnabled();

	void enableMultitouch();


private:
	TwoWire *_i2cPort = NULL; //The generic connection to user's chosen I2C hardware
	uint8_t _deviceAddress;	  //Keeps track of I2C address. setI2CAddress changes this.
	MAIN_CONTROL_REG _reg;

	// Read and write to registers
	byte readRegister(CAP1293_Register reg);
	void readRegisters(CAP1293_Register reg, byte *buffer, byte len);
	void writeRegister(CAP1293_Register reg, byte data);
	void writeRegisters(CAP1293_Register reg, byte *buffer, byte len);
};

#endif