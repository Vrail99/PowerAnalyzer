/**
 * @file CAP1293.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Microchip CAP1293 Touch Sense IC library. Based on Sparkfun CAP1203 library
 * @version 0.1
 * @date 2021-02-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include "CAP1293_Registers.h"
#include "CAP1293.h"

 /* CONSTRUCTOR
	 This function, called when you initialize the class will write
	 the varibale address into a private variable for future use.
	 The variable address should be 0x28.
 */
CAP1293::CAP1293(byte addr) {
	_deviceAddress = addr;
}

/* BEGIN INITIALIZATION
	This function initalizes the CAP1293 sensor.
*/
bool CAP1293::begin(TwoWire &wirePort, uint8_t deviceAddress) {
	// Set device address and wire port to private variable
	_deviceAddress = deviceAddress;
	_i2cPort = &wirePort;

	if (isConnected() == false) {
		return false;
	}
	// Read PROD_ID register
	byte prodIDValue = readRegister(PROD_ID);

	// PROD_ID should always be 0x6D
	if (prodIDValue != PROD_ID_VALUE) {
		return false;
	}
	_reg.MAIN_CONTROL_COMBINED = readRegister(MAIN_CONTROL);
	writeRegister(SIGNAL_GUARD_ENABLE_REG, 5); //Enable signal guard for both touch inputs
	setRepeatRateDisabled();        // disable repeat interrupts
	setForceCalibrateEnabled();     // force calibrate
	//setMultiTouchEnabled();         // enable Multiple touch SZ_MOD
	setReleaseInterruptDisabled();  // disable Release interrupt
	setSensitivity(SENSITIVITY_2X); // Set sensitivity to 2x on startup
	setInterruptEnabled();			// Enable INT as default, waiting for Interrupts
	setPowerButtonDisabled();		// Disable PowerButton feature SZ_MOD
	clearInterrupt();				// Clear interrupt on startup
	return true;
}

/* IS CONNECTED
	Returns true if the I2C Device acknowledgs a connection.
	Otherwise returns false.
*/
bool CAP1293::isConnected() {
	for (byte i = 0; i < 5; i++) {
		/* After inspecting with logic analyzer, the device fails
			to connect for unknown reasons. The device typically connects
			after two calls. We included a for loop to allow for
			multiple calls to the device.
		*/
		_i2cPort->beginTransmission((uint8_t)_deviceAddress);
		if (_i2cPort->endTransmission() == 0)
			return (true); //Sensor did not ACK
	}

	return (false);
}

/* CHECK MAIN CONTROL REGISTER
	Control the primary power state of the device. See data sheet
	on Main Control Register (pg. 22).
*/
uint8_t CAP1293::checkMainControl() {
	// MAIN_CONTROL_REG reg;
	return readRegister(MAIN_CONTROL);
}

/* CHECK STATUS
	Checks inputs in the general status register to ensure program
	is set up correctly. See data sheet on Status Registers (pg. 23).
*/
uint8_t CAP1293::checkStatus() {
	// GENERAL_STATUS_REG reg;
	return readRegister(GENERAL_STATUS);
}

/* CLEAR INTTERUPT
	Clears the interrupt (INT) bit by writing a logic 0 to it.
	This bit must be cleared in order to detec a new capacitive
	touch input. See datasheet on Main Control Register
*/
void CAP1293::clearInterrupt() { //FIXME omit register read, because it doesn't change.
	//MAIN_CONTROL_REG reg;
	//reg.MAIN_CONTROL_COMBINED = readRegister(MAIN_CONTROL);
	this->_reg.MAIN_CONTROL_FIELDS.INT = 0x00;
	writeRegister(MAIN_CONTROL, this->_reg.MAIN_CONTROL_COMBINED);
}

/* DISABLE INTERRUPTS
	This disables all the interrupts, so the alert LED will not turn on
	when a sensor is touched. Set on default in begin function See data
	sheet on Interrupt Enable Register
*/
void CAP1293::setInterruptDisabled() {
	INTERRUPT_ENABLE_REG reg;
	reg.INTERRUPT_ENABLE_COMBINED = readRegister(INTERRUPT_ENABLE);
	reg.INTERRUPT_ENABLE_FIELDS.CS1_INT_EN = 0x00;
	reg.INTERRUPT_ENABLE_FIELDS.CS2_INT_EN = 0x00;
	reg.INTERRUPT_ENABLE_FIELDS.CS3_INT_EN = 0x00;
	writeRegister(INTERRUPT_ENABLE, reg.INTERRUPT_ENABLE_COMBINED);
}

/* ENABLE INTERRUPTS
	This turns on all the interrupts, so the alert LED turns on when any
	sensor is touched. See data sheet on Interrupt Enable Register
*/
void CAP1293::setInterruptEnabled() {
	INTERRUPT_ENABLE_REG reg;
	reg.INTERRUPT_ENABLE_COMBINED = readRegister(INTERRUPT_ENABLE);
	reg.INTERRUPT_ENABLE_FIELDS.CS1_INT_EN = 0x01;
	reg.INTERRUPT_ENABLE_FIELDS.CS2_INT_EN = 0x00; //SZ_MOD
	reg.INTERRUPT_ENABLE_FIELDS.CS3_INT_EN = 0x01;
	writeRegister(INTERRUPT_ENABLE, reg.INTERRUPT_ENABLE_COMBINED);
}

/* IS INTERRUPT ENABLED
	Returns state of intterupt pin. Returns true if all interrupts enabled
	(0x07), otherwise returns false. When the interrupts are enabled, the
	LED on the CAP1293 Touch Slider Board turns on when it detects a touch
*/
bool CAP1293::isInterruptEnabled() {
	INTERRUPT_ENABLE_REG reg;
	reg.INTERRUPT_ENABLE_COMBINED = readRegister(INTERRUPT_ENABLE);
	if (reg.INTERRUPT_ENABLE_FIELDS.CS1_INT_EN == 0x01 && /*reg.INTERRUPT_ENABLE_FIELDS.CS2_INT_EN == 0x01 SZ_MOD && */reg.INTERRUPT_ENABLE_FIELDS.CS3_INT_EN == 0x01) {
		return true;
	}
	return false;
}

/* SET SENSITIVITY
	Sensitivity calibrated for SparkFun Capacitive Touch Slider. You may
	want to change sensitivity settings if creating your own capacitive
	touch pads. See datasheet on Sensitivity Control Register
*/
void CAP1293::setSensitivity(uint8_t sensitivity) {
	SENSITIVITY_CONTROL_REG reg;
	reg.SENSITIVITY_CONTROL_COMBINED = readRegister(SENSITIVITY_CONTROL);
	if (sensitivity == SENSITIVITY_128X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_128X;
	} else if (sensitivity == SENSITIVITY_64X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_64X;
	} else if (sensitivity == SENSITIVITY_32X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_32X;
	} else if (sensitivity == SENSITIVITY_16X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_16X;
	} else if (sensitivity == SENSITIVITY_8X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_8X;
	} else if (sensitivity == SENSITIVITY_4X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_4X;
	} else if (sensitivity == SENSITIVITY_1X) {
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_1X;
	} else {
		// Default case: calibrated for CAP1293 touch sensor
		reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE = SENSITIVITY_2X;
	}
	writeRegister(SENSITIVITY_CONTROL, reg.SENSITIVITY_CONTROL_COMBINED);
}

/* GET SENSITIVITY
	Returns the sensitivity multiplier for current sensitivity settings
*/
uint8_t CAP1293::getSensitivity() {
	SENSITIVITY_CONTROL_REG reg;
	reg.SENSITIVITY_CONTROL_COMBINED = readRegister(SENSITIVITY_CONTROL);
	uint16_t sensitivity = reg.SENSITIVITY_CONTROL_FIELDS.DELTA_SENSE;
	if (sensitivity == SENSITIVITY_128X) {
		return 128;
	} else if (sensitivity == SENSITIVITY_64X) {
		return 64;
	} else if (sensitivity == SENSITIVITY_32X) {
		return 32;
	} else if (sensitivity == SENSITIVITY_16X) {
		return 16;
	} else if (sensitivity == SENSITIVITY_8X) {
		return 8;
	} else if (sensitivity == SENSITIVITY_4X) {
		return 4;
	} else if (sensitivity == SENSITIVITY_2X) {
		return 2;
	} else if (sensitivity == SENSITIVITY_1X) {
		return 1;
	} else {
		// Error - no possible register value
		return 0;
	}
}

/* LEFT SENSOR TOUCHED
	Checks if touch input detected on left sensor (pad 1). Need to clear
	interrupt pin after touch occurs. See datasheet on Sensor Interrupt
	Status Reg
*/
bool CAP1293::isLeftTouched() {
	SENSOR_INPUT_STATUS_REG reg;
	reg.SENSOR_INPUT_STATUS_COMBINED = readRegister(SENSOR_INPUT_STATUS);

	GENERAL_STATUS_REG status_reg;
	status_reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	// Touch detected
	if (status_reg.GENERAL_STATUS_FIELDS.TOUCH == ON && reg.SENSOR_INPUT_STATUS_FIELDS.CS1 == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

/* MIDDLE SENSOR TOUCHED
	Checks if touch input detected on left sensor (pad 2). Need to clear
	interrupt pin after touch occurs. See datasheet on Sensor Interrupt
	Status Reg
*/
bool CAP1293::isMiddleTouched() {
	SENSOR_INPUT_STATUS_REG reg;
	reg.SENSOR_INPUT_STATUS_COMBINED = readRegister(SENSOR_INPUT_STATUS);

	GENERAL_STATUS_REG status_reg;
	status_reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	// Touch detected
	if (status_reg.GENERAL_STATUS_FIELDS.TOUCH == ON && reg.SENSOR_INPUT_STATUS_FIELDS.CS2 == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

/* RIGHT SENSOR TOUCHED
	Checks if touch input detected on left sensor (pad 3). Need to clear
	interrupt pin after touch occurs. See datasheet on Sensor Interrupt
	Status Reg
*/
bool CAP1293::isRightTouched() {
	SENSOR_INPUT_STATUS_REG reg;
	reg.SENSOR_INPUT_STATUS_COMBINED = readRegister(SENSOR_INPUT_STATUS);

	GENERAL_STATUS_REG status_reg;
	status_reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);
	// Touch detected
	if (status_reg.GENERAL_STATUS_FIELDS.TOUCH == ON && reg.SENSOR_INPUT_STATUS_FIELDS.CS3 == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

uint8_t CAP1293::isLeftOrRightTouched() {

	//SENSOR_INPUT_STATUS_REG reg;
	//reg.SENSOR_INPUT_STATUS_COMBINED = readRegister(SENSOR_INPUT_STATUS);
	byte sensor_status = readRegister(SENSOR_INPUT_STATUS);

	// Touch detected
	//if (reg.SENSOR_INPUT_STATUS_FIELDS.CS1 == ON) { //Right is touched
	if (sensor_status == 1)
		return 0;
	// else if (reg.SENSOR_INPUT_STATUS_FIELDS.CS3 == ON) { //Left is touched
	else if (sensor_status == 4)
		return 1;

	return 2;
}

/* DETECT TOUCH
	Checks if touch input detected on any sensor. Need to clear
	interrupt pin after touch occurs. See datasheet on Sensor Interrupt
	Status
*/
bool CAP1293::isTouched() {
	GENERAL_STATUS_REG status_reg;
	status_reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	// Touch detected
	if (status_reg.GENERAL_STATUS_FIELDS.TOUCH == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

/* IS RIGHT SWIPE
	Checks if a right swipe occured on the board. This method
	takes up all functionality due to implementation of
	while loop with millis().
*/
bool CAP1293::isRightSwipePulled() {
	bool swipe = false; // Tracks if conditions are being met
	unsigned long startTime = millis();

	// LEFT TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isLeftTouched() == true) {
			swipe = true;
			while (isLeftTouched() == true)
				;  // Wait for user to remove their finger
			break; // Break out of loop
		}
	}

	// Return if left not touched
	if (swipe == false)
		return false;

	startTime = millis(); // Reset start time
	swipe = false;		  // Reset check statement

	// MIDDLE TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isMiddleTouched() == true) {
			swipe = true;
			while (isMiddleTouched() == true)
				;  // Wait for user to remove their finger
			break; // Break out of loop
		}
	}

	// Return if middle not touched
	if (swipe == false) {
		return false;
	}

	startTime = millis(); // Reset start time
	swipe = false;		  // Reset check statement

	// RIGHT TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isRightTouched() == true) {
			return true;
		}
	}

	return false;
}

/* IS LEFT SWIPE PULLED
	Checks if a left swipe occured on the board. This method
	takes up all functionality due to implementation of
	while loop with millis().
*/
bool CAP1293::isLeftSwipePulled() {
	bool swipe = false; // Tracks if conditions are being met
	unsigned long startTime = millis();

	// RIGHT TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isRightTouched() == true) {
			swipe = true;
			while (isRightTouched() == true)
				;  // Wait for user to remove their finger
			break; // Break out of loop
		}
	}

	// Return if right not touched
	if (swipe == false)
		return false;

	startTime = millis(); // Reset start time
	swipe = false;		  // Reset check statement

	// MIDDLE TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isMiddleTouched() == true) {
			swipe = true;
			while (isMiddleTouched() == true)
				;  // Wait for user to remove their finger
			break; // Break out of loop
		}
	}

	// Return if middle not touched
	if (swipe == false)
		return false;

	startTime = millis(); // Reset start time
	swipe = false;		  // Reset check statement

	// LEFT TOUCH CONDITION
	while ((millis() - startTime) < 100) {
		if (isLeftTouched() == true) {
			return true;
		}
	}

	return false;
}

/* SET POWER BUTTON PAD
	Sets a specific pad to act as a power button. Function takes in which
	pad to set as power button. See datasheet on Power Button
*/
bool CAP1293::setPowerButtonPad(uint8_t pad) {
	POWER_BUTTON_REG reg;
	reg.POWER_BUTTON_COMBINED = readRegister(POWER_BUTTON);

	// Set pad to act as power button (pg. 43)
	if (pad == PAD_LEFT) {
		reg.POWER_BUTTON_FIELDS.PWR_BTN = PWR_CS1;
	} else if (pad == PAD_MIDDLE) {
		reg.POWER_BUTTON_FIELDS.PWR_BTN = PWR_CS2;
	} else if (pad == PAD_RIGHT) {
		reg.POWER_BUTTON_FIELDS.PWR_BTN = PWR_CS3;
	} else {
		// User input invalid pad number
		return false;
	}
	writeRegister(POWER_BUTTON, reg.POWER_BUTTON_COMBINED);
	return true;
}

/* GET POWER BUTTON PAD
	Returns which capacitive touch pad is currently set to act
	as a power button.

	Add 1 to return value so value matches pad number.
	See data sheet on Power Button
		REG VALUE   PAD
		0x00        1
		0x01        2
		0x02        3
*/
uint8_t CAP1293::getPowerButtonPad() {
	POWER_BUTTON_REG reg;
	reg.POWER_BUTTON_COMBINED = readRegister(POWER_BUTTON);

	return (reg.POWER_BUTTON_FIELDS.PWR_BTN + 1);
}

/* SET POWER BUTTON TIME
	Configure the length of time that the designated power button
	must indicate a touch before an interrupt is generated and the
	power status indicator is set. See data sheet on Power Button
	Configuration Register

	Possible inputs (represent time in ms): 280, 560, 1120, 2240
*/
bool CAP1293::setPowerButtonTime(uint8_t inputTime) {
	POWER_BUTTON_CONFIG_REG reg;
	reg.POWER_BUTTON_CONFIG_COMBINED = readRegister(POWER_BUTTON_CONFIG);
	if (inputTime == PWR_TIME_280_MS) {
		reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME = PWR_TIME_280_MS;
	} else if (inputTime == PWR_TIME_560_MS) {
		reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME = PWR_TIME_560_MS;
	} else if (inputTime == PWR_TIME_1120_MS) {
		reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME = PWR_TIME_1120_MS;
	} else if (inputTime == PWR_TIME_2240_MS) {
		reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME = PWR_TIME_2240_MS;
	} else {
		// User input invalid time
		return false;
	}
	writeRegister(POWER_BUTTON_CONFIG, reg.POWER_BUTTON_CONFIG_COMBINED);
	return true;
}

/* GET POWER BUTTON TIME
	Returns the length of the time designated time power button must
	indicate a touch before an interrupt is generated.

	See data sheet on Power Button Time
		REG VALUE   TIME
		0x00        280 MS
		0x01        560 MS
		0x02        1120 MS
		0x03        2240 MS
*/
uint16_t CAP1293::getPowerButtonTime() {
	POWER_BUTTON_CONFIG_REG reg;
	reg.POWER_BUTTON_CONFIG_COMBINED = readRegister(POWER_BUTTON_CONFIG);
	if (reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME == PWR_TIME_280_MS) {
		return 280;
	} else if (reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME == PWR_TIME_560_MS) {
		return 560;
	} else if (reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME == PWR_TIME_1120_MS) {
		return 1120;
	} else if (reg.POWER_BUTTON_CONFIG_FIELDS.PWR_TIME == PWR_TIME_2240_MS) {
		return 2240;
	}
	// Invalid data reading - check hook up
	return 0;
}

/* SET POWER BUTTON ENABLED
	Enables power button in active state. See data sheet on Power Button
	Configuration Register
*/
void CAP1293::setPowerButtonEnabled() {
	POWER_BUTTON_CONFIG_REG reg;
	reg.POWER_BUTTON_CONFIG_COMBINED = readRegister(POWER_BUTTON_CONFIG);
	reg.POWER_BUTTON_CONFIG_FIELDS.PWR_EN = 0x01;
	writeRegister(POWER_BUTTON_CONFIG, reg.POWER_BUTTON_CONFIG_COMBINED);
}

/* SET POWER BUTTON DISABLED
	Disables power button in active state. See data sheet on Power Button
	Configuration Register
*/
void CAP1293::setPowerButtonDisabled() {
	POWER_BUTTON_CONFIG_REG reg;
	reg.POWER_BUTTON_CONFIG_COMBINED = readRegister(POWER_BUTTON_CONFIG);
	reg.POWER_BUTTON_CONFIG_FIELDS.PWR_EN = 0x00;
	writeRegister(POWER_BUTTON_CONFIG, reg.POWER_BUTTON_CONFIG_COMBINED);
}

/* IS POWER BUTTON ENABLED
	Returns state of power button. Returns true if enabled (reg. value is
	0x01), otherwise returns false. Power button must be ENABLED to use.
	See data sheet on Power Button Configuration Register
*/
bool CAP1293::isPowerButtonEnabled() {
	POWER_BUTTON_CONFIG_REG reg;
	reg.POWER_BUTTON_CONFIG_COMBINED = readRegister(POWER_BUTTON_CONFIG);
	if (reg.POWER_BUTTON_CONFIG_FIELDS.PWR_EN == 0x01) {
		// Power button enabled
		return true;
	}
	// Power button disabled
	return false;
}

/* IS POWER BUTTON TOUCHED
	Once the power button has been held for designated time, an interrupt
	is generated and PWR bit is set in the General Status Register. See
	data sheet on Power Button (pg. 16), Power Button Register,
	and Power Button Configuration Register
*/
bool CAP1293::isPowerButtonTouched() {
	GENERAL_STATUS_REG reg;
	reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	if (reg.GENERAL_STATUS_FIELDS.PWR == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

uint8_t CAP1293::getTouchKeyStatus(bool keyStatus[], uint8_t num) {
	GENERAL_STATUS_REG status_reg;
	status_reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	uint8_t keyChanged = 0;
	uint8_t key = 0;
	uint8_t keyFilter = 0;

	// Touch detected
	if (status_reg.GENERAL_STATUS_FIELDS.TOUCH == ON) {
		key = readRegister(SENSOR_INPUT_STATUS);
		clearInterrupt();

		delay(45); // donnot delete

		keyFilter = readRegister(SENSOR_INPUT_STATUS);
		clearInterrupt();

		if (key & 0x01) // left key 
		{
			keyChanged |= 0x01;
			if (keyFilter & 0x01) {
				keyStatus[0] = true;
			} else {
				keyStatus[0] = false;
			}
		}

		if (key & 0x02) // middle key 
		{
			keyChanged |= 0x02;
			if (keyFilter & 0x02) {
				keyStatus[1] = true;
			} else {
				keyStatus[1] = false;
			}
		}

		if (key & 0x04) // right key
		{
			keyChanged |= 0x04;
			if (keyFilter & 0x04) {
				keyStatus[2] = true;
			} else {
				keyStatus[2] = false;
			}
		}
	}

	return keyChanged;
}


/* Enable MULTIPLE TOUCH
	Checks if multi touch input detected.
	See datasheet on Sensor Interrupt Status
*/
void CAP1293::setMultiTouchEnabled() {
	// config
	MTP_CONFIG_REG config;
	config.MTP_CONFIG_COMBINED = readRegister(MULTIPLE_TOUCH_CONFIG);

	config.MTP_CONFIG_FIELDS.MULT_BLK_EN = 0;

	writeRegister(MULTIPLE_TOUCH_CONFIG, config.MTP_CONFIG_COMBINED);
}

/* Disable MULTIPLE TOUCH
	Checks if multi touch input detected.
*/
void CAP1293::setMultiTouchDisabled() {
	MTP_CONFIG_REG config;
	config.MTP_CONFIG_COMBINED = readRegister(MULTIPLE_TOUCH_CONFIG);

	config.MTP_CONFIG_FIELDS.MULT_BLK_EN = 1;

	writeRegister(MULTIPLE_TOUCH_CONFIG, config.MTP_CONFIG_COMBINED);
}

/* Enable MTP
*/
void CAP1293::setMTPEnabled(bool left, bool middle, bool right) {
	MTP_ENABLE_REG reg;
	reg.MTP_ENABLE_COMBINED = readRegister(MULTIPLE_TOUCH_PATTERN_CONFIG);
	reg.MTP_ENABLE_FIELDS.MTP_EN = 1;
	//reg.MTP_ENABLE_FIELDS.COMP_PTRN = 1;
	//reg.MTP_ENABLE_FIELDS.MTP_ALERT = 1;

	writeRegister(MULTIPLE_TOUCH_PATTERN_CONFIG, reg.MTP_ENABLE_COMBINED);

	MULTIPLE_TOUCH_PATTERN_REG mtp_reg;
	mtp_reg.MULTIPLE_TOUCH_PATTERN_COMBINED = readRegister(MULTIPLE_TOUCH_PATTERN);
	mtp_reg.MULTIPLE_TOUCH_PATTERN_FIELDS.CS1_PTRN = left ? 1 : 0;
	mtp_reg.MULTIPLE_TOUCH_PATTERN_FIELDS.CS2_PTRN = middle ? 1 : 0;
	mtp_reg.MULTIPLE_TOUCH_PATTERN_FIELDS.CS3_PTRN = right ? 1 : 0;

	writeRegister(MULTIPLE_TOUCH_PATTERN, mtp_reg.MULTIPLE_TOUCH_PATTERN_COMBINED);
}

/* is MTP
	Checks if multi touch input detected.
	See datasheet on Sensor Interrupt Status
*/
bool CAP1293::isMTPStatus() {
	GENERAL_STATUS_REG reg;
	reg.GENERAL_STATUS_COMBINED = readRegister(GENERAL_STATUS);

	if (reg.GENERAL_STATUS_FIELDS.MTP == ON) {
		clearInterrupt();
		return true;
	}
	return false;
}

void CAP1293::enableMultitouch() {

	MTP_CONFIG_REG mtp_config; //Register 0x2a
	mtp_config.MTP_CONFIG_COMBINED = 0;
	mtp_config.MTP_CONFIG_FIELDS.MULT_BLK_EN = 1; //Raise flag on multiple touches
	mtp_config.MTP_CONFIG_FIELDS.B_MULT_T = 0; //1 = 2 simultaneous touches

	MTP_ENABLE_REG mtp_enable; //Register 0x2b
	mtp_enable.MTP_ENABLE_COMBINED = 0;

	writeRegister(MULTIPLE_TOUCH_CONFIG, mtp_config.MTP_CONFIG_COMBINED); //Enable multitouch for 2 inputs threshold
	writeRegister(MULTIPLE_TOUCH_PATTERN_CONFIG, mtp_enable.MTP_ENABLE_COMBINED);
}

/* DISABLE RELEASE INTERRUPTS
	Disable Interrupt on release
*/
void CAP1293::setReleaseInterruptDisabled() {
	byte controlReg2 = readRegister(CONFIG_2);
	controlReg2 = controlReg2 | 0b00000001;
	writeRegister(CONFIG_2, controlReg2);
}

/* Enable RELEASE INTERRUPTS
	Disable Interrupt on release
*/
void CAP1293::setReleaseInterruptEnabled() {
	byte controlReg2 = readRegister(CONFIG_2);
	controlReg2 = controlReg2 & 0b11111110;
	writeRegister(CONFIG_2, controlReg2);
}

/* Get Release Interrupt Enable status
*/
bool CAP1293::isReleaseInterruptEnabled() {
	byte controlReg2 = readRegister(CONFIG_2);
	if ((controlReg2 & 0b00000001) == 1) {
		return false;
	} else {
		return true;
	}
}


/* DISABLE Deep Sleep
   This turns on all sensor out of Deep Sleep state.
	See data sheet on main control register
*/
void CAP1293::setDeepSleepDisabled() {
	MAIN_CONTROL_REG reg;
	reg.MAIN_CONTROL_COMBINED = readRegister(MAIN_CONTROL);

	reg.MAIN_CONTROL_FIELDS.DSLEEP = 0x0;

	writeRegister(MAIN_CONTROL, reg.MAIN_CONTROL_COMBINED);
}

/* ENABLE Deep Sleep
   This turns on all sensor entering Deep Sleep and disabled all touch input scanning.
	See data sheet on main control register
*/
void CAP1293::setDeepSleepEnabled() {
	MAIN_CONTROL_REG reg;
	reg.MAIN_CONTROL_COMBINED = readRegister(MAIN_CONTROL);

	reg.MAIN_CONTROL_FIELDS.DSLEEP = 0x1;

	writeRegister(MAIN_CONTROL, reg.MAIN_CONTROL_COMBINED);
}

/* IS INTERRUPT ENABLED
	Returns whether touch intterupt is enable or not
	See data sheet on main control register
*/
bool CAP1293::isDeepSleepEnabled() {
	MAIN_CONTROL_REG reg;
	reg.MAIN_CONTROL_COMBINED = readRegister(MAIN_CONTROL);

	if (reg.MAIN_CONTROL_FIELDS.DSLEEP == 0x1) {
		return true;
	}

	return false;
}

/*  CLEAR STATUS REGISTERS
	at sometime, the input status cannot clear After clearing INT bit.
	See data sheet on 5.2.2 SENSOR INPUT STATUS.
*/
void CAP1293::clearStatus() {
	setDeepSleepEnabled();
	setDeepSleepDisabled();
}

/* Disabled Force Calibrate
	See data sheet on CALIBRATION_ACTIVATE_AND_STATUS register
*/
void CAP1293::setForceCalibrateDisabled() {
	CALIBRATION_ACTIVATE_AND_STATUS_REG reg;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_COMBINED = readRegister(CALIBRATION_ACTIVATE_AND_STATUS);

	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS1_CAL = 0x00;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS2_CAL = 0x00;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS3_CAL = 0x00;

	writeRegister(CALIBRATION_ACTIVATE_AND_STATUS, reg.CALIBRATION_ACTIVATE_AND_STATUS_COMBINED);
}

/* Enable Force Calibrate
   This forces the selected sensor inputs to be calibrated.
	See data sheet on CALIBRATION_ACTIVATE_AND_STATUS register
*/
void CAP1293::setForceCalibrateEnabled() {
	CALIBRATION_ACTIVATE_AND_STATUS_REG reg;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_COMBINED = readRegister(CALIBRATION_ACTIVATE_AND_STATUS);

	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS1_CAL = 0x01;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS2_CAL = 0x01;
	reg.CALIBRATION_ACTIVATE_AND_STATUS_FIELDS.CS3_CAL = 0x01;

	writeRegister(CALIBRATION_ACTIVATE_AND_STATUS, reg.CALIBRATION_ACTIVATE_AND_STATUS_COMBINED);
}

/* Disable REPEAT RATE
   Disables the repeat rate of the sensor inputs.
	See data sheet on REPEAT RATE ENABLE register
*/
void CAP1293::setRepeatRateDisabled() {
	REPEAT_RATE_ENABLE_REG reg;
	reg.REPEAT_RATE_ENABLE_COMBINED = readRegister(REPEAT_RATE_ENABLE);

	reg.REPEAT_RATE_ENABLE_FIELDS.CS1_RPT_EN = 0x00;
	reg.REPEAT_RATE_ENABLE_FIELDS.CS2_RPT_EN = 0x00;
	reg.REPEAT_RATE_ENABLE_FIELDS.CS3_RPT_EN = 0x00;

	writeRegister(REPEAT_RATE_ENABLE, reg.REPEAT_RATE_ENABLE_COMBINED);
}

/* Enable REPEAT RATE
   Enable the repeat rate of the sensor inputs.
	See data sheet on REPEAT RATE ENABLE register
*/
void CAP1293::setRepeatRateEnabled() {
	REPEAT_RATE_ENABLE_REG reg;
	reg.REPEAT_RATE_ENABLE_COMBINED = readRegister(REPEAT_RATE_ENABLE);

	reg.REPEAT_RATE_ENABLE_FIELDS.CS1_RPT_EN = 0x01;
	reg.REPEAT_RATE_ENABLE_FIELDS.CS2_RPT_EN = 0x01;
	reg.REPEAT_RATE_ENABLE_FIELDS.CS3_RPT_EN = 0x01;

	writeRegister(REPEAT_RATE_ENABLE, reg.REPEAT_RATE_ENABLE_COMBINED);
}

/* READ A SINGLE REGISTER
	Read a single byte of data from the CAP1293 register "reg"
*/
byte CAP1293::readRegister(CAP1293_Register reg) {
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);				// endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, (byte)1); // Ask for 1 byte, once done, bus is released by default

	// Wait for the data to come back
	if (_i2cPort->available()) {
		return _i2cPort->read(); // Return this one byte
	} else {
		return 0;
	}
}

/* READ MULTIPLE REGISTERS
	Read "en" bytes from the CAP1293, starting at register "reg." Bytes are
	stored in "buffer" on exit.
*/
void CAP1293::readRegisters(CAP1293_Register reg, byte *buffer, byte len) {
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);			// endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, len); // Ask for bytes, once done, bus is released by default

	// Wait for data to come back
	if (_i2cPort->available() == len) {
		// Iterate through data from buffer
		for (int i = 0; i < len; i++)
			buffer[i] = _i2cPort->read();
	}
}

/* WRITE TO A SINGLE REGISTER
	Wire a single btyte of data to a register in CAP1293
*/
void CAP1293::writeRegister(CAP1293_Register reg, byte data) {
	writeRegisters(reg, &data, 1);
}

/* WRITE TO MULTIPLE REGISTERS
	Write an array of "len" bytes ("buffer"), starting at register "reg,"
	and auto-incementing to the next
*/
void CAP1293::writeRegisters(CAP1293_Register reg, byte *buffer, byte len) {
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	for (int i = 0; i < len; i++)
		_i2cPort->write(buffer[i]);
	_i2cPort->endTransmission(); // Stop transmitting
}