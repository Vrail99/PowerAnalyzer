| <center><img src="./assets/rakstar.jpg" alt="RAKstar" width=25%></center>  | ![RAKWireless](./assets/RAK-Whirls.png) | [![Build Status](https://github.com/RAKWireless/RAK14002-CAP1293-Library/workflows/RAK%20Library%20Build%20CI/badge.svg)](https://github.com/RAKWireless/RAK14002-CAP1293-Library/actions) |
| -- | -- | -- |

# RAK14002 Capacitive Touch Button

| _**Based on [SparkFun Qwiic Capacitive Touch Slider - CAP1203](https://github.com/sparkfun/Qwiic_Capacitive_Touch_Slider_Arduino_Library).**_  |
| -- |    
| _**Changed to work with the CAP1293 touch pad chip.**_  |

<center><img src="./assets/touchpad.png" alt="RAKstar" width=50%></center>

[*RAKwireless RAK14002 Capacitive Touch Button*](https://store.rakwireless.com/products/3-channel-touch-pad-module-rak14002)

Arduino library for the I2C based capacitive touch pad module with 3 unique touch inputs. 

# Documentation

* **[Product Repository](https://github.com/RAKWireless/RAK14002-CAP1293-Library)** - Product repository for the RAKWireless RAK14002 Capacitive Touch Pad.
* **[Documentation](https://docs.rakwireless.com/Product-Categories/WisBlock/RAK14002/Overview/)** - Documentation and Quick Start Guide for the RAK14002 Capacitive Touch Pad.

# Installation

In Arduino IDE open Sketch->Include Library->Manage Libraries then search for RAK14002.    

In PlatformIO open PlatformIO Home, switch to libraries and search for RAK14002. 
Or install the library project depend by adding 
```log
lib_deps =
  rakwireless/RAKwireless CAP1293 Touch Pad library
```
into **`platformio.ini`**

For manual installation download the archive, unzip it and place the RAK14002-CAP1293-Library folder into the library directory.    
In Arduino IDE this is usually <arduinosketchfolder>/libraries/     
In PlatformIO this is usually <user/.platformio/lib>     

# Usage

The library provides a CAP1293 class, which allows communication to the CAP1293 touch button controller IC. Check out the examples how to use the library.

## This class provides the following methods:

**CAP1293(byte addr = CAP1293_I2C_ADDR);**     
Constructor for CAP1293 interface. Initialize touch button controller object    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | addr | I2C address of CAP1293 |
|  return |  | none  | 

**bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = CAP1293_I2C_ADDR);**     
Initialize the library. Optional parameters for device specific I2C bus and I2C address.        
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | wirePort | Hardware I2C port |
| in        | deviceAddress | I2C address of CAP1293 | 
|  return |  | Returns true if CAP1293 was detected. Otherwise returns false.  | 

**bool isConnected();`**     
Check if touch button module is connected.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | Returns true if the I2C Device acknowledges a connection. Otherwise returns false.  | 


**void setSensitivity(uint8_t sensitivity);**    
Sensitivity control. You may want to change sensitivity settings if creating your own capacitive touch pads. See datasheet on Sensitivity Control Register    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | sensitivity | Sensitivity (see below list)  |
|  return |  | void  | 

| Sensitivity level   | Defined name     | Value | 
| ------------------- | ---------------- | ----- |
| Highest sensitivity | SENSITIVITY_128X | 0x00 |
| High sensitivity | SENSITIVITY_64X | 0x01 |
| Higher sensitivity | SENSITIVITY_32X | 0x02 |
| Medium sensitivity | SENSITIVITY_16X | 0x03 |
| Lower sensitivity | SENSITIVITY_8X | 0x04 | 
| Low sensitivity | SENSITIVITY_4X | 0x05 |
| Low sensitivity | SENSITIVITY_2X | 0x06 |
| Lowest sensitivity | SENSITIVITY_1X | 0x07 |

**uint8_t getSensitivity();**    
Returns the current sensitivity sensitivity settings    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | Sensitivity level (see below list)  | 

| Sensitivity level   | Defined name     | Value | 
| ------------------- | ---------------- | ----- |
| Highest sensitivity | SENSITIVITY_128X | 0x00 |
| High sensitivity | SENSITIVITY_64X | 0x01 |
| Higher sensitivity | SENSITIVITY_32X | 0x02 |
| Medium sensitivity | SENSITIVITY_16X | 0x03 |
| Lower sensitivity | SENSITIVITY_8X | 0x04 | 
| Low sensitivity | SENSITIVITY_4X | 0x05 |
| Low sensitivity | SENSITIVITY_2X | 0x06 |
| Lowest sensitivity | SENSITIVITY_1X | 0x07 |


### Single Touch Functions

**bool isLeftTouched();**     
Checks if touch input detected on left sensor (pad 1). It is required to clear the interrupt pin after touch occurs. See datasheet on Sensor Interrupt Status Reg    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | True of touch was detected, otherwise false  | 

**bool isMiddleTouched();**    
Checks if touch input detected on middle sensor (pad 2). It is required to clear the interrupt pin after touch occurs. See datasheet on Sensor Interrupt Status Reg     
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | True of touch was detected, otherwise false  | 

**bool isRightTouched();**    
Checks if touch input detected on right sensor (pad 3). It is required to clear the interrupt pin after touch occurs. See datasheet on Sensor Interrupt Status Reg    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | True of touch was detected, otherwise false  | 

**bool isTouched();**    
Checks if touch input detected on any of the sensors. It is required to clear the interrupt pin after touch occurs. See datasheet on Sensor Interrupt Status Reg       
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return |  | True of touch was detected, otherwise false  | 

**uint8_t getTouchKeyStatus(bool keyStatus[], uint8_t num=3);**    
Checks for changes on the touch status of all three pads.   
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| out       | keyStatus | boolean array to hold the status of the three buttons |
| in        | num | size of boolean array keyStatus |
|  return | uint8_t | 0 if the status of all pads is unchanged or > 0 if any pad status has changed  | 

The status is returned in bool keyStatus[], where    
keyStatus[0] = true if left button is touched, else false    
keyStatus[1] = true if middle button is touched, else false    
keyStatus[2] = true if right button is touched, else false    
Return value is 0 if the status of all pads is unchanged or > 0 if any pad status has changed.    

### Swipe Detect Functions

**bool isRightSwipePulled();**    
Checks if a right swipe occurred on the board. This method is blocking and waits for 100ms to detect the start of a swipe motion.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | num | size of boolean array keyStatus |
|  return | bool | True if right swipe was detected  | 

**bool isLeftSwipePulled();**    
Checks if a left swipe occurred on the board. This method is blocking and waits for 100ms to detect the start of a swipe motion.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | num | size of boolean array keyStatus |
|  return | bool | True if left swipe was detected  | 

### Interrupt Control functions

**void clearInterrupt();**    
Clears the interrupt (INT) bit by writing a logic 0 to it. This bit must be cleared in order to detec a new capacitive touch input. See datasheet on Main Control Register    

**void setInterruptDisabled();**    
This disables all the interrupts, so the interrupt line is not set when a sensor is touched. Set on default in begin() function. See data sheet on Interrupt Enable Register

**void setInterruptEnabled();**        
This turns on all the interrupts, so the interrupt line is set when any sensor is touched. See data sheet on Interrupt Enable Register.    

**bool isInterruptEnabled();**    
Returns state of interrupt pin. Returns true if all interrupts enabled (0x07), otherwise returns false.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if all interrupts enabled, otherwise returns false  | 

**void setReleaseInterruptDisabled();**    
Disable interrupt on touch release.     

**void setReleaseInterruptEnabled();**    
Enable interrupt on touch release.    

**bool isReleaseInterruptEnabled();**    
Get current status of touch release interrupt. Returns true if interrupt on touch release is enabled, otherwise false.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if touch release interrupt is enabled, otherwise false  | 

### Sleep Functions

**void setDeepSleepDisabled();**    
Wake sensor up from deep sleep status.    

**void setDeepSleepEnabled();**    
Put sensor in deep sleep status. In deep sleep no touch events are detected.     

**bool isDeepSleepEnabled();**    
Check if sensor is in deep sleep. Returns true if sensor is in deep sleep, otherwise false.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if sensor is in deep sleep, otherwise false  | 

### Status Functions

**uint8_t checkMainControl();**    
Control the primary power state of the device. See data sheet on Main Control Register.    
Returns content of power status register
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | uint8_t | Content of power status register  | 

**uint8_t checkStatus();**    
Checks inputs in the general status register to ensure program is set up correctly. See data sheet on Status Registers
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | uint8_t | Content of status register  | 

### Power Button Functions

**bool setPowerButtonPad(uint8_t pad);**    
Sets a specific pad to act as a power button. Function parameter defines the pad. Allowed values `PWR_CS1` (left), `PWR_CS2` (middle), `PWR_CS3` (right). See datasheet on Power Button     
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | pad | Pad to be used as power button |
|  return | bool | True if success, false if invalid pad number was given  | 

**bool setPowerButtonTime(uint8_t time);**    
Configure the length of time that the designated power button must indicate a touch before an interrupt is generated and the power status indicator is set. See data sheet on Power Button Configuration Register.    
Possible inputs (represent time in ms): 280, 560, 1120, 2240    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | time | Power button detection time |
|  return | bool | True if success, false if invalid time was given  | 

**uint8_t getPowerButtonPad();**    
Returns which capacitive touch pad is currently set to act as a power button. Returned values are `PWR_CS1` (left), `PWR_CS2` (middle) or `PWR_CS3` (right)
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | uint8_t | Pad number currently assigned as power button  | 

**uint16_t getPowerButtonTime();**    
Returns the length of the time designated time power button must indicate a touch before an interrupt is generated. Return value is in milliseconds.
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | uint16_t | Power button detect time in milli seconds  | 

**void setPowerButtonEnabled();**    
Enables power button in active state. See data sheet on Power Button Configuration Register    

**void setPowerButtonDisabled();**
Disables power button in active state. See data sheet on Power Button Configuration Register    

**bool isPowerButtonEnabled();**    
Returns state of power button. Returns true if enabled, otherwise returns false.See data sheet on Power Button Configuration Register     
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if enabled, otherwise returns false  | 

**bool isPowerButtonTouched();**    
Once the power button has been held for designated time, an interrupt is generated and PWR bit is set in the General Status Register. See data sheet on Power Button, Power Button Register, and Power Button Configuration Register    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if button was touched, otherwise returns false  | 

### Multi Touch Functions

**void setMultiTouchEnabled();**     
Enables multi touch detection. See datasheet on Sensor Interrupt Status    

**void setMultiTouchDisabled();**    
Disables multi touch detection. See datasheet on Sensor Interrupt Status    

### Calibration Functions

**void setForceCalibrateEnabled();**    
Enable automatic force calibration. See data sheet on CALIBRATION_ACTIVATE_AND_STATUS register    

**void setForceCalibrateDisabled();**    
Disable automatic force calibration. See data sheet on CALIBRATION_ACTIVATE_AND_STATUS register    

### Repeat Control Functions

**void setRepeatRateEnabled();**    
Enables the repeat rate of the sensor inputs. See data sheet on REPEAT RATE ENABLE register     

**void setRepeatRateDisabled();**    
Disables the repeat rate of the sensor inputs. See data sheet on REPEAT RATE ENABLE register     

### Multi Touch Pattern Functions

**void setMTPEnabled(bool left=true, bool middle=true, bool right=true);**     
Enables multi touch pattern detection. Parameters left, middle and right define which pads are used for the multi touch pattern detection.     
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
| in        | left | True => use left button for MTP, false => ignore left button |
| in        | middle | True => use middle button for MTP, false => ignore middle button |
| in        | right | True => use right button for MTP, false => ignore right button |

**bool isMTPStatus();**    
Checks if multi touch pattern was detected.    
Returns true if the pattern was detected, otherwise false.    
Parameters:    
| Direction | Name | Function | 
| --------- | ---- | -------- |
|  return | bool | True if MTP was touched, otherwise returns false  | 

License Information    
-------------------

This library is based on [SparkFun Qwiic Capacitive Touch Slider - CAP1203](https://github.com/sparkfun/Qwiic_Capacitive_Touch_Slider_Arduino_Library) which is published under MIT license. See original license in the [Sparkfun repository](https://github.com/sparkfun/Qwiic_Capacitive_Touch_Slider_Arduino_Library)
