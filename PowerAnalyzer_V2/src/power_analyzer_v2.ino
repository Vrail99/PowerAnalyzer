/*
Teensy 4.0 Firmware for a Power Analyzer
Author: Sebastian Zisch
Date: 220320

MIT License

Copyright (c) 2021 Sebastian Zisch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <arm_math.h> //Include of DSP library
#include <TimeLib.h>  //Time Library for Real-Time Clock
#include <math.h>
#include <EEPROM.h>

// External Libraries
#include "OLED_Display.h" //Display Library
#include "Watchdog_t4.h"  //Watchdog Library for reset on error
#include "CAP1293.h"
#include "PA_DataLogger.h"

#include "PowerAnalyzer.h" //Includes ACS-Chip, pin and valuedefines
#include "harmonicCalculations.h"

// Touch-Controller
uint8_t currPage = 0;
uint8_t menuOption = 0;
const char *optionNames[MENU_OPTIONS] = {"V&C RMS", "POWER", "THD", "PWR_FAC", "ENERGY", "END"};

// Values predefined if EEPROM-read error
float conv_factor_VRMS = 8.384 * powf(10, -6);
float conv_factor_IRMS = 1.015 * powf(10, -3);

// Creation of a Chip instance
ACS71020 ACSchip(ACS_SPI_SPEED, ACS_CS, ACS_EN);

// Display instance
Adafruit_SSD1327 display(DISP_WIDTH, DISP_HEIGHT, &Wire, OLED_RST, 1000000);

DataLogger logger(&ACSchip, &SerialUSB1, SD_CS_PIN);

// Touch Sensor instance
CAP1293 touchSensor;
volatile boolean touchHappened = false;

// Character arrays for input commands
#define MAX_RECEIVEABLE 32 // Number of characters that can be read
char receivedChars[MAX_RECEIVEABLE];

const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

////////////////////////////////////////////////////////////////////////////////
// Volatile Memory Values
////////////////////////////////////////////////////////////////////////////////

// Timer for acquiring samples
IntervalTimer smplTimer;
boolean samplingDone = false;
uint32_t sampleCntr = 0;

uint16_t DISPLAY_UPD_RATE = 1000; // Display update time in Milliseconds

// Flags for enabling Streaming
bool sampling = false;
bool powerSampling = false;
bool calcFFT = false;
uint8_t runmode = 0;            // 0: Idle, 1: Voltage/Power, 2: Current, 3: Voltage & Current
bool computerConnected = false; // Display flag

// ARM_MATH FFT PARAMETERS
const uint16_t ifftFlag = 0;
arm_cfft_radix4_instance_f32 fftInstance;

// Buffer Arrays, Initialized to 0
// PC Sample Buffers
uint32_t vCodeBuffer[SAMPBUFFLEN] = {0};
uint32_t iCodeBuffer[SAMPBUFFLEN] = {0};

// Teensy Sample Buffers
float vSamps[FFTBUFFLEN] = {0.0};
float iSamps[FFTBUFFLEN] = {0.0};
float Mags[FFTLEN] = {0.0};

float zcd_thresh = 0.2;

struct PowerQuality pwr_qual = {0};

bool inCalibration = false;
uint8_t dataLogging = 0;
bool voltage_detected = true;

// ReadTimes
uint32_t newTime, streamTime, displayTime, logTime; // Time-Tracker, seconds-timer, minute-timer

// Instantiate Watchdogtimer 3
WDT_T4<WDT3> wdt;
uint32_t checkTime;
uint32_t wdTimeout = 30000; // ms

void checkCallback() {}

// Touch-IRQ
void touch_IRQ()
{
  touchHappened = true;
}

// Touch-Input handler for non-datalogging mode
void touchHandler_Default()
{
  if (computerConnected)
    return;

  uint8_t multitouch = (touchSensor.checkStatus() >> 2) & 0x1;

  if (multitouch)
  { // Both buttons pressed at the same time
    if (!dataLogging)
    {
      dataLogging = true;
      menuOption = 0;
    }
  }
  else
  {                                                         // Standard Menu
    uint8_t touchMode = touchSensor.isLeftOrRightTouched(); // 0: right, 1: left
    if (touchMode == 0 && currPage < NO_OF_PAGES)
    {
      currPage++;
    }
    else if (touchMode == 1 && currPage > 0)
      currPage--;
#ifdef DEBUG
    SerialUSB1.printf("CurrPage: %d. Sampling: %d\n", currPage, sampling);
#endif
  }
  touchSensor.clearInterrupt();
  calcFFT = false;
  switch (currPage)
  {
  case DISP_FFT_VOLT:
    calcFFT = true;
    ACSchip.activateVoltageAveraging();
    startSamplingFFT();
    break;
  case DISP_FFT_CURRENT:
    calcFFT = true;
    ACSchip.activateVoltageAveraging();
    startSamplingFFT();
    break;
  case DISP_VOLT:
    ACSchip.activateVoltageAveraging();
    break;
  case DISP_CURRENT:
    ACSchip.activateCurrentAveraging();
    break;
  case DISP_POWER:
    ACSchip.activateVoltageAveraging();
  }
  touchHappened = false;
}

void touchHandler_DataLogging()
{

  uint8_t multitouch = (touchSensor.checkStatus() >> 2) & 0x1;

  if (multitouch)
  { // Both buttons pressed at the same time
    switch (menuOption)
    {
    case 0:
    {
      logger.Log_RMS_VandC(true);
      break;
    }
    case 1:
    {
      SerialUSB1.println("Menu1 chosen");
      File a;
      logger.printDirectory(a, 0);
      break;
    }
    case 2:
    {
      SerialUSB1.println("Menu2 chosen");
      logger.LogAll();
      break;
    }
    case 3:
    {
      SerialUSB1.println("Menu3 chosen");
      break;
    }
    case 4:
    {
      SerialUSB1.println("Menu4 chosen");
      break;
    }
    case 5:
    {
      dataLogging = false;
      SerialUSB1.println("Exit DataLogging");
      break;
    }
    }
  }
  else
  {
    uint8_t touchMode = touchSensor.isLeftOrRightTouched(); // 0: right, 1: left
    if (touchMode == 0 && menuOption < MENU_OPTIONS)
    {
      menuOption++;
    }
    else if (touchMode == 1 && menuOption > 0)
      menuOption--;

    SerialUSB1.printf("MenuOption: %d\n", menuOption);
  }
  touchSensor.clearInterrupt();
  touchHappened = false;
}

Sd2Card _card;
SdVolume _volume;
SdFile _root;

void setup()
{
  // Configuration of watchdog Timer
  WDT_timings_t config;
  config.timeout = wdTimeout; /* in ms, 32ms to 522.232s */
  config.callback = checkCallback;
  wdt.begin(config);

  // Init Serial communication
  // Dual Channel Serial Communication
  Serial.begin(115200);     // Main transmit port
  SerialUSB1.begin(115200); // Debug and status port

#ifdef DEBUG
  while (!SerialUSB1)
  {
    yield();
  }
#endif
  printDebug("Initializing ACS-Chip");
  while (!ACSchip.init())
  {
    SerialUSB1.println("Unable to init ACS-Chip");
    delay(500);
  }
  printDebug("ACS-Chip initialized");

  // Display Initialization
  printDebug("Initializing Display");
#ifdef INIT_DISP
  while (!display.begin(0x3D))
  {
    SerialUSB1.println("Unable to initialize OLED");
    delay(500);
  }
  display.setTextSize(2);
  display.clearDisplay();
  display.display();
  printDebug("Display initialized");
#endif
  delay(500);
  printDebug("Initializing SD Card");
#ifdef INIT_SDCARD
  while (!logger.init())
  {
    SerialUSB1.println("SD initialization failed!");
    delay(5000);
  }
  printDebug("SD-Card initialized");
#endif

  printDebug("Touch Initialized");
#ifdef INIT_TOUCH
  while (!touchSensor.begin())
  {
    SerialUSB1.println("Unable to connect to TouchSensor");
    delay(500);
  }
  printDebug("Touch Sensor initialized");
  // Touch-PIN configure
  pinMode(CAP_IRQ, INPUT); // Pulled up in hardware
  attachInterrupt(digitalPinToInterrupt(CAP_IRQ), touch_IRQ, FALLING);
  touchSensor.setSensitivity(SENSITIVITY_64X);
  touchSensor.enableMultitouch();
#endif

  // Uncomment the next two rows, if accidentally written values too large or made a factory reset
  // MCU_write(VRMS_cal_address, 21280, false);
  // MCU_write(IRMS_cal_address, 257, false);

  // Read current calibration factors from the chip
  uint32_t tmpInt = MCU_read(VRMS_cal_address, false); // Read VRMS conversionFactor
  conv_factor_VRMS = ConvertUnsignedFixedPoint(tmpInt, 23, 24);
  tmpInt = MCU_read(IRMS_cal_address, false);
  conv_factor_IRMS = ConvertUnsignedFixedPoint(tmpInt, 23, 24);

  ACSchip.setConversionFactors(conv_factor_VRMS, conv_factor_IRMS);
#ifdef DEBUG
  SerialUSB1.printf("Init of Conversion Factors:\n VRMS-conv-factor:%e\n IRMS-conv-factor:%e\n", conv_factor_VRMS, conv_factor_IRMS);
#endif
  // Init of FFT Instance
  arm_status fft_err = arm_cfft_radix4_init_f32(&fftInstance, FFTLEN, 0, 1);
  if (fft_err == ARM_MATH_ARGUMENT_ERROR)
  {
    SerialUSB1.println("FFT Len Error");
    calcFFT = false;
  }
  delay(100);

  // Initialization of timers
  streamTime = millis();
  displayTime = millis();
  checkTime = millis();
  // End of Init
}

void loop()
{
  newTime = millis(); // Get current millis for timing

  if (samplingDone)
  {
    if (runmode == 1) // Transmit all voltage samples
    {
      transmitIntegerSamples(&Serial, vCodeBuffer, SAMPBUFFLEN);
      startSamplingPC();
    }
    else if (runmode == 2) // Transmit all current samples
    {
      transmitIntegerSamples(&Serial, iCodeBuffer, SAMPBUFFLEN);
      startSamplingPC();
    }
    else if (runmode == 3) // Transmit alls current and voltage samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u,%u\n", vCodeBuffer[i], iCodeBuffer[i]);
      startSamplingPC();
    }
    // Calculate FFT and the harmonic distortion
    else if (calcFFT)
    {
      arm_cfft_radix4_f32(&fftInstance, vSamps);     // In-Place FFT
      float angle_v = atan2(vSamps[21], vSamps[20]); // Angle from pure spectrum
      arm_cmplx_mag_f32(vSamps, Mags, FFTLEN);       // Calculate Magnitudes
      pwr_qual.thd_v = calcTHD(Mags, 17, BINSIZE);   // Calculate Total Harmonic Distortion
      if (pwr_qual.grouping_en)                      // If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = {0};
        pwr_qual.thdg_v = calcTHDG(Mags, fgroups, 17);
        pwr_qual.thdsg_v = calcTHDSG(Mags, fgroups, 17);
      }

      if (currPage == DISP_FFT_VOLT)
        displayFFT(&display, Mags, currPage);

      // FFT for Current samples
      arm_cfft_radix4_f32(&fftInstance, iSamps); // In-Place FFT
      float angle_i = atan2(iSamps[21], iSamps[20]);
      pwr_qual.phaseangle = (angle_v - angle_i) * (180.0F / PI);

      arm_cmplx_mag_f32(iSamps, Mags, FFTLEN);
      float base_mag_i = Mags[BINSIZE * 2] / (2 * FFTLEN);
      float tmp = ACSchip.readIRMS(0); // base in a
      // tmp = ConvertUnsignedFixedPoint(tmp, 15, 17);
      pwr_qual.distortion_factor = (tmp / base_mag_i);
      // SerialUSB1.printf("Dist.-fact:\n Base:%f\n irms:%f\n dist_fact:%f\n", base_mag_i, tmp, distortion_factor);
      pwr_qual.thd_i = calcTHD(Mags, 17, BINSIZE);
      if (pwr_qual.grouping_en) // If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = {0};
        pwr_qual.thdg_i = calcTHDG(Mags, fgroups, 17);
        pwr_qual.thdsg_i = calcTHDSG(Mags, fgroups, 17);
      }

      if (currPage == DISP_FFT_CURRENT)
        displayFFT(&display, Mags, currPage);

      startSamplingFFT();
    }
  }

  // Update Display with a fixed timing only
  if (newTime > (displayTime + DISPLAY_UPD_RATE))
  {
#ifdef DEBUG
    SerialUSB1.println("Updating Display");
#endif
    updateDisplay();
    displayTime = newTime;
  }

  // Check for Serial input
  getCommand();

  if (touchHappened)
  {
    if (dataLogging)
      touchHandler_DataLogging();
    else
      touchHandler_Default();
    updateDisplay();
  }

  if (newTime > checkTime + wdTimeout - 20000)
  {
#ifdef DEBUG
    SerialUSB1.println("Feeding the Dawg");
#endif
    wdt.feed();
    detectVoltage();
    checkTime = newTime;
  }
}

// Function to detect Zero Crossings
int detectCurrentZC(uint8_t orient)
{
  if (orient == 1) // Positive peak detected -> Detect Negative Slope
  {
    float currdata = ACSchip.readICODE();
    while (currdata > -zcd_thresh)
    {
      currdata = ACSchip.readICODE(); // readICodes();
    }
    return 0; // return a negative slope detection
  }
  if (orient == 0)
  {                                       // Negative peak detected: Detect positive Slope
    float currdata = ACSchip.readICODE(); // readICodes();
    while (currdata < zcd_thresh)
    {
      currdata = ACSchip.readICODE(); // readICodes();
    }
    return 1;
  }
  return 0;
}

void transmitIntegerSamples(usb_serial_class *sBus, uint32_t *samples, uint32_t length)
{
  for (uint32_t i = 0; i < length; i++)
  {
    sBus->printf("%u\n");
  }
}

/*
  Reads a single EEPROM Value
  After call, reads a Command String from the Serial Bus and converts the Single Values accordingly
  Send: ev<adr mask pos> with
        adr in hex-form  '0x..' (Address of EEPROM-Register)
        mask in hex-form '0x..' (Mask with zeros on the values' positions)
        pos in decimal-form (Position of LSB of Value)
*/
void readSingleEEPROM()
{
  receiveCommandString();
  char *endPointer;
  uint32_t adr = strtoul(receivedChars, &endPointer, 16);
  uint32_t mask = strtoul(endPointer, &endPointer, 16);
  uint8_t pos = strtoul(endPointer, NULL, 10);

  uint32_t data = ACSchip.readEEPROM(adr, mask, pos);
#ifdef DEBUG
  SerialUSB1.printf("Reading Adr %u, Mask %u, on pos %u\n Data: %u\n", adr, mask, pos, data);
#endif
  Serial.printf("%u\n", data);
  Serial.send_now(); // Sends instantly to avoid buffering
}

/*
 Writes to the MCU-internal EEPROM.

 Parameters:
  adr -- starting address to write to
  val -- 32-bit value to write
  read -- if true, read adr & val from serial bus, else take parameters

 To write, send we<address value>
 address -- as integer 0-1079
 value -- 32-bit integer value
*/
void MCU_write(uint32_t adr, uint32_t val, bool read)
{
  if (read)
  {
    receiveCommandString();

    char *endPointer;
    adr = strtoul(receivedChars, &endPointer, 10);
    val = strtoul(endPointer, NULL, 10);
  }
  if (adr <= 1079 && adr >= 0)
  {
#ifdef DEBUG
    SerialUSB1.printf("Writing MCU\n adr: %u\n val:%u\n", adr, val);
#endif
    // Write MSB first
    EEPROM.write(adr, (uint8_t)(val >> 24));
    EEPROM.write(adr + 1, (uint8_t)(val >> 16));
    EEPROM.write(adr + 2, (uint8_t)(val >> 8));
    EEPROM.write(adr + 3, (uint8_t)val);
  }
  else
    SerialUSB1.println("Address must be between 0 and 1079");
}

/*
 Reads from the MCU-internal EEPROM.

 Parameters:
  adr -- starting address to write to
  read -- if true, read adr from serial bus, else take parameters

 To read, send ee<address>
 address -- as integer 0-1079

 Return a 32-bit value from the starting address
*/
uint32_t MCU_read(uint32_t adr, bool read)
{
  if (read)
  {
    receiveCommandString();
    adr = strtoul(receivedChars, NULL, 10);
  }
  if (adr > 1079 || adr < 0)
  {
    SerialUSB1.println("Address must be between 0 and 1079");
    return 0;
  }
  // Read MSB First
  uint32_t data = (uint32_t)EEPROM.read(adr) << 24;
  data |= (uint32_t)EEPROM.read(adr + 1) << 16;
  data |= (uint32_t)EEPROM.read(adr + 2) << 8;
  data |= (uint32_t)EEPROM.read(adr + 3);
#ifdef DEBUG
  SerialUSB1.printf("Reading MCU\n adr: %u\n data:%u\n", adr, data);
#endif

  return data;
}

void readAddress()
{
  receiveCommandString();
  uint32_t adr = strtoul(receivedChars, NULL, 16);

  uint32_t data = ACSchip.readReg(adr);
#ifdef DEBUG
#endif
  SerialUSB1.printf("Reading Adress: %u, value: %u\n", adr, data);
  Serial.printf("%u\n", data);
  Serial.send_now();
}

void writeAddress()
{
  receiveCommandString();
  char *endPointer;
  uint32_t adr = strtoul(receivedChars, &endPointer, 16);
  uint32_t value = strtoul(endPointer, NULL, 10);
#ifdef DEBUG
  SerialUSB1.printf("Writing %u to address %u\n", value, adr);
#endif
  ACSchip.writeReg(adr, value);
}

/*
  Measures the frequency of a waveform, by detecting consecutive rising edges
  Takes the Pulsewidth into account

  Possible extension: Frequency-Measurement depending on halfcycle_en
*/
void measureFrequency()
{
  // Frequency-Measurement won't work
  if (!voltage_detected)
  {
    pwr_qual.pwr_f = 0.0F;
    return;
  }

  uint32_t firstTime;
  uint32_t secTime;
  uint8_t noOfTries = 0;
  // Read the ZC-Register until a ZC occurs
  uint8_t ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1 && noOfTries < 200)
  {
    ZC = ACSchip.readReg(0x2D) & 0x1; // update value
    noOfTries++;
  }
  firstTime = micros();  // Save the time of the first rising edge
  delayMicroseconds(32); // Wait for duration of pulse
  // Read the ZC-Register again until a ZC occurs
  noOfTries = 0;
  ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1 && noOfTries < 200)
  {
    ZC = ACSchip.readReg(0x2D) & 0x1; // update value
    noOfTries++;
  }
  secTime = micros(); // Save the time of the second rising edge
  uint32_t ti = secTime - firstTime;
  pwr_qual.pwr_f = 1.0F / (ti * 1000000.0F); // Convert time to seconds and calculate Frequency
}

/*
  Sets the calibration values. Send
  wc<calibration_factor exp_rms voltage(0)/current_mode(1)>
*/
void calibrateRMS()
{
  receiveCommandString();
  char *endPointer;
  // Conversion Factor
  uint32_t conv = strtoul(receivedChars, &endPointer, 10);
  // Mode either 0 or 1
  uint8_t mode = (uint8_t)strtoul(endPointer, NULL, 10);

  float converted = ConvertUnsignedFixedPoint(conv, 23, 24);
#ifdef DEBUG
  SerialUSB1.printf("Calibration: \n Conversion-f (int): %u,\n mode: %u,\n Converted: %e\n", conv, mode, converted);
#endif

  if (mode == 0)
  {
    conv_factor_VRMS = converted;
    MCU_write(VRMS_cal_address, conv, false);
  }
  else
  {
    conv_factor_IRMS = converted;
    MCU_write(IRMS_cal_address, conv, false);
  }

  Serial.printf("SUCCESS\n");
  Serial.send_now();
}

/*
  Receives a command String with delimiters. Can read up to MAX_RECEIVABLE chars

*/
void receiveCommandString()
{
  char stdel = '<';              // Start delimiter
  char enddel = '>';             // End delimiter
  bool receiving = false;        // Receiving flag
  uint8_t cntr = 0;              // Char counter
  while (Serial.available() > 0) // Receive while characters are on the Serial Bus / Buffer
  {
    char rc = Serial.read();

    if (receiving) // If receiving of values has started
    {
      if (rc != enddel)
      {
        receivedChars[cntr] = rc;
        cntr++;
        if (cntr > MAX_RECEIVEABLE)
          cntr = MAX_RECEIVEABLE - 1;
      }
      else // If end delimiter was sent
      {
        receivedChars[cntr] = '\0'; // String end delimiter
        receiving = false;          // Stop receiving and
        cntr = 0;                   // Reset counter
      }
    }
    else
    {
      if (rc == stdel) // Starting delimiter has been sent
        receiving = true;
    }
  }
}
/*
  Writes a value to the EEPROM
  Reads an incoming data String and converts each value
  Input:
    ww<Adr Value Mask Pos>
    Adr: hex-form '0x..' (Adress of EEPROM)
    Value: signed decimal (Value for EEPROM)
    Mask: hex-form '0x..' (Mask with zeros at value positions)
    Pos: decimal (Position of LSB of value)
*/
void writeEEPROMValue()
{
  receiveCommandString(); // Read incoming data
  uint32_t address = 0;
  int32_t value = 0; // Can be signed
  uint32_t mask = 0;
  uint8_t pos = 0;
  char *endPointer;

  address = strtoul(receivedChars, &endPointer, 16);
  value = strtol(endPointer, &endPointer, 10);
  mask = strtoul(endPointer, &endPointer, 16);
  pos = strtol(endPointer, NULL, 10);
#ifdef DEBUG
  SerialUSB1.printf("Writing Value %u to adr %u with mask %u on pos %u\n", value, address, mask, pos);
#endif
  ACSchip.writeEEPROM(address, value, mask, pos);
}

// Start FFT Sampling for Teensy calculation
void startSamplingFFT()
{
  if (!voltage_detected)
    return;
  // Make sure any other sampling is not running
  sampling = true;
  stopSampling();
  samplingDone = false;
  sampleCntr = 0;
  uint8_t z_count = 0;
  // Wait for zero crossing
  uint32_t zcd = ACSchip.readReg(0x2D) & 0x1;
  while (zcd != 1 && z_count < 30)
  {
    zcd = ACSchip.readReg(0x2D) & 0x1;
    z_count++;
  }
  bool err = smplTimer.begin(getSamples_FFT, FFTSAMPLERATE);
  if (!err)
    SerialUSB1.println("Error starting Timer");
}

// Start normal Sampling for PC calculation
void startSamplingPC()
{
  if (!voltage_detected)
    return;
  sampling = true;
  stopSampling();
  samplingDone = false;
  sampleCntr = 0;
  uint32_t zcd = ACSchip.readReg(0x2D) & 0x1;
  while (zcd != 1)
    zcd = ACSchip.readReg(0x2D) & 0x1;
  bool err = smplTimer.begin(getSamplesPC, SAMPLERATE);
  if (!err)
    SerialUSB1.println("Error starting Timer");

#ifdef DEBUG
  SerialUSB1.println("Started PC Sample Timer");
#endif
}

// Sampling for PC Calculation
void getSamplesPC()
{
  // Read Icodes and Vcodes as close as possible to each other
  // Adding power sampling
  if (powerSampling)
    vCodeBuffer[sampleCntr] = ACSchip.readPACTIVE_RAW(0);
  else
    vCodeBuffer[sampleCntr] = ACSchip.readVCODE_RAW();

  iCodeBuffer[sampleCntr] = ACSchip.readICODE_RAW();
  sampleCntr++;
  if (sampleCntr > SAMPBUFFLEN - 1) // If sampleBuffer is filled up
  {
    samplingDone = true;
    stopSampling();
  }
}

// Start Stream Sampling for PC
// Helper Function for streaming values at different speeds
void startStreamingPC()
{
  sampling = true;
  stopSampling();
  receiveCommandString();
  char *endPointer;

  float streamTime = strtof(receivedChars, &endPointer);
  SerialUSB1.printf("Stream time delay: %f\n", streamTime);
  bool err = smplTimer.begin(streamSampling, streamTime); // Maximum
  if (!err)
    SerialUSB1.println("Error starting Stream Timer");
}

// Sampling for Teensy-FFT Calculation
void getSamples_FFT()
{
  vSamps[sampleCntr] = ACSchip.readVCODE() * 366.94; // ConvertSignedFixedPoint(vcodes, 16, 17) * 366.94;
  iSamps[sampleCntr] = ACSchip.readICODE() * 15;     // ConvertSignedFixedPoint(icodes, 15, 17) * 15;
  vSamps[sampleCntr + 1] = 0.0;
  iSamps[sampleCntr + 1] = 0.0;
  sampleCntr += 2;
  if (sampleCntr >= 2 * FFTREALSAMPLES - 1) // Sample Buffer is filled up
  {
    for (uint32_t i = sampleCntr; i < FFTBUFFLEN; i++) // Zero-Pad the rest, because the FFT-Function works in-place
    {
      vSamps[i] = 0.0;
      iSamps[i] = 0.0;
    }
    samplingDone = true;
    stopSampling();
  }
}
/*
Helper to detect Voltage Level
*/

boolean detectVoltage()
{
  float tmp = ACSchip.readVRMS(0);
  if (tmp > 10) // At least 10V RMS voltage
  {
    SerialUSB1.printf("Voltage Detected: %.2f V_RMS\n", tmp);
    voltage_detected = true;
    return true;
  }
  voltage_detected = false;
  SerialUSB1.println("Voltage Level not high enough");
  return false;
}

/*
Stream vcodes and icodes to Serial. Helper
*/
void streamSampling()
{
  Serial.printf("%u,%u\n", ACSchip.readVCODE_RAW(), ACSchip.readICODE_RAW());
}
/*
  Stops the sampling timer
*/
void stopSampling()
{
  smplTimer.end();
}
////////////////////////////////////////////////////////////////////////////////
// Command Input
////////////////////////////////////////////////////////////////////////////////

/*
  Command structure:
  two-letter valuecode | < | Values, separated by whitespaces. Function dependent | >
*/

void getCommand()
{
  if (Serial.available() > 0)
  {
    char c1 = Serial.read(); // First letter
    char c2 = Serial.read(); // Second letter
#ifdef DEBUG
    SerialUSB1.printf("Got %d, %d", c1, c2);
#endif
    switch (c1)
    {
    case 't':
      timeSync();
      break;
    case 'v':
    { //**************************Voltage*****************************
      switch (c2)
      {
      case 'r': // RMS Voltage
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(0));
        break;
      case 's': // RMS Voltage per second
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(1));
        break;
      case 'm': // RMS Voltage per minute
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(2));
        break;
      case 'c': // Start Voltage Sampling
        connectComputer(true);
        startSamplingPC();
        runmode = 1;
        break;
      case 'd': // Single Voltage Value
        Serial.printf("%u\n", ACSchip.readVCODE_RAW());
        break;
      }
      break;
    }
    case 'i':
    { //**************************Current*****************************
      switch (c2)
      { // All current related measurements
      case 'r':
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(0));
        break;
      case 's':
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(1));
        break;
      case 'm':
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(2));
        break;
      case 'c':
        startSamplingPC();
        runmode = 2;
        break;
      case 'd':
        Serial.printf("%u\n", ACSchip.readICODE_RAW());
        break;
      }
      break;
    }
    case 'p':
    { //**************************Power*****************************
      switch (c2)
      {                // All power related measurements
        if (c2 == 'a') // Active Power Value
        case 'a':
          Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(0));
        break;
      case 's': // Active Power per second
        Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(1));
        break;
      case 'm': // Active Power per minute
        Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(2));
        break;
      case 'p': // Apparent Power
        Serial.printf("%u\n", ACSchip.readPAPP_RAW());
        break;
      case 'i': // Reactive Power
        Serial.printf("%u\n", ACSchip.readPIMAG_RAW());
        break;
      case 'f': // Power Factor
        Serial.printf("%u\n", ACSchip.readPOWERFACTOR_RAW());
        break;
      case 't':
        Serial.printf("%u\n", ACSchip.readPINST_RAW());
        break;
      case 'c':
        powerSampling = true;
        startSamplingPC();
        runmode = 1; // Voltage or active power
        break;
      }
      break;
    }
    case 's':
    { //**************************Status*****************************
      switch (c2)
      {
      case 'n':
        Serial.printf("%u\n", ACSchip.readNumOfPoints());
        break;
      case 'z':
        Serial.printf("%u\n", (ACSchip.readFlags() & 0x1));
        break; // Zero crossing
      case 'f':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 6) & 0x1));
        break; // pospf, 0: Generation, 1: Consumption
      case 'a':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 5) & 0x1));
        break; // posangle
      case 'u':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 4) & 0x1));
        break; // undervol
      case 'o':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 3) & 0x1));
        break; // overvol
      case 'l':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 2) & 0x1));
        break; // faultlatched
      case 'c':
        Serial.printf("%u\n", ((ACSchip.readFlags() >> 1) & 0x1));
        break; // faultout
      }
      break;
    }
    case 'w':
    { // Write to an EEPROM register
      switch (c2)
      {
      case 'w':
        writeEEPROMValue();
        break;
      case 'a':
        writeAddress();
        break;
      case 'e':
        MCU_write(0, 0, true);
        break;
      case 'c':
        calibrateRMS();
        break;
      }
      break;
    }
    case 'b':
    {
      switch (c2)
      { // Print Both Code Values
      case 'c':
        connectComputer(true);
        startSamplingPC();
        runmode = 3;
        break;
      case 'f':
        calcFFT = true;
        startSamplingFFT();
        break;
      case 't':
        SerialUSB1.println("Started Stream");
        startStreamingPC();
        break;
      }
      break;
    }
    case 'e':
    { // EEPROM-Operations
      switch (c2)
      {
      case 'v':
        readSingleEEPROM();
        break;
      case 'a': // Reads the content of a memory address
        readAddress();
        break;
      case 'e':
      { // Reads the MCU-EEPROM and sends the return
        uint32_t data = MCU_read(0, true);
        Serial.printf("%u\n", data);
        break;
      }
      case 's': // EEPROM "status"
        sendCalibration();
        break;
      }
      break;
    }
    case 'x':
    { // Set to Idle Mode
      stopSampling();
      sampling = false;
      samplingDone = false;
      powerSampling = false;
      // connectComputer(false);
      runmode = 0;
      inCalibration = false;
      calcFFT = false;
      break;
    }
    case 'c':
    { // Computer Commands
      switch (c2)
      {
      case 'c': // Connect
        connectComputer(true);
        break;
      case 'd': // Disconnect
        connectComputer(false);
        break;
      case 'i': // Connection test
        Serial.print("tconn\n");
        connectComputer(true);
        break;
      case 'a': // Calibration mode
        inCalibration = !inCalibration;
        Serial.printf("%u\n", inCalibration);
        toCalibrationMode();
        break;
      }
      break;
    }
    } // End switch(c1)
  }   // End if(Serial)
} // End getCommand()

////////////////////////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////////////////////////

/*
  Set the computerConnected Flag and displays a message

*/
void connectComputer(bool conn)
{
  computerConnected = conn;
  if (conn)
  {
#ifdef INIT_DISP
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Computer Connected.\nOutput Disabled");
    display.display();
#endif
    SerialUSB1.println("Computer connected");
  }
  else
  {
    SerialUSB1.println("Computer disconnected");
  }
}

void toCalibrationMode()
{
  uint32_t currTime = millis();
  uint32_t calTime = currTime;
  uint32_t timeDisplay = currTime;
  print_time_in_min_sec(&display, 5, 10, 300000);
  SerialUSB1.print("CalibrationMode:");
  SerialUSB1.print(inCalibration);
  SerialUSB1.println();
  while (inCalibration && (currTime < calTime + 300000))
  { // 5 min calibration time
    currTime = millis();
    if (currTime > timeDisplay + 10000)
    {
      print_time_in_min_sec(&display, 5, 10, (300000 - currTime));
      timeDisplay = currTime;
    }
    if (currTime > checkTime + wdTimeout - 5000)
    {
      wdt.feed();
      checkTime = currTime;
    }
    getCommand();
    calTime = currTime;
  }
  inCalibration = false;
  updateDisplay();
}

void timeSync()
{
  unsigned long pctime;
  receiveCommandString();                   // Receive Command
  pctime = strtol(receivedChars, NULL, 10); // Read Time Value
  SerialUSB1.print("Time received: ");
  SerialUSB1.print(pctime);
  SerialUSB1.println();
  if (pctime >= DEFAULT_TIME)
  {                  // check the integer is a valid time (greater than Jan 1 2013)
    setTime(pctime); // Sync Arduino clock to the time received on the serial port
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Time Syncronized");
    display.display();
    connectComputer(computerConnected);
  }
}

void sendCalibration()
{
  Serial.printf("%u,%u\n", MCU_read(VRMS_cal_address, false), MCU_read(IRMS_cal_address, false));
  SerialUSB1.println("Sent Calibration");
}

/*
  Updates the Display with values
*/
void updateDisplay()
{
#ifdef INIT_DISP
  if (currPage == DISP_FFT_VOLT || currPage == DISP_FFT_CURRENT || computerConnected)
    return;

  if (dataLogging)
    displayDataLogMenu(&display, menuOption, optionNames);
  else
    displayDefault(&display, &ACSchip, currPage, &pwr_qual);
#endif
  wdt.feed();
}

void printDebug(const char *obj)
{
#ifdef DEBUG
  SerialUSB1.println(obj);
#endif
}