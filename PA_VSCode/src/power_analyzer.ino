/*
Teensy 4.0 Firmware for a Power Analyzer
Author: Sebastian Zisch
Date: 210417

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

#include <arm_math.h>         //Include of DSP library
#include <Adafruit_SSD1327.h> //Display Library
#include <TimeLib.h>          //Time Library for Real-Time Clock
#include <math.h>
#include <Watchdog_t4.h> //Watchdog Library for reset on error
#include <EEPROM.h>
#include <SD.h>

#include "PowerAnalyzer.h"

//Uncomment for Debug messages on Serial Port 3
#define DEBUG

uint8_t currPage = 0;

//Values predefined if EEPROM-read error
float conv_factor_VRMS = 8.384 * pow(10, -6);
float conv_factor_IRMS = 1.015 * pow(10, -3);

//Creation of a Chip instance
ACS71020 ACSchip(ACS_SPI_SPEED, ACS_CS, ACS_CUSTOMER_CODE);

Adafruit_SSD1327 display(DISP_WIDTH, DISP_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);

//Character arrays for input commands
#define MAX_RECEIVEABLE 32 // Number of characters that can be read
char receivedChars[MAX_RECEIVEABLE];

extern volatile_memory *vol_mem;
extern harmonic_factors *fft;

//Timer for acquiring samples
IntervalTimer smplTimer;
boolean samplingDone = false;
uint32_t sampleCntr = 0;

//Flags for enabling Streaming
bool sampling = false;
bool powerSampling = false;
bool calcFFT = false;
uint8_t runmode = 0;            //0: Idle, 1: Voltage/Power, 2: Current, 3: Voltage & Current
bool computerConnected = false; // Display flag
bool voltage_detected = false;  //For frequency measurement, requires a voltage for zero crossings

//Buffer Arrays, Initialized to 0
//PC Sample Buffers
uint32_t vCodeBuffer[SAMPBUFFLEN] = { 0 };
uint32_t iCodeBuffer[SAMPBUFFLEN] = { 0 };

//Teensy Sample Buffers
float vSamps[FFTBUFFLEN] = { 0.0 };
float iSamps[FFTBUFFLEN] = { 0.0 };
float Mags[FFTLEN] = { 0.0 };

float zcd_thresh = 0.2;

bool inCalibration = false;

File root;


//ReadTimes
uint32_t newTime, streamTime, displayTime; //Time-Tracker, seconds-timer, minute-timer

//Instantiate Watchdogtimer 3
WDT_T4<WDT3> wdt;
uint32_t checkTime;
uint32_t wdTimeout = 30000; //ms

void checkCallback() {}

void setup() {
  pinMode(SD_CS_PIN, OUTPUT);
  initVolatileMemory(vol_mem);
  //Init Serial communication
  //Dual Channel Serial Communication
  Serial.begin(115200);     //Main transmit port
  SerialUSB1.begin(115200); //Debug and status port
  while (!Serial && !SerialUSB1) {};

  //Display Initialization
  if (!display.begin(SSD1327_I2C_ADDRESS)) {
    SerialUSB1.println("Unable to initialize OLED");
    while (1)
      yield();
  }
  display.clearDisplay();
  display.display();

  //Uncomment the next two rows, if accidentally written values too large or made a factory reset
  //MCU_write(VRMS_cal_address, 21280, false);
  //MCU_write(IRMS_cal_address, 257, false);

  /*if (!SD.begin(SD_CS_PIN)) {
    SerialUSB1.println("SD initialization failed!");
    return;
  }
  SerialUSB1.println("SD initialization done.");*/

  //Read current calibration factors from the chip
  uint32_t tmpInt = MCU_read(VRMS_cal_address, false); //Read VRMS conversionFactor
  conv_factor_VRMS = ConvertUnsignedFixedPoint(tmpInt, 23, 24);
  tmpInt = MCU_read(IRMS_cal_address, false);
  conv_factor_IRMS = ConvertUnsignedFixedPoint(tmpInt, 23, 24);

  SerialUSB1.printf("Init of Conversion Factors:\n VRMS-conv-factor:%e\n IRMS-conv-factor:%e\n", conv_factor_VRMS, conv_factor_IRMS);

  //Init of FFT Instance
  arm_status fft_err = arm_cfft_radix4_init_f32(&fftInstance, FFTLEN, 0, 1);
  if (fft_err == ARM_MATH_ARGUMENT_ERROR) {
    SerialUSB1.println("FFT Len Error");
    calcFFT = false;
  }

  //Voltage detection. Requires calibration of sensitivity
  float tmp = calcVRMS();
  if (tmp > 100) //At least 10V RMS voltage
  {
    calcFFT = true;
    startSamplingFFT();
    voltage_detected = true;
    SerialUSB1.printf("Voltage Detected: %.2f V_RMS", tmp);
  }

  //Touch-PIN configure
  attachInterrupt(digitalPinToInterrupt(TOUCH_NEXT), pageNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(TOUCH_PREV), pageBack, RISING);
  currPage = 1;

  //Configuration of watchdog Timer
  WDT_timings_t config;
  config.timeout = wdTimeout; /* in ms, 32ms to 522.232s */
  config.callback = checkCallback;
  wdt.begin(config);

  //Initialization of timers
  streamTime = millis();  //
  displayTime = millis(); //Display timing
  checkTime = millis();
  //End of Init
}

void loop() {
  newTime = millis(); //Get current millis for timing

  if (samplingDone) {
    if (runmode == 1) //Transmit all voltage samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u\n", vCodeBuffer[i]);
      startSamplingPC();
    } else if (runmode == 2) //Transmit all current samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u\n", iCodeBuffer[i]);
      startSamplingPC();
    } else if (runmode == 3) //Transmit alls current and voltage samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u,%u\n", vCodeBuffer[i], iCodeBuffer[i]);
      startSamplingPC();
    }
    //Calculate FFT and the harmonic distortion
    else if (calcFFT) {
      arm_cfft_radix4_f32(&fftInstance, vSamps);     //In-Place FFT
      float angle_v = atan2(vSamps[21], vSamps[20]); //Angle from pure spectrum
      arm_cmplx_mag_f32(vSamps, Mags, FFTLEN);       //Calculate Magnitudes
      fft->thd_v = calcTHD(17);                           //Calculate Total Harmonic Distortion
      if (fft->grouping_en)                               //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = { 0 };
        fft->thdg_v = calcTHDG(Mags, fgroups, 17);
        fft->thdsg_v = calcTHDSG(Mags, fgroups, 17);
      }
      if (currPage == 4)
        displayFFT();
      //FFT for Current samples
      arm_cfft_radix4_f32(&fftInstance, iSamps); //In-Place FFT
      float angle_i = atan2(iSamps[21], iSamps[20]);
      fft->phaseangle = (angle_v - angle_i) * (180 / PI);

      arm_cmplx_mag_f32(iSamps, Mags, FFTLEN);
      float base_mag_i = Mags[binSize * 2] / (2 * FFTLEN);
      float tmp = ACS_IRMS; // base in a
      tmp = ConvertUnsignedFixedPoint(tmp, 15, 17);
      fft->distortion_factor = (tmp / base_mag_i);
      //SerialUSB1.printf("Dist.-fact:\n Base:%f\n irms:%f\n dist_fact:%f\n", base_mag_i, tmp, distortion_factor);
      fft->thd_i = calcTHD(17);
      if (fft->grouping_en) //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = { 0 };
        fft->thdg_i = calcTHDG(Mags, fgroups, 17);
        fft->thdsg_i = calcTHDSG(Mags, fgroups, 17);
      }
      if (currPage == 5)
        displayFFT();

      updateDisplay();

      //Start Sampling again
      startSamplingFFT();
    }
  }
  //Update Display with a fixed timing only
  if (newTime > displayTime + DISPLAY_UPD_RATE && !sampling) {

    //Display Update if calculation of FFT was stopped and no computer is connected to avoid
    //Conflicts in SPI reading
    if (!calcFFT && !computerConnected) {
      //For Live Voltage detection
      float voltrms = calcVRMS();
      updateDisplay();
      //Voltage detection, only if no voltage was detected
      if (!voltage_detected && voltrms > 100) {
        //Start sampling FFT (default mode)
        voltage_detected = true;
        startSamplingFFT();
        calcFFT = true;
      }
    } else {
      //For Live Voltage detection
      float voltrms = calcVRMS();
      //If plug has been pulled out of the socket (no voltage)
      if (voltage_detected && voltrms <= 100) {
        updateDisplay();
        voltage_detected = false;
        calcFFT = false;
        stopSampling();
      }
    }
    displayTime = newTime;
  }
  //Check for Serial input
  getCommand();

  if (newTime > checkTime + wdTimeout - 5000) {
    wdt.feed();
    checkTime = newTime;
  }
}

void getCommand() {
  if (Serial.available() > 0) {
    char c1 = Serial.read(); //First letter
    char c2 = Serial.read(); //Second letter

    if (c1 == 't') //Time Sync
    {
      timeSync();
    }
    //All voltage related measurements
    else if (c1 == VOLTAGE) //**************************Voltage*****************************
    {
      if (c2 == RMS) //RMS Voltage
      {
        vol_mem->v_rms = ACS_VRMS;
        Serial.printf("%u\n", vol_mem->v_rms);
      } else if (c2 == PER_SECOND) //RMS Voltage per second
      {
        vol_mem->vrms_sec = ACSchip.readReg(0x26) & 0x7FFF; //15-bit FP, 15 frac
        Serial.printf("%u\n", vol_mem->vrms_sec);           //("%f\n", ConvertUnsignedFixedPoint(vrms_sec, 15, 15)*RATED_VOLTAGE);
      } else if (c2 == PER_MINUTE) //RMS Voltage per minute
      {
        vol_mem->vrms_min = ACSchip.readReg(0x27) & 0x7FFF; //15-bit FP, 15 frac
        Serial.printf("%u\n", vol_mem->vrms_min);           //("%f\n", ConvertUnsignedFixedPoint(vrms_min, 15, 15)*RATED_VOLTAGE);
      } else if (c2 == PC_SAMPLING) //Start Voltage Sampling
      {
        connectComputer(true);
        startSamplingPC();
        runmode = 1;
      } else if (c2 == SINGLE_VALUE) //Single Voltage Value
      {
        vol_mem->vcodes = ACS_VCODE;
        Serial.printf("%u\n", vol_mem->vcodes);
      }
    } else if (c1 == CURRENT) //**************************Current*****************************
    {                   //All current related measurements
      if (c2 == RMS)    //RMS Current
      {
        vol_mem->i_rms = ACS_IRMS;
        Serial.printf("%u\n", vol_mem->i_rms);
      } else if (c2 == PER_SECOND) //RMS Current per second
      {
        vol_mem->irms_sec = (ACSchip.readReg(0x26) >> 16) & 0x7FFF; //15-bit FP, 14 frac
        Serial.printf("%u\n", vol_mem->irms_sec);
      } else if (c2 == PER_MINUTE) //RMS Current per minute
      {
        vol_mem->irms_min = (ACSchip.readReg(0x27) >> 16) & 0x7FFF; //15-bit FP, 14 frac
        Serial.printf("%u\n", vol_mem->irms_min);
      } else if (c2 == PC_SAMPLING) //Start Current Sampling
      {
        startSamplingPC();
        runmode = 2;
      } else if (c2 == SINGLE_VALUE) //Single Current Value
      {
        vol_mem->icodes = ACS_ICODE;
        Serial.printf("%u\n", vol_mem->icodes);
      }
    } else if (c1 == POWER) //**************************Power*****************************
    {                   //All power related measurements
      if (c2 == ACTIVE_POWER)    //Active Power Value
      {
        vol_mem->pActive = ACS_PACTIVE;
        Serial.printf("%u\n", vol_mem->pActive);
      } else if (c2 == PER_SECOND) //Active Power per second
      {
        vol_mem->pact_sec = ACSchip.readReg(0x28) & 0x1FFFF; //17-bit FP, 15 frac
        Serial.printf("%u\n", vol_mem->pact_sec);
      } else if (c2 == PER_MINUTE) //Active Power per minute
      {
        vol_mem->pact_min = ACSchip.readReg(0x29) & 0x1FFFF; //17-bit FP, 15 frac
        Serial.printf("%u\n", vol_mem->pact_min);
      } else if (c2 == APPARENT_POWER) //Apparent Power
      {
        vol_mem->pApp = ACS_PAPPARENT;
        Serial.printf("%u\n", vol_mem->pApp);
      } else if (c2 == REACTIVE_POWER) //Reactive Power
      {
        vol_mem->pImag = ACS_PIMAG;
        Serial.printf("%u\n", vol_mem->pImag);
      } else if (c2 == POWER_FACTOR) //Power Factor
      {
        vol_mem->pfact = ACS_PF;
        Serial.printf("%u\n", vol_mem->pfact);
      } else if (c2 == INSTANT_POWER) //Instant Power Value
      {
        vol_mem->pinst = ACS_PINST;
        Serial.printf("%u\n", vol_mem->pinst);
      } else if (c2 == PC_SAMPLING) //Power codes
      {
        powerSampling = true;
        startSamplingPC();
        runmode = 1; //Voltage or active power
      }
    } else if (c1 == CHIP_STATUS) //**************************Status*****************************
    {                   //All status Values
      vol_mem->flags = ACSchip.readReg(0x2D);
      if (c2 == RMS_POINTS) {
        vol_mem->numptsout = ACSchip.readReg(0x25) & 0x1FF; //Unsigned 9-bit
        Serial.printf("%u\n", vol_mem->numptsout);
      } else if (c2 == ZERO_CROSS)
        Serial.printf("%u\n", (vol_mem->flags & 0x1)); //Zero crossing
      else if (c2 == PF_SIGN)
        Serial.printf("%u\n", ((vol_mem->flags >> 6) & 0x1)); //pospf, 0: Generation, 1: Consumption
      else if (c2 == LEAD_LAG)
        Serial.printf("%u\n", ((vol_mem->flags >> 5) & 0x1)); //posangle
      else if (c2 == UNDERVOLTAGE)
        Serial.printf("%u\n", ((vol_mem->flags >> 4) & 0x1)); //undervol
      else if (c2 == OVERVOLTAGE)
        Serial.printf("%u\n", ((vol_mem->flags >> 3) & 0x1)); //overvol
      else if (c2 == OVERCURRENT)
        Serial.printf("%u\n", ((vol_mem->flags >> 2) & 0x1)); //faultlatched
      else if (c2 == FAULTOUT)
        Serial.printf("%u\n", ((vol_mem->flags >> 1) & 0x1)); //faultout
    } else if (c1 == WRITE) { //Write to an EEPROM register
      if (c2 == WRITE_ACS)
        writeEEPROMValue();
      else if (c2 == W_ADDRESS)
        writeAddress();
      else if (c2 == W_MCU_EEPROM) //Writes a value to the MCU-EEPROM
        MCU_write(0, 0, true);
      else if (c2 == CALIBRATION) //Write calibration
        calibrateRMS();
    } else if (c1 == DUAL_CODES) { //Print Both Code Values
      if (c2 == PC_SAMPLING) {
        connectComputer(true);
        startSamplingPC();
        runmode = 3;
      } else if (c2 == FFT_SAMPLING) {
        calcFFT = true;
        startSamplingFFT();
      } else if (c2 == PC_STREAM) {
        SerialUSB1.println("Started Stream");
        startStreamingPC();
      }
    } else if (c1 == READ) //EEPROM-Read-Operations
    {
      if (c2 == READ_ACS) {
        readSingleEEPROM();
      } else if (c2 == R_ADDRESS) //Reads the content of a memory address
        readAddress();
      else if (c2 == R_MCU_EEPROM) { //Reads the MCU-EEPROM and sends the return
        uint32_t data = MCU_read(0, true);
        Serial.printf("%u\n", data);
      } else if (c2 == R_CALIBRATION) // EEPROM "status"
        sendCalibration();
    } else if (c1 == PA_IDLE) //Set to Idle Mode
    {
      stopSampling();
      sampling = false;
      samplingDone = false;
      powerSampling = false;
      connectComputer(false);
      runmode = 0;
      inCalibration = false;
      calcFFT = false;
    } else if (c1 == PC) //Computer Commands
    {
      if (c2 == CONNECT) //Connect
      {
        connectComputer(true);
      } else if (c2 == DISCONNECT) //Disconnect
      {
        connectComputer(false);
        updateDisplay();
      } else if (c2 == CONNTEST) { //Connection test
        Serial.print("tconn\n");
        connectComputer(true);
      } else if (c2 == VOLT_DETECT) //Ignore voltage detection
      {
        #ifdef DEBUG
        SerialUSB1.println("Ignoring Voltage Level");
        #endif
        voltage_detected = true;
      } else if (c2 == CALIBRATION_MODE) { //Calibration mode
        inCalibration = !inCalibration;
        Serial.printf("%u\n", inCalibration);
        toCalibrationMode();
      } else if (c2 == SD_READ) //SD-Read-mode
      {
        SerialUSB1.println("Printing SD Directory");
        root = SD.open("/");
        File f = root;
        while (!f) {
          if (f.isDirectory())
            printDirectory(f, 0);
        }
        f = f.openNextFile();
      }
    }
  }
}

//Function to detect Zero Crossings
int detectCurrentZC(uint8_t orient) {
  if (orient == 1) //Positive peak detected -> Detect Negative Slope
  {
    float currdata = readICodes();
    while (currdata > -zcd_thresh) {
      currdata = readICodes();
    }
    return 0; //return a negative slope detection
  }
  if (orient == 0) { //Negative peak detected: Detect positive Slope
    float currdata = readICodes();
    while (currdata < zcd_thresh) {
      currdata = readICodes();
    }
    return 1;
  }
}

uint32_t getMaxValueIndex(float values[], uint32_t arrlen) {
  uint32_t maxIndex = 0;
  float maxVal = values[maxIndex];

  for (uint32_t i = 0; i < arrlen; i++) {
    if (values[i] > maxVal) {
      maxIndex = i;
      maxVal = values[i];
    }
  }
  return maxIndex;
}

/*
  Reads a single EEPROM Value
  After call, reads a Command String from the Serial Bus and converts the Single Values accordingly
  Send: ev<adr mask pos> with
        adr in hex-form  '0x..' (Address of EEPROM-Register)
        mask in hex-form '0x..' (Mask with zeros on the values' positions)
        pos in decimal-form (Position of LSB of Value)
*/
void readSingleEEPROM() {
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
  Serial.send_now(); //Sends instantly to avoid buffering
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
void MCU_write(uint32_t adr, uint32_t val, bool read) {
  if (read) {
    receiveCommandString();

    char *endPointer;
    adr = strtoul(receivedChars, &endPointer, 10);
    val = strtoul(endPointer, NULL, 10);
  }
  if (adr <= 1079 && adr >= 0) {
    #ifdef DEBUG
    SerialUSB1.printf("Writing MCU\n adr: %u\n val:%u\n", adr, val);
    #endif
    //Write MSB first
    EEPROM.write(adr, (uint8_t)(val >> 24));
    EEPROM.write(adr + 1, (uint8_t)(val >> 16));
    EEPROM.write(adr + 2, (uint8_t)(val >> 8));
    EEPROM.write(adr + 3, (uint8_t)val);
  } else
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
uint32_t MCU_read(uint32_t adr, bool read) {
  if (read) {
    receiveCommandString();
    adr = strtoul(receivedChars, NULL, 10);
  }
  if (adr > 1079 || adr < 0) {
    SerialUSB1.println("Address must be between 0 and 1079");
    return 0;
  }
  //Read MSB First
  uint32_t data = (uint32_t)EEPROM.read(adr) << 24;
  data |= (uint32_t)EEPROM.read(adr + 1) << 16;
  data |= (uint32_t)EEPROM.read(adr + 2) << 8;
  data |= (uint32_t)EEPROM.read(adr + 3);
  #ifdef DEBUG
  SerialUSB1.printf("Reading MCU\n adr: %u\n data:%u\n", adr, data);
  #endif

  return data;
}

void readAddress() {
  receiveCommandString();
  uint32_t adr = strtoul(receivedChars, NULL, 16);

  uint32_t data = ACSchip.readReg(adr);
  #ifdef DEBUG
  SerialUSB1.printf("Reading Adress: %u, value: %u\n", adr, data);
  #endif
  Serial.printf("%u\n", data);
  Serial.send_now();
}

void writeAddress() {
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
void measureFrequency() {
  uint32_t firstTime;
  uint32_t secTime;
  //Read the ZC-Register until a ZC occurs
  uint32_t ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1) {
    ZC = ACSchip.readReg(0x2D) & 0x1; //update value
  }
  firstTime = micros();  //Save the time of the first rising edge
  delayMicroseconds(32); //Wait for duration of pulse
  //Read the ZC-Register again until a ZC occurs
  ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1) {
    ZC = ACSchip.readReg(0x2D) & 0x1; //update value
  }
  secTime = micros(); //Save the time of the second rising edge
  float ti = secTime - firstTime;
  fft->pwr_f = 1 / (ti / 1000000); //Convert time to seconds and calculate Frequency
}

/*
  Sets the calibration values. Send
  wc<calibration_factor exp_rms voltage(0)/current_mode(1)>
*/
void calibrateRMS() {
  receiveCommandString();
  char *endPointer;
  //Conversion Factor
  uint32_t conv = strtoul(receivedChars, &endPointer, 10);
  //Mode either 0 or 1
  uint8_t mode = (uint8_t)strtoul(endPointer, NULL, 10);

  float converted = ConvertUnsignedFixedPoint(conv, 23, 24);
  #ifdef DEBUG
  SerialUSB1.printf("Calibration: \n Conversion-f (int): %u,\n mode: %u,\n Converted: %e\n", conv, mode, converted);
  #endif

  if (mode == 0) {
    conv_factor_VRMS = converted;
    MCU_write(VRMS_cal_address, conv, false);
  } else {
    conv_factor_IRMS = converted;
    MCU_write(IRMS_cal_address, conv, false);
  }

  Serial.printf("SUCCESS\n");
  Serial.send_now();
}

/*
  Receives a command String with chosen delimiters. Can read up to MAX_RECEIVABLE chars
*/
void receiveCommandString() {
  char stdel = '<';              //Start delimiter
  char enddel = '>';             //End delimiter
  bool receiving = false;        //Receiving flag
  uint8_t cntr = 0;              //Char counter
  while (Serial.available() > 0) //Receive while characters are on the Serial Bus / Buffer
  {
    char rc = Serial.read();

    if (receiving) //If receiving of values has started
    {
      if (rc != enddel) {
        receivedChars[cntr] = rc;
        cntr++;
        if (cntr > MAX_RECEIVEABLE)
          cntr = MAX_RECEIVEABLE - 1;
      } else //If end delimiter was sent
      {
        receivedChars[cntr] = '\0'; //String end delimiter
        receiving = false;          //Stop receiving and
        cntr = 0;                   //Reset counter
      }
    } else {
      if (rc == stdel) //Starting delimiter has been sent
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
void writeEEPROMValue() {
  receiveCommandString(); //Read incoming data
  uint32_t address = 0;
  int32_t value = 0; //Can be signed
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
void startSamplingFFT() {
  //Make sure any other sampling is not running
  sampling = true;
  stopSampling();
  samplingDone = false;
  sampleCntr = 0;
  //Wait for zero crossing
  uint32_t zcd = ACSchip.readReg(0x2D) & 0x1;
  while (zcd != 1)
    zcd = ACSchip.readReg(0x2D) & 0x1;
  bool err = smplTimer.begin(getSamples_FFT, FFTSAMPLERATE);
  if (!err)
    SerialUSB1.println("Error starting Timer");
}

// Start normal Sampling for PC calculation
void startSamplingPC() {
  sampling = true;
  stopSampling();
  samplingDone = false;
  sampleCntr = 0;
  if (voltage_detected) {
    uint32_t zcd = ACSchip.readReg(0x2D) & 0x1;
    while (zcd != 1)
      zcd = ACSchip.readReg(0x2D) & 0x1;
  }
  bool err = smplTimer.begin(getSamplesPC, SAMPLERATE);
  if (!err)
    SerialUSB1.println("Error starting Timer");

  #ifdef DEBUG
  SerialUSB1.println("Started PC Sample Timer");
  #endif
}

// Sampling for PC Calculation
void getSamplesPC() {
  //Read Icodes and Vcodes as close as possible to each other
  //Adding power sampling
  if (powerSampling)
    vCodeBuffer[sampleCntr] = ACS_PACTIVE;
  else
    vCodeBuffer[sampleCntr] = ACS_VCODE;

  iCodeBuffer[sampleCntr] = ACS_ICODE;
  sampleCntr++;
  if (sampleCntr > SAMPBUFFLEN - 1) //If sampleBuffer is filled up
  {
    samplingDone = true;
    stopSampling();
  }
}

// Start Stream Sampling for PC
// Helper Function for streaming values at different speeds
void startStreamingPC() {
  sampling = true;
  stopSampling();
  receiveCommandString();
  char *endPointer;

  float streamTime = strtof(receivedChars, &endPointer);
  SerialUSB1.printf("Stream time delay: %f\n", streamTime);
  bool err = smplTimer.begin(streamSampling, streamTime); //Maximum
  if (!err)
    SerialUSB1.println("Error starting Stream Timer");
}

// Sampling for Teensy-FFT Calculation
void getSamples_FFT() {
  vol_mem->vcodes = ACS_VCODE;
  vol_mem->icodes = ACS_ICODE;
  vSamps[sampleCntr] = ConvertSignedFixedPoint(vol_mem->vcodes, 16, 17) * 366.94;
  iSamps[sampleCntr] = ConvertSignedFixedPoint(vol_mem->icodes, 15, 17) * 15;
  vSamps[sampleCntr + 1] = 0.0;
  iSamps[sampleCntr + 1] = 0.0;
  sampleCntr += 2;
  if (sampleCntr >= 2 * FFTREALSAMPLES - 1) //Sample Buffer is filled up
  {
    for (int i = sampleCntr; i < FFTBUFFLEN; i++) //Zero-Pad the rest, because the FFT-Function works in-place
    {
      vSamps[i] = 0.0;
      iSamps[i] = 0.0;
    }
    samplingDone = true;
    stopSampling();
  }
}

/*
Stream vcodes and icodes to Serial. Helper
*/
void streamSampling() {
  vol_mem->vcodes = ACS_VCODE;
  vol_mem->icodes = ACS_ICODE;
  Serial.printf("%u,%u\n", vol_mem->vcodes, vol_mem->icodes);
}
/*
  Stops the sampling timer
*/
void stopSampling() {
  smplTimer.end();
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      if (numTabs == 0)
        SerialUSB1.println("** Done **");
      return;
    }
    for (uint8_t i = 0; i < numTabs; i++)
      SerialUSB1.print('\t');
    SerialUSB1.print(entry.name());
    if (entry.isDirectory()) {
      SerialUSB1.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////////////////////////

/*
  Set the computerConnected Flag and displays a message

*/
void connectComputer(bool conn) {
  computerConnected = conn;
  if (conn) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Computer Connected.\nOutput Disabled");
    display.display();
    SerialUSB1.println("Computer connected");
  } else {
    SerialUSB1.println("Computer disconnected");
  }
}

void toCalibrationMode() {
  uint32_t currTime = millis();
  uint32_t calTime = millis();
  uint32_t timeDisplay = millis();
  print_time_in_min_sec(300000);
  SerialUSB1.print("CalibrationMode:");
  SerialUSB1.print(inCalibration);
  SerialUSB1.println();
  while (inCalibration && currTime < calTime + 300000) { //5 min calibration time
    currTime = millis();
    if (currTime > timeDisplay + 10000) {
      print_time_in_min_sec(300000 - currTime);
      timeDisplay = currTime;
    }
    if (currTime > checkTime + wdTimeout - 5000) {
      wdt.feed();
      checkTime = currTime;
    }
    getCommand();
    calTime = currTime;
  }
  inCalibration = false;
  updateDisplay();
}



float readICodes() {
  return ConvertSignedFixedPoint(ACS_ICODE, 15, 17) * 15;
}

//Time synchro
void timeSync() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  receiveCommandString();                        //Receive Command
  pctime = strtol(receivedChars, NULL, 10);      //Read Time Value
  SerialUSB1.print("Time received: ");
  SerialUSB1.print(pctime);
  SerialUSB1.println();
  if (pctime >= DEFAULT_TIME) {                  // check the integer is a valid time (greater than Jan 1 2013)
    setTime(pctime); // Sync Arduino clock to the time received on the serial port
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Time Syncronized");
    display.display();
    connectComputer(computerConnected);
  }
}

void print_time_in_min_sec(uint32_t ms) {
  if (ms < 60000) { //60s remaining
    int secs = (int)(ms / 1000) + 1;
    display.clearDisplay();
    display.setCursor(5, 10);
    display.printf("Calibration Mode\n");
    display.printf("%u s remaining", secs);
    display.display();
  } else {
    int mins = (int)(ms / 60000);
    display.clearDisplay();
    display.setCursor(5, 10);
    display.printf("Calibration Mode\n");
    display.printf("%u min remaining", mins);
    display.display();
  }
}
void sendCalibration() {
  uint32_t tmp = MCU_read(VRMS_cal_address, false);
  Serial.printf("VRMS,%u\n", tmp);
  tmp = MCU_read(IRMS_cal_address, false);
  Serial.printf("IRMS,%u\n", tmp);
}

////////////////////////////////////////////////////////////////////////////////
// Calculation Functions
////////////////////////////////////////////////////////////////////////////////

float calcVRMS() {
  vol_mem->v_rms = ACS_VRMS;
  //uint32_t calib_Code = 21280;              //Calibration Factor
  //conv_factor_VRMS = exp_VRMS / calFactor_VRMS; //Actual Sensitivity

  float vrms_rsense = vol_mem->v_rms * conv_factor_VRMS;      //Voltage over RS1
  float vrms_input = vrms_rsense * (4003000 / 3000); //Voltage at output
  return vrms_input;
}
float calcIRMS() {
  vol_mem->i_rms = ACS_IRMS;
  //uint32_t i_calib_Code = 257;
  //float conv_factor = exp_IRMS / calFactor_IRMS; //Calibration from Lightbulb
  float irms = vol_mem->i_rms * conv_factor_IRMS; //conv_factor;
  return irms;
}

float calcTHD(uint8_t order) {
  uint32_t s = 50 / binSize; //50Hz base frequency
  float thd = 0;
  float base = Mags[s];
  for (uint32_t i = s * 2; i < order * s; i += s) {
    thd += pow(Mags[i] / base, 2);
  }
  thd = sqrt(thd) * 100;

  return thd;
}

float calcTHDG(float frequencies[], float output[], int order) {
  float groupvalue = 0;
  for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
  {
    groupvalue = pow(frequencies[i * 10 - 5], 2) / 2; //Add 1/2*Value offset by 5 to the left
    float sumvalue = 0;
    for (int k = -4; k < 5; k++) //Add all values from -4 to 4 around the harmonic order
    {
      sumvalue += pow(frequencies[i * 10 + k], 2);
    }
    groupvalue += sumvalue + pow(Mags[i * 10 + 5], 2) / 2; //Add 1/2*Value offset by 5 to the right
    float result = sqrt(groupvalue);
    output[i] = result; //Save it in output array
  }
  //Calculate THDG
  float thdg = 0;
  float base = output[1];
  for (uint8_t i = 2; i <= order; i++) {
    thdg += pow(output[i] / base, 2);
  }
  thdg = sqrt(thdg) * 100;
  return thdg;
}
float calcTHDSG(float frequencies[], float output[], int order) {
  for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
  {
    float sumvalue = 0;
    for (int k = -1; k < 2; k++) //Add neighbor values
    {
      sumvalue += pow(frequencies[i * 10 + k], 2);
    }
    float result = sqrt(sumvalue);
    output[i] = result;
  }
  //Calculate THDSG
  float thdsg = 0;
  float base = output[1];
  for (uint8_t i = 2; i <= order; i++) {
    thdsg += pow(output[i] / base, 2);
  }
  thdsg = sqrt(thdsg) * 100;
  return thdsg;
}

/*
  Updates the Display with values
*/
void updateDisplay() {
  if (currPage == 4 || currPage == 5) {
  } else {
    float temp = calcVRMS();
    //Updates to the display
    display.clearDisplay();
    display.setCursor(0, 0); //Cursor Position top-left
    if (voltage_detected) {
      measureFrequency();
      if (now() < 1357041600) //Now() is less that year 2013
        display.printf("Zeit: no sync\n");
      else
        display.printf("Zeit: %02d:%02d:%02d\n", hour(), minute(), second());
      display.printf("THD (V): %.2f %%\n", fft->thd_v);
      if (fft->grouping_en) {
        display.printf("THDG: %.2f\n", fft->thdg_v);
        display.printf("THDSG: %.2f\n", fft->thdsg_v);
      }
      display.printf("THD (I): %.2f %%\n", fft->thd_i);
      if (fft->grouping_en) {
        display.printf("THDG: %.2f\n", fft->thdg_i);
        display.printf("THDSG: %.2f\n", fft->thdsg_i);
      }
      display.printf("Freq: %.2f Hz\n", fft->pwr_f);
      if (temp < 1)
        display.printf("Vrms: %.2f mV\n", temp * 1000.0);
      else
        display.printf("Vrms: %.2f V\n", temp);
      temp = calcIRMS();
      if (temp < 1)
        display.printf("Irms: %.2f mA\n", temp * 1000.0);
      else
        display.printf("Irms: %.2f A\n", temp);
      //pActive = ACS_PACTIVE;
      //display.printf("Power %.2f W\n", ConvertSignedFixedPoint(pActive, 15, 17) * MAXPOWER);
      vol_mem->pact_sec = ACSchip.readReg(0x28) & 0x1FFFF; //17-bit FP, 15 frac
      display.printf("Power %.2f W\n", ConvertSignedFixedPoint(vol_mem->pact_sec, 15, 17) * MAXPOWER);
      vol_mem->pfact = ACS_PF;
      temp = ConvertSignedFixedPoint(vol_mem->pfact, 9, 11);
      temp = temp * fft->distortion_factor;
      display.printf("P-Factor: %.2f\n", temp);
      //float ang = acos(temp) * (180 / PI);
      display.printf("Winkel: %.2fdeg\n", fft->phaseangle);
    } else {
      display.printf("Voltage too low:\n %2f", temp);
    }
    display.display();
  }
}

void displayFFT() {
  display.clearDisplay();
  display.setCursor(0, 0);
  if (currPage == 4)
    display.print("Voltage FFT 0-640 Hz");
  else
    display.print("Current-FFT 0-640 Hz");
  //Number of Magnitudes: 4096, with binSize = 5
  float maxMag = Mags[getMaxValueIndex(Mags, 4096)];
  float pix_per_volt = (DISP_HEIGHT - 20) / maxMag;
  for (int i = 0; i < DISP_WIDTH; i++) {
    uint16_t mag = round(Mags[i] * pix_per_volt); //Rounded value
    display.drawLine(i, DISP_HEIGHT, i, DISP_HEIGHT - 10 - mag, SSD1327_WHITE);
  }
  display.display();
}

void pageNext() {
  if (currPage < NO_OF_PAGES - 1)
    currPage++;
  SerialUSB1.printf("NEXT! -> Page Nr: %d\n", currPage);
}

void pageBack() {
  if (currPage > 0)
    currPage--;
  SerialUSB1.printf("PREV! -> Page Nr: %d\n", currPage);
}
