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
#include "TeensyDebug.h"
#include <TimerOne.h>

//Uncomment for Debug messages on Serial Port 3
#define DEBUG

//Uncomment for SD init
//#define INIT_SDCARD

//Uncomment for Display init
#define INIT_DISP
//#define USE_SPI_DISP

//Uncomment for Touch-Sensor init
#define INIT_TOUCH

//Uncomment for voltage detection, useful for off-grid testing
//#define DETECT_VOLTAGE

uint8_t currPage = 2;

//Creation of a Chip instance
ACS71020 ACSchip(ACS_SPI_SPEED, ACS_CS, ACS_CUSTOMER_CODE);
volatile_memory* vol_mem;

DataLogger logger(ACSchip);

#ifdef USE_SPI_DISP
//SPI Display instance
Adafruit_SSD1327 display(DISP_WIDTH, DISP_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);
#else
//I2C Display instance
Adafruit_SSD1327 display(DISP_WIDTH, DISP_HEIGHT, &Wire, OLED_RST, 1000000, 400000);
#endif

volatile boolean updateDisplay_flag = false;
//Touch Sensor instance
volatile boolean touchHappened = false;
CAP1293 touchSensor; //Defaults to 0x28 in library

extern PA_command *command;
struct harmonic_factors *fft;
arm_cfft_radix4_instance_f32 fftInstance;

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


//Watchdog Callback function
void checkCallback() {}

//DisplayUpdate()
uint8_t dis_count = 0;
void displayUpdate_IRQ() {
  dis_count++;
  if (dis_count > 20) {
    updateDisplay_flag = true;
    dis_count = 0;
  }
}

//Touch-IRQ
void touch_IRQ() {
  touchHappened = true;
}

//Touch-Input handler
void touchHandler() {
  /*if (touchSensor.isLeftTouched() && currPage > 0) {
    currPage--;
  } else if (touchSensor.isRightTouched() && currPage < NO_OF_PAGES) {
    currPage++;
  }*/
  uint8_t touchMode = touchSensor.isLeftOrRightTouched(); //0: right, 1: left
  if (touchMode == 0 && currPage < NO_OF_PAGES)
    currPage++;
  else if (touchMode == 1 && currPage > 0)
    currPage--;

  touchSensor.clearInterrupt();
}

void setup() {
  //Init Serial communication
  //Dual Channel Serial Communication
  Wire.begin();
  Serial.begin(115200);     //Main transmit port
  #ifdef DEBUG
  while (!SerialUSB1) { yield(); };
  #endif
  vol_mem = ACSchip.init_ACSchip();
  //logger.init(SD_CS_PIN, SerialUSB1, vol_mem);

  #ifdef INIT_DISP
  //Display Initialization 
  if (!display.begin(SSD1327_I2C_ADDRESS)) {
    SerialUSB1.println("Unable to initialize OLED");
    while (1)
      yield();
  }
  display.setTextSize(2);
  display.clearDisplay();
  #ifdef DEBUG
  display.println("DEBUG Mode\nactivated.\n" \
  "Bootup\nafter\nSerial\nconnection");
  display.display();
  #endif
  #endif

  #ifdef INIT_TOUCH
  while (touchSensor.begin() == false) {
    SerialUSB1.println("Unable to connect to TouchSensor");
    //while (1)
    yield();
  };
  #ifdef DEBUG
  SerialUSB1.println("Touch Sensor initialized");
  #endif 
  //Touch-PIN configure
  pinMode(CAP1293_IRQ, INPUT); //Pulled up in hardware
  attachInterrupt(digitalPinToInterrupt(CAP1293_IRQ), touchHandler, FALLING);
  touchSensor.setSensitivity(SENSITIVITY_128X);
  #endif

  //Uncomment the next two rows, if accidentally written values too large or made a factory reset
  //MCU_write(VRMS_cal_address, 21280, false);
  //MCU_write(IRMS_cal_address, 257, false);
  #ifdef INIT_SDCARD
  while (1) {
    if (!SD.begin(SD_CS_PIN)) {
      SerialUSB1.println("SD initialization failed!");
      //while (1)
      //  yield();
    } else {
      break;
    }
    delay(500);
  }
  #ifdef DEBUG
  SerialUSB1.println("SD initialization done.");
  #endif

  #endif

  //Read current calibration factors from the chip
  uint32_t tmpv = ConvertUnsignedFixedPoint(MCU_read(VRMS_cal_address, false), 23, 24); //Read VRMS conversionFactor
  uint32_t tmpi = ConvertUnsignedFixedPoint(MCU_read(IRMS_cal_address, false), 23, 24);
  ACSchip.setConversionFactors(tmpv, tmpi);

  #ifdef DEBUG
  SerialUSB1.printf("Init of Conversion Factors:\n VRMS-conv-factor:%e\n IRMS-conv-factor:%e\n", tmpv, tmpi);
  #endif


  //Init of FFT Instance
  arm_status fft_err = arm_cfft_radix4_init_f32(&fftInstance, FFTLEN, 0, 1);
  if (fft_err == ARM_MATH_ARGUMENT_ERROR) {
    SerialUSB1.println("FFT Len Error, cannot calculate FFT");
    calcFFT = false;
  }

  #ifdef DETECT_VOLTAGE
  //Voltage detection. Requires calibration of sensitivity
  float tmp = ACSchip.readVRMS(1);
  if (tmp > 100) //At least 10V RMS voltage
  {
    voltage_detected = true;
    calcFFT = true;
    startSamplingFFT();
    #ifdef DEBUG
    SerialUSB1.printf("Voltage Detected: %.2f V_RMS\n", tmp);
    #endif
  }
  #endif

  Timer1.initialize(1000000); //Time in Microseconds
  Timer1.attachInterrupt(displayUpdate_IRQ);
  //Configuration of watchdog Timer
  WDT_timings_t config;
  config.timeout = wdTimeout; /* in ms, 32ms to 522.232s */
  config.callback = checkCallback;
  //wdt.begin(config);

  //Initialization of timers
  streamTime = millis();  //
  displayTime = millis(); //Display timing
  checkTime = millis();

  #ifdef DEBUG
  SerialUSB1.printf("Init done\n");
  delay(3000);
  #endif
  //End of Init
}

void loop() {
  //newTime = millis(); //Get current millis for timing
  if (samplingDone) {
    #ifdef DEBUG
    SerialUSB1.println("Sampling Done");
    #endif
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
      fft->thd_v = calcTHD(17, Mags);                           //Calculate Total Harmonic Distortion
      if (fft->grouping_en)                               //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = { 0 };
        fft->thdg_v = calcTHDG(Mags, fgroups, Mags, 17);
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
      float tmp = ACSchip.readIRMS(1); // base in a
      tmp = ConvertUnsignedFixedPoint(tmp, 15, 17);
      fft->distortion_factor = (tmp / base_mag_i);
      //SerialUSB1.printf("Dist.-fact:\n Base:%f\n irms:%f\n dist_fact:%f\n", base_mag_i, tmp, distortion_factor);
      fft->thd_i = calcTHD(17, Mags);
      if (fft->grouping_en) //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = { 0 };
        fft->thdg_i = calcTHDG(Mags, fgroups, Mags, 17);
        fft->thdsg_i = calcTHDSG(Mags, fgroups, 17);
      }
      if (currPage == 5)
        displayFFT();

      updateDisplay();

      //Start Sampling again
      startSamplingFFT();
    }
  }
  /*
  //Update Display with a fixed timing only
  if (newTime > displayTime + DISPLAY_UPD_RATE && !sampling) {
    //Display Update if calculation of FFT was stopped and no computer is connected to avoid
    //Conflicts in SPI reading
    if (computerConnected) {
      #ifdef INIT_DISP
      display.clearDisplay();
      display.setCursor(0, 0);
      printTime();
      display.display();
      #endif
      #ifdef DEBUG
      SerialUSB1.println("Computer Connected, printTime");
      #endif
    } else if (!calcFFT) {
      SerialUSB1.printf("updating display, currPage: %u,  %u\n", currPage);
      updateDisplay(); //FIXME uncomment
      SerialUSB1.printf("Icode: %u \n", ACSchip.readReg(0x2A));
      SerialUSB1.printf("Vcode: %u \n", ACSchip.readReg(0x2B));
      //Voltage detection, only if no voltage was detected
      #ifdef DETECT_VOLTAGE
      float voltrms = ACSchip.readVRMS(1);
      if (!voltage_detected && voltrms > 100) {
        //Start sampling FFT (default mode)
        voltage_detected = true;
        startSamplingFFT();
        calcFFT = true;
      }
      #endif
    }
    #ifdef DETECT_VOLTAGE
    detectVoltage();
    #endif
    displayTime = newTime;
  }
  */
  if (updateDisplay_flag == true) {
    SerialUSB1.printf("updating display, currPage: %u\n", currPage);
    updateDisplay();
    noInterrupts();
    updateDisplay_flag = false;
    interrupts();
  }
  //Check for Serial input
  getCommand();
  //Feed the Watchdog. Woof
  /*if (newTime > checkTime + wdTimeout - 5000) {
    wdt.feed();
    #ifdef DEBUG
    SerialUSB1.println("Fed the dawg");
    #endif
    checkTime = newTime;
  }*/

  if (touchHappened == true) {
    touchHandler();
    noInterrupts();
    touchHappened = false;
    interrupts();

  }
}
bool displaytest = true;
/*
  Updates the Display with values
*/
void updateDisplay() {
  #ifdef INIT_DISP
  //Updates to the display
  float temp;
  display.clearDisplay();
  display.setCursor(0, 0); //Cursor Position top-left
  if (!voltage_detected && !displaytest) {
    display.printf("Voltage \ntoo low:\n%2f", ACSchip.readVRMS(1));
  } else {
    if (now() < 1357041600) //Now() is less that year 2013
      display.printf("No Time\n");
    //else
    //  printTime();
    if (currPage == 0) {
      display.printf("RMS values");
      temp = ACSchip.readVRMS(1);
      if (temp < 1)
        display.printf("Vrms: \n%.2f mV\n", temp * 1000.0);
      else
        display.printf("Vrms: \n%.2f V\n", temp);
      temp = ACSchip.readIRMS(1);
      if (temp < 1)
        display.printf("Irms: \n%.2f mA\n", temp * 1000.0);
      else
        display.printf("Irms: \n%.2f A\n", temp);
      //measureFrequency();
      display.printf("Frequency: \n%.2f Hz\n", 50.0/*fft->pwr_f*/);
    } else if (currPage == 1) {
      /*
      display.printf("Harm. Dist. (V)\n");
      display.printf("THD: %.2f %%\n", fft->thd_v);
      if (fft->grouping_en) {
        display.printf("THDG: %.2f\n", fft->thdg_v);
        display.printf("THDSG: %.2f\n", fft->thdsg_v);
      }*/
    } else if (currPage == 2) {
      /*display.printf("Harm. Dist. (I)\n");
      display.printf("THD: \n%.2f %%\n", fft->thd_i);
      if (fft->grouping_en) {
        display.printf("THDG: \n%.2f\n", fft->thdg_i);
        display.printf("THDSG: \n%.2f\n", fft->thdsg_i);
      }*/
    } else if (currPage == 3) {

      //SerialUSB1.println("Trying Power");
      //display.printf("Act. Power \n%.2f W\n", ACSchip.readPACTIVE(1) * MAXPOWER);
      //SerialUSB1.println("Trying Power");
      //display.printf("P-Factor: \n%.2f\n", ACSchip.readPOWERFACTOR() /* fft->distortion_factor*/);
      //SerialUSB1.println("Trying Power");
      //display.printf("Phase Angle\n: %.2fdeg\n", fft->phaseangle);
      //SerialUSB1.println("Finish power");

    }
  }
  display.display();
  #endif
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
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(0));
      } else if (c2 == PER_SECOND) //RMS Voltage per second
      {
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(1));           //("%f\n", ConvertUnsignedFixedPoint(vrms_sec, 15, 15)*RATED_VOLTAGE);
      } else if (c2 == PER_MINUTE) //RMS Voltage per minute
      {
        Serial.printf("%u\n", ACSchip.readVRMS_RAW(2));           //("%f\n", ConvertUnsignedFixedPoint(vrms_min, 15, 15)*RATED_VOLTAGE);
      } else if (c2 == PC_SAMPLING) //Start Voltage Sampling
      {
        connectComputer(true);
        startSamplingPC();
        runmode = 1;
      } else if (c2 == SINGLE_VALUE) //Single Voltage Value
      {
        Serial.printf("%u\n", ACSchip.readVCODE_RAW());
      }
    } else if (c1 == CURRENT) //**************************Current*****************************
    {                   //All current related measurements
      if (c2 == RMS)    //RMS Current
      {
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(0));
      } else if (c2 == PER_SECOND) //RMS Current per second
      {
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(1));
      } else if (c2 == PER_MINUTE) //RMS Current per minute
      {
        Serial.printf("%u\n", ACSchip.readIRMS_RAW(2));
      } else if (c2 == PC_SAMPLING) //Start Current Sampling
      {
        startSamplingPC();
        runmode = 2;
      } else if (c2 == SINGLE_VALUE) //Single Current Value
      {
        Serial.printf("%u\n", ACSchip.readICODE_RAW());
      }
    } else if (c1 == POWER) //**************************Power*****************************
    {                   //All power related measurements
      if (c2 == ACTIVE_POWER)    //Active Power Value
      {
        Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(0));
      } else if (c2 == PER_SECOND) //Active Power per second
      {
        Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(2));
      } else if (c2 == PER_MINUTE) //Active Power per minute
      {
        Serial.printf("%u\n", ACSchip.readPACTIVE_RAW(2));
      } else if (c2 == APPARENT_POWER) //Apparent Power
      {
        Serial.printf("%u\n", ACSchip.readPAPP_RAW());
      } else if (c2 == REACTIVE_POWER) //Reactive Power
      {
        Serial.printf("%u\n", ACSchip.readPIMAG_RAW());
      } else if (c2 == POWER_FACTOR) //Power Factor
      {
        Serial.printf("%u\n", ACSchip.readPOWERFACTOR_RAW());
      } else if (c2 == INSTANT_POWER) //Instant Power Value
      {
        Serial.printf("%u\n", ACSchip.readPINST_RAW());
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
      }
    }
  }
}

//Function to detect Zero Crossings
int detectCurrentZC(uint8_t orient) {
  if (orient == 1) //Positive peak detected -> Detect Negative Slope
  {
    float currdata = ACSchip.readICODE();
    while (currdata > -zcd_thresh) {
      currdata = ACSchip.readICODE();
    }
    return 0; //return a negative slope detection
  }
  if (orient == 0) { //Negative peak detected: Detect positive Slope
    float currdata = ACSchip.readICODE();
    while (currdata < zcd_thresh) {
      currdata = ACSchip.readICODE();
    }
    return 1;
  }
  return 0;
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
  receiveCommandString(&Serial, command->commandBuffer);
  char *endPointer;
  command->address = strtoul(command->commandBuffer, &endPointer, 16);
  command->mask = strtoul(endPointer, &endPointer, 16);
  command->mask = strtoul(endPointer, NULL, 10);

  uint32_t data = ACSchip.readEEPROM(command->address, command->mask, command->pos);
  #ifdef DEBUG
  SerialUSB1.printf("Reading Adr %u, Mask %u, on pos %u\n Data: %u\n",
                    command->address, command->mask, command->pos, data);
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
    receiveCommandString(&Serial, command->commandBuffer);

    char *endPointer;
    adr = strtoul(command->commandBuffer, &endPointer, 10);
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
    receiveCommandString(&Serial, command->commandBuffer);
    adr = strtoul(command->commandBuffer, NULL, 10);
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
  receiveCommandString(&Serial, command->commandBuffer);
  uint32_t adr = strtoul(command->commandBuffer, NULL, 16);

  uint32_t data = ACSchip.readReg(adr);
  #ifdef DEBUG
  SerialUSB1.printf("Reading Adress: %u, value: %u\n", adr, data);
  #endif
  Serial.printf("%u\n", data);
  Serial.send_now();
}

void writeAddress() {
  receiveCommandString(&Serial, command->commandBuffer);
  char *endPointer;
  uint32_t adr = strtoul(command->commandBuffer, &endPointer, 16);
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
  fft->pwr_f = 1 / (ti / 1000000.0); //Convert time to seconds and calculate Frequency
}

/*
  Sets the calibration values. Send
  wc<calibration_factor exp_rms voltage(0)/current_mode(1)>
*/
void calibrateRMS() {
  receiveCommandString(&Serial, command->commandBuffer);
  char *endPointer;
  //Conversion Factor
  uint32_t conv = strtoul(command->commandBuffer, &endPointer, 10);
  //Mode either 0 or 1
  uint8_t mode = (uint8_t)strtoul(endPointer, NULL, 10);

  float converted = ConvertUnsignedFixedPoint(conv, 23, 24);
  #ifdef DEBUG
  SerialUSB1.printf("Calibration: \n Conversion-f (int): %u,\n mode: %u,\n Converted: %e\n", conv, mode, converted);
  #endif

  if (mode == 0) {
    ACSchip.setConversionFactors(converted, -1);
    MCU_write(VRMS_cal_address, conv, false);
  } else {
    ACSchip.setConversionFactors(-1, converted);
    MCU_write(IRMS_cal_address, conv, false);
  }

  Serial.printf("SUCCESS\n");
  Serial.send_now();
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
  receiveCommandString(&Serial, command->commandBuffer); //Read incoming data
  char *endPointer;

  command->address = strtoul(command->commandBuffer, &endPointer, 16);
  command->value = strtol(endPointer, &endPointer, 10);
  command->mask = strtoul(endPointer, &endPointer, 16);
  command->pos = strtol(endPointer, NULL, 10);
  #ifdef DEBUG
  SerialUSB1.printf("Writing Value %u to adr %u with mask %u on pos %u\n",
                command->value, command->address, command->mask, command->pos);
  #endif
  ACSchip.writeEEPROM(command->address, command->value, command->mask, command->pos);
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
  smplTimer.priority(48);
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
  smplTimer.priority(48);
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
    vCodeBuffer[sampleCntr] = ACSchip.readPACTIVE_RAW(0);
  else
    vCodeBuffer[sampleCntr] = ACSchip.readVCODE_RAW();

  iCodeBuffer[sampleCntr] = ACSchip.readICODE_RAW();
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
  receiveCommandString(&Serial, command->commandBuffer);

  float streamTime = strtof(command->commandBuffer, NULL);
  SerialUSB1.printf("Stream time delay: %f\n", streamTime);
  bool err = smplTimer.begin(streamSampling, streamTime); //Maximum
  smplTimer.priority(48);
  if (!err)
    SerialUSB1.println("Error starting Stream Timer");
}

// Sampling for Teensy-FFT Calculation
void getSamples_FFT() {
  ACSchip.readVCODE_RAW();
  ACSchip.readICODE_RAW();
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
  ACSchip.readVCODE_RAW();
  ACSchip.readICODE_RAW();
  Serial.printf("%u,%u\n", vol_mem->vcodes, vol_mem->icodes);
}
/*
  Stops the sampling timer
*/
void stopSampling() {
  smplTimer.end();
}

////////////////////////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////////////////////////


/* prints the current time on the display

*/

void printTime() {
  display.printf("%02d:%02d:%02d\n", hour(), minute(), second());
}
/*
  Detect Voltage
*/
void detectVoltage() {
  //For Live Voltage detection
  float voltrms = ACSchip.readVRMS(1);
  //If plug has been pulled out of the socket (no voltage)
  if (voltrms <= 100) {
    voltage_detected = false;
    calcFFT = false;
    stopSampling();
  }
}
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

//Time synchro
void timeSync() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  receiveCommandString(&Serial, command->commandBuffer);                        //Receive Command
  pctime = strtol(command->commandBuffer, NULL, 10);      //Read Time Value
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
  } else {
    int mins = (int)(ms / 60000);
    display.clearDisplay();
    display.setCursor(5, 10);
    display.printf("Calibration Mode\n");
    display.printf("%u min remaining", mins);
  }
  display.display();
}
void sendCalibration() {
  uint32_t tmp = MCU_read(VRMS_cal_address, false);
  Serial.printf("VRMS,%u\n", tmp);
  tmp = MCU_read(IRMS_cal_address, false);
  Serial.printf("IRMS,%u\n", tmp);
}