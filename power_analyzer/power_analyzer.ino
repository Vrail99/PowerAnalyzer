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
#include <ACS71020.h>         //Include of the Chip-Library
#include <Adafruit_SSD1327.h> //Display Library
#include <TimeLib.h>          //Time Library for Real-Time Clock
#include <math.h>
#include <Watchdog_t4.h> //Watchdog Library for reset on error

//Uncomment for Debug messages on Serial Port 3
//#define DEBUG

//Defines for GPIO
#define ACS_CS 10              //Chip Select for the ACS Chip
#define ACS_SPI_SPEED 10000000 //SPI Speed setting for the ACS Chip (10 Mhz)
#define ACS_CUSTOMER_CODE 0x4F70656E

//Display GPIO defines
#define OLED_RST 14 //20
#define OLED_DC 15  //21
#define OLED_CS 16  //22

//Creation of a Chip instance
ACS71020 ACSchip(ACS_SPI_SPEED, ACS_CS, ACS_CUSTOMER_CODE);

//Display instance
Adafruit_SSD1327 display(128, 128, &SPI, OLED_DC, OLED_RST, OLED_CS);

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

//Characteristics
#define MAXVOLT 366               //Voltage divider at Input
#define MAXCURR 15                //Fixed for 5V SPI Version of the chip
#define MAXPOWER MAXVOLT *MAXCURR //For Power Calculations

//Character arrays for input commands
#define MAX_RECEIVEABLE 32 // Number of characters that can be read
char receivedChars[MAX_RECEIVEABLE];

//Frequency and FFT Defines
#define FFTSAMPLEFREQ 20408
#define FFTSAMPLERATE 49      //Samplerate in Microseconds
#define FFTREALSAMPLES 4082   // 200ms Sampling time with 49 us/Sa ~= 4082 Sa
#define FFTLEN 4096           //Full FFT length for CMSIS Function
#define FFTBUFFLEN 2 * FFTLEN //Buffer for CMSIS structure
#define binSize 5             //FFTSAMPLEFREQ / FFTLEN;
//PC
#define SAMPLERATE 32
#define SAMPBUFFLEN 6250

//Timer for acquiring samples
IntervalTimer smplTimer;
boolean samplingDone = false;
uint32_t sampleCntr = 0;

uint16_t DISPLAY_UPD_RATE = 1000; //Display update time in Milliseconds

//Flags for enabling Streaming
bool sampling = false;
bool powerSampling = false;
bool calcFFT = false;
uint8_t runmode = 0;            //0: Idle, 1: Voltage/Power, 2: Current, 3: Voltage & Current
bool computerConnected = false; // Display flag
bool voltage_detected = false;  //For frequency measurement, requires a voltage for zero crossings

//ARM_MATH FFT PARAMETERS
const uint16_t ifftFlag = 0;
arm_cfft_radix4_instance_f32 fftInstance;

//Buffer Arrays, Initialized to 0
//PC Sample Buffers
uint32_t vCodeBuffer[SAMPBUFFLEN] = {0};
uint32_t iCodeBuffer[SAMPBUFFLEN] = {0};

//Teensy Sample Buffers
float vSamps[FFTBUFFLEN] = {0.0};
float iSamps[FFTBUFFLEN] = {0.0};
float Mags[FFTLEN] = {0.0};

////////////////////////////////////////////////////////////////////////////////
// Volatile Memory Values
////////////////////////////////////////////////////////////////////////////////
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
uint32_t v_rms;
uint32_t i_rms;
uint32_t pact_sec = 0;
uint32_t irms_sec = 0;
uint32_t vrms_sec = 0;
uint32_t pact_min = 0;
uint32_t irms_min = 0;
uint32_t vrms_min = 0;

float zcd_thresh = 0.2;

//Variables for harmonic distortion calculation
float thd_v; //Total Harmonic Voltage Distortion
float thd_i; //Total Harmonic Current Distortion
float pwr_f; //Power Frequency
float phaseangle;

bool grouping_en = true;
float thdg_v = 0;
float thdsg_v = 0;
float thdg_i = 0;
float thdsg_i = 0;

//ReadTimes
uint32_t newTime, streamTime, displayTime; //Time-Tracker, seconds-timer, minute-timer

//Instantiate Watchdogtimer 3
WDT_T4<WDT3> wdt;
uint32_t checkTime;
uint32_t wdTimeout = 30000; //ms

void checkCallback()
{
}

void setup()
{
  //Configuration of watchdog Timer
  WDT_timings_t config;
  config.timeout = wdTimeout; /* in ms, 32ms to 522.232s */
  config.callback = checkCallback;
  wdt.begin(config);
  
  //Init Serial communication
  //Dual Channel Serial Communication
  Serial.begin(115200);     //Main transmit port
  SerialUSB1.begin(115200); //Debug and status port

  //Display Initialization
  if (!display.begin(0x3D))
  {
    SerialUSB1.println("Unable to initialize OLED");
    while (1)
      yield();
  }
  display.clearDisplay();
  display.display();

  //Init of FFT Instance
  arm_status fft_err = arm_cfft_radix4_init_f32(&fftInstance, FFTLEN, 0, 1);
  if (fft_err == ARM_MATH_ARGUMENT_ERROR)
  {
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

  //Initialization of timers
  streamTime = millis();  //
  displayTime = millis(); //Display timing
  checkTime = millis();

  //End of Init
}

void loop()
{
  newTime = millis(); //Get current millis for timing

  if (samplingDone)
  {
    if (runmode == 1) //Transmit all voltage samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u\n", vCodeBuffer[i]);
      startSamplingPC();
    }
    else if (runmode == 2) //Transmit all current samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u\n", iCodeBuffer[i]);
      startSamplingPC();
    }
    else if (runmode == 3) //Transmit alls current and voltage samples
    {
      for (uint16_t i = 0; i < SAMPBUFFLEN; i++)
        Serial.printf("%u,%u\n", vCodeBuffer[i], iCodeBuffer[i]);
      startSamplingPC();
    }
    //Calculate FFT and the harmonic distortion
    else if (calcFFT)
    {
      arm_cfft_radix4_f32(&fftInstance, vSamps);     //In-Place FFT
      float angle_v = atan2(vSamps[21], vSamps[20]); //Angle from pure spectrum
      arm_cmplx_mag_f32(vSamps, Mags, FFTLEN);       //Calculate Magnitudes
      thd_v = calcTHD(17);                           //Calculate Total Harmonic Distortion
      if (grouping_en)                               //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = {0};
        thdg_v = calcTHDG(Mags, fgroups, 17);
        thdsg_v = calcTHDSG(Mags, fgroups, 17);
      }
      //FFT for Current samples
      arm_cfft_radix4_f32(&fftInstance, iSamps); //In-Place FFT
      float angle_i = atan2(iSamps[21], iSamps[20]);
      phaseangle = (angle_v - angle_i) * (180 / PI);

      arm_cmplx_mag_f32(iSamps, Mags, FFTLEN);
      thd_i = calcTHD(17);
      if (grouping_en) //If grouping enabled, calculate Harmonic Groups
      {
        float fgroups[17] = {0};
        thdg_i = calcTHDG(Mags, fgroups, 17);
        thdsg_i = calcTHDSG(Mags, fgroups, 17);
      }

      updateDisplay();

      //Start Sampling again
      startSamplingFFT();
    }
  }
  //Update Display with a fixed timing only
  if (newTime > displayTime + DISPLAY_UPD_RATE && !sampling)
  {
    //Display Update if calculation of FFT was stopped and no computer is connected to avoid
    //Conflicts in SPI reading
    if (!calcFFT && !computerConnected)
    {
      //For Live Voltage detection
      float voltrms = calcVRMS();
      updateDisplay();
      //Voltage detection, only if no voltage was detected
      if (!voltage_detected && voltrms > 100)
      {
        //Start sampling FFT (default mode)
        voltage_detected = true;
        startSamplingFFT();
        calcFFT = true;
      }
    }
    else
    {
      //For Live Voltage detection
      float voltrms = calcVRMS();
      //If plug has been pulled out of the socket (no voltage)
      if (voltage_detected && voltrms <= 100)
      {
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

  if (newTime > checkTime + wdTimeout - 5000)
  {
    wdt.feed();
    checkTime = newTime;
  }
}

//Function to detect Zero Crossings
int detectCurrentZC(uint8_t orient)
{
  if (orient == 1) //Positive peak detected -> Detect Negative Slope
  {
    float currdata = readICodes();
    while (currdata > -zcd_thresh)
    {
      currdata = readICodes();
    }
    return 0; //return a negative slope detection
  }
  if (orient == 0)
  { //Negative peak detected: Detect positive Slope
    float currdata = readICodes();
    while (currdata < zcd_thresh)
    {
      currdata = readICodes();
    }
    return 1;
  }
}

float calcTHD(uint8_t order)
{
  uint32_t s = 50 / binSize; //50Hz base frequency
  float thd = 0;
  float base = Mags[s];
  for (uint32_t i = s * 2; i < order * s; i += s)
  {
    thd += pow(Mags[i] / base, 2);
  }
  thd = sqrt(thd) * 100;

  return thd;
}

uint32_t getMaxValueIndex(float values[], uint32_t arrlen)
{
  uint32_t maxIndex = 0;
  float maxVal = values[maxIndex];

  for (uint32_t i = 0; i < arrlen; i++)
  {
    if (values[i] > maxVal)
    {
      maxIndex = i;
      maxVal = values[i];
    }
  }
  return maxIndex;
}

/*
  Updates the Display with values
*/
void updateDisplay()
{
  float temp = calcVRMS();
  //Updates to the display
  display.clearDisplay();
  display.setCursor(0, 0); //Cursor Position top-left
  if (voltage_detected)
  {
    measureFrequency();
    if (now() < 1357041600) //Now() is less that year 2013
      display.printf("Zeit: no sync\n");
    else
      display.printf("Zeit: %d:%d:%d\n", hour(), minute(), second());
    display.printf("THD (V): %.2f %%\n", thd_v);
    if (grouping_en)
    {
      display.printf("THDG: %.2f\n", thdg_v);
      display.printf("THDSG: %.2f\n", thdsg_v);
    }
    display.printf("THD (I): %.2f %%\n", thd_i);
    if (grouping_en)
    {
      display.printf("THDG: %.2f\n", thdg_i);
      display.printf("THDSG: %.2f\n", thdsg_i);
    }
    display.printf("Freq: %.2f Hz\n", pwr_f);
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
    pact_sec = ACSchip.readReg(0x28) & 0x1FFFF; //17-bit FP, 15 frac
    display.printf("Power %.2f W\n", ConvertSignedFixedPoint(pact_sec, 15, 17) * MAXPOWER);
    pfact = ACS_PF;
    temp = ConvertSignedFixedPoint(pfact, 9, 11);
    display.printf("P-Factor: %.2f\n", temp);
    //float ang = acos(temp) * (180 / PI);
    display.printf("Winkel: %.2fdeg\n", phaseangle);
  }
  else
  {
    display.printf("Voltage too low: %2f", temp);
  }
  display.display();
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
  uint16_t adr = strtoul(receivedChars, &endPointer, 16);
  uint32_t mask = strtoul(endPointer, &endPointer, 16);
  uint8_t pos = strtoul(endPointer, NULL, 10);

  uint32_t data = ACSchip.readEEPROM(adr, mask, pos);

  Serial.printf("%u\n", data);
  Serial.send_now(); //Sends instantly to avoid buffering
}

/*
  Measures the frequency of a waveform, by detecting consecutive rising edges
  Takes the Pulsewidth into account

  Possible extension: Frequency-Measurement depending on halfcycle_en
*/
void measureFrequency()
{
  uint32_t firstTime;
  uint32_t secTime;
  //Read the ZC-Register until a ZC occurs
  uint32_t ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1)
  {
    ZC = ACSchip.readReg(0x2D) & 0x1; //update value
  }
  firstTime = micros();  //Save the time of the first rising edge
  delayMicroseconds(32); //Wait for duration of pulse
  //Read the ZC-Register again until a ZC occurs
  ZC = ACSchip.readReg(0x2D) & 0x1;
  while (ZC != 1)
  {
    ZC = ACSchip.readReg(0x2D) & 0x1; //update value
  }
  secTime = micros(); //Save the time of the second rising edge
  float ti = secTime - firstTime;
  pwr_f = 1 / (ti / 1000000); //Convert time to seconds and calculate Frequency
}

/*
  Receives a command String with chosen delimiters. Can read up to MAX_RECEIVABLE chars
*/
void receiveCommandString()
{
  char stdel = '<';              //Start delimiter
  char enddel = '>';             //End delimiter
  bool receiving = false;        //Receiving flag
  uint8_t cntr = 0;              //Char counter
  while (Serial.available() > 0) //Receive while characters are on the Serial Bus / Buffer
  {
    char rc = Serial.read();

    if (receiving) //If receiving of values has started
    {
      if (rc != enddel)
      {
        receivedChars[cntr] = rc;
        cntr++;
        if (cntr > MAX_RECEIVEABLE)
          cntr = MAX_RECEIVEABLE - 1;
      }
      else //If end delimiter was sent
      {
        receivedChars[cntr] = '\0'; //String end delimiter
        receiving = false;          //Stop receiving and
        cntr = 0;                   //Reset counter
      }
    }
    else
    {
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
void writeEEPROMValue()
{
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

  ACSchip.writeEEPROM(address, value, mask, pos);
}

// Start FFT Sampling for Teensy calculation
void startSamplingFFT()
{
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
#ifdef DEBUG
  if (!err)
    SerialUSB1.println("Error starting Timer");
  SerialUSB1.println("Started FFT Sample Timer");
#endif
}

// Start normal Sampling for PC calculation
void startSamplingPC()
{
  sampling = true;
  stopSampling();
  samplingDone = false;
  sampleCntr = 0;
  if (voltage_detected)
  {
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
void getSamplesPC()
{
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
void startStreamingPC()
{
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
void getSamples_FFT()
{
  vcodes = ACS_VCODE;
  icodes = ACS_ICODE;
  vSamps[sampleCntr] = ConvertSignedFixedPoint(vcodes, 16, 17) * 366.94;
  iSamps[sampleCntr] = ConvertSignedFixedPoint(icodes, 15, 17) * 15;
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
void streamSampling()
{
  vcodes = ACS_VCODE;
  icodes = ACS_ICODE;
  Serial.printf("%u,%u\n", vcodes, icodes);
}
/*
  Stops the sampling timer
*/
void stopSampling()
{
  smplTimer.end();
#ifdef DEBUG
  SerialUSB1.println("Stopped Sample Timer");
#endif
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
    char c1 = Serial.read(); //First letter
    char c2 = Serial.read(); //Second letter

    if (c1 == 't') //Time Sync
    {
      timeSync();
    }
    //All voltage related measurements
    else if (c1 == 'v') //**************************Voltage*****************************
    {
      if (c2 == 'r') //RMS Voltage
      {
        v_rms = ACS_VRMS;
        Serial.printf("%u\n", v_rms);
      }
      else if (c2 == 's') //RMS Voltage per second
      {
        vrms_sec = ACSchip.readReg(0x26) & 0x7FFF; //15-bit FP, 15 frac
        Serial.printf("%u\n", vrms_sec);           //("%f\n", ConvertUnsignedFixedPoint(vrms_sec, 15, 15)*RATED_VOLTAGE);
      }
      else if (c2 == 'm') //RMS Voltage per minute
      {
        vrms_min = ACSchip.readReg(0x27) & 0x7FFF; //15-bit FP, 15 frac
        Serial.printf("%u\n", vrms_min);           //("%f\n", ConvertUnsignedFixedPoint(vrms_min, 15, 15)*RATED_VOLTAGE);
      }
      else if (c2 == 'c') //Start Voltage Sampling
      {
        connectComputer(true);
        startSamplingPC();
        runmode = 1;
      }
      else if (c2 == 'd') //Single Voltage Value
      {
        vcodes = ACS_VCODE;
        Serial.printf("%u\n", vcodes);
      }
    }
    else if (c1 == 'i') //**************************Current*****************************
    {                   //All current related measurements
      if (c2 == 'r')    //RMS Current
      {
        i_rms = ACS_IRMS;
        Serial.printf("%u\n", i_rms);
      }
      else if (c2 == 's') //RMS Current per second
      {
        irms_sec = (ACSchip.readReg(0x26) >> 16) & 0x7FFF; //15-bit FP, 14 frac
        Serial.printf("%u\n", irms_sec);
      }
      else if (c2 == 'm') //RMS Current per minute
      {
        irms_min = (ACSchip.readReg(0x27) >> 16) & 0x7FFF; //15-bit FP, 14 frac
        Serial.printf("%u\n", irms_min);
      }
      else if (c2 == 'c') //Start Current Sampling
      {
        startSamplingPC();
        runmode = 2;
      }
      else if (c2 == 'd') //Single Voltage Value
      {
        icodes = ACS_ICODE;
        Serial.printf("%u\n", icodes);
      }
    }
    else if (c1 == 'p') //**************************Power*****************************
    {                   //All power related measurements
      if (c2 == 'a')    //Active Power Value
      {
        pActive = ACS_PACTIVE;
        Serial.printf("%u\n", pActive);
      }
      else if (c2 == 's') //Active Power per second
      {
        pact_sec = ACSchip.readReg(0x28) & 0x1FFFF; //17-bit FP, 15 frac
        Serial.printf("%u\n", pact_sec);
      }
      else if (c2 == 'm') //Active Power per minute
      {
        pact_min = ACSchip.readReg(0x29) & 0x1FFFF; //17-bit FP, 15 frac
        Serial.printf("%u\n", pact_min);
      }
      else if (c2 == 'p') //Apparent Power
      {
        pApp = ACS_PAPPARENT;
        Serial.printf("%u\n", pApp);
      }
      else if (c2 == 'i') //Reactive Power
      {
        pImag = ACS_PIMAG;
        Serial.printf("%u\n", pImag);
      }
      else if (c2 == 'f') //Power Factor
      {
        pfact = ACS_PF;
        Serial.printf("%u\n", pfact);
      }
      else if (c2 == 't') //Instant Power Value
      {
        pinst = ACS_PINST;
        Serial.printf("%u\n", pinst);
      }
      else if (c2 == 'c') //Power codes
      {
        powerSampling = true;
        startSamplingPC();
        runmode = 1; //Voltage or active power
      }
    }
    else if (c1 == 's') //**************************Status*****************************
    {                   //All status Values
      flags = ACSchip.readReg(0x2D);
      if (c2 == 'n')
      {
        numptsout = ACSchip.readReg(0x25) & 0x1FF; //Unsigned 9-bit
        Serial.printf("%u\n", numptsout);
      }
      else if (c2 == 'z')
        Serial.printf("%u\n", (flags & 0x1)); //Zero crossing
      else if (c2 == 'f')
        Serial.printf("%u\n", ((flags >> 6) & 0x1)); //pospf, 0: Generation, 1: Consumption
      else if (c2 == 'a')
        Serial.printf("%u\n", ((flags >> 5) & 0x1)); //posangle
      else if (c2 == 'u')
        Serial.printf("%u\n", ((flags >> 4) & 0x1)); //undervol
      else if (c2 == 'o')
        Serial.printf("%u\n", ((flags >> 3) & 0x1)); //overvol
      else if (c2 == 'l')
        Serial.printf("%u\n", ((flags >> 2) & 0x1)); //faultlatched
      else if (c2 == 'c')
        Serial.printf("%u\n", ((flags >> 1) & 0x1)); //faultout
    }
    else if (c1 == 'w') //Write to an EEPROM register
    {
      writeEEPROMValue();
    }
    else if (c1 == 'b')
    { //Print Both Code Values
      if (c2 == 'c')
      {
        connectComputer(true);
        startSamplingPC();
        runmode = 3;
      }
      else if (c2 == 'f')
      {
        calcFFT = true;
        startSamplingFFT();
      }
      else if (c2 == 't')
      {
        SerialUSB1.println("Started Stream");
        startStreamingPC();
      }
    }
    else if (c1 == 'e') //EEPROM-Operations
    {
      if (c2 == 'v')
      {
        readSingleEEPROM();
      }
    }
    else if (c1 == 'x') //Set to Idle Mode
    {
      stopSampling();
      sampling = false;
      samplingDone = false;
      powerSampling = false;
      connectComputer(false);
      runmode = 0;
      calcFFT = false;
    }
    else if (c1 == 'c') //Computer Commands
    {
      if (c2 == 'c') //Connect
      {
        connectComputer(true);
      }
      else if (c2 == 'd') //Disconnect
      {
        connectComputer(false);
        updateDisplay();
      }
      else if (c2 == 'i')
      { //Connection test
        Serial.print("tconn\n");
        connectComputer(true);
      }
      else if (c2 == 'v') //Ignore voltage detection
      {
#ifdef DEBUG
        SerialUSB1.println("Ignoring Voltage Level");
#endif
        voltage_detected = true;
      }
    }
  }
}

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
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Computer Connected.Output Disabled");
    display.display();
    SerialUSB1.println("Computer connected");
  }
  else
  {
    SerialUSB1.println("Computer disconnected");
  }
}

//Conversion helpers
/*
 * Convert an unsigned bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
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

/*
 * Convert a signed bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be sign extended then converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
{
  int32_t signedValue = SignExtendBitfield(inputValue, width);
  return (float)signedValue / (float)(1L << binaryPoint);
}

/*
 * Sign extend a bitfield which if right justified
 *
 *    data        - the bitfield to be sign extended
 *    width       - the width of the bitfield
 *    returns     - the sign extended bitfield
 */
int32_t SignExtendBitfield(uint32_t data, uint16_t width)
{
  // If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
  if (width == 32)
  {
    return (int32_t)data;
  }

  int32_t x = (int32_t)data;
  int32_t mask = 1L << (width - 1);

  x = x & ((1 << width) - 1); // make sure the upper bits are zero

  return (int32_t)((x ^ mask) - mask);
}

float readICodes()
{
  return ConvertSignedFixedPoint(ACS_ICODE, 15, 17) * 15;
}

void timeSync()
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  receiveCommandString();                        //Receive Command
  pctime = strtol(receivedChars, NULL, 10);      //Read Time Value
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

////////////////////////////////////////////////////////////////////////////////
// Calculation Functions
////////////////////////////////////////////////////////////////////////////////

float calcVRMS()
{
  v_rms = ACS_VRMS;
  uint32_t calib_Code = 21280;              //Calibration Factor
  float exp_RMS = 0.1784;                   //Expected Input VRMS from Calibration
  float conv_factor = exp_RMS / calib_Code; //Actual Sensitivity

  float vrms_rsense = v_rms * conv_factor;           //Voltage over RS1
  float vrms_input = vrms_rsense * (4003000 / 3000); //Voltage at output
  return vrms_input;
}
float calcIRMS()
{
  i_rms = ACS_IRMS;
  uint32_t i_calib_Code = 257;
  float conv_factor = 0.26086 / i_calib_Code; //Calibration from Lightbulb
  float irms = i_rms * conv_factor;
  return irms;
}

float calcTHDG(float frequencies[], float output[], int order)
{
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
  for (uint8_t i = 2; i <= order; i++)
  {
    thdg += pow(output[i] / base, 2);
  }
  thdg = sqrt(thdg) * 100;
  return thdg;
}
float calcTHDSG(float frequencies[], float output[], int order)
{
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
  for (uint8_t i = 2; i <= order; i++)
  {
    thdsg += pow(output[i] / base, 2);
  }
  thdsg = sqrt(thdsg) * 100;
  return thdsg;
}
