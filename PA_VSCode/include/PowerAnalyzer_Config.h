#include <Arduino.h>


//Touch-Inputs:
#define TOUCH_PREV 2
#define TOUCH_NEXT 3

#define NO_OF_PAGES 6 //Number of pages


//Defines for ACS71020 Chip
#define ACS_CS 10 //Chip Select for ACS71020
#define ACS_SPI_SPEED 10000000 //SPI Speed setting for the ACS Chip (10 Mhz)
#define ACS_CUSTOMER_CODE 0x4F70656E //Customer code for unlocking

//Calibration
#define VRMS_cal_address 0
#define IRMS_cal_address VRMS_cal_address + 4


//Defines for Display SSD1327
#define OLED_RST 14 //20
#define OLED_DC 15  //21
#define OLED_CS 16  //22

//Display Measurements
#define DISP_WIDTH 128
#define DISP_HEIGHT 128
#define DISPLAY_UPD_RATE 1000 //Display update time in Milliseconds


//Defines for the SD-Card
#define SD_CS_PIN 9


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

//ARM_MATH FFT PARAMETERS
#define ifftFlag 0
arm_cfft_radix4_instance_f32 fftInstance;

//Characteristics
#define MAXVOLT 366               //Voltage divider at Input
#define MAXCURR 15                //Fixed for 5V SPI Version of the chip
#define MAXPOWER MAXVOLT *MAXCURR //For Power Calculations

