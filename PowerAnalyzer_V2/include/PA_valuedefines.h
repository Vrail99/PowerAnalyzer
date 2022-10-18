#ifndef PA_VALUEDEFINES_H
#define PA_VALUEDEFINES_H

#define ACS_SPI_SPEED 10000000 //SPI Speed setting for the ACS Chip (10 Mhz)

//Addresses in Teensy-EEPROm for calibration values
#define VRMS_cal_address 0
#define IRMS_cal_address VRMS_cal_address + 4

//Characteristics
#define MAXVOLT 366             //Voltage divider at Input
#define MAXCURR 15                //Fixed for 5V SPI Version of the chip
#define MAXPOWER (MAXVOLT * MAXCURR) //For Power Calculations


//Frequency and FFT Defines
#define FFTSAMPLEFREQ  20408
#define FFTSAMPLERATE  49      //Samplerate in Microseconds
#define FFTREALSAMPLES  4082   // 200ms Sampling time with 49 us/Sa ~= 4082 Sa
#define FFTLEN  4096           //Full FFT length for CMSIS Function
#define FFTBUFFLEN  (2 * FFTLEN) //Buffer for CMSIS structure
#define BINSIZE 5             //FFTSAMPLEFREQ / FFTLEN;

//For streaming to a computer, more samplerate possible
#define SAMPLERATE 32
#define SAMPBUFFLEN 6250

#endif