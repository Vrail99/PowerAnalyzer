#ifndef PA_DATALOGGER_H
#define PA_DATALOGGER_H


#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "ACS71020.h"
//#include "Adafruit_SSD1327.h"
#include <TimeLib.h>

#define BUFSIZE 300

typedef struct {
    uint32_t prevHour;
    float perHour[24]; //Up to 24 Hours
    float perMonth[31]; //Up to 31 Days
    float perYear[12]; //12 Months
    bool logHourly;
    bool logDaily;
    bool logMonthly;
}EnergyLogger;

typedef struct {
    float pInstant;
    float activeAvgSec;
    float apparentPower;
    float imagPower;
    float powerFactor;
}PowerLogger;

typedef struct {
    float VoltRMS;
    float CurrentRMS;
}RMSLogger;

typedef union {
    struct {
        uint32_t _logVoltageAndCurrent : 1;      //Logs Raw Voltags and Current Values
        uint32_t _logRMSVoltageAndCurrent : 1;   //Logs  RMS voltages and currents averaged over one second
        uint32_t _logPowerFactor : 1;            //Logs the Power Factor
        uint32_t _logPower : 1;                  //Logs the active Power averaged over one second
        uint32_t _logEnergy : 1;                 //Logs the Energy consumed
        uint32_t _logEvents : 1;                 //Power Events, such as
        uint32_t _logTHD : 1;                    //Logs the THD calculated by FFT
        uint32_t _logFFT : 1;                    //Logs the harmonics calculated by FFTs
        uint32_t _printToSerial : 1;             //Prints the logs to the SerialMonitor instead of the SD Card
        uint32_t _emtpy : 7;                      //Empty field for further flags
        uint32_t _sampleTime : 16;                //Time for logging in milliseconds
    }flags;
    uint32_t CONFIG;
}loggerConfig;

class DataLogger {
private:
    const char* fft_prefix = "fft";
    const char* rms_prefix = "rms";
    const char* power_prefix = "power";
    const char* energy_prefix = "energy";

    // SD CARD
    Sd2Card _card;
    SdVolume _volume;
    SdFile _root;

    boolean _cs = 0;
    usb_serial2_class* _serialBus;              //Debug Serial Port

    //Flags for logging
    loggerConfig* _config;
    EnergyLogger energy;                        //Energy in Watt over various times
    PowerLogger power;
    RMSLogger rms;

    ACS71020* _acsChip;                         //Instance for ACS71020

    char fileNameBuffer[BUFSIZE];
    char logBuffer[BUFSIZE];

    char currentDateString[5];
    uint32_t currDay;
    uint32_t currMonth;
    uint32_t currYear;

    void _formatLogValues();        //Formats all enabled Logging measurements to one string
    void _formatFFTLog();
    void _logEnergy();
    void _logFFT();
    void _logRMS();
    void _initEnergy();
    void _saveEnergy();
    void _resetEnergy();
    void _updateLogfile();

public:
    DataLogger(ACS71020* chip, usb_serial2_class* serialBus, uint16_t cs);
    boolean init();    //Inits Communication with SD-Card
    boolean writeLog(File logFile, String log);                 //Writes a line to a logFile
    void printDirectory(File dir, int numTabs);                 //Prints the whole directory to the SerialUSB1 monitor
    uint32_t getFileCount();
    void Log_RMS_VandC(boolean state);
    void Log_RAW_VandC(boolean state);
    void Log_PowerFactor(boolean state);
    void Log_Power(boolean state);
    void Log_Energy(boolean state);
    void Log_Events(boolean state);
    void Log_THD(boolean state);
    void Log_FFT(boolean state);
    void LogAll();                             //Gets log values for every entry
};

#endif