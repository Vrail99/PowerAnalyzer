#ifndef PowerAnalyzer_H
#define PowerAnalyzer_H

#include <Arduino.h>
#include "PowerAnalyzer_Config.h"
#include "ACS71020.h"
#include "ConversionHelpers.h"

enum serialCommands {
    TIMESYNC = 't',

    VOLTAGE = 'v',
    CURRENT = 'i',
    RMS = 'r',
    PER_SECOND = 's',
    PER_MINUTE = 'm',
    PC_SAMPLING = 'c',
    SINGLE_VALUE = 'd',

    POWER = 'p',
    ACTIVE_POWER = 'a',
    APPARENT_POWER = 'p',
    REACTIVE_POWER = 'i',
    POWER_FACTOR = 'f',
    INSTANT_POWER = 't',

    CHIP_STATUS = 's',
    RMS_POINTS = 'n',
    ZERO_CROSS = 'z',
    PF_SIGN = 'f',
    LEAD_LAG = 'a',
    UNDERVOLTAGE = 'u',
    OVERVOLTAGE = 'o',
    OVERCURRENT = 'l',
    FAULTOUT = 'c',

    WRITE = 'w',
    WRITE_ACS = 'w',
    W_ADDRESS = 'a',
    W_MCU_EEPROM = 'e',
    CALIBRATION = 'c',

    DUAL_CODES = 'b',
    FFT_SAMPLING = 'f',
    PC_STREAM = 't',

    READ = 'e',
    READ_ACS = 'v',
    R_ADDRESS = 'a',
    R_MCU_EEPROM = 'e',
    R_CALIBRATION = 's',

    PA_IDLE = 'x',

    PC = 'c',
    CONNECT = 'c',
    DISCONNECT = 'd',
    CONNTEST = 'i',
    VOLT_DETECT = 'v',
    CALIBRATION_MODE = 'a',
    SD_READ = 's'
};

struct volatile_memory
{
    uint32_t tmp;
    uint32_t vcodes;
    uint32_t icodes;
    uint32_t flags;
    uint32_t pinst;
    uint32_t numptsout;
    uint32_t pfact;
    uint32_t pImag;
    uint32_t pApp;
    uint32_t pActive;
    uint32_t v_rms;
    uint32_t i_rms;
    uint32_t pact_sec;
    uint32_t irms_sec;
    uint32_t vrms_sec;
    uint32_t pact_min;
    uint32_t irms_min;
    uint32_t vrms_min;
}acs71020_mem;

struct volatile_memory *vol_mem = &acs71020_mem;

void initVolatileMemory(volatile_memory *ptr) {
    ptr->tmp = 0;
    ptr->vcodes = 0;
    ptr->icodes = 0;
};

struct harmonic_factors {
    //Variables for harmonic distortion calculation
    float thd_v; //Total Harmonic Voltage Distortion
    float thd_i; //Total Harmonic Current Distortion
    float pwr_f; //Power Frequency
    float phaseangle;
    float distortion_factor; //Distortion factor for the pf-calculation

    bool grouping_en = true;
    float thdg_v = 0; //Group distortion voltage
    float thdsg_v = 0; //Subgroup distortion voltage
    float thdg_i = 0; //Group distortion current
    float thdsg_i = 0; //Subgroup distortion current
}harmonics;

struct harmonic_factors *fft = &harmonics;


#endif