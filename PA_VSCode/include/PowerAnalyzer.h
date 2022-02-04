#ifndef PowerAnalyzer_H
#define PowerAnalyzer_H

#include <Arduino.h>
#include "PowerAnalyzer_Config.h"
#include "ACS71020.h"
#include "CAP1293.h"
#include "ConversionHelpers.h"
#include "HarmonicCalculations.h"
#include "Input_Output.h"
#include "PA_DataLogger.h"

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

struct harmonic_factors {
    //Variables for harmonic distortion calculation
    float thd_v; //Total Harmonic Voltage Distortion
    float thd_i; //Total Harmonic Current Distortion
    float pwr_f; //Power Frequency
    float phaseangle;
    float distortion_factor; //Distortion factor for the pf-calculation

    bool grouping_en;
    float thdg_v; //Group distortion voltage
    float thdsg_v; //Subgroup distortion voltage
    float thdg_i; //Group distortion current
    float thdsg_i; //Subgroup distortion current
};

#endif