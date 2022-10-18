#ifndef POWERANALYZER_H
#define POWERANALYZER_H

#include "PA_valuedefines.h"
#include "PA_pindefines.h"
#include "ACS71020.h"         //Include of the Chip-Library

//Uncomment for Debug messages on Serial Port 3
#define DEBUG

//Uncomment for SD init
#define INIT_SDCARD

//Uncomment for Display init
#define INIT_DISP

//Uncomment for Touch-Sensor init
#define INIT_TOUCH

//Uncomment for voltage detection, useful for off-grid testing
//#define DETECT_VOLTAGE

#endif