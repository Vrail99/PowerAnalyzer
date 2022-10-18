#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "Adafruit_SSD1327.h" //Display Library
#include "PowerAnalyzer.h"
#include "harmonicCalculations.h"

//Default mode pages
#define NO_OF_PAGES 6

//DataLogging Menug options
#define MENU_OPTIONS 6

#define DISP_HEIGHT 128
#define DISP_WIDTH 128

enum display_pages {
    DISP_VOLT = 0,
    DISP_CURRENT = 1,
    DISP_POWER = 2,
    DISP_THD_I = 3,
    DISP_THD_V = 4,
    DISP_FFT_VOLT = 5,
    DISP_FFT_CURRENT = 6
};

void print_time_in_min_sec(Adafruit_SSD1327* display, int16_t x, int16_t y, uint32_t ms);
void displayDefault(Adafruit_SSD1327* display, ACS71020* ACSchip, uint8_t currPage, PowerQuality* pwr_qual);
void displayFFT(Adafruit_SSD1327* display, float* Mags, uint8_t currPage);
void displayDataLogMenu(Adafruit_SSD1327* display, uint8_t menuOption, const char* optionName[]);

#endif