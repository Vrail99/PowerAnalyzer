#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "Adafruit_SSD1327.h" //Display Library

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

#endif