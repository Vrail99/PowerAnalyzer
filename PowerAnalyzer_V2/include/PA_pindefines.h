#ifndef PA_PINDEFINES_H
#define PA_PINDEFINES_H

#define VERSION 2.2

/*                __________USB__________
GND        GND   |                       |   +5V
SD_CS       0    |                       |   GND
            1    |                       |   +3V3
            2    |                       |   23
            3    |                       |   22
            4    |                       |   21
            5    |                       |   20
            6    |                       |   19      SCL0 (Display I2C)
            7    |                       |   18      SDA0 (Display I2C)
            8    |                       |   17
ACS_EN      9    |                       |   16      CAP_IRQ
ACS_CS      10   |                       |   15      OLED_DC
ACS_MISO    11   |                       |   14      OLED_RST
ACS_MOSI    12   |                       |   13      ACS_SCK
                  _______________________
*/
#include <Arduino.h>

//Defines for GPIO
#define ACS_CS 10              //Chip Select for the ACS Chip, active-low
#define ACS_EN 9               //enable MISO line, active-high

  //Display
#define OLED_RST 14 //20
#define OLED_DC 15  //21

//Touch-Controller Interrupt-Pin (ALERT)
#define CAP_IRQ 16

//SD-Card CS-Pin
#define SD_CS_PIN 0

#endif