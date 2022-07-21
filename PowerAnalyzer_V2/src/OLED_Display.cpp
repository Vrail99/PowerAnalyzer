#include "OLED_Display.h"

void displayDefault(Adafruit_SSD1327* display, ACS71020* ACSchip, uint8_t currPage) {
    float temp;
    display->clearDisplay();
    display->setCursor(0, 0); //Cursor Position top-left
    //Updates to the display
    if (currPage == DISP_CURRENT) {
        display->printf("Current\n");
        temp = ACSchip->readIRMS(1);
        if (temp < 1.0F)
            display->printf("Irms:\n%.2f mA\n", temp * 1000.0F);
        else
            display->printf("Irms:\n%.2f A\n", temp);
    } else if (currPage == DISP_VOLT) {
        display->printf("Voltage\n");
        temp = ACSchip->readVRMS(0);
        if (temp < 1.0F)
            display->printf("Vrms:\n%.2f mV\n", temp * 1000.0F);
        else
            display->printf("Vrms:\n%.2f V\n", temp);
        temp = ACSchip->readPACTIVE(1) * 5490.0;
        display->printf("Power\n%.2f W\n", temp);


    } /*else if (!voltage_detected) {
        display->printf("Voltage \ntoo low:\n%2f", ACSchip->readVRMS(0));
        display->display();
        return;
    }*/ else if (currPage == DISP_THD_V) {
        if (false) { //!voltage_detected
            display->printf("Voltage \ntoo low:\n%2f", ACSchip->readVRMS(1));
        } else {
            if (currPage == 0) {
                display->printf("RMS values");
                temp = ACSchip->readVRMS(1);
                if (temp < 1)
                    display->printf("Vrms:\n%.2f mV\n", temp * 1000.0F);
                else
                    display->printf("Vrms:\n%.2f V\n", temp);
                temp = ACSchip->readIRMS(1);
                if (temp < 1)
                    display->printf("Irms:\n%.2f mA\n", temp * 1000.0F);
                else
                    display->printf("Irms:\n%.2f A\n", temp);

                /*display->printf("Harmonics(V)\n");
                display->printf("THD:\n%.2f %%\n", thd_v);
                if (grouping_en) {
                    display->printf("THDG: \n%.2f%%\n", thdg_v);
                    display->printf("THDSG: \n%.2f%%\n", thdsg_v);
                }*/
            } else if (currPage == DISP_THD_I) {
                display->printf("Harmonics(I)\n");
                /*display->printf("THD:\n%.2f%%\n", thd_i);
                if (grouping_en) {
                    display->printf("THDG:\n%.2f%%\n", thdg_i);
                    display->printf("THDSG:\n%.2f%%\n", thdsg_i);
                }*/
            } else if (currPage == DISP_POWER) {
                /*
                display->printf("Act. Power \n%.2f W\n", ACSchip->readPACTIVE(1) * MAXPOWER);
                display->printf("P-Factor: \n%.2f\n", ACSchip->readPOWERFACTOR() * distortion_factor);
                display->printf("Phase Angle\n: %.2fdeg\n", phaseangle);*/
            }
        }
    }
    display->display();
}
void displayFFT(Adafruit_SSD1327* display, float* Mags, uint8_t currPage) {
    display->clearDisplay();
    display->setCursor(0, 0);
    if (currPage == DISP_FFT_VOLT)
        display->print("Voltage FFT 0-640 Hz");
    else
        display->print("Current-FFT 0-640 Hz");
    //Number of Magnitudes: 4096, with binSize = 5
    float maxMag = Mags[getMaxValueIndex(Mags, 4096)];
    float pix_per_volt = (128 - 20) / maxMag;
    for (int i = 0; i < 128; i++) {
        uint16_t mag = round(Mags[i] * pix_per_volt); //Rounded value
        display->drawLine(i, DISP_HEIGHT, i, DISP_WIDTH - 10 - mag, SSD1327_WHITE);
    }
    display->display();
    #ifdef DEBUG
    SerialUSB1.printf("Display update Time: %u\n", micros() - starttime);
    #endif
}

void displayDataLogMenu(Adafruit_SSD1327* display, uint8_t menuOption, const char* optionName[]) {
    display->clearDisplay();
    display->setCursor(0, 0);
    for (uint8_t i = 0; i < MENU_OPTIONS; i++) {
        if (menuOption == i)
            display->print(">");
        display->printf("%s\n", optionName[i]);
    }
    display->display();
}

uint32_t getMaxValueIndex(float* values, uint32_t arrlen) {
    uint32_t maxIndex = 0;
    float maxVal = values[maxIndex];

    for (uint32_t i = 0; i < arrlen; i++) {
        if (values[i] > maxVal) {
            maxIndex = i;
            maxVal = values[i];
        }
    }
    return maxIndex;
}