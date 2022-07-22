#include "OLED_Display.h"

void displayDefault(Adafruit_SSD1327* display, ACS71020* ACSchip, uint8_t currPage, PowerQuality* pwr_qual) {
    float temp;
    display->clearDisplay();
    display->setCursor(0, 0); //Cursor Position top-left
    //Updates to the display
    switch (currPage) {
    case DISP_CURRENT: {
        display->printf("Current\n");
        temp = ACSchip->readIRMS(1);
        if (temp < 1.0F)
            display->printf("Irms:\n%.2f mA\n", temp * 1000.0F);
        else
            display->printf("Irms:\n%.2f A\n", temp);
        break;
    } case DISP_VOLT: {
        display->printf("Voltage\n");
        temp = ACSchip->readVRMS(0);
        if (temp < 1.0F)
            display->printf("Vrms:\n%.2f mV\n", temp * 1000.0F);
        else
            display->printf("Vrms:\n%.2f V\n", temp);
        temp = ACSchip->readPACTIVE(1) * 5490.0;
        display->printf("Power\n%.2f W\n", temp);
        break;}
    case DISP_THD_V: {
        if (false) { //!voltage_detected
            display->printf("Voltage \ntoo low:\n%2f", ACSchip->readVRMS(1));
        }
        display->printf("Harm.(V)\n");
        display->printf("THD:\n%.2f%%\n", pwr_qual->thd_v);
        if (pwr_qual->grouping_en) {
            display->printf("THDG:\n%.2f%%\n", pwr_qual->thdg_v);
            display->printf("THDSG:\n%.2f%%\n", pwr_qual->thdsg_v);
        }
        break;}
    case DISP_THD_I: {
        display->printf("Harm.(I)\n");
        display->printf("THD:\n%.2f%%\n", pwr_qual->thd_i);
        if (pwr_qual->grouping_en) {
            display->printf("THDG:\n%.2f%%\n", pwr_qual->thdg_i);
            display->printf("THDSG:\n%.2f%%\n", pwr_qual->thdsg_i);
        } break;
    } case DISP_POWER: {
        display->printf("Act. Power \n%.2f W\n", ACSchip->readPACTIVE(1) * MAXPOWER);
        display->printf("P-Factor: \n%.2f\n", ACSchip->readPOWERFACTOR() * pwr_qual->distortion_factor);
        display->printf("Phase\n: %.2fdeg\n", pwr_qual->phaseangle);
        break;}
    }
    display->display();
}
/*
Displays the FFT with Bars on the display
*/
void displayFFT(Adafruit_SSD1327 * display, float* Mags, uint8_t currPage) {
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
}

void displayDataLogMenu(Adafruit_SSD1327 * display, uint8_t menuOption, const char* optionName[]) {
    display->clearDisplay();
    display->setCursor(0, 0);
    for (uint8_t i = 0; i < MENU_OPTIONS; i++) {
        if (menuOption == i)
            display->print(">");
        display->printf("%s\n", optionName[i]);
    }
    display->display();
}

void print_time_in_min_sec(Adafruit_SSD1327 * display, int16_t x, int16_t y, uint32_t ms) {
    if (ms < 60000) { //60s remaining
        int secs = (int)(ms / 1000) + 1;
        display->clearDisplay();
        display->setCursor(x, y);
        display->printf("Calibration Mode\n");
        display->printf("%02d s remaining", secs);
        display->display();
    } else {
        int mins = (int)(ms / 60000);
        display->clearDisplay();
        display->setCursor(x, y);
        display->printf("Calibration Mode\n");
        display->printf("%02d min remaining", mins);
        display->display();
    }
}