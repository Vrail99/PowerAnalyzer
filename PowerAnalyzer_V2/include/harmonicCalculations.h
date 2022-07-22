#ifndef HARMONIC_CALCULATIONS_H
#define HARMONIC_CALCULATIONS_H

#include <Arduino.h>

//Variables for harmonic distortion calculation
struct PowerQuality {
    float thd_v; //Total Harmonic Voltage Distortion
    float thd_i; //Total Harmonic Current Distortion
    float pwr_f; //Power Frequency
    float phaseangle;
    float distortion_factor; //Distortion factor for the pf-calculation

    bool grouping_en;
    float thdg_v;
    float thdsg_v;
    float thdg_i;
    float thdsg_i;
};

float calcTHD(float* Magnitudes, uint8_t order, uint32_t binSize);
float calcTHDG(float* frequencies, float* output, int order);
float calcTHDSG(float* frequencies, float* output, int order);

float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
int32_t SignExtendBitfield(uint32_t data, uint16_t width);
uint32_t getMaxValueIndex(float* values, uint32_t arrlen);

#endif