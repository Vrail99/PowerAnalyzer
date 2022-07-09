#ifndef HARMONIC_CALCULATIONS_H
#define HARMONIC_CALCULATIONS_H

#include <Arduino.h>

float calcTHD(float* Magnitudes, uint8_t order, uint32_t binSize);
float calcTHDG(float* frequencies, float* output, int order);
float calcTHDSG(float* frequencies, float* output, int order);

float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
int32_t SignExtendBitfield(uint32_t data, uint16_t width);

#endif