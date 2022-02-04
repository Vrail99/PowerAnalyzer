#ifndef HarmonicCalculations_H
#define HarmonicCalculations_H

#include <Arduino.h>
#include "ACS71020.h"
#include "PowerAnalyzer.h"

////////////////////////////////////////////////////////////////////////////////
// Calculation Functions
////////////////////////////////////////////////////////////////////////////////


float calcTHD(uint8_t order, float* Mags) {
    uint32_t s = 50 / binSize; //50Hz base frequency
    float thd = 0;
    float base = Mags[s];
    for (uint32_t i = s * 2; i < order * s; i += s) {
        thd += pow(Mags[i] / base, 2);
    }
    thd = sqrt(thd) * 100;

    return thd;
}

float calcTHDG(float *frequencies, float *output, float *Mags, int order) {
    float groupvalue = 0;
    for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
    {
        groupvalue = pow(frequencies[i * 10 - 5], 2) / 2; //Add 1/2*Value offset by 5 to the left
        float sumvalue = 0;
        for (int k = -4; k < 5; k++) //Add all values from -4 to 4 around the harmonic order
        {
            sumvalue += pow(frequencies[i * 10 + k], 2);
        }
        groupvalue += sumvalue + pow(Mags[i * 10 + 5], 2) / 2; //Add 1/2*Value offset by 5 to the right
        float result = sqrt(groupvalue);
        output[i] = result; //Save it in output array
    }
    //Calculate THDG
    float thdg = 0;
    float base = output[1];
    for (uint8_t i = 2; i <= order; i++) {
        thdg += pow(output[i] / base, 2);
    }
    thdg = sqrt(thdg) * 100;

    return thdg;
}

float calcTHDSG(float *frequencies, float* output, int order) {
    for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
    {
        float sumvalue = 0;
        for (int k = -1; k < 2; k++) //Add neighbor values
        {
            sumvalue += pow(frequencies[i * 10 + k], 2);
        }
        float result = sqrt(sumvalue);
        output[i] = result;
    }
    //Calculate THDSG
    float thdsg = 0;
    float base = output[1];
    for (uint8_t i = 2; i <= order; i++) {
        thdsg += pow(output[i] / base, 2);
    }
    thdsg = sqrt(thdsg) * 100;
    return thdsg;
}

#endif