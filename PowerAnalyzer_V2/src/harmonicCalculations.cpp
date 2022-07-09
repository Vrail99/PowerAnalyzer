#include "harmonicCalculations.h"


////////////////////////////////////////////////////////////////////////////////
// Calculation Functions
////////////////////////////////////////////////////////////////////////////////

float calcTHD(float* Magnitudes, uint8_t order, uint32_t binSize) {
    uint32_t s = 50 / binSize; //50Hz base frequency
    float thd = 0;
    float base = Magnitudes[s];
    for (uint32_t i = s * 2; i < order * s; i += s) {
        thd += powf(Magnitudes[i] / base, 2);
    }
    thd = sqrtf(thd) * 100.0;

    return thd;
}

float calcTHDG(float* frequencies, float* output, int order) {
    float groupvalue = 0;
    for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
    {
        groupvalue = pow(frequencies[i * 10 - 5], 2) / 2; //Add 1/2*Value offset by 5 to the left
        float sumvalue = 0;
        for (int k = -4; k < 5; k++) //Add all values from -4 to 4 around the harmonic order
        {
            sumvalue += powf(frequencies[i * 10 + k], 2);
        }
        groupvalue += sumvalue + powf(frequencies[i * 10 + 5], 2) / 2; //Add 1/2*Value offset by 5 to the right
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

float calcTHDSG(float* frequencies, float* output, int order) {
    for (uint8_t i = 1; i <= order; i++) //Loop over Harmonic Orders
    {
        float sumvalue = 0;
        for (int k = -1; k < 2; k++) //Add neighbor values
        {
            sumvalue += powf(frequencies[i * 10 + k], 2);
        }
        float result = sqrtf(sumvalue);
        output[i] = result;
    }
    //Calculate THDSG
    float thdsg = 0;
    float base = output[1];
    for (uint8_t i = 2; i <= order; i++) {
        thdsg += powf(output[i] / base, 2);
    }
    thdsg = sqrtf(thdsg) * 100.0;
    return thdsg;
}

//Conversion helpers
/*
 * Convert an unsigned bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width) {
    uint32_t mask;

    if (width == 32) {
        mask = 0xFFFFFFFF;
    } else {
        mask = (1UL << width) - 1UL;
    }

    return (float)(inputValue & mask) / (float)(1L << binaryPoint);
}

/*
 * Convert a signed bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be sign extended then converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width) {
    int32_t signedValue = SignExtendBitfield(inputValue, width);
    return (float)signedValue / (float)(1L << binaryPoint);
}

/*
 * Sign extend a bitfield which if right justified
 *
 *    data        - the bitfield to be sign extended
 *    width       - the width of the bitfield
 *    returns     - the sign extended bitfield
 */
int32_t SignExtendBitfield(uint32_t data, uint16_t width) {
    // If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
    if (width == 32) {
        return (int32_t)data;
    }

    int32_t x = (int32_t)data;
    int32_t mask = 1L << (width - 1);

    x = x & ((1 << width) - 1); // make sure the upper bits are zero

    return (int32_t)((x ^ mask) - mask);
}
