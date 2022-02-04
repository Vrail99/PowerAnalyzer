
#ifndef ConversionHelpers_H
#define ConversionHelpers_H
#include <Arduino.h>

//Conversion helpers

/*
 * Sign extend a bitfield which if right justified
 *
 *    data        - the bitfield to be sign extended
 *    width       - the width of the bitfield
 *    returns     - the sign extended bitfield
 */
int32_t SignExtendBitfield(uint32_t data, uint16_t width) {
    // If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
    if (width == 32)     {
        return (int32_t)data;
    }

    int32_t x = (int32_t)data;
    int32_t mask = 1L << (width - 1);

    x = x & ((1 << width) - 1); // make sure the upper bits are zero

    return (int32_t)((x ^ mask) - mask);
}

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

    if (width == 32)     {
        mask = 0xFFFFFFFF;
    }     else     {
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

uint32_t getMaxValueIndex(float *values, uint32_t arrlen) {
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

#endif