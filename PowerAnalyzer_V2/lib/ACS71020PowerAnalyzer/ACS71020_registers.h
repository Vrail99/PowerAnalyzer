#ifndef ACS71020_registers_H
#define ACS71020_registers_H

#include "Arduino.h"

//Easier access to Register names
enum ACS_registers {
    ICODE_REG = 0x2A,
    VCODE_REG = 0x2B,
};

//Easier access to Register masks
enum ACS_register_masks {
    ICODE_MASK = 0x0001FFFF,
    VCODE_MASK = 0x0001FFFF,
};

typedef union {
    struct settings
    {
        uint32_t ECC : 6;
        uint32_t skip : 4;
        uint32_t iavgselen : 1;
        uint32_t crs_sns : 3;
        uint32_t sns_fine : 9;
        uint32_t qvo_fine : 9;
    };
    uint32_t complete;
}regB;

#endif