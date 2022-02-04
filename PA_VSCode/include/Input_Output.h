#ifndef Input_Output_H
#define Input_Output_H

#include <Arduino.h>
#include <PowerAnalyzer_Config.h>

#define MAX_RECEIVEABLE 32 // Number of characters that can be read

struct PA_command {
    uint32_t address;
    int32_t value; //Can be signed
    uint32_t mask;
    uint8_t pos;
    char commandBuffer[MAX_RECEIVEABLE];
} c;

struct PA_command *command = &c;

/*
  Receives a command String with chosen delimiters. Can read up to MAX_RECEIVABLE chars
*/
void receiveCommandString(usb_serial_class *ser, char* receivedChars) {
    char stdel = '<';              //Start delimiter
    char enddel = '>';             //End delimiter
    bool receiving = false;        //Receiving flag
    uint8_t cntr = 0;              //Char counter
    while (ser->available() > 0) //Receive while characters are on the Serial Bus / Buffer
    {
        char rc = ser->read();

        if (receiving) //If receiving of values has started
        {
            if (rc != enddel) {
                receivedChars[cntr] = rc;
                cntr++;
                if (cntr > MAX_RECEIVEABLE)
                    cntr = MAX_RECEIVEABLE - 1;
            } else //If end delimiter was sent
            {
                receivedChars[cntr] = '\0'; //String end delimiter
                receiving = false;          //Stop receiving and
                cntr = 0;                   //Reset counter
            }
        } else {
            if (rc == stdel) //Starting delimiter has been sent
                receiving = true;
        }
    }
}

#endif