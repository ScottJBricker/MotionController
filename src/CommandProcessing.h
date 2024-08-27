#pragma once
#ifndef _COMMANDPROCESSING_H_
#define _COMMANDPROCESSING_H_

// === Header File Declaration ===

#include "BoardInfo.h"
#include "ProgramMemoryStrings.h"
#include "Enumerators.h"

#include <stdint.h>                     // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>             // Arduino built in header file (<SerialCommands.h>)... (local file here)

extern char *__brkval;
extern char __heap_start;
extern char *__malloc_heap_start;
extern size_t __malloc_margin;
extern char *__data_start;
extern char *__bss_start;
extern char *__bss_end;
extern char *__brkval;

class CommandProcessing {
  
private:
    static unsigned long timeoutMS;       // Communication timeout in milliseconds
    uint8_t currSequence;                                           // (1 byte)WARNING : Encoded Variable, currSequence = [x7x6x5x4 x3x2x1x0] where x7 indicates if it is a special sequence or not. (x& == TRUE -> special sequence)
    char myParameters[MAX_PARAMETERS][MAX_PARAMETER_LENGTH + 1];    // (4 * 21 bytes)  Assuming a max parameter length of MAX_PARAMETER_LENGTH characters + null terminator
    uint8_t numParameters = 0;                                      // (1 byte)        Number of parameters parsed

    void clearParameters(void) {
        // Initialize parameters to empty strings
        for (uint8_t iter = 0; iter < MAX_PARAMETERS; ++iter)
            myParameters[iter][0] = '\0';
        this->numParameters = 0;
    }

    void parseCMD(const char* cmd);             // Used by 'processSerialPort' to transfer data contents to their corresponding parameter indexes

public:
    CommandProcessing() : numParameters(0) {
        this->clearParameters();    // initialize parameters to empty strings
    }

    static unsigned long getTimeoutMS() {
      return CommandProcessing::timeoutMS;
    }

    static void setTimeout(double durationSeconds) {
      CommandProcessing::timeoutMS = abs(durationSeconds * 1000);
    }

    bool processSerialPort(void);               // Process the data pending on the serial port and place 
    bool inputValidation(void);                 // Check if input is valid and assigns the encoded sequence associated with the current cmd

    uint8_t getNumParameters(void) {  return this->numParameters; }
    bool validateParameters(bool valueIsString = false);    // Performs input validation on the members of 'this' and returns non-zero if the command sequence was found to be programmed. The encoded sequence is stored to 'this' if the command was valid.
    uint8_t getSpecialParameterSequence(void);              // Returns the sequence # of the special sequences that the current cmd is associated with    (non-encoded)
    uint8_t getParameterSequence(void);                     // Returns the sequence # of the normal sequences that the current cmd is associated with     (non-encoded)
    const char* getParameter(uint8_t parameter);            // Returns the pointer that contains the specified parameter (1-based) from 'this'
    void displayCMD(void);                                  // Prints the full cmd associated with 'this'
    uint8_t getCurrSequence(void) { return this->currSequence; }  // returns the encoded sequence #
    uint8_t computeCorrespondingMotor(uint8_t parameter);
    void processParameters(const char* srcParameter);

    // Function used to compute remaining free memory during run-time
    static int freeMemory(void) {
      char top;

      #if MICRO_CONTROLLER == TEENSY
        // PaulStoffregen Code Snipped Used
        extern unsigned long _heap_end;
        extern char *__brkval;
        return (char *)&_heap_end - __brkval;
      #elif __arm__
        return &top - reinterpret_cast<char*>(sbrk(0));
      #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
        return &top - __brkval;
      #else
        return __brkval ? &top - __brkval : &top - __malloc_heap_start;
      #endif
    }
};
#endif