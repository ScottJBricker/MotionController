#pragma once
#ifndef _COMMANDPROCESSING_H_
#define _COMMANDPROCESSING_H_

// === Header File Declaration ===
#include "BoardInfo.h"
#include "ProgramMemoryStrings.h"

#include <stdint.h>                     // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>             // Arduino built in header file (<SerialCommands.h>)... (local file here)


#define MAX_PARAMETERS 8                // Store up to 4 parameters. Defined at compile time to minimize dynamic memory usage.
#define MAX_PARAMETER_LENGTH 20
#define INPUT_BUFFER_SIZE 64

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
    static uint8_t currSequence;                                  // (1 byte)   WARNING : Encoded Variable, currSequence = [x7x6x5x4 x3x2x1x0] where x7 indicates if it is a special sequence or not. (x& == TRUE -> special sequence)
    static uint16_t timeoutMS;                                    // (2 bytes)  Communication timeout in milliseconds (up to 65 seconds)
    
    static char inputBuffer[INPUT_BUFFER_SIZE];                   // (64 bytes) Assuming a max buffer length of 63 characters + null terminator
    static uint8_t numParameters;                                 // (1 byte)   Number of parameters parsed
    static uint8_t numLen;                                        // (1 byte) Number of valid chars in 'inputBuffer'.
    static uint8_t indexOffset[MAX_PARAMETERS];                   // (4 bytes)  Index for each parameter of the input buffer, indexing into 'inputBuffer'

    void clearParameters(void) {
        // Initialize parameters to empty strings
        for (uint8_t iter = 0; iter < MAX_PARAMETERS; ++iter)
          this->indexOffset[iter] = 0;
        this->inputBuffer[0] = '\0';
        this->numParameters = 0;
    }

public:
    static uint8_t getParameterLen(uint8_t parameter) {
      if (CommandProcessing::numParameters < 1 || parameter < 1 || parameter > CommandProcessing::numParameters)
        return 0;
      else if (parameter == CommandProcessing::numParameters)
        return CommandProcessing::numLen - CommandProcessing::indexOffset[CommandProcessing::numParameters - 1];
      else
        return CommandProcessing::indexOffset[parameter] - CommandProcessing::indexOffset[parameter - 1] - 1; // Remove the '/0' from the count
    }

    static bool cmdValid;
    static uint8_t axis;
    static int8_t cs;
    static double value;

    CommandProcessing() {
        this->clearParameters();    // initialize parameters to empty strings
    }

    static unsigned long getTimeoutMS() {
      return CommandProcessing::timeoutMS;
    }

    static void setTimeout(double durationSeconds) {
      CommandProcessing::timeoutMS = abs(durationSeconds * 1000);
    }
    static bool processSerialPort(void);               // Process the data pending on the serial port and place 
    static bool processString(const char* inputBuffer, const char *delimiter = " ");
    static bool inputValidation(void);                 // Check if input is valid and assigns the encoded sequence associated with the current cmd
    static void parseCMD(const char* cmd, const char *delimiter = " ");             // Used by 'processSerialPort' to transfer data contents to their corresponding parameter indexes
    static void processCMD(uint8_t prompt = 2);                      // Processes the static vars AFTER parseCMD() has duplicated and parsed the input buffer.
    
    static uint8_t getCMDLength(void) { return CommandProcessing::numLen; }
    static uint8_t getNumParameters(void) {  return CommandProcessing::numParameters; }
    static bool validateParameters(bool valueIsString = false);    // Performs input validation on the members of 'this' and returns non-zero if the command sequence was found to be programmed. The encoded sequence is stored to 'this' if the command was valid.
    static uint8_t getSpecialParameterSequence(void);              // Returns the sequence # of the special sequences that the current cmd is associated with    (non-encoded)
    static uint8_t getParameterSequence(void);                     // Returns the sequence # of the normal sequences that the current cmd is associated with     (non-encoded)
    static const char* getParameter(uint8_t parameter);            // Returns the pointer that contains the specified parameter (1-based) from 'this'
    static void displayCMD(void);                                  // Prints the full cmd associated with 'this'
    static uint8_t getCurrSequence(void) { return CommandProcessing::currSequence; }  // returns the encoded sequence #
    static uint8_t* getStrLen(void) { return CommandProcessing::indexOffset; }
    // Function used to compute remaining free memory during run-time
    static int freeMemory(void) {
      

      #if MICRO_CONTROLLER == MC_TEENSY
        // PaulStoffregen Code Snipped Used
        extern unsigned long _heap_end;
        extern char *__brkval;
        return (char *)&_heap_end - __brkval;
      #elif __arm__
        char top;
        return &top - reinterpret_cast<char*>(sbrk(0));
      #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
        char top;
        return &top - __brkval;
      #else
        char top;
        return __brkval ? &top - __brkval : &top - __malloc_heap_start;
      #endif
    }
};

/*
class Task {
  static:
    uint16_t computeCommandDesignation(uint16_t encodedTask) {
      if (encodedTask == 9 || encodedTask == 12 || encodedTask == 13 || encodedTask == 16)
        return 10;      // Timed control commands (Set rotation, set distance, ...)
      if (encodedTask >= 1 && encodedTask <= 8 || encodedTask == 10 || encodedTask == 11 || encodedTask == 14 || encodedTask == 15 || encodedTask == 17 || )
10, 11, 14, 15, 17, 18, 19, 23, 24, 25, 27
Microcontroller Programming : 20, 21, 22, 26,28,29
    }

private:
    uint16_t task;        // Keep track of order of tasks receieved.
    uint16_t commandDesignation;  // "Task" command designation used for internal processes
    uint16_t encodedTask;         // Task to execute
    uint8_t axis;                 // Axis corresponding to task command
    double parameterValue;
    char parameterString[30];     // Parameter string associated with the command


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
*/
#endif