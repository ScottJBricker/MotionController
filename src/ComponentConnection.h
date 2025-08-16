#pragma once
#ifndef _COMPONENTCONNECTION_H_
#define _COMPONENTCONNECTION_H_

// === Header File Declaration ===
#include "BoardInfo.h"

#include <stdint.h>             // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>     // Arduino built in header file (<SerialCommands.h>)... (local file here)

#if defined(IS_ARDUINO)
  #include <TimerOne.h>
#endif


// State of a device, in general
class ComponentConnection {
private:
  uint8_t *correspondingMotors; 
  uint8_t numPins;
public:
  static uint8_t *isrPins;
  static bool *isrStartState;
  static uint8_t isrSIZE;
  static uint32_t interruptCount;
  static uint32_t currInterruptCount;

  static bool isInitial;
  static float errorPerStep;
  static float accumulatedTimingError;

  static uint8_t *resumePins;
  static uint8_t *resumeState;
  static uint8_t resumeSIZE;
    static uint32_t isrMax[2];         // 
  static uint32_t isrPeriodMicroseconds[2];
  static uint8_t currISRStage;
  static uint8_t isrPeriods;
  static uint32_t netCount;

  uint8_t *axis;                 // Axis relative to the Class of the holders of these 'CurrentState' obj's
  uint8_t *axisCS;              // CS relative to a specific axis of a class
  uint8_t* pinout;              // Points to a location in the Class data (contains numPins elements)
  uint8_t csIndex;
  uint8_t SIZE;
  
  ComponentConnection(void) {  // Nothing is known about the generic motor upon startup
    this->correspondingMotors = nullptr;
    this->numPins = 0;
    this->pinout = nullptr;
    this->axis = nullptr;
    this->axisCS = nullptr;
    this->csIndex = 0;
    this->SIZE = 0;
  }

  ~ComponentConnection(void) {
    delete[] this->correspondingMotors;
    delete[] this->axis;
    delete[] this->axisCS;
  }

  // Static function called by the timer to produce desired signal duty cycle profile
  static void timerISR(void) {
    #if defined(IS_ARDUINO)
  
      ++ComponentConnection::interruptCount;
      ++ComponentConnection::currInterruptCount;
      for (uint8_t pinIndex = 0; pinIndex < ComponentConnection::isrSIZE; ++pinIndex) {
        digitalWrite(ComponentConnection::isrPins[pinIndex], ComponentConnection::isrStartState[pinIndex]);
        ComponentConnection::isrStartState[pinIndex] = !ComponentConnection::isrStartState[pinIndex];
      }

      if (ComponentConnection::currISRStage >= ComponentConnection::isrPeriods || ComponentConnection::interruptCount >= ComponentConnection::netCount)
        Timer1.detachInterrupt(); // Disable the timer interrupt
      else if (ComponentConnection::currInterruptCount >= ComponentConnection::isrMax[ComponentConnection::currISRStage]) {
        ++ComponentConnection::currISRStage;        // Split ISR into two different periods to compensate for error accumulation
        ComponentConnection::currInterruptCount = 0;  
        if (ComponentConnection::currISRStage < ComponentConnection::isrPeriods)
          Timer1.setPeriod(ComponentConnection::isrPeriodMicroseconds[ComponentConnection::currISRStage]);
      }
      
    #endif


    /*
    old stuff that should likely be deleted...
    ComponentConnection::isrPeriodMicroseconds[0] = (uint32_t)onPulseMicroseconds;
        ComponentConnection::isrPeriodMicroseconds[1] = (uint32_t)(onPulseMicroseconds + 1);  // Round up

        for (uint8_t iter = 0; iter < SIZE; ++iter) {
          ComponentConnection::isrPins[iter] = pins[iter];
          ComponentConnection::isrStartState[iter] = startState[iter] != 0;
        }

        // Initialize timer period, Assume Duty Cycle 
        float percentageMissing = (clockDuration - (ComponentConnection::isrPeriodMicroseconds[0] + ComponentConnection::isrPeriodMicroseconds[1])) / 1; // These will need to round up to compensate for the error accumulation
        ComponentConnection::isrPeriods = 2;  // 2 unique periods (ceil and floor to compensate for error accumulation)
        ComponentConnection::isrMax[0] = numPulses * 2 * (1 - percentageMissing);
        ComponentConnection::isrMax[1] = numPulses * 2 * percentageMissing; 
    
        // Initialize Timer ISR Vars
        ComponentConnection::currISRStage = 0;  // Restart the ISR processing
        ComponentConnection::isInitial = true;  // alternate between states and pulse duration times
        ComponentConnection::interruptCount = 0;

    */  
    //ComponentConnection::isInitial = !ComponentConnection::isInitial;
    //ComponentConnection::isInitial ? Timer1.setPeriod(ComponentConnection::timerPeriodMicroSeconds[0]) : Timer1.setPeriod(ComponentConnection::timerPeriodMicroSeconds[1]);
  }


  void sendPulses(const uint8_t *pins, const uint8_t *startState, int8_t pulseDurationExponent, float pulseDuration, int8_t offDurationExponent, float offDuration, uint32_t numPulses, uint8_t SIZE);
  void specifyConnection(uint8_t *motors, uint8_t *axis, uint8_t *cs, uint8_t SIZE);
  void setCS(uint8_t cs);
  void begin(void);
  void selectPinout(uint8_t cs);
  void selectPinout(uint8_t correspondingMotor, uint8_t axis);
  void setPinout(const uint8_t* pinout, uint8_t numPins);
  uint8_t* getPinout(void) {  return this->pinout; }
  uint8_t getNumPins(void);
  uint8_t getCorrespondingMotor(void) { return this->correspondingMotors[this->csIndex];  }
  uint8_t getAxis(void) { return this->axis[this->csIndex]; }
  void displayPinout(void);
};

#endif