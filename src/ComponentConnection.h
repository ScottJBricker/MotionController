#pragma once
#ifndef _COMPONENTCONNECTION_H_
#define _COMPONENTCONNECTION_H_

// === Header File Declaration ===
#include "Enumerators.h"

#include <stdint.h>             // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>     // Arduino built in header file (<SerialCommands.h>)... (local file here)

// State of a device, in general
class ComponentConnection {
private:
  const uint8_t correspondingMotor; 
  uint8_t numPins;
public:
  uint8_t axis;                 // Axis relative to the Class of the holders of these 'CurrentState' obj's
  uint8_t* pinout;              // Points to a location in the Class data (contains numPins elements)

  ComponentConnection(uint8_t correspondingMotor, uint8_t axis = 1) : correspondingMotor(correspondingMotor) {  // Nothing is known about the generic motor upon startup
    this->numPins = 0;
    this->pinout = NULL;
    this->axis = 0;
  }

  void begin(uint8_t motor);
  void selectPinout(uint8_t axis);
  void setPinout(const uint8_t* pinout, uint8_t numPins);
  uint8_t getNumPins(void);
  uint8_t *getAxisPtr(void) { return &this->axis; }
  uint8_t getCorrespondingMotor(void) { return this->correspondingMotor;  }
  uint8_t getAxis(void) { return this->axis; }
};

#endif