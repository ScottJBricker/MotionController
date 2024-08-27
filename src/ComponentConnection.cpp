#include "ComponentConnection.h"

uint8_t ComponentConnection::getNumPins(void) {
  return this->numPins;
}

void ComponentConnection::begin(uint8_t motor) {
  switch (motor) {
    #if (NEMA17_SUPPORTED)
      case Enumerators::NEMA17 :
        pinMode(this->pinout[Enumerators::NEMA17Pins[Enumerators::EN]], OUTPUT);      // Set EN pin to Output
        pinMode(this->pinout[Enumerators::NEMA17Pins[Enumerators::CS]], OUTPUT);      // Set CS pin to Output
        digitalWrite(this->pinout[Enumerators::NEMA17Pins[Enumerators::EN]], LOW);    // Set EN pin to LOW
        digitalWrite(this->pinout[Enumerators::NEMA17Pins[Enumerators::CS]], HIGH);   // Set CS pin to HIGH
        break;
    #endif
    #if (NEMA23_SUPPORTED)
      case Enumerators::NEMA23:
        pinMode(this->pinout[Enumerators::NEMA23Pins[Enumerators::PUL]], OUTPUT);    // Set PULSE pin to OUTPUT
        pinMode(this->pinout[Enumerators::NEMA23Pins[Enumerators::DIR]], OUTPUT);    // Set DIRECTION pin to OUTPUT
        break;
    #endif
    #if (G2_SUPPORTED)
      case Enumerators::G2 :
      
        pinMode(this->pinout[Enumerators::G2Pins[Enumerators::IN1]], INPUT);
        pinMode(this->pinout[Enumerators::G2Pins[Enumerators::IN2]], INPUT);
        break;
    #endif
  }
}

void ComponentConnection::selectPinout(uint8_t axis) {
  if (axis < 1 || axis > 2)
    return;
  uint8_t numPins;
  uint8_t **tempPinouts; // Each chip has up to 3 unique pinouts pre-defined
  switch (this->correspondingMotor) {
    #if (NEMA17_SUPPORTED)
      case Enumerators::NEMA17 :
        numPins = Enumerators::nema17_numPins;
        tempPinouts = Enumerators::nema17_MotorPinouts;
        break;
    #endif
    #if (NEMA23_SUPPORTED)
      case Enumerators::NEMA23:
      numPins = Enumerators::nema23_numPins;
        tempPinouts = Enumerators::nema23_MotorPinouts;
        break;
    #endif
    #if (G2_SUPPORTED)
      case Enumerators::G2 :
        numPins = Enumerators::g2_numPins;
        tempPinouts = Enumerators::g2_MotorPinouts;
        break;
    #endif
    default:
      break;
  }
  this->axis = axis;
  this->numPins = numPins;
  this->pinout = tempPinouts[this->axis - 1];
}