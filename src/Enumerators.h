#pragma once
#ifndef _ENUMERATORS_H_
#define _ENUMERATORS_H_

#include "BoardInfo.h"
#include <stdint.h>                         // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>                 

class Enumerators {
  public:
    enum SupportedMotors { G2 = 0, NEMA17, NEMA23 };
    enum pinoutTasks { CS = 0, EN, DIR, PUL, PWM, SLEEP, FAULT, C_SENSE, IN1, IN2 };   // Chip-Select, Enable, Direction, Pulse, Pulse Width Modulation (PWM), Sleep, Fault, Current Sense, Feedback 1, Feedback 2
    static const uint8_t NUM_MOTOR_MODELS = 3;  // G2, NEMA17, NEMA23

    static bool validateMotor(const char *motorString);
    
    // NEMA17 Components
    #if (NEMA17_SUPPORTED)
    static const uint8_t nema17_numPins = 4;
    static const char nema17_description[];
    static const uint8_t NEMA17Pins[];
    static uint8_t nema17_m1Pinout[]; // The motor could be connected to axis 1
    static uint8_t nema17_m2Pinout[]; // The motor could be connected to axis 2
    static uint8_t nema17_m3Pinout[]; // The motor could be connected to axis 3
    static uint8_t *nema17_MotorPinouts[];
    #endif

    // Nema23 Components
    #if (NEMA23_SUPPORTED)
    static const uint8_t nema23_numPins = 2;
      static const char nema23_description[];
      static const uint8_t NEMA23Pins[];
      static uint8_t nema23_m1Pinout[]; // The motor could be connected to axis 1
      static uint8_t nema23_m2Pinout[]; // The motor could be connected to axis 2
      static uint8_t nema23_m3Pinout[]; // The motor could be connected to axis 3
      static uint8_t *nema23_MotorPinouts[];
    #endif

    // G2 Components
    #if (G2_SUPPORTED)
    static const uint8_t g2_numPins = 7;
      static const char g2_description[];
      static const uint8_t G2Pins[]; // The G2 Motor has 7 pins to use
      static uint8_t g2_m1Pinout[]; // The motor could be connected to axis 1
      static uint8_t g2_m2Pinout[]; // The motor could be connected to axis 2
      //static constexpr struct PORTDemulation M1_I = B00000100;  // M1_I is in-phase component of the Hall sensor (Digital Pin 2)
      //static constexpr struct PORTDemulation M1_Q = B00001000;  // M1_Q is quadrature component of the Hall sensor (Digital Pin 3)
      //static constexpr struct PORTDemulation M2_I = B00010000;  // M2_I is in-phase component of the Hall sensor (Digital Pin 4)
      //static constexpr struct PORTDemulation M2_Q = B00100000;  // M2_Q is in-quadrature component of the Hall sensor (Digital Pin 5)
      static uint8_t *g2_MotorPinouts[];
    #endif
};
#endif