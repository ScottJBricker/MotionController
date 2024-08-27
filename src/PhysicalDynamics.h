#pragma once
#ifndef _PHYSICALDYNAMICS_H_
#define _PHYSICALDYNAMICS_H_

#include "Enumerators.h"
#include <stdint.h>  // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h> 

// Base 'PhysicalDynamics' Class for generic polymorphism
class PhysicalDynamics {
  public:
    float resolution;                                      // Resolution of motor to move a finite distance
    volatile uint32_t speed;                                        // Microsteps per second


    virtual ~PhysicalDynamics(void) {}

    // Pure Virtual Functions for Derived Classes
    virtual void setPositionRate(uint32_t rate, uint8_t derivative = 0) = 0;
    virtual uint32_t getPositionRate(uint8_t derivative = 0) = 0;
    virtual uint32_t getTotalStates(void) = 0;
    virtual uint16_t getMicroSteps(void) = 0;
    virtual void setMicroSteps(uint16_t microsteps) = 0;
};

// Physical Dynamics for stepper motors. 
// Supported motors are : NEMA17, NEMA23
class PhysicalDynamics_Stepper : public PhysicalDynamics {
  private: 
    void setDynamics(uint16_t stepsPerRev, uint16_t microsteps, uint32_t speed, uint32_t acceleration);
  public:
    // Additional members of Generic NEMA stepper motor
    
    volatile uint32_t acceleration;                                 // Microsteps per second ^2
    uint16_t stepsPerRev;                                           // # of steps per revolution without micro-steps
    uint16_t microsteps;                                            // # of driver steps per Full-Step of stepper motor
    uint8_t correspondingMotor;

    // Constructor function of Generic NEMA stepper motor
    PhysicalDynamics_Stepper(uint8_t correspondingMotor = 0) {
      this->selectStepperMotor(correspondingMotor);
    }

    // Destructor function of Generic NEMA stepper motor
    virtual ~PhysicalDynamics_Stepper(void) { }

    void setPositionRate(uint32_t rate, uint8_t derivative) override {
      switch (derivative) {
        case 1:     // (Velocity)
          this->speed = rate;
        break;
        case 2:     // (Acceleration)
          this->acceleration = rate;
        break;
      }
    }

    uint32_t getPositionRate(uint8_t derivative) override {
      switch (derivative) {
        case 1:     // (Velocity)
          return this->speed;
        break;
        case 2:     // (Acceleration)
          return this->acceleration;
        break;
        default:
          return 0;
      }
    }

    // Class utilities
    void selectStepperMotor(uint8_t correspondingMotor);

    // Class variable getters/setters
    uint32_t getTotalStates(void) override;
    uint16_t getMicroSteps(void) override {  return this->microsteps;  }
    void setMicroSteps(uint16_t microsteps) override;
};

#if (G2_SUPPORTED)
class PhysicalDynamics_G2 : public PhysicalDynamics {
  public:
    PhysicalDynamics_G2(uint8_t correspondingMotor = 0) {
      this->selectMotor(correspondingMotor);
    }

  // Class utilities
  void selectMotor(uint8_t correspondingMotor);
  void selectAxis(uint8_t correspondingAxis);

  // Override Pure Virtual Functions
  ~PhysicalDynamics_G2(void) override {

  }

  void setPositionRate(uint32_t rate, uint8_t derivative = 0) override {
    switch (derivative) {
      case 1:     // (Velocity)
        this->speed = rate;
      break;
    }
  }

  uint32_t getPositionRate(uint8_t derivative = 0) override {
    switch (derivative) {
      case 1:     // (Velocity)
        return this->speed;   // In mm/s
      break;
      default:
        return 0;
    }
  }
  uint32_t getTotalStates(void) override {
    // TODO: DEFINE ME
    return 0;
  }

  uint16_t getMicroSteps(void) override {
    // TODO: DEFINE ME
    return 0;
  }
  void setMicroSteps(uint16_t microsteps) override {

  }

};
#endif
#endif