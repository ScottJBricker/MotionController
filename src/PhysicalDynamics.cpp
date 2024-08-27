#include "PhysicalDynamics.h"

void PhysicalDynamics_Stepper::setDynamics(uint16_t stepsPerRev, uint16_t microsteps, uint32_t speed, uint32_t acceleration) {
  this->speed = speed;
  this->acceleration = acceleration;
  this->stepsPerRev = stepsPerRev;
  this->microsteps = microsteps;
}

void PhysicalDynamics_Stepper::setMicroSteps(uint16_t microsteps) {
  this->microsteps = microsteps;
}

uint32_t PhysicalDynamics_Stepper::getTotalStates(void) {
  return this->stepsPerRev * this->microsteps;  // Total states of the stepper motor for each complete revolution.
}

void PhysicalDynamics_Stepper::selectStepperMotor(uint8_t correspondingMotor) {
  uint16_t stepsPerRev;     // Stepper Motor resolution [Steps/rev] (typically 200)
  uint16_t microsteps;      // Driver step size [Microsteps/full step]
  uint32_t speed;           // Stepper speed [Microsteps per second] (4k max for ATmega328 @16MHz)
  uint32_t acceleration;    // Stepper Acceleration [Microsteps per second^2]

  this->correspondingMotor = correspondingMotor;
  switch (correspondingMotor) {
    #if (NEMA17_SUPPORTED)
      case Enumerators::NEMA17:
        stepsPerRev = 200;    // Stepper Motor resolution [Steps/rev] (typically 200)
        microsteps = 128;     // Driver step size [Microsteps/full step]
        speed = 4000;         // Stepper speed [Microsteps per second] (4k max for ATmega328 @16MHz)
        acceleration = 40000; // Stepper Acceleration [Microsteps per second^2]
      break;
    #endif
    #if (NEMA23_SUPPORTED)
      case Enumerators::NEMA23:
        stepsPerRev = 200;    // Stepper Motor resolution [Steps/rev] (typically 200)
        microsteps = 125;     // Driver step size [Microsteps/full step]
        speed = 0;           // Stepper speed [Microsteps per second] (unknown quantity - managed by a 3rd party stepper motor driver)
        acceleration = 0;    // Stepper Acceleration [Microsteps per second^2] (unknown quantity - managed by a 3rd party stepper motor driver)
      break;
    #endif
    default:
      stepsPerRev = 0 - 1;  // OVERFLOW (forces maximum value, closest to inf)
      microsteps = 0 - 1;   // OVERFLOW (forces maximum value, closest to inf)
      speed = 0;
      acceleration = 0;
  }
  this->setDynamics(stepsPerRev, microsteps, speed, acceleration);
}

#if (G2_SUPPORTED)
void PhysicalDynamics_G2::selectMotor(uint8_t correspondingMotor) {
  this->resolution = 1.08;    // 1.08 um resolution;
}

void PhysicalDynamics_G2::selectAxis(uint8_t correspondingAxis) {
  switch (correspondingAxis -1) {
    case 0: // x-axis
      this->speed = 4;          // [1 to 5] recommended (mm/s) (up to 7.5 mm/s)
    break;
    case 1: // y-axis
      this->speed = 4;       // [1 to 5] recommended (up to 7.5 mm/s)
    break;
    default:

    break;
  }
}

#endif
