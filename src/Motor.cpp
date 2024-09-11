#include "Motor.h"

// General Default Motor Class

float Motor::getPositionRate(uint8_t derivative) {
  switch (derivative) {
  case 0:   // (Position) : Current State
    return this->getPosition();
  case 1:    // (Velocity) : Rate of change of position
  case 2:    // (Acceleration) : Rate of change of the velocity 
    return this->driver.physicalDynamics->getPositionRate(derivative);
  default:
    return -1;
  }
}

void Motor::setPositionRate(float desiredState, uint8_t derivative) {
  if (derivative == 0) {    // Position (0)
    this->setPosition(desiredState);
  }
  else if (derivative > 0)  {// Velocity (1), Acceleration (2)
    this->driver.physicalDynamics->setPositionRate((uint32_t)desiredState, derivative);
    // TODO: Make changes to the actual physical hardware. 'setPositionRate' only 
    // alters the parameter set desired state.
  }
}

// Works for any stepper motors (INVOKES THE BASE CLASS FCN OF PHYSICAL DYNAMICS)
uint32_t Motor_Stepper::computeOverallStep(float rotorAngle) {
  // Perform the multiplication by the angle last to prevent overflow errors for large angles.
  return (long)(((float)this->gearRatio * (float)this->driver.physicalDynamics->getTotalStates() / 360.0) * rotorAngle);
}

// Works for any stepper motors
float Motor_Stepper::getPosition(void) {
  return 360.0 * (this->relativeStep / (float)(this->gearRatio * this->driver.physicalDynamics->getTotalStates()));
}

#if NEMA17_SUPPORTED
void Motor_Nema17::initializeDevice(uint8_t SIZE) {

  for (uint8_t i = 0; i < SIZE; ++i) {
      // Initialize motor parameters
    this[i].driver.motor.selectPinout(*(this[i].axis));                         // Set real-time vars for each motor dynamics
    this[i].tmcDriver = new TMC2130Stepper(this[i].driver.motor.pinout[Enumerators::NEMA17Pins[Enumerators::CS]]);
    this[i].driver.motor.begin((uint8_t)Enumerators::NEMA17);                 // Define pinout for their respective connections (input/output etc.)
    this[i].stepper = AccelStepper(this[i].stepper.DRIVER, this[i].driver.motor.pinout[Enumerators::NEMA17Pins[Enumerators::PUL]], this[i].driver.motor.pinout[Enumerators::NEMA17Pins[Enumerators::DIR]]);

    this[i].tmcDriver->begin();                                          // Initiate pins and registeries
    this[i].tmcDriver->rms_current(this[i].driver.electricalDynamics.current);  // Set stepper current
    
    this[i].tmcDriver->stealthChop(1);
    this[i].tmcDriver->stealth_autoscale(1);                             // Enable quiet stepping (smoother)
    
    
    this[i].tmcDriver->microsteps(this[i].driver.physicalDynamics->getMicroSteps());  // Set cumulative step size
    digitalWrite(this[i].driver.motor.pinout[Enumerators::NEMA17Pins[Enumerators::CS]], LOW);   // Set CS pin to LOW

    this[i].stepper.setMaxSpeed(this[i].driver.physicalDynamics->getPositionRate(1));       // Set stepper speed
    this[i].stepper.setAcceleration(this[i].driver.physicalDynamics->getPositionRate(2));   // Set stepper acceleration
  }
}

void Motor_Nema17::originSearch(void) {  
  this->setPosition(0);   // No feedback is possible. Therefore, our only possible origin is the previously defined origin.
}

void Motor_Nema17::setPosition(float desiredPositionDegrees) {
  uint32_t finalRelativeStep = this->computeRelativeStep(desiredPositionDegrees);   // Compute the final stepper position from 0 to N steps (Output periodic with N steps)
  int32_t numSteps = this->computeNumSteps(finalRelativeStep);                      // Compute the number of steps to move in the direction of the polarity of the value.
  this->stepper.move((long)numSteps);                                           // Instruct motor driver of the desired steps to move.
  this->stepper.runToPosition();                                                // Instruct motor driver to move NEMA17 motor to the desired position.
  this->relativeStep = this->computeRelativeStep(desiredPositionDegrees);       // Store MotionControllerAPI position in order to maintain rotation from 0 to 360 degrees.
}

#endif

#if NEMA23_SUPPORTED

void Motor_Nema23::initializeDevice(uint8_t SIZE) {
  for (uint8_t currMotor = 0; currMotor < SIZE; ++currMotor) {
    this[currMotor].driver.motor.selectPinout(*(this[currMotor].axis));                         // Set real-time vars for each motor dynamics
    this[currMotor].driver.motor.begin((uint8_t)Enumerators::NEMA23);      // Define pinout for their respective connections (input/output etc.)
  }
}

void Motor_Nema23::originSearch(void) {  
  this->setPosition(0);   // No feedback is possible. Therefore, our only possible origin is the previously defined origin.
}

void Motor_Nema23::setPosition(float desiredState) {
  uint32_t finalRelativeStep = this->computeRelativeStep(desiredState);
  int32_t numSteps = this->computeNumSteps(finalRelativeStep);
  
  digitalWrite(this->driver.motor.pinout[Enumerators::NEMA23Pins[Enumerators::DIR]], numSteps > 0);  // Set the direction to step
  
  int32_t currStep = 0;
  numSteps = abs(numSteps);
  for (currStep = 0; currStep < numSteps; ++currStep) {
      digitalWrite(this->driver.motor.pinout[Enumerators::NEMA23Pins[Enumerators::PUL]], HIGH);
      delayMicroseconds(this->driver.electricalDynamics.micro_pulseDuration);
      digitalWrite(this->driver.motor.pinout[Enumerators::NEMA23Pins[Enumerators::PUL]], LOW);
      delayMicroseconds(this->driver.electricalDynamics.micro_pulseDuration);
  }
  this->relativeStep = finalRelativeStep;
}
#endif

#if G2_SUPPORTED

// Initialize G2 Chip State
bool Motor_G2::g2Direction[2] = { true, true };                               // Actuator direction of travel 
int32_t Motor_G2::g2Steps[2] = { 0, 0 };                                   // Actuator overall steps
volatile uint32_t Motor_G2::unaccountedSteps[2] = { 0, 0 };     // Steps accumulated in motor ISR awaiting transfer to overall steps var, 'g2Steps'. Use 'g2Direction' to determine direction   
    

void intializeAxis(uint8_t axis) {
  
}

void Motor_G2::initializeDevice(uint8_t SIZE) {
  for (uint8_t currMotor = 0; currMotor < SIZE; ++currMotor) {
    this[currMotor].driver.motor.selectPinout(*this[currMotor].axis);    // Sets the desired axis for use
    this[currMotor].setPositionRate(4, 1);                      //  Set 4 mm/s  (Derivative 1 : Velocity)

    // Initialize Motor Hardware
    switch (*(this[currMotor].axis)) {
      case 1:
        this[currMotor].g2Driver->calibrateM1CurrentOffset();
      break;
      case 2:
        this[currMotor].g2Driver->calibrateM2CurrentOffset();
      break;
    }
  }
  
  // Initialize Feedback Software
  for (uint8_t i = 0; i < SIZE; ++i) {
    //this[i].driver.motor.selectPinout(i + 1);               // Sets (i + 1) as the motor axis and assigns the corresponding pinout
    this[i].driver.motor.begin((uint8_t)Enumerators::G2);   // Begin the connection sequence for a G2 motor
    switch (*this[i].axis) {
      case 1:   // Axis 1: M1
        attachInterrupt(digitalPinToInterrupt(this[i].driver.motor.pinout[Enumerators::G2Pins[Enumerators::IN1]]), Motor_G2::M1_ISR, RISING); // Attach the in-phase pin to the handler_hall0 ISR
        attachInterrupt(digitalPinToInterrupt(this[i].driver.motor.pinout[Enumerators::G2Pins[Enumerators::IN2]]), Motor_G2::M1_ISR, RISING); // Attach the quadrature pin to the handler_hall1 ISR
      break;
      case 2:   // Axis 2: M2
        attachInterrupt(digitalPinToInterrupt(this[i].driver.motor.pinout[Enumerators::G2Pins[Enumerators::IN1]]), Motor_G2::M2_ISR, RISING); // Attach the in-phase pin to the handler_hall0 ISR
        attachInterrupt(digitalPinToInterrupt(this[i].driver.motor.pinout[Enumerators::G2Pins[Enumerators::IN2]]), Motor_G2::M2_ISR, RISING); // Attach the quadrature pin to the handler_hall1 ISR
      break;
    }
  }
}

void Motor_G2::originSearch(void) {                  
  uint8_t constCount = 0;                                   // Counter of consecutive loops where the pulse count from the Motor ISR remains constant
  int32_t prevSteps;

  float initialVelocity = this->getPositionRate(1);         // Velocity in mm/s
  this->setPositionRate(Motor_G2::originSearchSpeed, 1);    // Set velocity to origin search velocity
  float pwmVal = this->computePWMVal();
  this->setDirection(RETRACT);                                                              // Set direction of travel
  analogWrite(this->driver.motor.pinout[Enumerators::G2Pins[Enumerators::PWM]], pwmVal);    // Set motor speed (0 to 255)

  // Observe steps counter while motor is operating.
  // Stop the motor when the pulses stop incrementing
  while (constCount++ < 3) {
    prevSteps = this->pullSteps(false);   // Observe the motor ISR steps counter for changes
    delay(50);                            // allow time for changes to take effect
    
    if (abs(prevSteps - (int32_t)this->pullSteps(false)) > 2)
      constCount = 0;
  }
  analogWrite(this->driver.motor.pinout[Enumerators::G2Pins[Enumerators::PWM]], 0);   // Motor reached origin. Turn it off.
  this->redefineOrigin();
  this->setPositionRate(initialVelocity, 1);            // Set the initial velocity
  this->setPosition(Motor_G2::ORIGIN_OFFSET);           // Offset from hardware origin to prevent drift of position which occurs often near the limits
  this->redefineOrigin();
}

// Interrupt service routine for motor 1 hall sensors
void Motor_G2::M1_ISR(void) {
  Motor_G2::unaccountedSteps[0]++;  // Increment #pulses detected on motor 1
}

// Interrupt service routine for motor 2 hall sensors
void Motor_G2::M2_ISR(void) {
  Motor_G2::unaccountedSteps[1]++;  // Increment #pulses detected on motor 2
}

#endif