#pragma once
#ifndef _MOTOR_H_
#define _MOTOR_H_

// ############   Scott Header Files  ##############
#include "BoardInfo.h"
#include "PhysicalDynamics.h"
#include "ComponentConnection.h"

// ###########   Built-in Header Files   #############
#include <stdint.h>                          // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <math.h>
#include <SerialCommands.h>                 // Arduino built in header file (<SerialCommands.h>)... (local file here)

// ##########   3rd Party Header Files  ############
// Arduino : Position Control
#if (G2_SUPPORTED)
  #include "DualG2HighPowerMotorShield.h"     // Motor Header for 2D Position Control (Hall Sensor will use the Encoder.h)
#endif

// Arduino : Rotational Control
#if (NEMA17_SUPPORTED)
  #include <TMC2130Stepper.h>                 // Needed for Arduino Leonardo for laser attenuation
  #include <AccelStepper.h>                   // Needed for Arduino Leonardo for laser attenuation
#endif
// #################################################

#define MOTOR_DEBUGGER (false && DEBUGGER)  // TURN OFF THE DEBUGGER HERE!! (show debugger stuff here, so long as the debugger is on)

struct ElectricalDynamics {
    uint32_t micro_pulseDuration;                                               // Pulse duration in microseconds
    float current;                                                             // Stepper current in mA

    ElectricalDynamics(void) {
        this->micro_pulseDuration = 0;
        this->current = 0;
    }
};

// Abstract motor driver class
struct MotorDriver {
  public : 

  struct PhysicalDynamics *physicalDynamics;    // Generic array of Physical Dynamic Instances
  struct ElectricalDynamics electricalDynamics;
  struct ComponentConnection motor;
  
  MotorDriver(uint8_t motorModel, uint8_t axis) : motor(motorModel, axis) {
    switch (motorModel) {   // There is at least one motor of the current motor class
      #if NEMA17_SUPPORTED
        case Enumerators::NEMA17 :
      #endif
      #if NEMA23_SUPPORTED
        case Enumerators::NEMA23 :
      #endif
        this->physicalDynamics = new struct PhysicalDynamics_Stepper(motorModel);
        break;
      #if G2_SUPPORTED
        case Enumerators::G2 :
          this->physicalDynamics = new struct PhysicalDynamics_G2(motorModel);
        break;
      #endif
      default:
        this->physicalDynamics = nullptr;
        break;
    }
  }

  virtual ~MotorDriver() {
    if (this->physicalDynamics)
      delete this->physicalDynamics;  // release dynamic memory
  }
};

class Motor {
private:
  
public: 
  const uint8_t *axis;  // Pointer to the axis value stored in the 'ComponentConnection' instance

  //volatile long currStep;     // State relative to the startup position
  volatile bool motorAtOrigin;                // tells if motor is at origin (needed due to discrete limitations)
  volatile bool moving;         // is the actuator moving?
  
  struct MotorDriver driver;
  
  Motor(uint8_t motorModel = 0) : driver(motorModel, 1) {   // WARNING : Default axis is (axis = 1)
    this->axis = this->driver.motor.getAxisPtr();           // Enables fast monitoring of the axis for a unique motor instance
    this->motorAtOrigin = true;
    this->moving = false;
  }

  void statusPrintout(void) {
    if (!Serial)
      Serial.begin(LEONARDO_BAUD);
    while (!Serial)
      delay(200);                                                             // Allow time for Serial port to initialize, otherwise upcoming commands involving the Serial port may fail
    delay(50);
    Serial.print("Corresponding model : ");           Serial.print(this->driver.motor.getCorrespondingMotor());           Serial.print("\r\n"); // Terminate the line
    Serial.print("Generic Dynamics initialized : ");  Serial.print((uint8_t)(this->driver.physicalDynamics != nullptr));  Serial.print("\r\n"); // Terminate the line
  }

  float getPositionRate(uint8_t derivative = 0);                      // Function for setting position rate        { 0|Position, 1|Velocity, 2|Acceleration }
  void setPositionRate(float desiredState, uint8_t derivative = 0);   // Pure virtual function for retrieving the positing rate { 0|Position, 1|Velocity, 2|Acceleration }

  // Virtual Functions which allow for derived instances to refer to their most recently defined function for each virtual function
  virtual ~Motor() {                                                          // Virtual destructor for cleanup
    
  }               

  virtual void selectAxis(uint8_t axis) {
    this->driver.motor.selectPinout(axis);
  }

  // Pure Virtual Functions (MUST be overwritten by derived classes)
  virtual void redefineOrigin(void) = 0;                                      // Pure virtual function for resetting the coordinate system such that new values are relative to the current position
  virtual bool atOrigin(void) = 0;
  virtual void initializeDevice(uint8_t SIZE = 1) = 0;                        // Pure virtual function for initializing device
  virtual float getPosition(void) = 0;                                        // Pure virtual function for retrieving the position
  virtual void setPosition(float desiredState) = 0;                           // Pure virutal function for setting position
  virtual void originSearch(void) = 0;                                        // Pure virtual function for executing commands in order for hardware to find the hardware origin
};

class Motor_Stepper : public Motor {
  protected:
    uint32_t relativeStep;      // Current step relative to total steps in a revolution
    uint32_t relativeState;     // State relative to the origin of a periodic output (ex: 360 degrees is equivalent to 0 degrees)
    float gearRatio;            // Ratio of Drive Gear teeth to NEMA Gear teeth

    // Methods characteristic of Stepper Motors ONLY : 
    uint32_t computeOverallStep(float rotorAngle);    // Utility to assist 'this' with positioning

  public:
    Motor_Stepper(uint8_t correspondingMotor = 0) : Motor(correspondingMotor), relativeStep(0), relativeState(0), gearRatio(1) {     }

    uint32_t computeRelativeStep(double desiredState) {
      float N = abs(desiredState / 360);
      float eqDesiredPos = desiredState + ((desiredState < 0) ? ceil(N) : -floor(N)) * 360;             //      0 <=   eqDesiredPos  <= 360
      float currPos = this->getPosition();                                                              //      0 <=   currPos       <= 360
      uint32_t stepsPerRev = this->gearRatio * this->driver.physicalDynamics->getTotalStates();         // # of steps to perform a complete rotation
      uint32_t finalRelativeStep = 0.5 /* moves us to closest integer */ + (eqDesiredPos * stepsPerRev / 360);  // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here;
      return finalRelativeStep;
    }
    
    int32_t computeNumSteps(double desiredState) {
      uint32_t finalRelativeStep = (int32_t)this->computeRelativeStep(desiredState);                           // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here
      return this->computeNumSteps(finalRelativeStep);
    }

    // Overloaded function for computing # steps
    int32_t computeNumSteps(uint32_t finalRelativeStep) {
      int32_t stepsPerRev = this->gearRatio * (int32_t)this->driver.physicalDynamics->getTotalStates();       // # of steps to perform a complete rotation
      uint32_t directPathNumSteps = abs((int32_t)finalRelativeStep - (int32_t)this->relativeStep);
      
      bool directPath = directPathNumSteps < (stepsPerRev / 2.0);
      int32_t numSteps = finalRelativeStep - this->relativeStep;
      if (!directPath) {
        // numSteps = -(((finalRelativeStep < (int32_t)this->relativeStep) ? -stepsPerRev : stepsPerRev) - numSteps);    // Works excellently
        numSteps = ((finalRelativeStep < (int32_t)this->relativeStep) ? stepsPerRev : -stepsPerRev) + numSteps;
        //numSteps = stepsPerRev + ((finalRelativeStep < (int32_t)this->relativeStep) ? -numSteps : numSteps); //1 : -1) * abs(stepsPerRev - (int32_t)this->relativeStep + (int32_t)finalRelativeStep);
      }
      return numSteps;
    }



    // ##### Polymorphism Methods #####
    virtual ~Motor_Stepper(void) override {
      // Cleanup here..
    }

    // ############### DEFINE PURE VIRTUAL FUNCTIONS (REQUIRED BY COMPILER) ################
    virtual void redefineOrigin(void) override {                                     // Virtual function for resetting the coordinate system such that new values are relative to the current position
      this->relativeState = 0;
      this->relativeStep = 0;
      this->motorAtOrigin = true;
    }

    virtual bool atOrigin(void) override {
      return this->relativeStep == 0;
    }

    float getPosition() override;

    // ############### Empty Pure Virtual Definitions (Must be overridden by next derived class) ################
    virtual void initializeDevice(uint8_t SIZE = 1) override {  }   // Characteristics of 'initializeDevice' are inherent of the class derived from this class
    virtual void setPosition(float desiredState) override {  }   // Characteristics of 'initializeDevice' are inherent of the class derived from this class
    virtual void originSearch(void) override {  }   // Characteristics of 'initializeDevice' are inherent of the class derived from this class
};

#if NEMA17_SUPPORTED

  class Motor_Nema17 : public Motor_Stepper {
    private:
    // ############### DEFINE PURE VIRTUAL FUNCTIONS ################
    void originSearch(void) override;

    public:
      // Non-Static Members
      struct TMC2130Stepper* tmcDriver;
      struct AccelStepper stepper;

      // Constructor Function for a NEMA17 stepper motor
      Motor_Nema17(uint8_t axis = 1) : Motor_Stepper(Enumerators::NEMA17) {
        #if (MOTOR_DEBUGGER)
          if (Serial)
            Serial.print("NEMA17 Constructor\r\n");
        #endif

        this->tmcDriver = nullptr;
        this->gearRatio = 2;                        // Number of rotations of drive for each rotation of output
        this->driver.electricalDynamics.current = 700;     // Stepper strength [mA]
        this->driver.motor.selectPinout(axis);
      }

      // Destructor Function for a NEMA17 stepper motor
      virtual ~Motor_Nema17() override {
        if (this->tmcDriver)
          delete this->tmcDriver;
      }
      
    // ############### DEFINE PURE VIRTUAL FUNCTIONS (REQUIRED BY COMPILER) ################
    void initializeDevice(uint8_t SIZE = 1) override;
    virtual void setPosition(float degrees) override;

    // ############### DEFINE VIRTUAL FUNCTIONS ################
    
  };
#endif

#if NEMA23_SUPPORTED
  

  class Motor_Nema23 : public Motor_Stepper {
  private:
    // ############### DEFINE PURE VIRTUAL FUNCTIONS ################
    void originSearch(void) override;


  public:
    // Constructor Function for a NEMA23 stepper motor
    Motor_Nema23(uint8_t axis = 1) : Motor_Stepper(Enumerators::NEMA23) {
      #if (MOTOR_DEBUGGER)
        if (Serial)
          Serial.print("NEMA23 Constructor\r\n");
      #endif
      
      this->driver.electricalDynamics.micro_pulseDuration = 500; // Pulse duration in microseconds (t >= 2.5us, 50% duty cycle recommended)
      this->gearRatio = 57/24;  // Number of rotations of drive for each rotation of output
      this->driver.motor.selectPinout(axis);
    }

    // Destructor Function for a NEMA23 stepper motor
    virtual ~Motor_Nema23() override {
      // Cleanup here...
    }

    // ############### DEFINE PURE VIRTUAL FUNCTIONS (REQUIRED BY COMPILER) ################
    void initializeDevice(uint8_t SIZE = 1) override;
    virtual void setPosition(float degrees) override;

    // ############### DEFINE VIRTUAL FUNCTIONS ################
    
  };
#endif

#if G2_SUPPORTED

#define EXTEND false
#define RETRACT true

  struct G2_Driver : public MotorDriver {
    struct DualG2HighPowerMotorShield24v14 driver;
    public : 

    G2_Driver(unsigned char M1nSLEEP = 1, unsigned char M1DIR = 2, unsigned char M1PWM = 22,
              unsigned char M1nFAULT = 19, unsigned char M1CS = 23, unsigned char M2nSLEEP = 3,
              unsigned char M2DIR = 4, unsigned char M2PWM = 15, unsigned char M2nFAULT = 18, unsigned char M2CS = 14) : 
              MotorDriver(Enumerators::G2, 1),
              driver(M1nSLEEP, M1DIR, M1PWM, M1nFAULT, M1CS, M2nSLEEP, M2DIR, M2PWM, M2nFAULT, M2CS) {
      this->driver.init();
    }
    struct DualG2HighPowerMotorShield *getDriver(void) {  return &this->driver; }
  };

  class Motor_G2 : public Motor {
  private:
    const float ORIGIN_OFFSET = 5;                      // Offset origin from hardware limit by 5 mm
    float pulsesPerMM = 99.3103;                        // 72 * 10mm desired dist / 7.25 mm actual dist = 99.3103 : Firgelli Automations (FA-BS16-11-12-60) Linear Actuator with 11bs Force and 60mm stroke (Normally only 36 pulses/mm but we have 2 hall effect sensors which will double the pulse count)
    struct DualG2HighPowerMotorShield *g2Driver;        // Pololu Dual G2 High-Power Motor Driver 24v14 Shield

    // ############### DEFINE PURE VIRTUAL FUNCTIONS ################
    void originSearch(void) override;
    
  public:
    // Static Members
    static const int16_t originSearchSpeed = 5;         // Actuator velocity (default) when searching for the actuator origin (mm/s)
    static bool g2Direction[2];                         // Actuator direction of travel 
    static int32_t g2Steps[2];                          // Actuator overall steps
    static volatile uint32_t unaccountedSteps[2];       // Steps accumulated in motor ISR awaiting transfer to overall steps var, 'g2Steps'. Use 'g2Direction' to determine direction   
    
    // Non-Static Members (These vars point to the static members corresponding to the axis in use)
    bool *direction;                                    // Actuator direction of travel
    int32_t *steps;                                     // Actuator overall steps
    
    void stopAxis(uint8_t axis = 1);
    void moveAxis(uint8_t axis);

    // Constructor Function for a G2 Actuator
    Motor_G2(uint8_t axis = 1, struct DualG2HighPowerMotorShield *driverChip = nullptr) : Motor(Enumerators::G2) {
      #if (MOTOR_DEBUGGER)
        if (Serial)
          Serial.print("G2 Constructor\r\n");
      #endif

      if (axis < 1 || axis > 2) {
        delete this;
      }

      this->steps = &(Motor_G2::g2Steps[axis - 1]);       // Point to the actuator steps status register (ISR needs to change status reg at any time)
      this->direction = &(Motor_G2::g2Direction[axis - 1]);

      this->g2Driver = driverChip;              // Assign the shared driver obj to the local motor instance
      this->driver.motor.selectPinout(axis);    // Sets the desired axis for use
    }



    // Destructor Function for a G2 Actuator
    virtual ~Motor_G2() override {
      if (this->g2Driver)
        delete this->g2Driver;
      // Cleanup here...
    }

    // ############### DEFINE PURE VIRTUAL FUNCTIONS (REQUIRED BY COMPILER) ################
    void redefineOrigin(void) override {          // Virtual function for resetting the coordinate system such that new values are relative to the current position
      this->motorAtOrigin = true;
      *this->steps = 0;                                   // Set the current position as the origin
      Motor_G2::unaccountedSteps[*this->axis - 1] = 0;    // Resets the pulses count in the motor hall effect ISR
    }

    virtual bool atOrigin(void) override {
      return this->motorAtOrigin;
    }

    void initializeDevice(uint8_t SIZE = 1) override;

    float getPosition(void) override {
      int32_t isrSteps = ((*this->direction == EXTEND) ? 1 : -1) * this->pullSteps(false);     // Pull steps accumulated in motor ISR - DO NOT TRANSFER THE STEPS as doing so often will cause drift on the position due to modulating ISR's on/off
      return (*this->steps + isrSteps) / this->pulsesPerMM;
    }

    // Updates the overall position by incrementing/decrementing total pulses accumulated but not accounted for from the ISR
    void setDirection(bool direction) {
      if (direction != *this->direction)
        this->pullSteps(true);                                                                    // Pull steps accumulated on ISR and account them to the direction of previous travel
      *this->direction = direction;                                                               // Set the new direction of travel
      digitalWrite(this->driver.motor.pinout[Enumerators::G2Pins[Enumerators::DIR]], direction);  // Set motor direction
    }    

    // ############### DEFINE VIRTUAL FUNCTIONS ################

    void selectAxis(uint8_t axis) override {
      // This function is still implemented because it could be used to implement travel distance per pulse. 
      // This would be important because the travel distance per pulse can vary based on the actuator.
      this->driver.motor.selectPinout(axis);
    }

    uint32_t pullSteps(bool transferPulses) {
      uint8_t index = *this->axis - 1;
      if (transferPulses) {
        cli();                                                                // Disable ALL interrupts briefly as we interface and reset a volatile var
        *this->steps += (*this->direction == EXTEND) ? Motor_G2::unaccountedSteps[index] : - Motor_G2::unaccountedSteps[index];
        Motor_G2::unaccountedSteps[index] = 0;                                // The steps from the ISR have now been accounted for 
        sei();                                                                // Enable ALL interrupts
        return 0;                                                             // There are no steps pending on the ISR counter - They are accounted for in the non-volatile steps counter var
      }
      else {
        return Motor_G2::unaccountedSteps[index];                             // ONLY return the steps accumulated on the motor pulse counter ISR
      }
    }
    
    void setPosition(float positionMM) override {
      if (positionMM < 0 - (2 * Motor_G2::ORIGIN_OFFSET / 3))
        positionMM = 0;
      else if (positionMM > 60 - (2 * Motor_G2::ORIGIN_OFFSET / 3))
        positionMM = 60;
      
      uint8_t bufferIndex = 0;
      float currPosition = this->getPosition();
      float prevPosition;
      float eps = 0.02;                                // 0.1mm resolution
      uint8_t prevIndex = 0;

      bool changeSensed[3] = { true, true, true };    // The state must remain the same for 3 consecutive iterations to "reach the limit"
      uint8_t constantStates = 0; 

      while (constantStates < 3 && abs(currPosition - positionMM) > eps) {
        this->openLoopSetPosition(positionMM);    // Set position using trapezoidal velocity profile. Designed to undershoot estimated distance required
        prevPosition = currPosition;
        delay(5);                                // allow time for changes to take effect
        currPosition = getPosition();
        
        // Logic to ensure loop can exit when motor does not move after 3 consecutive loops.
        if (abs(currPosition - prevPosition) < eps) {
          if (changeSensed[bufferIndex]) {
            changeSensed[bufferIndex] = false;
            constantStates++;
          }
        }
        else {
          if (changeSensed[bufferIndex] == false) {
            changeSensed[bufferIndex] = true;
            constantStates--;
          }
        }
        bufferIndex = (bufferIndex + 1) % 3;
      }
    }

    void moveAxis(float speed = 0, float durationMS = -1);
    
    // ISR Routines :
    static void M1_ISR(void);
    static void M2_ISR(void);

    private:

    void moveDuration(bool direction, float durationMs) {
      float delayInterval = min(durationMs, 200);             // Pause in intervals of 200ms to observe the sensor states as motor is moving
      float totalDelay = 0;
      float pwmVal = this->computePWMVal();
      
      if (pwmVal < 1)
        return;

      uint8_t constCount = 0;                                 // Counter of consecutive loops where the pulse count from the Motor ISR remains constant
      int32_t prevSteps;
      int32_t currSteps;

      this->setDirection(direction);                                                                // Set direction of travel
      analogWrite(this->driver.motor.pinout[Enumerators::G2Pins[Enumerators::PWM]], pwmVal);        // Set motor speed (0 to 255)

      // Observe steps counter while motor is operating.
      // Stop the motor when the pulses stop incrementing
      while (constCount++ < 3 && totalDelay < durationMs) {
        prevSteps = (int32_t)Motor_G2::unaccountedSteps[*this->axis - 1];  
        delay(delayInterval);           // allow time for changes to take effect
        totalDelay += delayInterval;
        currSteps = (int32_t)Motor_G2::unaccountedSteps[*this->axis - 1];
        if (abs(prevSteps - currSteps) > 2)
          constCount = 0;
      }
      analogWrite(this->driver.motor.pinout[Enumerators::G2Pins[Enumerators::PWM]], 0);           // Motor reached origin. Turn it off.
    }

    float computePWMVal(void) {
      return (this->getPositionRate(1) + 0.398) / 0.0331;     // Conversion from actuator velocity (mm/s to approximate pwm value to set)
    }

    void openLoopSetPosition(float positionMM) {
      float currPos = this->getPosition();
      float direction = currPos > positionMM;

      #if DEBUGGER_OVERRIDE
        Serial.print("Curr Position : ");   Serial.print(currPos);    Serial.print("\r\n"); // Terminate the line
        Serial.print("Dest Position : ");   Serial.print(positionMM); Serial.print("\r\n"); // Terminate the line
        Serial.print("Direction Travel : ");   
        if (direction == RETRACT)
          Serial.print("RETRACT\r\n");
        else if (direction == EXTEND)
          Serial.print("EXTEND\r\n");
      #endif
        
      float totTravelDist = abs(positionMM - currPos);
      float velocityMin = 0.75;                             // mm/s
      float velocityMax = 4;                                // mm/s : ACTUAL VELOCITY MAX = 7.5 mm/s
      float peakVelocity = velocityMax;                     // Assume the max velocity will be the peak  (ie. the velocity that is/may be held constant)
      float Tstep = 0.01;                                   // 10 ms
      uint16_t rampSteps = 5;                               // Use 5 units of time to move from min velocity to max velocity
      uint16_t constVelocitySteps = 0;                      // Initialize var (uint32_t may be possible, but to ensure unrealistically long commands are not waiting to be executed, truncation is preferred and resorted to if needed)
      float minDist = velocityMin * Tstep;                  // Minimum distance to travel in a single increment of sending motor pulses (T = Tstep, V = Vmin)
      float dStartupTot = 0;
      
      if (rampSteps == 1)
        rampSteps = 2;                                      // DO NOT use 1 step as it WILL cause division by zero
      
      if (totTravelDist < 2 * rampSteps * minDist) {
        rampSteps = 0;                                      // It takes too long to ramp to the peak velocity
        peakVelocity = velocityMin;                         // The velocity MUST be the min velocity (The constant velocity portion )
        constVelocitySteps = 1;                             // Use 1 constant pulse to move to position
        Tstep = totTravelDist / (velocityMin);              // Stretch the pulse to the whole duration of constant velocity
      }
      else {
        // Assume the ramp to max velocity can be achieved until found otherwise
        dStartupTot = Tstep * (velocityMin * rampSteps + (rampSteps + 1) * (velocityMax - velocityMin) / 2);
        if (totTravelDist < 2 * dStartupTot) {              // The ramp must be able to process within the time frame required to move to the final position
          peakVelocity = (((totTravelDist / (2 * Tstep)) - velocityMin * rampSteps) / 2) + velocityMin;
          dStartupTot = Tstep * (velocityMin * rampSteps + (rampSteps + 1) * (peakVelocity - velocityMin) / 2);
        }
        constVelocitySteps = ceil(0.85 * (totTravelDist - 2 * dStartupTot) / (peakVelocity * Tstep));      // milli prefix of position and velocity cancel out 
      }

      // Ramp up to max velocity
      for (uint16_t iter = 0; iter < rampSteps; ++iter) {
        this->setPositionRate(velocityMin + iter * (peakVelocity - velocityMin) / (rampSteps - 1), 1);
        this->moveDuration(direction, Tstep * 1000);        // Move for the specified duration (ms)
      }
      
      #if DEBUGGER_OVERRIDE
        Serial.print("Your peak velocity : ");  Serial.print(peakVelocity);   Serial.print("\r\n"); // Terminate the line
      #endif

      // Hold max velocity
      this->setPositionRate(peakVelocity, 1);
      for (uint16_t iter = 0; iter < constVelocitySteps; ++iter) {
        this->moveDuration(direction, Tstep * 1000);   // Move for the specified duration
      }

      // Ramp down to min velocity
      for (uint16_t iter = rampSteps; iter > 0; --iter) {
        this->setPositionRate(velocityMin + (iter - 1) * (peakVelocity - velocityMin) / (rampSteps - 1), 1);
        this->moveDuration(direction, Tstep * 1000);        // Move for the specified duration (ms)
      }
    }
  };
#endif

#endif