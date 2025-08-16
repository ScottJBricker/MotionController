#pragma once
#ifndef _PHYSICALDYNAMICS_H_
#define _PHYSICALDYNAMICS_H_

#include "BoardInfo.h"
#include "ComponentConnection.h"
#include <stdint.h>  // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h> 

// ##########   3rd Party Header Files  ############
// Arduino : Position Control
#if (G2_SUPPORTED)
  #define EXTEND false
  #define RETRACT true
  #include "DualG2HighPowerMotorShield.h"     // Motor Header for 2D Position Control (Hall Sensor will use the Encoder.h)
#endif

class MotorState {
  public:
    // User Preferences : 
    uint8_t correspondingMotor;                             // Motor from BoardInfo::

    volatile bool   motorAtOrigin;  // tells if motor is at origin (needed due to discrete limitations)
    float frequency;                                      // Cycle frequency
    volatile uint16_t velocity;                          // Units per second (ex: microsteps/sec, mm/sec)
    volatile uint16_t pulseRate;                          // Pulses per second (ex: N/sec)
    volatile uint32_t acceleration;                       // Microsteps per second ^2
    float resolution;                                     // Resolution of motor to move a finite distance

    // Driver State : Enable stored via D Flip-Flop, Fault signal generated via MotorDriver
    bool enable;                                          // Output enabled
    bool fault;                                           // Fault detected

    uint32_t relativeStep;      // Current step relative to total steps in a revolution
    uint32_t relativeState;     // State relative to the origin of a periodic output (ex: 360 degrees is equivalent to 0 degrees)

    MotorState(void) {
      this->resetState();
      this->resetSpecs();
    }

    virtual void resetState(void) {
      this->enable = false;
      this->fault = false;
    }

    virtual void resetSpecs(void) {
      this->correspondingMotor = 0;
      this->frequency = 0;
      this->pulseRate = 0;
      this->velocity = 0;
      this->acceleration = 0;
      this->resolution = 999;
    }

    // Add virtual destructor to make the class polymorphic
    virtual ~MotorState() {}
};

class StepperMotorState : public MotorState {
  private:
    uint32_t minPulseDuration;          // Pulse duration in seconds (0 to 127s)
    int8_t minPulseDurationExponent;    // Pulse duration, Base-10 exponent. -> actualPulseDuration = pulseDuration * pow(10, pulseDurationExponent)
    uint8_t maxDutyCycle;
  public:
    // Motor Driver Specs 
    uint8_t dutyCycle;                  // Pulse on-duration duty cycle
    float cycleDuration;             // Pulse duration in seconds (0 to 127s) (Used to manipulate angular velocity)
    int32_t cycleDurationExponent;      // Pulse duration, Base-10 exponent. -> actualPulseDuration = pulseDuration * pow(10, pulseDurationExponent)


   // uint32_t pulseDuration;              // Pulse duration in seconds (0 to 127s) (Used to manipulate angular velocity)
   // int8_t pulseDurationExponent;       // Pulse duration, Base-10 exponent. -> actualPulseDuration = pulseDuration * pow(10, pulseDurationExponent)
   // uint32_t offDuration;
   // int8_t offDurationExponent;
    

    // Motor Specs
    uint16_t stepsPerCycle;           // # of steps per cycle without micro-steps
    uint16_t microsteps;                                            // # of driver steps per Full-Step of stepper motor
    float gearRatio;



    float getMinPulseDuration(void) { return this->minPulseDuration * pow(10, this->minPulseDurationExponent);}
    uint8_t getMaxDutyCycle(void) { return this->maxDutyCycle; }
    int8_t getMinPulseDurationExponent(void) { return this->minPulseDurationExponent; }
    uint32_t getNormalizedMinPulseDuration(void) { return this->minPulseDuration; }

    StepperMotorState(uint8_t correspondingMotor = 0) : MotorState() {
      this->resetState();
      this->resetSpecs();
      this->correspondingMotor = correspondingMotor;
    }

    void resetState(void) override {
      MotorState::resetState();
      this->relativeStep = 0;
      this->relativeState = 0;
    }

    void resetSpecs(void) override {
      MotorState::resetSpecs();
    //  this->pulseDuration = 1;
    //  this->pulseDurationExponent = 0;
    //  this->offDuration = 0;
    //  this->offDurationExponent = 0;
      this->stepsPerCycle = 1;
      this->microsteps = 1;
      this->gearRatio = 1;
      this->dutyCycle = 0;
    }

    void setStatics(uint32_t minPulseDuration, int8_t pulseDurationExponent, uint8_t maxDutyCycle) {
      this->minPulseDuration = minPulseDuration;
      this->minPulseDurationExponent = pulseDurationExponent;
      this->maxDutyCycle = maxDutyCycle;
    }
};


class ActuatorState : public MotorState {
  public: 

  ActuatorState(uint8_t correspondingMotor = 0) : MotorState() {
    
  }

  void setStatics(void) {
    
  }
};


// Base 'PhysicalDynamics' Class for generic polymorphism
class DeviceDynamics {
  private : 
    // Pure Virtual Functions (MUST be overwritten by derived classes)
    
  protected:
  // Shared variables across the hierarchy
  uint32_t currPulseDuration;          // Pulse duration in seconds (0 to 127s)
  int8_t currPulseDurationExponent;   // Pulse duration, Base-10 exponent. -> actualPulseDuration = pulseDuration * pow(10, pulseDurationExponent)
  uint16_t current;                   // Stepper current in mA            (65A max)

  // Preallocate DeviceDyanmics in program memory
  #if (NEMA17_SUPPORTED)
    static struct ComponentConnection nema17Connection[NEMA17_SIZE];
    static struct StepperMotorState *nema17State[NEMA17_SIZE];                      // Current state for each CS line corresponding to a single comms bus
  #endif
  #if (STEPPER_ONLINE_SUPPORTED)
    static struct ComponentConnection stepperOnlineConnection[STEPPER_ONLINE_SIZE];
    static struct StepperMotorState *stepperOnlineState[STEPPER_ONLINE_SIZE];       // Current state for each CS line corresponding to a single comms bus
  #endif
  #if (G2_SUPPORTED)
    static struct ComponentConnection g2Connection[G2_SIZE];
    static struct ActuatorState *g2State[G2_SIZE];                         // Current state for each CS line corresponding to a single comms bus
  #endif

  public:
    // Current pinout status
    bool direction;                                       // Motor direction
  //  bool enable;                                          // Output enabled
  //  bool fault;                                           // Fault detected
    uint8_t cs;                                           // Chip select (0-255)
    uint8_t csSize;

    uint8_t *connectionAxis;  // Axis for each element
    uint8_t *connectionCS;  // CS for each element of 'this' corresponding to a ComponentConnection instance
    struct ComponentConnection **motorConnection;          // Points to a static private MotorDriver ComponentConnection instance
    struct MotorState **primitiveState;                         // Current state for each CS line corresponding to a single comms bus

    DeviceDynamics(const uint8_t *correspondingMotors, const uint8_t *motorAxis, const uint8_t *motorCS, const uint8_t SIZE, const uint8_t finalMotorCS = 0) {
      if (correspondingMotors == nullptr || motorAxis == nullptr || motorCS == nullptr || SIZE < 1)
        return;
        
      this->csSize = 0;
      this->resetState(); 

      // Step 1 : Allocate Static Vars (Assume not initialized)
      DeviceDynamics::allocateStaticVars(correspondingMotors, motorAxis, SIZE);

      this->allocateObjVars(SIZE);
      for (uint8_t index = 0; index < this->csSize; ++index) {
        this->connectionCS[index] = motorCS[index];
        this->connectionAxis[index] = motorAxis[index];

        if (this->primitiveState[index] != nullptr) {
          this->primitiveState[index]->enable = false;
          this->primitiveState[index]->fault = false;
          this->primitiveState[index]->correspondingMotor = correspondingMotors[index];
        }
      }

      this->linkObjPtrs(correspondingMotors, motorAxis, motorCS, SIZE);  // Link the obj pointers for this motor-axis pair
      this->cs = finalMotorCS;                                  // Leave cs line selected for parameter 'motorCS'.
    }

    virtual ~DeviceDynamics(void) {
      this->deallocateObjVars();
    }

    // Static Var Functions
    static void resetStaticVars(void);
    static void allocateStaticVars(void);
    static void allocateStaticVars(const uint8_t *motorModel, const uint8_t *axis, const uint8_t SIZE);
    static void initializeStaticVars(const uint8_t *motorModel, const uint8_t *axis, const uint8_t SIZE);
    static struct MotorState* fetchMotorState(const uint8_t motorModel, const uint8_t axis);
    static struct ComponentConnection* fetchComponentConnection(const uint8_t motorModel, const uint8_t axis);

    
    // Obj Var Functions
    void resetState(void);
    void deallocateObjVars(void);
    void allocateObjVars(uint8_t SIZE);
    void linkObjPtrs(const uint8_t *motorModel, const uint8_t *motorAxis, const uint8_t *motorCS, const uint8_t SIZE);
    
    
    void setDynamics(uint16_t current, uint16_t pulseRate, uint32_t acceleration);

    // Virtual Functions
    virtual void enableMotor(bool toEnable);
    virtual void setCS(uint8_t cs);

    // Pure Virtual Functions (MUST be overwritten by derived classes)
    virtual void initializeDevice(void) = 0;           // Pure virtual function for initializing all device cs
    virtual void initializeCS(uint8_t cs = 0) = 0;     // Pure virutal function for initializing the specified cs
    virtual float getPositionRate(uint8_t derivative = 0) = 0;
    virtual float getPosition(void) = 0;                                        // Pure virtual function for retrieving the position
    virtual uint32_t getTotalStates(void) = 0;
    virtual uint16_t getMicroSteps(void) = 0;
 //   virtual float getPulseDuration(void) = 0;
    virtual uint16_t getCurrent(void) = 0;
    virtual void selectMotor(uint8_t correspondingMotor) = 0;
    virtual void syncMotorDynamics(void) = 0;
    virtual void redefineOrigin(void) = 0;                                      // Pure virtual function for resetting the coordinate system such that new values are relative to the current position
    virtual bool atOrigin(void) = 0;
    virtual void setPosition(float desiredState) = 0;                           // Pure virutal function for setting position
    virtual void setPositionRate(double rate, uint8_t derivative = 0) = 0;
    virtual void move(float) = 0;
    virtual void originSearch(void) {};
};

// Physical Dynamics for stepper motors. 
// Supported motors are : NEMA17, STEPPER_ONLINE
class PhysicalDynamics_Stepper : public DeviceDynamics {
  private: 
    

    void setDynamics(uint16_t current, uint32_t acceleration, uint8_t dutyCycle, float frequency, uint16_t stepsPerRev, uint16_t microsteps, float gearRatio);
    
  public:
    // Additional members of Generic NEMA stepper motor

    struct StepperMotorState **currState;

    // Base OBJ Functions
    PhysicalDynamics_Stepper(const uint8_t *motorModels = nullptr, const uint8_t *motorAxis = nullptr, const uint8_t *elementSIZE = nullptr, uint8_t SIZE = 1, uint8_t motorCS = 0) : DeviceDynamics(motorModels, motorAxis, elementSIZE, SIZE, motorCS) {
      if (motorAxis == nullptr || motorModels == nullptr || elementSIZE == nullptr)
        return;

      //this->currState = dynamic_cast<StepperMotorState*>(this->primitiveState);
      //this->currState = static_cast<StepperMotorState**>(this->primitiveState);
      this->currState = reinterpret_cast<StepperMotorState**>(this->primitiveState);

     

      this->initializeDevice();

      
    }

    // Destructor function of Generic NEMA stepper motor
    virtual ~PhysicalDynamics_Stepper(void) override { 
      delete[] this->currState;
    }

    int32_t computeRelativeStep(double desiredState);
    uint32_t computeShortestPath(double desiredState); 
    int32_t computeShortestPath(int32_t moveSteps); 
    int32_t computeNumSteps(double desiredState);
    int32_t computeNumSteps(uint32_t finalRelativeStep);

/*
    uint32_t getAdjustedPulseDuration(void) { return this->d}

    uint32_t getAdjustedPulseDuration(void) { return this->currState[this->cs]->pulseDuration; }
    int8_t getPulseDurationExponent(void) { return this->currState[this->cs]->pulseDurationExponent; }
    float getPulseDuration(void) { return this->currState[this->cs]->pulseDuration * pow(10, this->currState[this->cs]->pulseDurationExponent); } 
    float getOffDuration(void) { return this->currState[this->cs]->offDuration * pow(10, this->currState[this->cs]->offDurationExponent); } 
    */
    float getRateLimit(uint8_t derivative);

    float getGearRatio(void) { return this->currState[this->cs]->gearRatio; }
    

    // Virtual Functions
    // Class utilities
    void initializeDevice(void) override;
    void initializeCS(uint8_t cs = 0) override;
    void selectMotor(uint8_t correspondingMotor) override;
    void syncMotorDynamics(void) override;
    void setPositionRate(double rate, uint8_t derivative = 0) override;
    float getPositionRate(uint8_t derivative = 0) override;
    void setCS(uint8_t cs) override; 

    // Class variable getters/setters
    // Getters
    float getPosition() override;
    uint32_t getTotalStates(void) override;
    uint16_t getMicroSteps(void) override {  return this->currState[this->cs]->microsteps;  }
    uint16_t getCurrent(void) override { return this->current; }
    
    // Setters
    void setMicroSteps(uint16_t microsteps);
    void setDutyCycle(uint8_t dutyCycle = 50);

    virtual void enableMotor(bool toEnable) override {
      switch (this->currState[this->cs]->correspondingMotor) {
        #if NEMA17_SUPPORTED
          case BoardInfo::NEMA17 :
          if (toEnable) {
            // Turn the motor driver on
            digitalWrite(this->driver->motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::EN]], LOW);    // Set EN pin to LOW  (Turns the motor driver on)
            digitalWrite(this->driver->motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], HIGH);   // Set CS pin to HIGH (Selects the motor driver)
          }
          else {
            // Turn the motor driver off
            digitalWrite(this->driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::EN]], HIGH);    // Set EN pin to LOW  (Turns the motor driver on)
            digitalWrite(this->driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], HIGH);   // Set CS pin to HIGH (Selects the motor driver)
          }
          break;
        #endif
        #if STEPPER_ONLINE_SUPPORTED
          case BoardInfo::STEPPER_ONLINE :
            // Assume CS Line updated already
            pinout = this->motorConnection[this->cs]->getPinout();
            digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);                    // Disable asynchronous state changes (added for reliability)
            delayMicroseconds(1);                                                                       // Give time for gates to update (>59ns)

            #if (DEBUGGER_OVERRIDE)
              this->motorConnection[this->cs]->displayPinout();
            #endif

            digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::EN]], toEnable ? HIGH : LOW);   // Enable/Disable CS 0 Motor Driver, setup time (>25ns)
            delayMicroseconds(1);                                                                       // Setup time before CLK (36ns)
            digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], HIGH);                   // Trigger a state change on the enable flip-flop
            delayMicroseconds(1);
          break;
        #endif
      }
      this->currState[this->cs]->enable = toEnable;
    }

  virtual void move(float delta) override {
    Serial.println("Move()");
    int32_t numSteps = (int32_t)(delta + (delta > 0 ? 0.5 : -0.5));   // Round to nearest integer
    switch (this->currState[this->cs]->correspondingMotor) {
        #if NEMA17_SUPPORTED
          case BoardInfo::NEMA17 :
            Serial.println(numSteps);   // Use 'numSteps' to clear warnings of unused variables

          break;
        #endif
        #if STEPPER_ONLINE_SUPPORTED
          case BoardInfo::STEPPER_ONLINE :
            bool initialEN = this->currState[this->cs]->enable;                                                                   // Store initial state so we can leave state as is.
            float cycleDuration = this->currState[this->cs]->cycleDuration;
            int8_t cycleDurationExponent = this->currState[this->cs]->cycleDurationExponent;
            uint8_t dutyCycle = this->currState[this->cs]->dutyCycle;

           //  Serial.print("Cycle Duration : ");Serial.print(this->currState[this->cs]->cycleDuration);Serial.print(" x 10^");Serial.println(this->currState[this->cs]->cycleDurationExponent);
            // Serial.print("Duty Cycle : ");Serial.println(dutyCycle);

        //    float pulseDuration = this->getPulseDuration();
            uint8_t startState = HIGH;
            (initialEN) ? Serial.println("initial = enabled") : Serial.println("initial = disabled");
         //   Serial.println("CLK Low");delay(3000);
            // Assume CS Line updated already
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);             // Disable asynchronous state changes
            delayMicroseconds(1);                                                                                                 // Give time for gates to update (>59ns)
        //    Serial.println("Update PUL, EN, DIR");
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], LOW);             // Turn off pulses - Trigger pulses in the FOR loop
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::EN]], HIGH);             // Ensure motor driver is on
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::DIR]], numSteps > 0);    // Set Direction (Timing Irrelevant Here) (>0ns)

            

            if (initialEN == false) {
              this->currState[this->cs]->enable = true;
              delay(240); // Motor Driver Enable time must be at least 200 ms
            }
            else
              delayMicroseconds(6);                                                                                               // Direction must be set at least 5 us before pulse signal

        //    this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]] = true;   

          //  Serial.println("CLK High");
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], HIGH);            // Pulses can only be sent if CLK is HIGH
          //  delayMicroseconds(1);                                                                                                 // Give time for gates to update (>59ns)
          //  delay(3000);
            
         //   Serial.println("Send pulses");
            // Send pulses to move to desired state
            this->motorConnection[this->cs]->sendPulses(&this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], 
              &startState, cycleDurationExponent, cycleDuration * dutyCycle / 100, cycleDurationExponent, cycleDuration * (100 - dutyCycle) / 100, abs(numSteps), 1);

            // Commented out because ISR needs EN to remain true for pulses to successfully send (look for way to bypass)
            //if (initialEN == false)
            //  this->enableMotor(false);
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);             // Disable asynchronous state changes

            // Update system position
            int32_t incSteps = this->computeShortestPath(numSteps);
            uint32_t stepsPerRev = this->getTotalStates();                                                    // # of steps to perform a complete rotation

            #if (DEBUGGER_OVERRIDE)
              Serial.print("Curr state : ");Serial.println(this->currState[this->cs]->relativeStep);
              Serial.print("inc steps = ");Serial.print(incSteps);Serial.print(", stepsPerRev =");Serial.println(stepsPerRev);
            #endif

            this->currState[this->cs]->relativeStep = (this->currState[this->cs]->relativeStep + incSteps) % stepsPerRev;

            #if (DEBUGGER_OVERRIDE)
              Serial.print("Set state : ");Serial.println(this->currState[this->cs]->relativeStep);
            #endif
          break;
        #endif
        default :
          Serial.println(numSteps);   // Use 'numSteps' to clear warnings of unused variables

      }

  }

  virtual void setPosition(float desiredState) override {
    switch (this->currState[this->cs]->correspondingMotor) {
        #if NEMA17_SUPPORTED
          case BoardInfo::NEMA17 :
            uint32_t finalRelativeStep = this->computeRelativeStep(desiredState);   // Compute the final stepper position from 0 to N steps (Output periodic with N steps)
            int32_t numSteps = this->computeNumSteps(finalRelativeStep);                      // Compute the number of steps to move in the direction of the polarity of the value.
            //Serial.println("Motor.h specs : STUCK HERE");
            //Serial.println(finalRelativeStep);
            //Serial.println(numSteps);
            this->stepper.move((long)numSteps);                                           // Instruct motor driver of the desired steps to move.
            //Serial.println("a");
            this->stepper.runToPosition();                                                // Instruct motor driver to move NEMA17 motor to the desired position.
            //Serial.println("B");
            this->relativeStep = this->computeRelativeStep(desiredState);       // Store MotionControllerAPI position in order to maintain rotation from 0 to 360 degrees.
            //Serial.println(this->relativeStep);
          
          break;
        #endif
        #if STEPPER_ONLINE_SUPPORTED
          case BoardInfo::STEPPER_ONLINE :
            // int32_t finalRelativeStep = this->computeRelativeStep(desiredState);   // Compute the final relative step and do not take a shortcut
            uint32_t finalRelativeStep = this->computeShortestPath(desiredState);   // Compute the final relative step and take the shortest path
            int32_t numSteps = this->computeNumSteps(finalRelativeStep);
            this->move(numSteps);
            /*
            bool initialEN = this->currState[this->cs]->enable;

            // Assume CS Line updated already
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);             // Disable asynchronous state changes
            delayMicroseconds(1);                                                                                                 // Give time for gates to update (>59ns)
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], LOW);
            digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::DIR]], numSteps > 0);    // Set Direction (Timing Irrelevant Here) (>0ns)
            if (this->currState[this->cs]->enable == false) {
              this->enableMotor(true);    // Turn the motor driver on
              delay(300); // Enable time must be at least 200 ms
            }
            delayMicroseconds(5.5);                                                                                               // Direction must be set at least 5 us before pulse signal

            int32_t currStep = 0;
            float pulseDuration = this->getPulseDuration();
            
            numSteps = abs(numSteps);
            for (currStep = 0; currStep < numSteps; ++currStep) {
                digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], HIGH);
                delayMicroseconds(pulseDuration);
                digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], LOW);
                delayMicroseconds(pulseDuration);
            }

            if (initialEN == false) {
              // Turn the motor driver off (Reduce system vibrations and power consumption since torque isn't needed)
              this->enableMotor(false);   // Turn the motor driver off
            }
            
            // Update system position
            this->currState[this->cs]->relativeStep = finalRelativeStep;
*/
          break;
        #endif
      }
  }

    virtual void redefineOrigin(void) override {                                     // Virtual function for resetting the coordinate system such that new values are relative to the current position
      this->currState[this->cs]->relativeState = 0;
      this->currState[this->cs]->relativeStep = 0;
      this->currState[this->cs]->motorAtOrigin = true;
    }

    virtual bool atOrigin(void) override {
      return this->currState[this->cs]->relativeStep == 0;
    }
};



  #if (G2_SUPPORTED)
  class PhysicalDynamics_G2 : public DeviceDynamics {
      private:
      static bool g2API_initialized;
      static struct DualG2HighPowerMotorShield24v14 g2API;
      struct DualG2HighPowerMotorShield *g2Driver;        // Pololu Dual G2 High-Power Motor Driver 24v14 Shield
      

      // ############### DEFINE PURE VIRTUAL FUNCTIONS ################

    public:
      const float ORIGIN_OFFSET = 0;                    // Offset origin from hardware limit by 0 mm
      float pulsesPerMM = 200;                           // Firgelli Automations (FA-BS16-11-12-60) Linear Actuator with 11bs Force and 60mm stroke (Normally only 36 pulses/mm but we have 2 hall effect sensors which will double the pulse count)
    
      // Static Members
      static const int16_t originSearchSpeed = 150;         // Actuator velocity (default) when searching for the actuator origin (speed = conversionFactor * 150 mm/s)

      // vars which can be removed because their information is stored in the driver->componentConnection
      static bool g2Direction[2];                         // Actuator direction of travel 
      static int32_t g2Steps[2];                          // Actuator overall steps
      static volatile int32_t unaccountedSteps[2];       // Steps accumulated in motor ISR awaiting transfer to overall steps var, 'g2Steps'. Use 'g2Direction' to determine direction   
      static volatile bool hallSensorPhase[4];                     // phase of the hall effect sensors (2 sensors per motor - with sensors 90 degrees out of phase producing a square wave)
      static bool g2Calibrated[2];      // This is false until a SEARCH ORIGIN is performed. Until then, max position is restricted to +5mm to enforce user to calibrate on startup to prevent damage.

      // Non-Static Members (These vars point to the static members corresponding to the axis in use)
      bool *direction;                                    // Actuator direction of travel
      int32_t *steps;                                     // Actuator overall steps
      
      struct ActuatorState **currState;
      void originSearch(void);
      
      PhysicalDynamics_G2(const uint8_t *motorModels = nullptr, const uint8_t *motorAxis = nullptr, const uint8_t *elementSIZE = nullptr, uint8_t SIZE = 1, uint8_t motorCS = 0) : DeviceDynamics(motorModels, motorAxis, elementSIZE, SIZE, motorCS) {
        if (motorAxis == nullptr || motorModels == nullptr || elementSIZE == nullptr)
          return;

        this->currState = reinterpret_cast<ActuatorState**>(this->primitiveState);

        // Initialize Motor Driver api
        this->initializeDriverAPI(SIZE);
        
        // Initialize Motor
        this->initializeDevice();
      }

      // ISR Routines :
      static void M1_ISR(void);
      static void M2_ISR(void);

      //  static void ISR(void);


      // Class utilities
      void selectMotor(uint8_t correspondingMotor);
      float getRateLimit(uint8_t derivative);
    
      // Override Pure Virtual Functions
      ~PhysicalDynamics_G2(void) override {
        if (this->g2Driver)
            delete this->g2Driver;
      }

      void initializeDevice(void);

      void initializeDriverAPI(uint8_t SIZE) {
        if (SIZE < 2)
          return;   // This algorithm assumes 2 CS are used. Errors will occur if not filtered.

        this->g2Driver = static_cast<DualG2HighPowerMotorShield*>(&this->g2API);

        if (!PhysicalDynamics_G2::g2API_initialized) {
          this->g2Driver->init();
          PhysicalDynamics_G2::g2API_initialized = true;
        }

        for (uint8_t axis = 1; axis <= SIZE; ++axis) {
          this->initializeCS(axis - 1);
          this->setCS(axis - 1);
          this->setPositionRate(5, 1);    //  Set 5 mm/s  (Derivative 1 : Velocity)

          // Initialize Motor Hardware
          // IF THIS WORKS... THEN THIS IMPLEMENTATION SHOULD BE USED AS THE REFERENCE FORMAT TO REPLICATE 
          // This is in regards to the indexing into the motor
          switch (this->connectionAxis[axis - 1]) {
          //switch (this->motorConnection[axis - 1]->axisCS[this->connectionCS[axis - 1]]) {
            case 1:
              this->g2Driver->calibrateM1CurrentOffset();
            break;
            case 2:
              this->g2Driver->calibrateM2CurrentOffset();
            break;
          }
        }
    }

    void setCS(uint8_t cs) override;

    void setPositionRate(double rate, uint8_t derivative = 0) override;

    float getPositionRate(uint8_t derivative = 0) override;

    // Updates the overall position by incrementing/decrementing total pulses accumulated but not accounted for from the ISR
    void setDirection(bool direction) {
      *this->direction = direction;                                                               // Set the new direction of travel
      digitalWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::DIR]], direction);  // Set motor direction
    }    

    int32_t pullSteps(bool transferPulses) {
      if (transferPulses) {
        cli();                                                                // Disable ALL interrupts briefly as we interface and reset a volatile var
        int32_t isrSteps = PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1];
        this->primitiveState[cs]->relativeStep += PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1];
        this->primitiveState[cs]->relativeState += PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1];
        PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1] = 0;                                        // The steps from the ISR have now been accounted for 
        sei();                                                                // Enable ALL interrupts
        return isrSteps;                                                      // Return the steps which were transferred to the obj steps counter
        //return 0;                                                             // There are no steps pending on the ISR counter - They are accounted for in the non-volatile steps counter var
      }
      else {
        return PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1];                             // ONLY return the steps accumulated on the motor pulse counter ISR
      }
    }

    float getPosition(void) override {
      #if (DEBUGGER_OVERRIDE)
      if (Serial) {
        Serial.print("this->CS : ");Serial.println(this->cs);
        Serial.print("Steps var : ");Serial.println(this->primitiveState[cs]->relativeStep);
        Serial.print("ISR counter : ");Serial.println(this->pullSteps(false));
      }
      #endif
      int32_t currSteps = this->primitiveState[cs]->relativeStep;  // Hold current state before pulling from ISR to ensure accurate calculation
      return (currSteps + this->pullSteps(true)) / this->pulsesPerMM; // Pull steps accumulated in motor ISR - DO NOT TRANSFER THE STEPS as doing so often will cause drift on the position due to modulating ISR's on/off
      //return (currSteps + this->pullSteps(false)) / this->pulsesPerMM; // Pull steps accumulated in motor ISR - DO NOT TRANSFER THE STEPS as doing so often will cause drift on the position due to modulating ISR's on/off
    }

    uint32_t getTotalStates(void) override {
      // TODO: DEFINE ME
      return 0;
    }

    uint16_t getMicroSteps(void) override {
      // TODO: DEFINE ME
      return 0;
    }

    void setDynamics(uint16_t velocity, uint32_t acceleration);

    void redefineOrigin(void) override {                                              // Virtual function for resetting the coordinate system such that new values are relative to the current position
      this->currState[this->cs]->motorAtOrigin = true;
      this->primitiveState[cs]->relativeStep = 0;                                     // Set the current position as the origin
      this->primitiveState[cs]->relativeState = 0;                                    // Set the current position as the origin
      PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1] = 0;  // Resets the pulses count in the motor hall effect ISR
    }

    virtual bool atOrigin(void) override {
      return this->currState[this->cs]->motorAtOrigin;
    }

    virtual void initializeCS(uint8_t cs) override;
    virtual void move(float delta) override;
    virtual void syncMotorDynamics(void) override;

    void moveAxis(float speed = 0, float durationMS = -1);  // TODO : Define me
    void stopAxis(uint8_t axis = 1);  // TODO : Define me
    void moveAxis(uint8_t axis);  // TODO : Define me

    virtual void setPosition(float positionMM) override {
      uint8_t maxPosition = PhysicalDynamics_G2::g2Calibrated[this->connectionAxis[this->cs] - 1] ? 58 : 10; // Distance is limited to 58mm; limit to 10mm if not calibrated to ensure positioner does not damage the frame
      uint8_t minPosition = 58;             // Negative
        this->motorConnection[this->cs]->setCS(this->connectionCS[this->cs]);

        if (positionMM < -minPosition + (2 * PhysicalDynamics_G2::ORIGIN_OFFSET / 3))
          positionMM = -minPosition;
        else if (positionMM > maxPosition - (2 * PhysicalDynamics_G2::ORIGIN_OFFSET / 3))
          positionMM = maxPosition;
        
        uint8_t bufferIndex = 0;
        float currPosition = this->getPosition();
        float prevPosition;

        float resolution = 1 / PhysicalDynamics_G2::pulsesPerMM;  // Sensor resolution (mm/pulse)
       // float tolerance = 0.0000200;                                  // 20 um positional accuracy tolerance
        float toleranceMM = 0.02;       // 20 um positional accuracy tolerance
        uint8_t constantStates = 0; 

        #if (DEBUGGER_OVERRIDE)
        if (Serial) {
          Serial.println("G2: Set Position");        
          Serial.print("cs : "); Serial.println(this->cs);
          Serial.print("unnacountedSteps : ");Serial.println(PhysicalDynamics_G2::unaccountedSteps[this->cs]);
          Serial.print("current step count : ");Serial.println(this->primitiveState[cs]->relativeStep);
        }
        #endif

        while (constantStates < 3 && abs(currPosition - positionMM) > toleranceMM) {
          prevPosition = currPosition;
          this->openLoopSetPosition(positionMM);    // Set position using trapezoidal velocity profile. Designed to undershoot estimated distance required
          //delay(200); // Ensure measurement is stable
          currPosition = getPosition();

          #if (DEBUGGER_OVERRIDE)
          if (Serial) {
            Serial.print("Prev position : ");Serial.println(prevPosition * pow(10, 6));
            Serial.print("Curr position : ");Serial.println(currPosition * pow(10, 6));
            Serial.println();
            Serial.println();
          }
          #endif

          // Logic to ensure loop can exit when motor does not move after 3 consecutive loops.
          if (abs(currPosition - prevPosition) < resolution)
            constantStates++;
          else
            constantStates = 0; 
          bufferIndex = (bufferIndex + 1) % 3;
        }
        
      }

    void openLoopSetPosition(float positionMM) {
      unsigned long last;
     
      int32_t prevSteps;
      int32_t currSteps;
      float conversionFactor = 35 * pow(10, -6);              // 35.0 um/pwm
      uint8_t currVelocity = 0, currAcceleration = 0;
      float rampPos, decPos;
      float currPos = this->getPosition();
      bool direction = currPos > positionMM;
      
      #if DEBUGGER_OVERRIDE
      if (Serial) {
        Serial.print("Curr Position : ");   Serial.print(currPos);    Serial.print("\r\n"); // Terminate the line
        Serial.print("Dest Position : ");   Serial.print(positionMM); Serial.print("\r\n"); // Terminate the line
        Serial.print("Direction Travel : ");   
        if (direction == RETRACT)
          Serial.print("RETRACT\r\n");
        else if (direction == EXTEND)
          Serial.print("EXTEND\r\n");
      }
      #endif

      //    Progessive Trapezoidal Velocity Profile
      float totTravelDist = abs(positionMM - currPos) / 1000;
      float dt = 50 * pow(10, -6);                               // During the ramp, velocity changes at a rate of 'acc' every 'dt' = 50 us
      uint8_t acc = 1, vMax = 200;

      if (totTravelDist < pow(10, -5) /* 10 um */) {        // tMove < 267 ms
        acc = 1;
        vMax = 15;
      }
      else if (totTravelDist < 5 * pow(10, -5) /* 50 um */) {   // tMove < 444 ms
        acc = 4;
        vMax = 40;
      }
      else if (totTravelDist < pow(10, -4) /* 100 um */) {  // tMove < 267 ms
        //acc = 3;
        //vMax = 66;
        acc = 5;
        vMax = 80;
      }
      else if (totTravelDist < pow(10, -3) /* 1 mm */) {    // tMove < 267 ms
        acc = 5;
        vMax = 120;
      }
      else if (totTravelDist < pow(10, -2) /* 10 mm */) {   // tMove < 1.78 seconds
        acc = 10;
        vMax = 180;
      }
      else { /* 100 mm */                                // tMove < 13.3 seconds
        acc = 20;
        vMax = 200;                                         // 7.5 mm/s (MAX SPEED)
      }

      uint8_t rampSteps = max(floor(vMax / acc), 1);        // Compute # ramp steps - there must be at least 1 step
      float delayInterval = 1 / (1000 * PhysicalDynamics_G2::pulsesPerMM * rampSteps * acc * conversionFactor);//min(pow(10, -3), );  // Delay interval in which we should expect 1 pulses to have been counted
      
      // Pre-Process For Trapezoidal Move
      // 1. Pre-Process to compute estimated ramp distance 
      float rampDistance = conversionFactor * dt * acc * rampSteps * (rampSteps + 1) / 1000;
      
      // 2. Extract required high speed on-duration
      //float peakVelocityDuration =  (totTravelDist - rampDistance) / (conversionFactor * rampSteps * acc);//max(pow(10, -3), 0.25 *);
      this->setDirection(direction);                                                                            // Set direction of travel
      
      // Perform rising velocity ramp
      analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], 0);               // Set motor speed (0 to 255)
      for (uint8_t currStep = 0; currStep < rampSteps; ++currStep) {
        currVelocity = currVelocity + acc;
        analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], currVelocity);  // Set motor speed (0 to 255)
        delay(dt * 1000);   // Ramp PWM voltage slowly to prevent large EMF force
      }

      // Hold the peak velocity until the distance - deltaD is achieved.
      uint32_t intervalsPerPulse = 100000;
      uint8_t constCountTolerance = 3;  // wait long enough for 3 missed pulses..
      uint32_t constCount = 0;                                                     // Counter of consecutive loops where the pulse count from the Motor ISR remains constant
      currSteps = this->primitiveState[cs]->relativeStep + this->pullSteps(false);

      if (!direction) { // moving forward
        decPos = positionMM - 2 * (this->getPosition() - currPos);
        while (constCount < constCountTolerance * intervalsPerPulse && this->getPosition() < decPos) {
          prevSteps = currSteps;
          delay(delayInterval);                                          // Delay a microsecond scale duration
          currSteps = this->primitiveState[cs]->relativeStep + this->pullSteps(false);
          abs(prevSteps - currSteps) > 0 ? constCount = 0 : ++constCount;
        }
      }
      else {
        decPos = positionMM + 2 * (currPos - this->getPosition());
        while (constCount < constCountTolerance * intervalsPerPulse && this->getPosition() > decPos) {
          prevSteps = currSteps;
          delay(delayInterval);                                          // Delay a microsecond scale duration
          currSteps = this->primitiveState[cs]->relativeStep + this->pullSteps(false);
          abs(prevSteps - currSteps) > 0 ? constCount = 0 : ++constCount;
        }
      }

      if (constCount < 3) {
        // Perform falling velocity ramp
        for (uint8_t currStep = 0; currStep < rampSteps; ++currStep) {
          currVelocity -= acc;
          analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], currVelocity);    // Set motor speed (0 to 255)
          delay(dt * 1000);   // Ramp PWM voltage slowly to prevent large EMF force
        }
      }
      analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], 0);    // Motor reached desired position. Turn it off.
    }

    PhysicalDynamics_G2& operator=(const PhysicalDynamics_G2& other) {
      if (this != &other) {  // Prevent self-assignment
          // Copy necessary members from `other`
          
      }
      return *this;
    }

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
      //analogWrite(this->driver.motorConnection->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], pwmVal);        // Set motor speed (0 to 255)
      analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], pwmVal);    // Set motor speed (0 to 255)

      // Observe steps counter while motor is operating.
      // Stop the motor when the pulses stop incrementing
      while (constCount++ < 3 && totalDelay < durationMs) {
        prevSteps = PhysicalDynamics_G2::unaccountedSteps[this->cs];  
        delay(delayInterval);           // allow time for changes to take effect
        totalDelay += delayInterval;
        currSteps = PhysicalDynamics_G2::unaccountedSteps[this->cs];
        if (abs(prevSteps - currSteps) > 2)
          constCount = 0;
      }
      //analogWrite(this->driver.motorConnection->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], 0);           // Motor reached origin. Turn it off.
      analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], 0);    // Motor reached origin. Turn it off.
    }

    float computePWMVal(void) {
      // This function computes the PWM value which results in the actuator 
      // traveling at the specified velocity in mm/s.
      return (this->getPositionRate(1) + 0.398) / 0.0331;
    }

    uint16_t getCurrent(void) override {
      return 0; // TODO : Define me
    }
  };

  #endif
#endif