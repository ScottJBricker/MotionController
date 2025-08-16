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

// Arduino : Rotational Control
#if (NEMA17_SUPPORTED)
  #include <TMC2130Stepper.h>                 // Needed for Arduino Leonardo for laser attenuation
  #include <AccelStepper.h>                   // Needed for Arduino Leonardo for laser attenuation
#endif
// #################################################

#define MOTOR_DEBUGGER (false && DEBUGGER)  // TURN OFF THE DEBUGGER HERE!! (show debugger stuff here, so long as the debugger is on)

//extern const uint8_t supportedMotorsAxisSize[];



// Abstract motor driver class
struct MotorDriver {
  private : 
  // Preallocate DeviceDyanmics in program memory
  #if (NEMA17_SUPPORTED)
    static struct PhysicalDynamics_Stepper *nema17Driver[NEMA17_SIZE];
  #endif
  #if (STEPPER_ONLINE_SUPPORTED)
    static struct PhysicalDynamics_Stepper *stepperOnlineDriver[STEPPER_ONLINE_SIZE];
  #endif
  #if (G2_SUPPORTED)
    //static struct G2_Driver g2Driver;
    static struct PhysicalDynamics_G2 *g2Driver[G2_SIZE];
    /*
  // Each device string identifier is separated by a ' ' (space)
    #if (G2_SUPPORTED)
  Device(void) : g2Driver(), g2Motors{Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)} {       // The G2 Motor class is designed for a chip capable of controlling 2-axis
    this->resetState();
  }
  #else
  */
  #endif

  public : 

  struct DeviceDynamics *deviceDriver;                // Points to an array of 'DeviceDynamics' (ie. Driver for N chips on a single comms bus using a CS line)
  static void resetStaticVars(void);
  static void allocateStaticVars(void);
  static void allocateObjVars(uint8_t *motorModel, uint8_t *axis, uint8_t SIZE);

  static void initializeStaticVar(uint8_t motorModel, uint8_t axis = 1);

  void linkDeviceDriver(uint8_t motorModel, uint8_t axis);
  

  MotorDriver(uint8_t motorModel, uint8_t axis, uint8_t motorCS = 0) {
    this->deviceDriver = nullptr;

    // Step 1 : Allocate Vars (Assume not initialized)
    MotorDriver::allocateObjVars(&motorModel, &axis, 1);

    // Step 2 : Fetch the motor axis instance
    this->linkDeviceDriver(motorModel, axis);

    #if (DEBUGGER_OVERRIDE)
      if (this->deviceDriver == nullptr) {
        Serial.println("ERROR : ptr = nullptr. State held, see MotorDriver()");
      while(1);
      }
    #endif
  }
  
  virtual ~MotorDriver() {
    delete this->deviceDriver;  // release dynamic memory
  }

};

class Motor {
private:
  
public: 
  const uint8_t *axis;  // Pointer to the axis value stored in the 'ComponentConnection' instance

  //volatile long currStep;     // State relative to the startup position
  
  volatile bool moving;         // is the actuator moving?
  int8_t controlCS;            // desired cs to control with next cmd
  uint8_t motorSIZE;            // max motors controller on a single comms bus

  struct MotorDriver driver;
  //struct MotorDriver *driver;

  Motor(uint8_t motorModel, uint8_t axis = 1, uint8_t motorCS = 0) : driver(motorModel, axis, motorCS)  {  
    this->moving = false;
    this->motorSIZE = BoardInfo::motorConfigsAxisCSSize[motorModel][axis - 1];
  }
  
  virtual ~Motor() { }                                        // Virtual destructor for cleanup

  // Base Motor Functions ONLY
  void statusPrintout(void);
  void setCS(uint8_t cs);
  void setPositionRate(float desiredState, uint8_t derivative = 0);   // Pure virtual function for retrieving the positing rate { 0|Position, 1|Velocity, 2|Acceleration }
  void move(float);

  bool isValid(int8_t controlCS);
  bool isValid(void) {    return this->isValid(this->controlCS);  }
  uint8_t getMotorSIZE(void) {    return this->motorSIZE;  }
  float getPositionRate(uint8_t derivative = 0);                      // Function for setting position rate        { 0|Position, 1|Velocity, 2|Acceleration }
  void selectAxis(uint8_t axis) {   this->driver.deviceDriver->setCS(axis - 1); }
  bool getMotorEnable(void) {   return this->driver.deviceDriver->primitiveState[this->controlCS]->enable;  }

  // Virtual Functions
  void initializeDevice(uint8_t *csList, uint8_t SIZE = 0) { this->driver.deviceDriver->initializeDevice(); }                        // virtual function for initializing device
  void initializeDevice(void) { this->driver.deviceDriver->initializeDevice(); }                        // virtual function for initializing device
  

  // Pure Virtual Functions (MUST be overwritten by derived classes)
  virtual void originSearch(void) = 0;                                        // Pure virtual function for executing commands in order for hardware to find the hardware origin

  
};

class Motor_Stepper : public Motor {
  protected:


    // Methods characteristic of Stepper Motors ONLY : 
    uint32_t computeOverallStep(float rotorAngle);    // Utility to assist 'this' with positioning

  public:
    Motor_Stepper(uint8_t motorModel, uint8_t axis = 1, uint8_t motorCS = 0) : Motor(motorModel, axis, motorCS) {  
      
    }
    
    // ##### Polymorphism Methods #####
    virtual ~Motor_Stepper(void) override {
      // Cleanup here..
    }

    // ############### Empty Pure Virtual Definitions (Must be overridden by next derived class) ################
    //virtual void initializeDevice(uint8_t SIZE = 1) override {  }   // Characteristics of 'initializeDevice' are inherent of the class derived from this class
    
    virtual void originSearch(void) override {  }   // Characteristics of 'initializeDevice' are inherent of the class derived from this class
    //virtual void initializeDevice(uint8_t *csList, uint8_t SIZE = 0) override { }
    //virtual void initializeDevice(void) override { };
};

#if STEPPER_ONLINE_SUPPORTED
  

  class Motor_StepperOnline : public Motor_Stepper {
  public:
    // Constructor Function for a StepperOnline stepper motor
    Motor_StepperOnline(uint8_t axis = 1, uint8_t motorCS = 0) : Motor_Stepper(BoardInfo::STEPPER_ONLINE, axis, motorCS) {
      #if (MOTOR_DEBUGGER)
        if (Serial)
          Serial.print("StepperOnline Constructor\r\n");
      #endif
    }

    // Destructor Function for a StepperOnline stepper motor
    virtual ~Motor_StepperOnline() override {
      // Cleanup here...
    }

    // ############### DEFINE VIRTUAL FUNCTIONS ################
    virtual void originSearch(void) override;
    //virtual void initializeDevice(uint8_t *csList, uint8_t SIZE = 0) override;                    // virtual function for initializing device
    //virtual void initializeDevice(void) override;                        // virtual function for initializing device
    };
#endif


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
      Motor_Nema17(uint8_t axis = 1, uint8_t motorCS = 0) : Motor_Stepper(BoardInfo::NEMA17, axis, motorCS) {
        #if (MOTOR_DEBUGGER)
          if (Serial)
            Serial.print("NEMA17 Constructor\r\n");
        #endif

        this->tmcDriver = nullptr;
        this->driver.motorConnection->selectPinout(axis);
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



#if G2_SUPPORTED
  class Motor_G2 : public Motor {

  public:


    // Constructor Function for a G2 Actuator
    Motor_G2(uint8_t axis = 1, uint8_t motorCS = 0) : Motor(BoardInfo::G2, axis, motorCS) {
      #if (MOTOR_DEBUGGER)
          if (Serial)
            Serial.print("G2 Constructor\r\n");
        #endif
    }

    // Destructor Function for a G2 Actuator
    virtual ~Motor_G2() override {
      // Cleanup here...
    }

    virtual void originSearch(void) override {
      this->driver.deviceDriver->originSearch();
     }                                        // Pure virtual function for executing commands in order for hardware to find the hardware origin


  };
#endif

#endif