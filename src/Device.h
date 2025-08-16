#pragma once
#ifndef _DEVICE_H_
#define _DEVICE_H_

// ############   Scott Header Files  ##############
#include "BoardInfo.h"
#include "CommandProcessing.h"

#include "Motor.h"                        // Class of motors that are pre-programmed for use
#include "PhysicalDynamics.h"

// ##########   Built in Header Files   ############
#include <stdio.h>
#include <stdint.h>                       // Standard datatype sizing : (stdint.h for c, cstdint for c++)

#define DEV_DEBUGGER (true && DEBUGGER)   // show debugger stuff here, as long as the debugger is on

extern const bool USE_DEFAULT_LIST;
extern const bool USE_DEFAULT_BOARD;
extern const char DEFAULT_MOTORS[];
extern const char DEFAULT_BOARD[];
extern const uint8_t startPrompt;

// A base class which stores all motors involved in the functionality of a device
class Device {
  private:
    #if (NEMA17_SUPPORTED)
      static struct Motor_Nema17 *nema17Motors[NEMA17_SIZE];
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      static struct Motor_StepperOnline *stepperOnlineMotors[STEPPER_ONLINE_SIZE];
    #endif
    #if (G2_SUPPORTED)
      //static struct G2_Driver g2Driver;    
      static struct Motor_G2 *g2Motors[G2_SIZE];          // The G2 Motor class is designed for a chip capable of controlling 2-axis
    #endif

    int8_t boardConfiguration;
    uint8_t numUniqueMotors;
    uint8_t numAxis[BoardInfo::NUM_MOTOR_MODELS];     // Num motor of each unique motor used : G2, TMC2130, Nema23

    uint8_t totalAxis;
    uint8_t currDevCS[BOARD_SIZE];                      // Current CS for each axis of the dev controller. Used for function overloading
    uint8_t devAxisCSSize[BOARD_SIZE];                  // CSSize for each axis of the dev controller. Used for CS input validation.
    uint8_t *motorAxis[BOARD_SIZE];                      // axis for each axis relative to a specific/physical device/class obj : G2, TMC2130, Nema23
    uint8_t *motorAxisCS[BOARD_SIZE];                    // CS corresponding to each controller axis
    int8_t *correspondingMotor[BOARD_SIZE];             // Contains unique motor identifier : (-1: No motor), G2, TMC2130, Nema23

    //struct MotorDriver *extDriver[BOARD_SIZE];
    void resetState(void);
    
  public:
    enum inputPrompts { MC_NAME = 0, MOTOR_LIST, DONE };
    struct Motor** motors[BOARD_SIZE];            // Array of pointers to support multiple unique motors
    
    Device(void) { 
      for (uint8_t axisIndex = 0; axisIndex < BOARD_SIZE; ++axisIndex) {
        // Initialize pointers
        this->motorAxis[axisIndex] = nullptr;
        this->motorAxisCS[axisIndex] = nullptr;
        this->correspondingMotor[axisIndex] = nullptr;
        this->motors[axisIndex] = nullptr;
      }
      this->resetState();
    }

    ~Device(void) {
      this->deallocateObjVars();
      this->resetState();
    }

    // Copy constructor
    Device(const Device &dev1) {
      // device copy constructor
    }
    
    // Move Constructor
    Device(Device &&dev1) noexcept {
      // device move constructor

      // move heap contect address?
    }

    static void resetStaticVars(void);
    void displayDeviceState(void);
    static void allocateStaticVars(void);
    static void allocateStaticMotors(const uint8_t *correspondingMotor, uint8_t *axis, uint8_t SIZE);
    void initializeDevice(void);            // Function used for selecting board config for controller use
    bool loadMotors(const uint8_t* deviceAxis, const uint8_t* deviceCS, const uint8_t* correspondingMotors, const uint8_t* motorAxis, const uint8_t* motorCS, const uint8_t* csSize, uint8_t SIZE);
    void allocateObjVars(uint8_t *axis, uint8_t *csSize, uint8_t SIZE);
    void initializeDeviceProcessing(void);
    void initializeDeviceProcessing(uint8_t axis, uint8_t cs);
    void deallocateObjVars(void);
    void deallocateObjVars(uint8_t devAxis);
    void removeAxis(uint8_t axis);
    void linkDevVars(void);
    
    // Old and needs updated...
    uint8_t openMotor(uint8_t motorSpecifier);
    
  /*
    // Copy assignment operator
    Device& operator=(const Device &dev1) {
      if (this != &dev1)   {  // Prevent self-assignment
        this->totalAxis = dev1.totalAxis;
        this->numUniqueMotors = dev1.numUniqueMotors;
        for (uint8_t axis = 0; axis < this->totalAxis; ++axis) {
          this->uniqueMotorAxis[axis] = dev1.uniqueMotorAxis[axis];
          this->motorAxis[axis] = dev1.motorAxis[axis];
          this->correspondingMotor[axis] = dev1.correspondingMotor[axis];
        }
        for (uint8_t iter = 0; iter < BoardInfo::NUM_MOTOR_MODELS; ++iter)
          this->numAxis[iter] = dev1.numAxis[iter];
      }
      return *this;
    }

    // Move assignment operator
    Device& operator=(Device &&dev1) noexcept {
      if (this != &dev1)   {  // Prevent self-assignment
        this->totalAxis = dev1.totalAxis;
        this->numUniqueMotors = dev1.numUniqueMotors;
        for (uint8_t axis = 0; axis < this->totalAxis; ++axis) {
          this->uniqueMotorAxis[axis] = dev1.uniqueMotorAxis[axis];
          this->motorAxis[axis] = dev1.motorAxis[axis];
          this->correspondingMotor[axis] = dev1.correspondingMotor[axis];
        }
        for (uint8_t iter = 0; iter < BoardInfo::NUM_MOTOR_MODELS; ++iter)
          this->numAxis[iter] = dev1.numAxis[iter];
      }
      return *this;
    }
  */
      int8_t getMotorAxis(uint8_t controllerAxis, uint8_t controllerCS) {
      return (controllerAxis < 1 || controllerAxis > BOARD_SIZE) ? -1 : this->motorAxis[controllerAxis - 1][controllerCS];
    }

    int8_t getMotorCS(uint8_t controllerAxis, uint8_t controllerCS) {
      return (controllerAxis < 1 || controllerAxis > BOARD_SIZE) ? -1 : this->motorAxisCS[controllerAxis - 1][controllerCS];
    }
    
    int8_t getMotorAxis(uint8_t controllerAxis) {
      return (controllerAxis < 1 || controllerAxis > BOARD_SIZE) ? -1 : this->motorAxis[controllerAxis - 1][this->currDevCS[controllerAxis - 1]];
    }

    int8_t getMotorCS(uint8_t controllerAxis) {
      return (controllerAxis < 1 || controllerAxis > BOARD_SIZE) ? -1 : this->motorAxisCS[controllerAxis - 1][this->currDevCS[controllerAxis - 1]];
    }

    void setDevCS(uint8_t controllerAxis, uint8_t controllerCS) {
      this->currDevCS[controllerAxis - 1] = controllerCS;
    }

    uint8_t getCurrDevCS(uint8_t controllerAxis) {
      return this->currDevCS[controllerAxis - 1];
    }

    uint8_t getNumDimensions(void) {  return this->totalAxis;   }
};
#endif
