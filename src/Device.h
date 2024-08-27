#pragma once
#ifndef _DEVICE_H_
#define _DEVICE_H_

// ############   Scott Header Files  ##############
#include "CommandProcessing.h"
#include "Enumerators.h"
#include "Motor.h"                        // Class of motors that are pre-programmed for use

// ##########   Built in Header Files   ############
#include <stdio.h>
#include <stdint.h>                       // Standard datatype sizing : (stdint.h for c, cstdint for c++)

#define DEV_DEBUGGER (true && DEBUGGER)   // show debugger stuff here, as long as the debugger is on
#define MAX_AXIS  3

extern const bool USE_DEFAULT_LIST;
extern const char DEFAULT_MOTORS[];

// A base class which stores all motors involved in the functionality of a device
class Device {

private:
  #if (NEMA17_SUPPORTED)
    struct Motor_Nema17 nema17Motors[1];
  #endif
  #if (NEMA23_SUPPORTED)
    struct Motor_Nema23 nema23Motors[2];
  #endif
  #if (G2_SUPPORTED)
    struct G2_Driver g2Driver;    // Using default constructor
    struct Motor_G2 g2Motors[2] = {Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)};          // The G2 Motor class is designed for a chip capable of controlling 2-axis
  #endif

  uint8_t numUniqueMotors;
  uint8_t uniqueMotorAxis[MAX_AXIS];                // Axis for a motor relative to its unique class (Usually front end users do not want this)
  uint8_t numAxis[Enumerators::NUM_MOTOR_MODELS];   // Num motor of each unique motor used : G2, TMC2130, Nema23

  uint8_t totalAxis;
  uint8_t motorAxis[MAX_AXIS];                      // axis for each axis relative to a specific/physical device/class obj : G2, TMC2130, Nema23
  uint8_t correspondingMotor[MAX_AXIS];             // Contains unique motor identifier : G2, TMC2130, Nema23
  struct MotorDriver *extDriver[MAX_AXIS];

  void resetState(void) {
    this->numUniqueMotors = 0;
    this->totalAxis = 0;
    for (uint8_t iter = 0; iter < MAX_AXIS; ++iter) {
      this->uniqueMotorAxis[iter] = 0;
      this->motorAxis[iter] = 0;
      this->correspondingMotor[iter] = 0;
    }
    for (uint8_t iter = 0; iter < Enumerators::NUM_MOTOR_MODELS; ++iter)
      this->numAxis[iter] = 0;
  }
  void initializeDeviceProcessing(void);
  
public:

  uint8_t getMotorAxis(uint8_t controllerIndex) {
    return this->motorAxis[controllerIndex];
  }
/*
  // Each device string identifier is separated by a ' ' (space)
    #if (G2_SUPPORTED)
  Device(void) : g2Driver(), g2Motors{Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)} {       // The G2 Motor class is designed for a chip capable of controlling 2-axis
    this->resetState();
  }
  #else
  */
  Device(void) { 
    this->resetState();
  }

//    #endif

  struct Motor* motors[MAX_AXIS];            // Array of pointers to support multiple unique motors

  void allocateDevVars(void);
  void initializeDevice(void);

  uint8_t openMotor(uint8_t motorSpecifier) {
    uint8_t axisIndex = 0;
    uint8_t iter;
    bool needsInitialized = false;
    uint8_t axisSet;

    // Traverse through motorAxis arr until all elements of this same motor are identified
    uint8_t currElement = 0;
    uint8_t correspondingElements[MAX_AXIS];
    uint8_t matchesFound = 0;
    uint8_t destElement = 0;
    
    while (currElement < this->totalAxis) {
      if (this->correspondingMotor[currElement] == motorSpecifier) {
        correspondingElements[matchesFound++] = currElement;
      }
      if (motorAxis[currElement] == 0) {
        destElement = currElement;
      }
      ++currElement;
    }

    // Find a free axis to designate for this motor
    bool duplicateFound;
    for (axisSet = 1; iter < MAX_AXIS; ++iter) {
      currElement = 0;
      
      duplicateFound = false;
      while (currElement < matchesFound && !duplicateFound ) {
        duplicateFound = axisSet == motorAxis[correspondingElements[currElement]];
        ++currElement;
      }
      if (!duplicateFound)
        break;
    }
    
    if (duplicateFound && axisSet > MAX_AXIS)
      return 0;                                             // No more motors can be controlled
    
    this->motorAxis[destElement] = axisSet;                  // Axis
    this->correspondingMotor[destElement] = motorSpecifier;   // Store the motor specifier which identifies the type of motor this instance controls
    ++this->numAxis[motorSpecifier];                        // Increment # of axis of this specific motor
    ++this->totalAxis;                                      // Increment total # of axis
  }

  void removeAxis(uint8_t axis) {
    uint8_t motorSpecifier = this->correspondingMotor[axis - 1];
    --this->totalAxis;                                          // Decrement total # of axis
    --this->numAxis[motorSpecifier];                            // Decrement # of axis of this specific motor
    this->motorAxis[axis - 1] = 0;                              // Clear axis specifier - no longer in use.
    this->correspondingMotor[axis - 1] = 0;                     // Clear corresponding motor - no longer in use.
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
      for (uint8_t iter = 0; iter < Enumerators::NUM_MOTOR_MODELS; ++iter)
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
      for (uint8_t iter = 0; iter < Enumerators::NUM_MOTOR_MODELS; ++iter)
        this->numAxis[iter] = dev1.numAxis[iter];
    }
    return *this;
  }

  void displayDeviceState(void) {
    if (!Serial)
      Serial.begin(LEONARDO_BAUD);
    Serial.print("Device State Printout\r\n");
    for (uint8_t iter = 0; iter < this->totalAxis; ++iter) {
      this->motors[iter]->statusPrintout();
    }
  }

  void loadMotors(uint8_t* correspondingMotors, uint8_t SIZE = 0) {
    if (!SIZE)
      return;
    else if (SIZE > 4)
      SIZE = 4;
    uint8_t motorSearch;
    uint8_t motorIndex;
    bool isFound, currIsFound;
    uint8_t index = 0;
    this->resetState();
    this->totalAxis = SIZE;
    for (uint8_t iter = 0; iter < SIZE; ++iter) {
      // Search for the 1st available index
      while (index < MAX_AXIS && this->motorAxis[index] > 0) {
        index = index + 1;
      }
      if (index >= MAX_AXIS) {
        Serial.print("No more available axis.\r\n");
        break;  // No more free elements where we can place this motor
      }
      this->correspondingMotor[index] = correspondingMotors[iter];
      
      if (this->numAxis[correspondingMotors[iter]] == 0) {
        this->uniqueMotorAxis[index] = 1; // This is the 1st instance of this motor type. Therefore it must be the first axis of its type.
      }
      else {
        motorIndex = 1;
        motorSearch = 0;
        isFound = true;
        // Search until we find
        while (isFound && motorIndex < MAX_AXIS) {
          // Need to identify specs from motor instance starting with axis 1 then 2, ...
          motorSearch = 0;
          currIsFound = false;
          while (!currIsFound && motorSearch < MAX_AXIS) {
            currIsFound = this->correspondingMotor[motorIndex] == correspondingMotors[iter] && motorIndex == this->uniqueMotorAxis[motorIndex];
            motorSearch = motorSearch + 1;
          }
          // An axis of this motor was found. Increment the counter and search for the next axis identifier
          if (currIsFound) {
            motorIndex = motorIndex + 1;
          }
          else {  // Ax axis of this motor of this 'motorIndex' was not found. It is the motor index relative to this motor instance
            isFound = false;
          }
        }
        if (!isFound) // Which means there is an empty element to place the motor instance
          this->uniqueMotorAxis[index] = motorIndex;
      }
      ++this->numAxis[correspondingMotors[iter]];   // Keep track of the axis for each axis of control under a single motor class object
      this->motorAxis[index] = index + 1;

      // Increment the unique motors counter when the 1st motor of the 'correspondingMotors[iter]' motor class is initialized.
      if (this->numAxis[correspondingMotors[iter]] == 1)
        ++this->numUniqueMotors;  
    }
    //this->uniqueMotorAxis = new uint8_t[this->numUniqueMotors];  //Needed for Dynamic Implementations
    for (uint8_t iter = 0; iter < SIZE; ++iter) {
      if (this->motorAxis[iter] == 1)
        this->uniqueMotorAxis[iter] = this->correspondingMotor[iter];
    }
  }

  ~Device() {

  }

  uint8_t getNumDimensions(void) {  return this->totalAxis;   }
};
#endif
