#include "Motor.h"

#if (NEMA17_SUPPORTED)
  //struct Motor_Nema17 nema17Motors[NEMA17_SIZE];
  struct PhysicalDynamics_Stepper *MotorDriver::nema17Driver[NEMA17_SIZE];
#endif
#if (STEPPER_ONLINE_SUPPORTED)
  struct PhysicalDynamics_Stepper *MotorDriver::stepperOnlineDriver[STEPPER_ONLINE_SIZE] = {};
#endif
#if (G2_SUPPORTED)
  struct PhysicalDynamics_G2 *MotorDriver::g2Driver[G2_SIZE] = {};    
 // struct Motor_G2 MotorDriver::g2Motors[G2_SIZE] = {Motor_G2(1, &MotorDriver::g2API), Motor_G2(2, &MotorDriver::g2API)};       // The G2 Motor class is designed for a chip capable of controlling 2-axis
#endif


// MOTOR DRIVER FUNCTIONS
void MotorDriver::resetStaticVars(void) {
  for (uint8_t motor = 0; motor < BoardInfo::NUM_MOTOR_MODELS; ++motor) {
    for (uint8_t axisIndex = 0; axisIndex < BoardInfo::supportedMotorsAxisSize[motor]; ++axisIndex) {
      #if (NEMA17_SUPPORTED)
        delete MotorDriver::nema17Driver[axisIndex];
        MotorDriver::nema17Driver[axisIndex] = nullptr;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        delete MotorDriver::stepperOnlineDriver[axisIndex];
        MotorDriver::stepperOnlineDriver[axisIndex] = nullptr;
      #endif
      #if (G2_SUPPORTED)
        delete MotorDriver::g2Driver[axisIndex];
        MotorDriver::g2Driver[axisIndex] = nullptr;
      #endif
    }
  }
}

void MotorDriver::allocateStaticVars(void) {
  // For each motor
  for (uint8_t correspondingMotor = 0; correspondingMotor < BoardInfo::NUM_MOTOR_MODELS; ++correspondingMotor) {
    // For each axis
    for (uint8_t axis = 1; axis <= BoardInfo::motorConfigsAxisSize[correspondingMotor]; ++axis) {
      MotorDriver::allocateObjVars(&correspondingMotor, &axis, 1);
    }
  }
}

void MotorDriver::allocateObjVars(uint8_t *motorModel, uint8_t *axis, uint8_t SIZE) {
    const uint8_t *axisSizePtr = BoardInfo::motorConfigsAxisSize;
    const uint8_t **csSizePtr = BoardInfo::motorConfigsAxisCSSize;
    uint8_t axisIndex;

    uint8_t *axisPtr, *motorModels, *csPtr;
    uint8_t csSet = 0;

    for (uint8_t element = 0; element < SIZE; ++element) {
      #if (DEBUGGER_OVERRIDE)
        if (Serial)
          Serial.print("Motor Model : ");Serial.print(motorModel[element]);Serial.print(", axis = ");Serial.println(axis[element]);
      #endif

      axisIndex = axis[element] - 1;  // overflow if axis[element] == 0 (thats fine)
      if (axisIndex >= axisSizePtr[motorModel[element]])
        continue;

      #if (DEBUGGER_OVERRIDE)
        if (Serial) {
          Serial.println("Allocating memory...");
          Serial.print("Motor : ");Serial.println(motorModel[element]);
          Serial.print("For size : ");Serial.println(csSizePtr[motorModel[element]][axisIndex]);
          Serial.print("CS Size -------> "); Serial.println(BoardInfo::motorConfigsAxisCSSize[motorModel[element]][axisIndex]);
        }
     #endif

      axisPtr = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[element]][axisIndex]];
      motorModels = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[element]][axisIndex]];
      csPtr = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[element]][axisIndex]];
      for (uint8_t cs = 0; cs < BoardInfo::motorConfigsAxisCSSize[motorModel[element]][axisIndex]; ++cs) {
        axisPtr[cs] = axis[element];
        motorModels[cs] = motorModel[element];
        csPtr[cs] = cs;         
      }

      switch (motorModel[element]) {
        // Check for nullptr to prevent run-away memory usage. For instance, parameter set may contain duplicate 'controllerAxisIndex' values.
        #if (NEMA17_SUPPORTED)
          case BoardInfo::NEMA17 :
            if (MotorDriver::nema17Driver[axisIndex] == nullptr) {
              MotorDriver::nema17Driver[axisIndex] = new PhysicalDynamics_Stepper(motorModels, axisPtr, csPtr, csSizePtr[motorModel[element]][axisIndex], csSet);
            }
          break;
        #endif
        #if (STEPPER_ONLINE_SUPPORTED)
          case BoardInfo::STEPPER_ONLINE : 
            if (MotorDriver::stepperOnlineDriver[axisIndex] == nullptr) {
              MotorDriver::stepperOnlineDriver[axisIndex] = new PhysicalDynamics_Stepper(motorModels, axisPtr, csPtr, csSizePtr[motorModel[element]][axisIndex], csSet);
            }
          break;
        #endif
        #if (G2_SUPPORTED)
          case BoardInfo::G2 :
            if (MotorDriver::g2Driver[axisIndex] == nullptr) {
              MotorDriver::g2Driver[axisIndex] = new PhysicalDynamics_G2(motorModels, axisPtr, csPtr, csSizePtr[motorModel[element]][axisIndex], csSet);
            }
          break;
        #endif
      }

      // Free dynamic memory
      delete[] axisPtr;
      delete[] motorModels;
      delete[] csPtr;
    }
  }



void MotorDriver::initializeStaticVar(uint8_t motorModel, uint8_t axis) {
  uint8_t axisIndex = axis - 1;
  if (motorModel >= BoardInfo::NUM_MOTOR_MODELS || axisIndex >= BoardInfo::motorConfigsAxisSize[motorModel])
    return;
  
  #if (NEMA17_SUPPORTED)
    if (MotorDriver::nema17Driver[axisIndex] != nullptr)
      *MotorDriver::nema17Driver[axisIndex] = PhysicalDynamics_Stepper(&motorModel, &axis, &BoardInfo::motorConfigsAxisCSSize[motorModel][axisIndex], 1, 0);
  #endif
  #if (STEPPER_ONLINE_SUPPORTED)
    if (MotorDriver::stepperOnlineDriver[axisIndex] != nullptr)
      *MotorDriver::stepperOnlineDriver[axisIndex] = PhysicalDynamics_Stepper(&motorModel, &axis, &(BoardInfo::motorConfigsAxisCSSize[motorModel][axisIndex]), 1, 0);
  #endif
  #if (G2_SUPPORTED)
    if (MotorDriver::g2Driver[axisIndex] != nullptr)
      *MotorDriver::g2Driver[axisIndex] = PhysicalDynamics_G2(&motorModel, &axis, &(BoardInfo::motorConfigsAxisCSSize[motorModel][axisIndex]), 1, 0);
    //for (uint8_t iter = 0; iter < NEMA17_SIZE; ++iter) {
    //  MotorDriver::g2Driver[iter].selectMotor(BoardInfo::NEMA17);  // Set the motor type
    // }
    //struct G2_Driver g2Driver;    // Using default constructor
    //struct Motor_G2 g2Motors[G2_SIZE] = {Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)};          // The G2 Motor class is designed for a chip capable of controlling 2-axis
  #endif
}

void MotorDriver::linkDeviceDriver(uint8_t motorModel, uint8_t axis) {
  switch (motorModel) {   // There is at least one motor of the current motor class
    #if NEMA17_SUPPORTED
      case BoardInfo::NEMA17 :
        this->deviceDriver = static_cast<DeviceDynamics*>(MotorDriver::nema17Driver[axis - 1]);
      break;
    #endif
    #if STEPPER_ONLINE_SUPPORTED
      case BoardInfo::STEPPER_ONLINE :
        this->deviceDriver = static_cast<DeviceDynamics*>(MotorDriver::stepperOnlineDriver[axis - 1]);
      //this->testPtr = MotorDriver::stepperOnlineDriver[axis - 1];
      //this->currState = static_cast<StepperMotorState*>(this->primitiveState);
      break;
    #endif

    #if G2_SUPPORTED
      case BoardInfo::G2 :
        this->deviceDriver = static_cast<DeviceDynamics*>(MotorDriver::g2Driver[axis - 1]);
      break;
    #endif
    default:
      this->deviceDriver = nullptr;
      break;
  }
}

// MOTOR FUNCTIONS

void Motor::setCS(uint8_t cs) {
  this->controlCS = cs;
  this->driver.deviceDriver->setCS(cs);
}

bool Motor::isValid(int8_t controlCS) {
  // We can add more logic in future for more robust error checking.
  return controlCS >= 0 && controlCS < this->motorSIZE;
}

void Motor::statusPrintout(void) {
  #if (DEBUGGER_OVERRIDE)
    if (Serial) {
      //Serial.print("Corresponding model : ");           Serial.print(this->driver.deviceDriver->motorConnection)->getCorrespondingMotor());           Serial.print("\r\n"); // Terminate the line
      Serial.print("Generic Dynamics initialized : ");  Serial.print((uint8_t)(this->driver.deviceDriver != nullptr));  Serial.print("\r\n"); // Terminate the line
      
    }
  #endif
}

float Motor::getPositionRate(uint8_t derivative) {
  if (this->driver.deviceDriver == nullptr)  {
    //Serial.println("NEEDS ALLOCATED!!! STATE HELD HERE, see Motor::getPositionRate().");
    while(1);
  }
  else 
    return this->driver.deviceDriver->getPositionRate(derivative);
}

void Motor::setPositionRate(float desiredState, uint8_t derivative) {
  this->driver.deviceDriver->setPositionRate(desiredState, derivative);
}

void Motor::move(float delta) {
  this->driver.deviceDriver->move(delta);
}

// Works for any stepper motors (INVOKES THE BASE CLASS FCN OF PHYSICAL DYNAMICS)
uint32_t Motor_Stepper::computeOverallStep(float rotorAngle) {
  // Perform the multiplication by the angle last to prevent overflow errors for large angles.
  return (long)(((float)this->driver.deviceDriver->getTotalStates() / 360.0) * rotorAngle);
}





#if NEMA17_SUPPORTED
void Motor_Nema17::initializeDevice(uint8_t SIZE) {

  for (uint8_t i = 0; i < SIZE; ++i) {
      // Initialize motor parameters
    this[i].driver.motorConnection->selectPinout(*(this[i].axis));                         // Set real-time vars for each motor dynamics
    this[i].tmcDriver = new TMC2130Stepper(this[i].driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]]);
    this[i].driver.motorConnection->begin((uint8_t)BoardInfo::NEMA17);                 // Define pinout for their respective connections (input/output etc.)
    this[i].driver.enable = false;

    

    this[i].stepper = AccelStepper(this[i].stepper.DRIVER, this[i].driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::PUL]], this[i].driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::DIR]]);

    this[i].tmcDriver->begin();                                          // Initiate pins and registeries
    this[i].tmcDriver->rms_current(this[i].driver.deviceDriver->getCurrent());  // Set stepper current
    
    this[i].tmcDriver->stealthChop(1);
    this[i].tmcDriver->stealth_autoscale(1);                             // Enable quiet stepping (smoother)
    
    /*
    if (Serial) {
      Serial.print("Microsteps:");  Serial.println(this[i].driver.deviceDriver->getMicroSteps());
      Serial.print("Max Speed:");  Serial.println(this[i].driver.deviceDriver->getPositionRate(1));
      Serial.print("Acceleration:");  Serial.println(this[i].driver.deviceDriver->getPositionRate(2));
    }
    */

    this[i].tmcDriver->microsteps(this[i].driver.deviceDriver->getMicroSteps());  // Set cumulative step size
    digitalWrite(this[i].driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], LOW);   // Set CS pin to LOW
    digitalWrite(this[i].driver.motorConnection->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], LOW);   // Set CS pin to LOW

    this[i].stepper.setMaxSpeed(this[i].driver.deviceDriver->getPositionRate(1));       // Set stepper speed
    this[i].stepper.setAcceleration(this[i].driver.deviceDriver->getPositionRate(2));   // Set stepper acceleration
  }
}

void Motor_Nema17::originSearch(void) {  
  this->setPosition(0);   // No feedback is possible. Therefore, our only possible origin is the previously defined origin.
}



#endif

#if STEPPER_ONLINE_SUPPORTED

//static struct PhysicalDynamics_Stepper MotorDriver::stepperOnlineDriver = {};

//MotorDriverPhysicalDynamics_Stepper stepperOnlineDriver[STEPPER_ONLINE_SIZE];


//struct PhysicalDynamics_Stepper MotorDriver::stepperOnlineDriver[] = { PhysicalDynamics_Stepper() };

/*
void Motor_StepperOnline::initializeDevice(void) {
  //this[currMotor].driver.deviceDriver->motorConnection->selectPinout(*(this[currMotor].axis));          // Set real-time vars for each motor dynamics
  for (uint8_t cs = 0; cs < this->motorSIZE; ++cs) {
    this->setCS(cs);
  }
}


void Motor_StepperOnline::initializeDevice(uint8_t *csList, uint8_t SIZE) {
  
  // see pysical dynamics.initializedevice()
}
*/

void Motor_StepperOnline::originSearch(void) {  
  this->driver.deviceDriver->setPosition(0);    // No feedback is possible. Therefore, our only possible origin is the previously defined origin.
}

#endif

#if G2_SUPPORTED



void intializeAxis(uint8_t axis) {
  
}







#endif