#include "PhysicalDynamics.h"

#if (NEMA17_SUPPORTED)
  //struct Motor_Nema17 nema17Motors[NEMA17_SIZE];
  struct ComponentConnection DeviceDynamics::nema17Connection[NEMA17_SIZE] = {};  // Initialize to zero by default
  struct StepperMotorState* DeviceDynamics::nema17State[NEMA17_SIZE] = {};
#endif
#if (STEPPER_ONLINE_SUPPORTED)
  struct ComponentConnection DeviceDynamics::stepperOnlineConnection[STEPPER_ONLINE_SIZE] = {};  // Initialize to zero by default
  struct StepperMotorState* DeviceDynamics::stepperOnlineState[STEPPER_ONLINE_SIZE] = {};               // Current state for each CS line corresponding to a single comms bus
#endif
#if (G2_SUPPORTED)
  struct ComponentConnection DeviceDynamics::g2Connection[G2_SIZE] = {};  // Initialize to zero by default
  struct ActuatorState* DeviceDynamics::g2State[G2_SIZE] = {};               // Current state for each CS line corresponding to a single comms bus  

/*
  struct DualG2HighPowerMotorShield24v14 PhysicalDynamics_G2::g2API(BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::SLEEP]], BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::DIR]], BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::PWM]],
                                                            BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::FAULT]], BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::CS]], BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::SLEEP]],
                                                            BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::DIR]], BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::PWM]], BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::FAULT]],
                                                            BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::CS]]);
  */                                              
  bool PhysicalDynamics_G2::g2API_initialized = false;          
  struct DualG2HighPowerMotorShield24v14 PhysicalDynamics_G2::g2API(1, 2, A8, A5, A9, 3, 4, A1, A4, A0);  // Precision Plunge V1.0 Pinout
  //struct DualG2HighPowerMotorShield24v14 PhysicalDynamics_G2::g2API(1, 2, A1, A4, A0, 3, 4, A8, A5, A9);  // Precision Plunge V2.0 Pinout

  // struct PhysicalDynamics_Stepper MotorDriver::g2Motors[G2_SIZE] = {PhysicalDynamics_Stepper(1, &MotorDriver::g2API), PhysicalDynamics_Stepper(2, &MotorDriver::g2API)};       // The G2 Motor class is designed for a chip capable of controlling 2-axis


  // Initialize G2 Chip State
  bool PhysicalDynamics_G2::g2Calibrated[2] = { false, false };
  bool PhysicalDynamics_G2::g2Direction[2] = { true, true };                               // Actuator direction of travel 
  int32_t PhysicalDynamics_G2::g2Steps[2] = { 0, 0 };                                   // Actuator overall steps
  volatile int32_t PhysicalDynamics_G2::unaccountedSteps[2] = { 0, 0 };     // Steps accumulated in motor ISR awaiting transfer to overall steps var, 'g2Steps'. Use 'g2Direction' to determine direction   
  volatile bool PhysicalDynamics_G2::hallSensorPhase[4] = { false, false, false, false };                     // phase of the hall effect sensors (2 sensors per motor - with sensors 90 degrees out of phase producing a square wave)

#endif

// Allocate ALL static vars
void DeviceDynamics::resetStaticVars(void) {
  for (uint8_t motor = 0; motor < BoardInfo::NUM_MOTOR_MODELS; ++motor) {
    for (uint8_t axisIndex = 0; axisIndex < BoardInfo::supportedMotorsAxisSize[motor]; ++axisIndex) {
      #if (NEMA17_SUPPORTED)
        delete DeviceDynamics::nema17State[axisIndex];
        DeviceDynamics::nema17State[axisIndex] = nullptr;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        delete DeviceDynamics::stepperOnlineState[axisIndex];
        DeviceDynamics::stepperOnlineState[axisIndex] = nullptr;
      #endif
      #if (G2_SUPPORTED)
        delete DeviceDynamics::g2State[axisIndex];
        DeviceDynamics::g2State[axisIndex] = nullptr;
      #endif
    }
  }
}

void DeviceDynamics::allocateStaticVars(void) {
  // For each motor
  for (uint8_t correspondingMotor = 0; correspondingMotor < BoardInfo::NUM_MOTOR_MODELS; ++correspondingMotor) {
    // For each axis
    for (uint8_t axis = 1; axis <= BoardInfo::motorConfigsAxisSize[correspondingMotor]; ++axis) {
      DeviceDynamics::allocateStaticVars((const uint8_t*)&correspondingMotor,(const uint8_t*) &axis, 1);
    }
  }
}

void DeviceDynamics::allocateStaticVars(const uint8_t *motorModel, const uint8_t *axis, uint8_t SIZE) {
    const uint8_t *axisSizePtr = BoardInfo::motorConfigsAxisSize;
    const uint8_t** csSizePtr = BoardInfo::motorConfigsAxisCSSize;
    uint8_t axisIndex;

    for (uint8_t element = 0; element < SIZE; ++element) {
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Motor Model : ");Serial.print(motorModel[element]);Serial.print(", axis = ");Serial.println(axis[element]);
      #endif

      axisIndex = axis[element] - 1;  // overflow if axis[element] == 0 (thats fine)
      if (motorModel[element] >= BoardInfo::NUM_MOTOR_MODELS || axisIndex >= axisSizePtr[motorModel[element]])
        continue;

      #if (DEBUGGER_OVERRIDE)
        Serial.println("Allocating memory...");
        Serial.print("Motor : ");Serial.println(motorModel[element]);
        Serial.print("For size : ");Serial.println(csSizePtr[motorModel[element]][axisIndex]);
      #endif

      switch (motorModel[element]) {
        // Check for nullptr to prevent run-away memory usage. For instance, parameter set may contain duplicate 'controllerAxisIndex' values.
        #if (NEMA17_SUPPORTED)
          case BoardInfo::NEMA17 :
            if (DeviceDynamics::nema17Connection == nullptr) {
             // DeviceDynamics::nema17Connection = new ComponentConnection[NEMA17_SIZE];
              myPtr = DeviceDynamics::nema17Connection;
            }

            if (DeviceDynamics::nema17State[axisIndex] == nullptr)
              DeviceDynamics::nema17State[axisIndex] = new StepperMotorState(BoardInfo::NEMA17);
          break;
        #endif
        #if (STEPPER_ONLINE_SUPPORTED)
          case BoardInfo::STEPPER_ONLINE : 
            if (DeviceDynamics::stepperOnlineConnection == nullptr) {
              //  DeviceDynamics::stepperOnlineConnection = new ComponentConnection[STEPPER_ONLINE_SIZE]; // This line is for when 'stepperOnlineConnection' is pointer '*' instead of array '[]'
              //myPtr = DeviceDynamics::stepperOnlineConnection;
            }
            
            if (DeviceDynamics::stepperOnlineState[axisIndex] == nullptr) {
              DeviceDynamics::stepperOnlineState[axisIndex] = new StepperMotorState[csSizePtr[motorModel[element]][axisIndex]];
              for (uint8_t cs = 0; cs < csSizePtr[motorModel[element]][axisIndex]; ++cs)
                DeviceDynamics::stepperOnlineState[axisIndex][cs] = StepperMotorState(BoardInfo::STEPPER_ONLINE);
            }
          break;
        #endif
        #if (G2_SUPPORTED)
          case BoardInfo::G2 : 
          if (DeviceDynamics::g2State[axisIndex] == nullptr) {
              DeviceDynamics::g2State[axisIndex] = new ActuatorState[csSizePtr[motorModel[element]][axisIndex]];
              for (uint8_t cs = 0; cs < csSizePtr[motorModel[element]][axisIndex]; ++cs)
                DeviceDynamics::g2State[axisIndex][cs] = ActuatorState(BoardInfo::G2);
          }
          break;
        #endif
      }
    }

    // Step 2 : Initialize Vars
    DeviceDynamics::initializeStaticVars(motorModel, axis, SIZE);
  }

  // Initializes static vars as a function of the parameters
    void DeviceDynamics::initializeStaticVars(const uint8_t *motorModel, const uint8_t *axis, const uint8_t SIZE) {
      uint8_t axisIndex;
      struct MotorState *myPtr; 
      struct ComponentConnection *motorConnection;
      uint8_t *motorModels, *motorAxisList, *motorCSList;
      bool *axisInitialized = new bool[SIZE];
      for (uint8_t index = 0; index < SIZE; ++index)
        axisInitialized[index] = false;

      for (uint8_t index = 0; index < SIZE; ++index) {
        if (axisInitialized[index])
          continue; // only initialize motorModel-axis once

         axisIndex  = axis[index] - 1;
        if (motorModel[index] >= BoardInfo::NUM_MOTOR_MODELS || axis[index] > BoardInfo::motorConfigsAxisSize[motorModel[index]])
          continue;
        
        myPtr = DeviceDynamics::fetchMotorState(motorModel[index], axis[index]);                                    // Fetches the static dynamically allocated var
        motorConnection = DeviceDynamics::fetchComponentConnection(motorModel[index], axis[index]);        // Points to a static private MotorDriver ComponentConnection instance

        if (myPtr == nullptr || motorConnection == nullptr)
          continue; // memory is not allocated. Do not allocate now because it was not explicitly requested.

        if (BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex] > 0) {
          // allocate connection elements (each cs has its own specs to store: motorModels, motorAxis, wherein the motorModels can be unique) in connection instance.
          motorModels = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex]];
          motorAxisList = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex]];
          motorCSList = new uint8_t[BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex]];

          for (uint8_t cs = 0; cs < BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex]; ++cs) {
            myPtr[cs].motorAtOrigin = true;
            motorModels[cs] = motorModel[index];   // Assume each cs is for the same motor model
            motorAxisList[cs] = axis[index];
            motorCSList[cs] = cs;         // Assume each cs is connected to the same controller axis -> comms is over the same bus -> each cs is a unique integer

            //  Flag future parameter indexes as 'processed' if their motor exists on the same comms bus (hence they share the same connection)
            for (uint8_t testIndex = index + 1; testIndex < SIZE; ++testIndex) 
              axisInitialized[testIndex] = motorModel[index] == motorModel[testIndex] && axis[index] == axis[testIndex]; // If they have the same motor model and are on the same axis, then it will not need motor connection specified again

          }
          
          motorConnection->specifyConnection(motorModels, motorAxisList, motorCSList, BoardInfo::motorConfigsAxisCSSize[motorModel[index]][axisIndex]);
          delete[] motorModels;
          delete[] motorAxisList;
          delete[] motorCSList;
        }
      }
      delete[] axisInitialized;
    }
  
  struct MotorState* DeviceDynamics::fetchMotorState(const uint8_t motorModel, const uint8_t axis) {
    switch (motorModel) {
      #if (NEMA17_SUPPORTED)
        case BoardInfo::NEMA17
          return (DeviceDynamics::nema17State == nullptr) ? nullptr : DeviceDynamics::nema17State[axis - 1];
          break;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        case BoardInfo::STEPPER_ONLINE : 
          return (DeviceDynamics::stepperOnlineState[axis - 1] == nullptr) ? nullptr : DeviceDynamics::stepperOnlineState[axis - 1];
          break; 
        break;
      #endif
      #if (G2_SUPPORTED)
        case BoardInfo::G2 : 
          return (DeviceDynamics::g2State[axis - 1] == nullptr) ? nullptr : DeviceDynamics::g2State[axis - 1];
          break;
      #endif
    }
  }

  struct ComponentConnection*  DeviceDynamics::fetchComponentConnection(const uint8_t motorModel, const uint8_t axis) {
    switch (motorModel) {
      #if (NEMA17_SUPPORTED)
        case BoardInfo::NEMA17 : 
          return (DeviceDynamics::nema17Connection == nullptr) ? nullptr : &DeviceDynamics::nema17Connection[axis - 1];
          break;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        case BoardInfo::STEPPER_ONLINE : 
          return (DeviceDynamics::stepperOnlineConnection == nullptr) ? nullptr : &DeviceDynamics::stepperOnlineConnection[axis - 1];
          break;
      #endif
      #if (G2_SUPPORTED)
        case BoardInfo::G2 : 
          return (DeviceDynamics::g2Connection == nullptr) ? nullptr : &DeviceDynamics::g2Connection[axis - 1];
          break;
      #endif
    }
  }

  // OBJ Var Functions
  void DeviceDynamics::resetState(void) {
    // Free dynamic memory
    // Pointers are no longer valid, set to nullptr
    this->deallocateObjVars();
    this->direction = true;
    this->cs = 0;
    for (uint8_t cs = 0; cs < this->csSize; ++cs) {
      this->primitiveState[cs]->resetState();
    }
  }

  // (FAST) - Deallocate ALL dynamic memory
  void DeviceDynamics::deallocateObjVars(void) {
    // Free Dynamic Memory
    if (this->csSize > 0) {
      delete[] this->primitiveState;
      delete[] this->motorConnection;
      delete[] this->connectionCS;
      delete[] this->connectionAxis;
    }
    this->primitiveState = nullptr;
    this->motorConnection = nullptr;
    this->connectionCS = nullptr;
    this->connectionAxis = nullptr;
    this->csSize = 0;
  }

  void DeviceDynamics::allocateObjVars(uint8_t SIZE) {
    // Check if obj needs to release memory for the previous csSize
    if (this->csSize != 0)
      return; // Must release memory first
      
    // Prevent runaway memory usage - check for null
    if (this->primitiveState == nullptr) {
        this->primitiveState = new MotorState*[SIZE];
    }
    
    if (this->motorConnection == nullptr)
      this->motorConnection = new ComponentConnection*[SIZE];
    if (this->connectionCS == nullptr)
      this->connectionCS = new uint8_t[SIZE];
    if (this->connectionAxis == nullptr)
      this->connectionAxis = new uint8_t[SIZE];

    for (uint8_t index = 0; index < SIZE; ++index) {
      this->primitiveState[index] = nullptr;
      this->motorConnection[index] = nullptr;
    }

    this->csSize = SIZE;
  }

  void DeviceDynamics::linkObjPtrs(const uint8_t *motorModel, const uint8_t *motorAxis, const uint8_t *motorCS, const uint8_t SIZE) {
      uint8_t axisIndex;
      
      for (uint8_t index = 0; index < SIZE; ++index) {
        axisIndex = motorAxis[index] - 1;

        switch (motorModel[index]) {   // There is at least one motor of the current motor class
          #if NEMA17_SUPPORTED
            case BoardInfo::NEMA17 :
              if (axisIndex < NEMA17_SIZE) {
                this->motorConnection[index] = &DeviceDynamics::nema17Connection[axisIndex];
                this->primitiveState[index] = &DeviceDynamics::nema17State[axisIndex][motorCS[index]];
              }
              break;
          #endif
          #if STEPPER_ONLINE_SUPPORTED
            case BoardInfo::STEPPER_ONLINE :
              if (axisIndex < STEPPER_ONLINE_SIZE) {
                this->motorConnection[index] = &DeviceDynamics::stepperOnlineConnection[axisIndex];    // this is original instance. assigned address will not be nullptr
                this->primitiveState[index] = &DeviceDynamics::stepperOnlineState[axisIndex][motorCS[index]];
              }
              break;
          #endif
          #if G2_SUPPORTED
            case BoardInfo::G2 :
              if (axisIndex < G2_SIZE) {
                this->motorConnection[index] = &this->g2Connection[axisIndex];
                this->primitiveState[index] = &DeviceDynamics::g2State[axisIndex][motorCS[index]];
              }
              break;
          #endif
          default:
            this->primitiveState[index] = nullptr;
            this->motorConnection[index] = nullptr;
          break;
        }
      }
    }

    void DeviceDynamics::setCS(uint8_t cs) {
      this->cs = cs;
      this->motorConnection[this->connectionAxis[this->cs]]->selectPinout(this->primitiveState[this->cs]->correspondingMotor, cs + 1);    // Optimal Method
    }

    void DeviceDynamics::setDynamics(uint16_t current, uint16_t pulseRate, uint32_t acceleration) {
      this->primitiveState[this->cs]->pulseRate = pulseRate;                    // [Stepper Motors] : Steps per second || [Actuators] : mm per second
      this->primitiveState[this->cs]->acceleration = acceleration;
      this->current = current;                                // Motor current
    }

    // Virtual Functions
    
    void DeviceDynamics::enableMotor(bool toEnable) {
      this->primitiveState[this->cs]->enable = toEnable;

    };   

    // ------ END OF DEVICE DYNAMICS CLASS FUNCTION DEFINITIONS ------ \\


    void PhysicalDynamics_Stepper::initializeDevice(void) {
      // For each cs of the motor obj axis
      for (uint8_t cs = 0; cs < this->csSize; ++cs) {
        this->cs = cs;
        this->motorConnection[cs]->selectPinout(this->connectionCS[cs]);    // Set pointers to address of our motor specs
        this->motorConnection[cs]->begin();                                 // Perform startup routine for this cs
        this->primitiveState[cs]->enable = false;                           // Motor turned off in motorConnection->begin()
        this->syncMotorDynamics();                                          // Update obj motor dynamics for the motorModel specified by this->primitiveState[this->cs].motorModel
        this->setPositionRate(0.1, 1);                                     //  Set frequency to 0.5 Hz           Set initial angular velocity to 1.57 rad/s
        this->initializeCS(cs);
      }
    }

    void PhysicalDynamics_Stepper::initializeCS(uint8_t cs) {
      // Call the base class's functiond
      //this->DeviceDynamics::initializeCS(cs);
      //DeviceDynamics::initializeCS(cs);        // CANNOT INVOKE, function is pure virtual function -> it cannot be called and must be invoked via a derived override function
      this->currState[cs]->relativeStep = 0;
      this->currState[cs]->relativeState = 0;
    }

    float PhysicalDynamics_Stepper::getRateLimit(uint8_t derivative) {
      switch (derivative) {
        case 0:     // Position

        break;
        case 1:      // Velocity : Hz
          return this->currState[this->cs]->getMaxDutyCycle() / (100 * this->getTotalStates() * this->currState[this->cs]->getMinPulseDuration()); // 100 is used to normalize duty cycle
      }
    }

    void PhysicalDynamics_Stepper::setDutyCycle(uint8_t dutyCycle) {
      this->currState[this->cs]->dutyCycle = dutyCycle <= 100 ? dutyCycle : 50;
    }

    void PhysicalDynamics_Stepper::setPositionRate(double rate, uint8_t derivative) {
      double rateLimit;
      switch (derivative) {
        case 0:     // Position (0)
          this->setPosition(rate);
          break;
        case 1:     // (Velocity : Hz)
          // Input validation
          rateLimit = this->getRateLimit(1);

        //  Serial.print("rate limit : ");Serial.println(rateLimit);
        //  Serial.print("set rate : ");Serial.print(rate);

          this->primitiveState[this->cs]->frequency = rate > rateLimit ? (rateLimit > 0 ? rateLimit : 1) : (rate > 0 ? rate : 1);       // Limit maximum speed because they are not achievable with our hardware.
          this->currState[this->cs]->cycleDuration = (1/(this->getTotalStates() * this->primitiveState[this->cs]->frequency)) * pow(10, 6); // (Duration in us)
          this->currState[this->cs]->cycleDurationExponent = -6;    // Duration in us
       //   Serial.print("Total states : ");Serial.println(this->getTotalStates());
        // Serial.print("Cycle Duration : ");Serial.print(this->currState[this->cs]->cycleDuration);Serial.print(" x 10^");Serial.println(this->currState[this->cs]->cycleDurationExponent);
          break;
        case 2:     // (Acceleration)
          this->currState[this->cs]->acceleration = rate;
        break;
      }
    }

    float PhysicalDynamics_Stepper::getPositionRate(uint8_t derivative) {
      switch (derivative) {
        case 0:   // (Position) : Current State
          return this->getPosition();
        case 1:     // (Velocity)
          return this->primitiveState[this->cs]->frequency;
        break;
        case 2:     // (Acceleration)
          return this->currState[this->cs]->acceleration;
        break;
        default:
          return 0;
      }
    }

    int32_t PhysicalDynamics_Stepper::computeRelativeStep(double desiredState) {
      return (desiredState * this->getTotalStates() / 360) + desiredState > 0 ? 0.5 : -0.5;   // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here;
    }

    uint32_t PhysicalDynamics_Stepper::computeShortestPath(double desiredState) {
      float N = abs(desiredState / 360);
      float eqDesiredPos = desiredState + ((desiredState < 0) ? ceil(N) : -floor(N)) * 360;             //      0 <=   eqDesiredPos  <= 360
      //  float currPos = this->getPosition();                                                              //      0 <=   currPos       <= 360
      uint32_t stepsPerRev = this->getTotalStates();                                                    // # of steps to perform a complete rotation
      uint32_t finalRelativeStep = 0.5 /* moves us to closest integer */ + (eqDesiredPos * stepsPerRev / 360);  // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here;
      return finalRelativeStep;
    }

    int32_t PhysicalDynamics_Stepper::computeShortestPath(int32_t moveSteps) {
      int32_t stepsPerRev = (int32_t)this->getTotalStates();                                                    // # of steps to perform a complete rotation
      float N = abs(moveSteps / stepsPerRev);
      int32_t eqStepsToMove = moveSteps + ((moveSteps < 0) ? ceil(N) : -floor(N)) * stepsPerRev;        // 0 <= eqStepsToMove <= stepsPerRev
      return eqStepsToMove;
    }
 
    int32_t PhysicalDynamics_Stepper::computeNumSteps(double desiredState) {
      return this->computeRelativeStep(desiredState);                           // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here
      
      //uint32_t finalRelativeStep = this->computeRelativeStep(desiredState);                           // Only integer micro-steps can be performed. As a result, we need to choose the closest discrete step as is done here
      //return this->computeNumSteps(finalRelativeStep);
    }

    // Overloaded function for computing # steps
    int32_t PhysicalDynamics_Stepper::computeNumSteps(uint32_t finalRelativeStep) {
      int32_t stepsPerRev = (int32_t)this->getTotalStates();       // # of steps to perform a complete rotation
      uint32_t directPathNumSteps = abs((int32_t)finalRelativeStep - (int32_t)this->currState[this->cs]->relativeStep);
      
      bool directPath = directPathNumSteps < (stepsPerRev / 2.0);
      int32_t numSteps = finalRelativeStep - this->currState[this->cs]->relativeStep;
      if (!directPath) {
        // numSteps = -(((finalRelativeStep < (int32_t)this->relativeStep) ? -stepsPerRev : stepsPerRev) - numSteps);    // Works excellently
        numSteps = ((finalRelativeStep < this->currState[this->cs]->relativeStep) ? stepsPerRev : -stepsPerRev) + numSteps;
        //numSteps = stepsPerRev + ((finalRelativeStep < (int32_t)this->relativeStep) ? -numSteps : numSteps); //1 : -1) * abs(stepsPerRev - (int32_t)this->relativeStep + (int32_t)finalRelativeStep);
      }
      return numSteps;
    }

void PhysicalDynamics_Stepper::setDynamics(uint16_t current, uint32_t acceleration, uint8_t dutyCycle, float frequency, uint16_t stepsPerRev, uint16_t microsteps, float gearRatio) {
  // State Management
  this->current = current;
  this->currState[this->cs]->stepsPerCycle = stepsPerRev;                       // Steps per revolution (360 / dx)
  this->currState[this->cs]->microsteps = microsteps;                           // Microsteps per step
  this->currState[this->cs]->gearRatio = gearRatio;                             // Gear Ratio of stepper motor

  // Pulse Timing
  this->currState[this->cs]->dutyCycle = dutyCycle;                             // Set pulse duration duty cycle
  
  // High Dependencies assignments
  this->setPositionRate((uint16_t)(frequency), 1);                              // Set the speed (Cycle duration will be set)
  this->setPositionRate(acceleration, 2);                                       // Set the speed (Cycle duration will be set)
}

void PhysicalDynamics_Stepper::setCS(uint8_t cs) {
  this->cs = cs;

  // Set CS line and EN line
  this->motorConnection[this->cs]->setCS(this->connectionCS[this->cs]);         // Set CS on motor connection obj (updates the pinout)
  uint8_t *pinout = this->motorConnection[this->cs]->getPinout();

  switch (this->currState[this->cs]->correspondingMotor) {
    #if NEMA17_SUPPORTED

    #endif
    #if STEPPER_ONLINE_SUPPORTED
      case BoardInfo::STEPPER_ONLINE :   

        digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);        // Disable asynchronous state changes (added for reliability)
        delayMicroseconds(1);                                                           // Give time for gates to update (>59ns)

        // Set CS Line Voltage
        switch (this->connectionCS[this->cs]) {
          case 0:
            digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]], LOW);     // Select CS Line 0, setup time (>36ns)
          break;
          case 1:
            digitalWrite(pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]], HIGH);    // Select CS Line 0, setup time (>36ns)
          break;
        }
      break;
    #endif
    #if G2_SUPPORTED
      case BoardInfo::G2 :
        Serial.println(*pinout);  // TODO : do something with pinout : this cmd is performed to remove compiler warnings
        break;
    #endif
  }
}

void PhysicalDynamics_Stepper::setMicroSteps(uint16_t microsteps) {
  this->currState[this->cs]->microsteps = microsteps;
}

uint32_t PhysicalDynamics_Stepper::getTotalStates(void) {
  return this->currState[this->cs]->stepsPerCycle * this->currState[this->cs]->microsteps * this->currState[this->cs]->gearRatio;  // Total states of the stepper motor for each complete revolution.
}

void PhysicalDynamics_Stepper::selectMotor(uint8_t correspondingMotor) {
  this->primitiveState[this->cs]->correspondingMotor = correspondingMotor;
  this->syncMotorDynamics();
}

void PhysicalDynamics_Stepper::syncMotorDynamics(void) {
  uint8_t correspondingMotor = this->primitiveState[this->cs]->correspondingMotor;

  // Static Vars
  uint8_t maxDutyCycle;
  uint32_t minPulseDuration;
  int8_t minPulseDurationExponent;

  // Dynamic Vars
  uint16_t current;               // Stepper strength [mA]
  uint32_t acceleration;          // Stepper Acceleration [Microsteps per second^2]
  uint8_t dutyCycle;
  float frequency;
  uint16_t stepsPerRev;           // Stepper Motor resolution [Steps/rev] (typically 200)
  uint16_t microsteps;            // Driver step size [Microsteps/full step]
  float gearRatio;                // Ratio of Drive Gear teeth to Output Gear teeth

  // uint16_t speed;                 // Stepper speed [Microsteps per second] (4k max for ATmega328 @16MHz)

  switch (correspondingMotor) {
    #if (NEMA17_SUPPORTED)
      case BoardInfo::NEMA17:
        stepsPerRev = 200;            // Stepper Motor resolution [Steps/rev] (typically 200)
        microsteps = 128;             // Driver step size [Microsteps/full step]
        frequency = 0.5;
     //   speed = 4000;                 // Stepper speed [Microsteps per second] (4k max for ATmega328 @16MHz)
        acceleration = 40000;         // Stepper Acceleration [Microsteps per second^2]
        gearRatio = 2;                // Number of rotations of drive for each rotation of output
        pulseDuration = 0;            // Handled by api
        pulseDurationExponent = 0;    // Handled by api
        dutyCycle = 50;            // Handled by api
        current = 700;                // [mA]
      break;
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      case BoardInfo::STEPPER_ONLINE:
        acceleration = 0;             // Stepper Acceleration [Microsteps per second^2] (unknown quantity - managed by a 3rd party stepper motor driver)
        current = 0;                  // handled by driver

        
        // Testing with LED
        stepsPerRev = 1;
        microsteps = 1;
        gearRatio = 1;
        minPulseDuration = 50;
        minPulseDurationExponent = -3;
        //pulseDuration = 1000000;
        //pulseDurationExponent = -6;
        
        
        // Stepper Motor Specs
        // Static Specs
        maxDutyCycle = 50;
        /*
        minPulseDuration = 20;            // At least 2.5us is recommended (Provide extra 20us for pulse shortening for synchronization purposes)
        minPulseDurationExponent = -6;    // Pulse duration in microseconds
        

        // Dynamic Specs
        stepsPerRev = 200;            // Stepper Motor resolution [Steps/rev] (typically 200)
        microsteps = 128;             // Driver step size [Microsteps/full step]
        gearRatio = 7.793203125;             // Number of rotations of drive for each rotation of output
        */
        dutyCycle = 50;               // 50% duty cycle is recommended for motion smoothing
        frequency = 0.1;             // Stepper speed [Hz]
      break;
    #endif
    default:
      // Static Specs
      minPulseDuration = 1;            // 
      minPulseDurationExponent = -6;   // Cycle duration in microseconds
      maxDutyCycle = 50;

      // Dynamic Specs    
      current = 0;
      acceleration = 0;
      dutyCycle = 0;
      frequency = 0;
      stepsPerRev = 0 - 1;  // OVERFLOW (forces maximum value, closest to inf)
      microsteps = 0 - 1;   // OVERFLOW (forces maximum value, closest to inf)
      gearRatio = 0;
  }
  this->currState[this->cs]->setStatics(minPulseDuration, minPulseDurationExponent, maxDutyCycle);
  this->setDynamics(current, acceleration, dutyCycle, frequency, stepsPerRev, microsteps, gearRatio);
}

// Works for any stepper motors
float PhysicalDynamics_Stepper::getPosition(void) {
  return 360.0 * ((float)this->currState[this->cs]->relativeStep / this->getTotalStates());
}


#if (G2_SUPPORTED)

    // Interrupt service routine for motor 1 hall sensors
    void PhysicalDynamics_G2::M1_ISR(void) {
      //    hallSensorPhase
      bool x0 = digitalRead(BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::IN1]]);
      bool x1 = digitalRead(BoardInfo::g2_MotorPinouts[0][BoardInfo::G2Pins[BoardInfo::IN2]]);

      bool x0_i = PhysicalDynamics_G2::hallSensorPhase[0];
      bool x1_i = PhysicalDynamics_G2::hallSensorPhase[1];

      //bool decrement = ((x0_i && x1_i) && (x0 && !x1)) || ((!x0 && x1) && (x0 && x1)) || ((x0 && !x1) && (!x0 && !x1)) || ((!x0 && !x1) && (!x0 && x1));
      //bool increment = ((x0_i && x1_i) && (!x0 && x1)) || ((!x0 && x1) && (!x0 && !x1)) || ((x0 && !x1) && (x0 && x1)) || ((!x0 && !x1) && (x0 && !x1));
      bool decrement = ((x0_i && x1_i) && (x0 && !x1)) || ((!x0_i && x1_i) && (x0 && x1)) || ((x0_i && !x1_i) && (!x0 && !x1)) || ((!x0_i && !x1_i) && (!x0 && x1));
      bool increment = ((x0_i && x1_i) && (!x0 && x1)) || ((!x0_i && x1_i) && (!x0 && !x1)) || ((x0_i && !x1_i) && (x0 && x1)) || ((!x0_i && !x1_i) && (x0 && !x1));

      if (increment && !decrement)
        PhysicalDynamics_G2::unaccountedSteps[0]++;  // Increment #pulses detected on motor 1
      else if (decrement && !increment)
        PhysicalDynamics_G2::unaccountedSteps[0]--;  // Decrement #pulses detected on motor 1
      
      PhysicalDynamics_G2::hallSensorPhase[0] = x0;
      PhysicalDynamics_G2::hallSensorPhase[1] = x1;
    }

    // Interrupt service routine for motor 2 hall sensors
    void PhysicalDynamics_G2::M2_ISR(void) { 
      bool x0 = digitalRead(BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::IN1]]);
      bool x1 = digitalRead(BoardInfo::g2_MotorPinouts[1][BoardInfo::G2Pins[BoardInfo::IN2]]);
      bool x0_i = PhysicalDynamics_G2::hallSensorPhase[2];
      bool x1_i = PhysicalDynamics_G2::hallSensorPhase[3];
      //bool decrement = ((x0_i && x1_i) && (x0 && !x1)) || ((!x0 && x1) && (x0 && x1)) || ((x0 && !x1) && (!x0 && !x1)) || ((!x0 && !x1) && (!x0 && x1));
      //bool increment = ((x0_i && x1_i) && (!x0 && x1)) || ((!x0 && x1) && (!x0 && !x1)) || ((x0 && !x1) && (x0 && x1)) || ((!x0 && !x1) && (x0 && !x1));
      bool decrement = ((x0_i && x1_i) && (x0 && !x1)) || ((!x0_i && x1_i) && (x0 && x1)) || ((x0_i && !x1_i) && (!x0 && !x1)) || ((!x0_i && !x1_i) && (!x0 && x1));
      bool increment = ((x0_i && x1_i) && (!x0 && x1)) || ((!x0_i && x1_i) && (!x0 && !x1)) || ((x0_i && !x1_i) && (x0 && x1)) || ((!x0_i && !x1_i) && (x0 && !x1));


      if (increment && !decrement) 
        PhysicalDynamics_G2::unaccountedSteps[1]++;  // Increment #pulses detected on motor 2
      else if (decrement && !increment)
        PhysicalDynamics_G2::unaccountedSteps[1]--;  // Decrement #pulses detected on motor 2

      PhysicalDynamics_G2::hallSensorPhase[2] = x0;
      PhysicalDynamics_G2::hallSensorPhase[3] = x1;
    }

    void PhysicalDynamics_G2::initializeDevice(void) {
      this->direction = &(PhysicalDynamics_G2::g2Direction[this->cs]);
      
      // For each cs of the motor obj axis
      for (uint8_t cs = 0; cs < this->csSize; ++cs) {
        this->setCS(cs);                                                            // Sets the desired axis for use (sets this->cs)

        this->motorConnection[this->cs]->begin();                                       // Perform startup routine for this cs
        this->primitiveState[this->cs]->enable = false;                                 // Motor turned off in motorConnection->begin()
        this->syncMotorDynamics();                                                      // Update obj motor dynamics for the motorModel specified by this->primitiveState[this->cs].motorModel
        this->setPositionRate(1, 1);                                                    // Set velocity to 5 mm/s
        this->initializeCS(cs);

        // Initialize ISR to detect pulses from hall effect sensors
        switch (this->connectionAxis[this->cs] - 1) {
          case 0:   // M1
            /*
            if (Serial)
              Serial.println("M1 attach");
              Serial.print("In1 : ");Serial.println(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN1]]);
              Serial.print("In2 : ");Serial.println(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN2]]);
              */
            attachInterrupt(digitalPinToInterrupt(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN1]]), M1_ISR, CHANGE);   // Attach the in-phase pin to the handler_hall0 ISR
            attachInterrupt(digitalPinToInterrupt(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN2]]), M1_ISR, CHANGE); // Attach the quadrature pin to the handler_hall1 ISR
          break;
          case 1:   // M2
            /*
            if (Serial)
              Serial.println("M2 attach");
              Serial.print("In1 : ");Serial.println(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN1]]);
              Serial.print("In2 : ");Serial.println(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN2]]);
              */
            attachInterrupt(digitalPinToInterrupt(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN1]]), M2_ISR, CHANGE);   // Attach the in-phase pin to the handler_hall0 ISR
            attachInterrupt(digitalPinToInterrupt(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::IN2]]), M2_ISR, CHANGE); // Attach the quadrature pin to the handler_hall1 ISR
          break;
        }
      }
    }

  void PhysicalDynamics_G2::setDynamics(uint16_t velocity, uint32_t acceleration) {
      
  // High Dependencies assignments
  this->setPositionRate(velocity, 1);                              // Set the speed (Cycle duration will be set)
  this->setPositionRate(acceleration, 2);                                       // Set the speed (Cycle duration will be set)
}
    
  void PhysicalDynamics_G2::syncMotorDynamics(void) {
    uint8_t correspondingMotor = this->primitiveState[this->cs]->correspondingMotor;

    // Static Vars

    // Dynamic Vars
    float velocity = 1; // mm/s
    uint32_t acceleration = 1;

    switch (correspondingMotor) {
      #if (G2_SUPPORTED)
        // Static Specs
        
        // Dynamic Specs    
        velocity = 1;   // mm/s

        break;
      #endif
      default:
        // Static Specs
        

        // Dynamic Specs    
        velocity = 1;   // mm/s
        
    }
    this->currState[this->cs]->setStatics();
    this->setDynamics(velocity, acceleration);
  }

  void PhysicalDynamics_G2::setCS(uint8_t cs) {
    this->cs = cs;

    // Set CS line and EN line
    this->motorConnection[this->cs]->setCS(this->connectionCS[this->cs]);         // Set CS on motor connection obj (updates the pinout)
    uint8_t *pinout = this->motorConnection[this->cs]->getPinout();

    switch (this->currState[this->cs]->correspondingMotor) {
        
      #if G2_SUPPORTED
        case BoardInfo::G2 :
          switch (this->connectionAxis[this->cs]) {
            case 1: // x-axis
              this->primitiveState[this->cs]->velocity = 1;          // [1 to 5] recommended (mm/s) (up to 7.5 mm/s)
            break;
            case 2: // y-axis
              this->primitiveState[this->cs]->velocity = 1;       // [1 to 5] recommended (up to 7.5 mm/s)
            break;
            default:

            break;
          }

          break;
      #endif
    }
    #if (DEBUGGER_OVERRIDE)
    if (Serial)
      Serial.println("G2 Set CS Done");
    #endif
  }

  void PhysicalDynamics_G2::originSearch(void) {        
    #if (DEBUGGER_OVERRIDE)
    if (Serial) {
      Serial.println("G2: Origin Search");        
      Serial.print("cs : "); Serial.println(this->cs);
      Serial.print("Connection axis : ");Serial.println(this->connectionAxis[this->cs]);
      Serial.print("unnacountedSteps : ");Serial.println(PhysicalDynamics_G2::unaccountedSteps[this->connectionAxis[this->cs] - 1]);
      Serial.print("current step count : ");Serial.println(this->primitiveState[cs]->relativeStep);
    }
    #endif

  //uint32_t intervalsPerPulse = 100000;
  uint8_t constCountTolerance = 3;  // wait long enough for 3 missed pulses..
  uint32_t constCount = 0;    
  int32_t prevSteps;
  int32_t currSteps = this->primitiveState[cs]->relativeStep + this->pullSteps(false);

  float initialVelocity = this->getPositionRate(1);         // Velocity in mm/s
  this->setPositionRate(PhysicalDynamics_G2::originSearchSpeed, 1);    // Set velocity to origin search velocity
  float pwmVal = this->computePWMVal();
  this->setDirection(RETRACT);                                                              // Set direction of travel
  analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], pwmVal);    // Set motor speed (0 to 255)

  // Observe steps counter while motor is operating.
  // Stop the motor when the pulses stop incrementing
  while (constCount < constCountTolerance) {
    prevSteps = currSteps;
    delay(50);                                           // Delay a microsecond scale duration
    currSteps = this->primitiveState[cs]->relativeStep + this->pullSteps(false);       // Observe the motor ISR steps counter for changes
    abs(prevSteps - currSteps) > 0 ? constCount = 0 : ++constCount;
  }

  analogWrite(this->motorConnection[this->cs]->pinout[BoardInfo::G2Pins[BoardInfo::PWM]], 0);   // Motor reached origin. Turn it off.
  this->redefineOrigin();
  this->setPositionRate(initialVelocity, 1);                                // Set the initial velocity
  this->setPosition(PhysicalDynamics_G2::ORIGIN_OFFSET);                    // Offset from hardware origin
  this->g2Calibrated[this->connectionAxis[this->cs] - 1] = true;            // This will enable user to use full operation limits
  this->redefineOrigin();
}

  void PhysicalDynamics_G2::setPositionRate(double rate, uint8_t derivative) {
    double rateLimit;
    switch (derivative) {
      case 0:     // Position (0)
        this->setPosition(rate);
        break;
      case 1:     // (Velocity : Hz)
        // Input validation
        rateLimit = this->getRateLimit(1);
        this->primitiveState[this->cs]->velocity = rate > rateLimit ? (rateLimit > 0 ? rateLimit : 3) : (rate > 0 ? rate : 3);       // Limit maximum speed because they are not achievable with our hardware (Set 3mm/s by default)
        //this->primitiveState[this->cs]->frequency = rate > rateLimit ? (rateLimit > 0 ? rateLimit : 1) : (rate > 0 ? rate : 1);       // Limit maximum speed because they are not achievable with our hardware.

        /*
        // Stepper motor implementation below
        this->currState[this->cs]->cycleDuration = (1/(this->getTotalStates() * this->primitiveState[this->cs]->frequency)) * pow(10, 6); // (Duration in us)
        this->currState[this->cs]->cycleDurationExponent = -6;    // Duration in us
        */
        break;
      case 2:     // (Acceleration)
        this->currState[this->cs]->acceleration = rate;
      break;
    }
  }

  float PhysicalDynamics_G2::getPositionRate(uint8_t derivative)  {
    switch (derivative) {
      case 0:   // (Position) : Current State
        return this->getPosition();
      case 1:     // (Velocity)
        return this->primitiveState[this->cs]->velocity;  // Velocity in terms of mm/s
      break;
      case 2:     // (Acceleration)
        return this->primitiveState[this->cs]->acceleration;
      break;
      default:
        return 0;
    }
  }

  float PhysicalDynamics_G2::getRateLimit(uint8_t derivative) {
    switch (derivative) {
      case 0:         // Position
        return 58;    // Stroke length is 60 mm but platform is limited to 58mm
      break;
      case 1:         // Velocity : mm/s
        return 7.5;   // Limit is 7.5 mm/s
        default : 
          return -1;
    }
  }


void PhysicalDynamics_G2::selectMotor(uint8_t correspondingMotor) {
  this->primitiveState[this->cs]->resolution = 1.08;    // 1.08 um resolution;
}

void PhysicalDynamics_G2::initializeCS(uint8_t cs) {
  this->primitiveState[cs]->relativeStep = 0;
  this->primitiveState[cs]->relativeState = 0;
}

void PhysicalDynamics_G2::move(float delta) {
  // TODO: Define me
    switch (this->currState[this->cs]->correspondingMotor) {
        #if G2_SUPPORTED
          case BoardInfo::G2 :


          break;
        #endif
      }
  }

#endif
