#include "Device.h"

#if (NEMA17_SUPPORTED)
  struct Motor_Nema17* nema17Motors[NEMA17_SIZE] = {};     // Initialize to zero by default
#endif
#if (STEPPER_ONLINE_SUPPORTED)
  struct Motor_StepperOnline* Device::stepperOnlineMotors[STEPPER_ONLINE_SIZE] = {};     // Initialize to zero by default
#endif
#if (G2_SUPPORTED)
  struct Motor_G2* Device::g2Motors[G2_SIZE] = {};     // Initialize to zero by default
  //struct Motor_G2 Device::g2Motors[G2_SIZE] = {Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)};          // The G2 Motor class is designed for a chip capable of controlling 2-axis
#endif

void Device::resetState(void) {
  this->numUniqueMotors = 0;
  this->totalAxis = 0;
  for (uint8_t iter = 0; iter < BOARD_SIZE; ++iter) {
    this->currDevCS[iter] = 0;        // Set to control CS 0
    this->devAxisCSSize[iter] = 0;    // Set axis size to 0 elements
  }
  for (uint8_t iter = 0; iter < BoardInfo::NUM_MOTOR_MODELS; ++iter)
    this->numAxis[iter] = 0;          // Reset # instances of each motor type
}

void Device::displayDeviceState(void) {
  if (!Serial)
    Serial.begin(BAUD_RATE);
    
  ProgMemFcns::displayProgramMemoryString(deviceStatePrintoutHeader);
  Serial.print("\r\n"); // Terminate the line
  for (uint8_t iter = 0; iter < this->totalAxis; ++iter) {
    for (uint8_t cs = 0; cs < this->devAxisCSSize[iter]; ++cs) 
      this->motors[iter][cs]->statusPrintout();
  }
}

void Device::resetStaticVars(void) {
  uint8_t axisSize;
  for (uint8_t motor = 0; motor < BoardInfo::NUM_MOTOR_MODELS; ++motor) {
    for (uint8_t axisIndex = 0; axisIndex < BoardInfo::supportedMotorsAxisSize[motor]; ++axisIndex) {
      #if (NEMA17_SUPPORTED)
        delete Device::nema17Motors[axisIndex];
        Device::nema17Motors[axisIndex] = nullptr;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        delete Device::stepperOnlineMotors[axisIndex];
        Device::stepperOnlineMotors[axisIndex] = nullptr;
      #endif
      #if (G2_SUPPORTED)
        delete Device::g2Motors[axisIndex];
        Device::g2Motors[axisIndex] = nullptr;
      #endif
    }
  }
}

void Device::allocateStaticVars(void) {
  for (uint8_t motorModel = 0; motorModel < BoardInfo::NUM_MOTOR_MODELS; ++motorModel) {
    for (uint8_t axis = 1; axis <= BoardInfo::motorConfigsAxisSize[motorModel]; ++axis) {
      Device::allocateStaticMotors((const uint8_t*)&motorModel, &axis, 1);
    }
  }
}

void Device::allocateStaticMotors(const uint8_t *correspondingMotor, uint8_t *axis, uint8_t SIZE) {
  uint8_t axisIndex;
  for (uint8_t index = 0; index < SIZE; ++index) {
    if (correspondingMotor[index] >= BoardInfo::NUM_MOTOR_MODELS || axis[index] > BoardInfo::motorConfigsAxisSize[correspondingMotor[index]])
      continue;

    axisIndex = axis[index] - 1;
    switch (correspondingMotor[index]) {
      #if (NEMA17_SUPPORTED)
        case BoardInfo::NEMA17 : 
          if (Device::nema17Motors[axisIndex] == nullptr)
            Device::nema17Motors[axisIndex] = new Motor_Nema17(axisIndex + 1, *DeviceDynamics::getCSPtr(BoardInfo::NEMA17, axisIndex + 1));   // Construct NEMA17 motor obj
        break;
      #endif
      #if (STEPPER_ONLINE_SUPPORTED)
        case BoardInfo::STEPPER_ONLINE : 
          if (Device::stepperOnlineMotors[axisIndex] == nullptr)
            Device::stepperOnlineMotors[axisIndex] = new Motor_StepperOnline([index], 0); // Each instance controls 2 motors via CS : (D Flip-Flops used to retain motor state for each discrete motor driver);
        break;
      #endif
      #if (G2_SUPPORTED)
        case BoardInfo::G2 : 
          if (Device::g2Motors[axisIndex] == nullptr)
            Device::g2Motors[axisIndex] = new Motor_G2(axis[index], 0);                       // Each instance controls 2 motors
        break;
        //struct Motor_G2 g2Motors[G2_SIZE] = {Motor_G2(1, &g2Driver.driver), Motor_G2(2, &g2Driver.driver)};          // The G2 Motor class is designed for a chip capable of controlling 2-axis
      #endif
    }
  }
}

void Device::initializeDevice(void) {
  // Initialize axis' one or multiple at a time in order to satisfy board requirements 
  // which need both axis specifications during initialization. Do not change this implementation
  // until a solution can be found which works for the G2 actuators. They currently require 
  // an instance of each motor object to be supplied during device initialization.
  //struct CommandProcessing cmdProcess;                 // Utility used to facilitate with device connection commands, as done in a setup routine
  int awaitingInput = !USE_DEFAULT_BOARD;                     // 1st input will use default parameter to specify motors
  int promptOffset[3];
  bool isComplete = false;                        // Input will always need validated and complete constructor procedure
  
  uint8_t deviceAxis[MAX_PARAMETERS];
  uint8_t deviceCS[MAX_PARAMETERS];
  uint8_t motorAxis[MAX_PARAMETERS];
  uint8_t motorCS[MAX_PARAMETERS];
  int8_t correspondingMotors[MAX_PARAMETERS];

  // Assume all possible parameters will be used. A SIZE parameter will be used to relay the first N valid elements
  uint8_t axisCSSize[MAX_PARAMETERS];
    for (uint8_t index = 0; index < MAX_PARAMETERS; ++index)
      axisCSSize[index] = 1;

  uint8_t currParameter;
  uint8_t **boardConfigsAxisCSSize;   // [board][axisIndex] -> value is cs size for the axis
  uint8_t iter;
  bool isValid;
  const char ***supportedBoardsAxisDescription = BoardInfo::supportedBoardsAxisDescription;

  Serial.begin(BAUD_RATE);                    // The specified baud rate will be used for serial communication across the USB cable

  promptOffset[0] = USE_DEFAULT_BOARD ? 1 : 0;
  promptOffset[1] = USE_DEFAULT_LIST ? 1 : 0;

  switch (startPrompt) {
    case Device::MC_NAME:
      awaitingInput = 1;        // Skips prompt 1 (ie. use default for input 1)
    break;
    case Device::MOTOR_LIST:    // Skips prompt 1 and 2 (ie. use default for input 1 and 2)
      awaitingInput = 2;
    break;
    case Device::DONE:
      awaitingInput = 3;
    break;
    default:
      awaitingInput = 1;
  }

  uint8_t currStep;
  uint8_t actualNumDevs = 0;
  while (!Serial)
    delay(500);               // Allow time for Serial port to initialize, otherwise upcoming commands involving the Serial port may fail

  #if DEBUGGER_OVERRIDE
    #if defined(IS_ARDUINO) 
      Serial.print(F("Begin microcontroller setup\r\nWaiting for a valid device list cmd\r\n"));
    #endif
  #endif

  while (!isComplete) {                                     // Prompt the user for the device list until there is at least one valid axis allocated for by the 'struct Device myController' object
    currStep = 0;
    if (awaitingInput + promptOffset[0] == 1) {
      ProgMemFcns::displayProgramMemoryString(boardInputPrompt);  // Prompt the user to input a device list   
      Serial.print("\r\n"); // Terminate the line
      while (!Serial.available())                       // Wait for the master to transmit data accross the serial port
        delay(50);                                      // Wait for the master to specify a valid device list
    }
    
    #if (DEBUGGER_OVERRIDE)
      // Display Free Dynamic Memory (Before cmd process)
      ProgMemFcns::displayProgramMemoryString(freeMemoryPrompt);
      Serial.print(CommandProcessing::freeMemory());
      Serial.print("\r\n"); // Terminate the line
    #endif

    // Board Alias Validation
    ProgMemFcns::displayProgramMemoryString(validatingBoardPrompt);
    Serial.print("\r\n");   // Terminate the line

    if (awaitingInput + promptOffset[0] == 1) {
      CommandProcessing::processSerialPort();               // Process the user input
      awaitingInput = 2;                                    // Proceed to the next prompt
      
    }
    else
      CommandProcessing::processString(DEFAULT_BOARD);           // Hardcoded to prevent need to manually enter each time (use serial port otherwise)
    promptOffset[0] = 0;                                    // Only allow default input for 1st attempt

    this->boardConfiguration = BoardInfo::computeBoardConfiguration();// Compute and store the board configuration associated with this board description

    #if (DEBUGGER_OVERRIDE)
      Serial.print("your board config : ");Serial.println(this->boardConfiguration);
    #endif

    if (this->boardConfiguration < 0 || BoardInfo::supportedBoardsAxisDescription[this->boardConfiguration] == nullptr) {
      awaitingInput = 1;                                    // Start over at prompt 1
      continue;
    }

    if (awaitingInput + promptOffset[1] == 2) {
      ProgMemFcns::displayProgramMemoryString(devListInputPrompt);  // Prompt the user to input a device list        
      
      Serial.print("\r\n"); // Terminate the line
      while (!Serial.available())                       // Wait for the master to transmit data accross the serial port
        delay(50);                                      // Wait for the master to specify a valid device list
    }

    // Board Alias Validation
    ProgMemFcns::displayProgramMemoryString(validatingListPrompt);
    Serial.print("\r\n");                                         // Terminate the line

    // Set motor parameters
    if (awaitingInput + promptOffset[1] == 2) {

      // Check if motors in the board configuration are valid
      isValid = true;
      currParameter = 0;   // total string count
      while (isValid && currParameter < CommandProcessing::getNumParameters()) {
        CommandProcessing::processString(CommandProcessing::getParameter(currParameter + 1), "::");

        // WARNING : MAJOR PROBLEM HERE : We need to create multiple instance of CommandProcessing
        // because with the static member, we can only process one buffer at a time. 
        // This is a problem here because we must process the input buffer string and also 
        // process each individual string using a different delimiter, '::'.
        // For now, the code below is WRONG!!!!!!
        if (CommandProcessing::getNumParameters() >= 5) {
          deviceAxis[currParameter] = atoi(CommandProcessing::getParameter(1));
          deviceCS[currParameter] = atoi(CommandProcessing::getParameter(2));
          correspondingMotors[currParameter] = BoardInfo::computeMotorIndex(CommandProcessing::getParameter(3));   // Store the motor index associated with this motor model 
          motorAxis[currParameter] = atoi(CommandProcessing::getParameter(4));
          motorCS[currParameter] = atoi(CommandProcessing::getParameter(5));

          #if (DEBUGGER_OVERRIDE)
            Serial.print("Dev Axis : ");Serial.println(deviceAxis[currParameter]);
            Serial.print("Dev CS: ");Serial.println(deviceCS[currParameter]);
            Serial.print("motor : ");Serial.println(correspondingMotors[currParameter]);
            Serial.print("motor axis: ");Serial.println(motorAxis[currParameter]);
            Serial.print("motor CS: ");Serial.println(motorCS[currParameter]);
            Serial.print("axisCSSize : ");Serial.println(axisCSSize[currParameter]);
          #endif
        }

        #if (DEBUGGER_OVERRIDE)
          #if defined(IS_ARDUINO) 
            Serial.print(F("Corresponding motor (SEE BoardInfo.h) : ")); Serial.print(correspondingMotors[currParameter]);   Serial.print("\r\n"); // Terminate the line
          #endif
        #endif

        isValid = correspondingMotors[currParameter++] >= 0;   // motor index MUST be >= 0 (This is why this comparison is performed before assignment)
      }
      awaitingInput = 3;                                    // Proceed to the next prompt
    }
    else {
      // Check if motors in the board configuration are valid
      isValid = true;
      currParameter = 0;   // total string count
      while (isValid && BoardInfo::supportedBoardsAxisDescription[this->boardConfiguration][currParameter] != nullptr) {
        #if (DEBUGGER_OVERRIDE)
          if (Serial) {
            Serial.print("initializeDevice() : Analyzing vs board axis description : ");
            Serial.println(BoardInfo::supportedBoardsAxisDescription[this->boardConfiguration][currParameter]);
          }
        #endif

        CommandProcessing::processString(supportedBoardsAxisDescription[this->boardConfiguration][currParameter], "::");

        if (CommandProcessing::getNumParameters() >= 5) {
          deviceAxis[currParameter] = atoi(CommandProcessing::getParameter(1));
          deviceCS[currParameter] = atoi(CommandProcessing::getParameter(2));
          correspondingMotors[currParameter] = BoardInfo::computeMotorIndex(CommandProcessing::getParameter(3));   // Store the motor index associated with this motor model 
          motorAxis[currParameter] = atoi(CommandProcessing::getParameter(4));
          motorCS[currParameter] = atoi(CommandProcessing::getParameter(5));

        #if (DEBUGGER_OVERRIDE)
            Serial.print("Dev Axis : ");Serial.println(deviceAxis[currParameter]);
            Serial.print("Dev CS: ");Serial.println(deviceCS[currParameter]);
            Serial.print("motor : ");Serial.println(correspondingMotors[currParameter]);
            Serial.print("motor axis: ");Serial.println(motorAxis[currParameter]);
            Serial.print("motor CS: ");Serial.println(motorCS[currParameter]);
            Serial.print("axisCSSize : ");Serial.println(axisCSSize[currParameter]);
          #endif

          #if (DEBUGGER_OVERRIDE)
            #if defined(IS_ARDUINO) 
              Serial.print(F("Corresponding motor (SEE BoardInfo.h) : ")); Serial.print(correspondingMotors[currParameter]);   Serial.print("\r\n"); // Terminate the line
            #endif
          #endif

          isValid = correspondingMotors[currParameter++] >= 0;   // motor index MUST be >= 0 (This is why this comparison is performed before assignment) 
        }
        else {
          isValid = false;
          Serial.println("Warning : Expected 5 Parameters, received only ");Serial.print(CommandProcessing::getNumParameters());Serial.println(" parameters.");
          delay(100);
        }
      }
    }
    promptOffset[1] = 0;                                    // Only allow default input for 1st attempt

    isComplete = isValid;

    // 'isComplete' : is only TRUE at this point IFF all motors are valid
    if (isValid) {
      isComplete = this->loadMotors(deviceAxis, deviceCS, correspondingMotors, motorAxis, motorCS, axisCSSize, currParameter);

      #if DEBUGGER_OVERRIDE
        #if defined(IS_ARDUINO) 
          Serial.print(F("Allocating dev vars... Please wait.\r\n"));
        #endif
      #endif
      if (isComplete) {
        #if DEBUGGER_OVERRIDE
          #if defined(IS_ARDUINO) 
            Serial.print(F("Initializing Controller... Please wait.\r\n"));
          #endif
        #endif
        
        this->initializeDeviceProcessing();         // Initializes motor driver for each controller axis-cs pair
      }
    }
  
    #if (DEBUGGER_OVERRIDE)
      // Display Free Dynamic Memory (After cmd process)
      ProgMemFcns::displayProgramMemoryString(freeMemoryPrompt);
      Serial.print(CommandProcessing::freeMemory());
      Serial.print("\r\n"); // Terminate the line
    #endif
  }
  ProgMemFcns::displayProgramMemoryString(successfullInitializationPrompt);
  Serial.print("\r\n"); // Terminate the line
  
  #if DEBUGGER_OVERRIDE
    Serial.println("Done with Arduino setup()\r\n");
  #endif
}

bool Device::loadMotors(const uint8_t* deviceAxis, const uint8_t* deviceCS, const uint8_t* correspondingMotors, const uint8_t* motorAxis, const uint8_t* motorCS, const uint8_t* csSize, uint8_t SIZE) {
    if (deviceAxis == nullptr || deviceCS == nullptr || correspondingMotors == nullptr || motorAxis == nullptr || motorCS == nullptr || csSize == nullptr)
      return false;

    uint8_t overallElement;
    uint8_t parameterIndex;
    uint8_t devAxisIndex;
    bool *isValid = new bool[SIZE];
    for (uint8_t parameterIndex = 0; parameterIndex < SIZE; ++parameterIndex) {
      isValid[parameterIndex] = (correspondingMotors[parameterIndex] < BoardInfo::NUM_MOTOR_MODELS && motorAxis[parameterIndex] > 0 && 
      motorAxis[parameterIndex] <= BoardInfo::supportedBoardsAxisSize[this->boardConfiguration][correspondingMotors[parameterIndex]] && 
      motorCS[parameterIndex] < BoardInfo::supportedBoardsAxisCSSize[this->boardConfiguration][correspondingMotors[parameterIndex]][motorAxis[parameterIndex] - 1]);
    }

    this->deallocateObjVars();
    this->resetState();

    overallElement = 0;
    uint8_t devAxisCSSize[BOARD_SIZE];
    uint8_t axisPtr[BOARD_SIZE];

    for (uint8_t devAxisIndex = 0; devAxisIndex < BOARD_SIZE; ++devAxisIndex) {
      devAxisCSSize[devAxisIndex] = 0;
      axisPtr[devAxisIndex] = 0;
    }
    
    for (uint8_t parameterIndex = 0; parameterIndex < SIZE; ++parameterIndex) {
      if (isValid[parameterIndex]) {
        for (uint8_t cs = 0; cs < csSize[parameterIndex]; ++cs) {
          devAxisCSSize[deviceAxis[overallElement + cs] - 1]++;       // Increment cs counter for this axis
          this->numAxis[correspondingMotors[overallElement + cs]]++;    // Increment corresponding motor counter
        }
      }
      overallElement += csSize[parameterIndex];
    }
    
    // Pre-process 'this' state
    this->totalAxis = 0;
    for (devAxisIndex = 0; devAxisIndex < BOARD_SIZE; ++devAxisIndex) {
      axisPtr[devAxisIndex] = devAxisIndex + 1;                           // Set this var for use in 'allocateObjVars()'
      if (devAxisCSSize[devAxisIndex] > 0) {
        ++this->totalAxis;
    }
    }
    
    this->allocateObjVars(axisPtr, devAxisCSSize, BOARD_SIZE);

    // Set obj state vars
    overallElement = 0;
    for (parameterIndex = 0; parameterIndex < SIZE; ++parameterIndex) {
      if (isValid[parameterIndex]) {
        #if (DEBUGGER_OVERRIDE)
          Serial.print("Element ");Serial.print(parameterIndex);Serial.print(" : motor ");Serial.print(correspondingMotors[parameterIndex]);Serial.print(" : devAxis ");Serial.print(deviceAxis[parameterIndex]);Serial.print(" : CS ");
          Serial.print(deviceCS[parameterIndex]); 
          Serial.print(" = ");  
          Serial.print(motorAxis[parameterIndex]);
          Serial.print("-");Serial.println(motorCS[parameterIndex]);
        #endif
        
        devAxisIndex = deviceAxis[parameterIndex] - 1;
        this->motorAxis[devAxisIndex][deviceCS[parameterIndex]] = motorAxis[parameterIndex];                     // Store the motor axis associated with this 'element-cs' pair
        this->motorAxisCS[devAxisIndex][deviceCS[parameterIndex]] = motorCS[parameterIndex];                     // Store the motor CS associated with this 'element-cs' pair
        this->correspondingMotor[devAxisIndex][deviceCS[parameterIndex]] = correspondingMotors[parameterIndex];  // Store the corresponding motor associated with this 'element-cs' pair
        
        #if (DEBUGGER_OVERRIDE)
          Serial.print("Store obj spec : ");
          Serial.print("devAxisIndex = ");Serial.println(devAxisIndex);
          Serial.print("deviceCS     = ");Serial.println(deviceCS[parameterIndex]);
          Serial.print("dv cs = ");Serial.println(deviceCS[parameterIndex]);
          Serial.print("Dev : axis = ");Serial.print(devAxisIndex + 1);Serial.print(", cs = ");Serial.println(deviceCS[parameterIndex]);
          Serial.print("Mot : motor = ");Serial.print(correspondingMotors[parameterIndex]);Serial.print(", axis = ");Serial.print(motorAxis[parameterIndex]);Serial.print(", cs = ");Serial.println(motorCS[parameterIndex]);

          Serial.print("Stored obj spec : ");
          Serial.print("Mot : motor = ");Serial.print(this->correspondingMotor[devAxisIndex][deviceCS[parameterIndex]]);Serial.print(", axis = ");Serial.print(this->motorAxis[devAxisIndex][deviceCS[parameterIndex]]);Serial.print(", cs = ");Serial.println(this->motorAxisCS[devAxisIndex][deviceCS[parameterIndex]]);
        #endif
      }
      ++overallElement;
    }

    this->linkDevVars();    // Link the 'motors' pointers to their motors
    for (uint8_t iter = 0; iter < BoardInfo::NUM_MOTOR_MODELS; ++iter) {
      this->numUniqueMotors += (this->numAxis[iter] > 0) ? 1 : 0;
      #if DEBUGGER_OVERRIDE
        Serial.print("Motor model : "); Serial.print(BoardInfo::supportedMotorsDescription[iter]);  Serial.print(" has ");  Serial.print(this->numAxis[iter]);  Serial.println(" num axis.\r\n");   // Display stored information, for debugging
      #endif
    }
    delete[] isValid; // Free dynamic memory
    return true;
  }

  void Device::allocateObjVars(uint8_t *axis, uint8_t *csSize, uint8_t SIZE) {
    uint8_t axisIndex;
    for (uint8_t element = 0; element < SIZE; ++element) {
      axisIndex = axis[element] - 1;  // overflow if axis[element] == 0 (thats fine)

      #if (DEBUGGER_OVERRIDE)
        Serial.print("Process dev axis allocation, axis : ");Serial.println(axis[element]);
        Serial.print("curr size = ");Serial.print(this->devAxisCSSize[axisIndex]);Serial.print(", new size = ");Serial.println(csSize[element]);
      #endif

      if (axisIndex >= BOARD_SIZE || this->devAxisCSSize[axisIndex] == csSize[element])
        continue;
      if (this->devAxisCSSize[axis[element] - 1] > 0)
        this->deallocateObjVars(axis[element]);                                              // Deallocate axis pointer since the size is changing.

      #if (DEBUGGER_OVERRIDE)
        Serial.println("Allocating memory...");
      #endif

      // Check for nullptr to prevent run-away memory usage. For instance, parameter set may contain duplicate 'controllerAxisIndex' values.
      if (this->motorAxis[axisIndex] == nullptr)
        this->motorAxis[axisIndex] = new uint8_t[csSize[element]];
      if (this->motorAxisCS[axisIndex] == nullptr)
        this->motorAxisCS[axisIndex] = new uint8_t[csSize[element]];
      if (this->correspondingMotor[axisIndex] == nullptr)
        this->correspondingMotor[axisIndex] = new int8_t[csSize[element]];
      if (this->motors[axisIndex] == nullptr) 
        this->motors[axisIndex] = new Motor*[csSize[element]];
      this->devAxisCSSize[axisIndex] = csSize[element];   // Update the pointer size

      // Initialize vars
      for (uint8_t cs = 0; cs < csSize[element]; ++cs) {
        this->motorAxis[axisIndex][cs] = 0;     // Motor axis is not known. Set to 0 for now.
        this->motorAxisCS[axisIndex][cs] = 0;   // Assume each controller cs will each use cs = 0 to control that output.
        this->correspondingMotor[axisIndex][cs] = -1;  // Motor for each cs is unknown at this time.
        this->motors[axisIndex][cs] = nullptr;
      }
    }
  }

  void Device::initializeDeviceProcessing(void) {
    for (uint8_t axisIndex = 0; axisIndex < this->totalAxis; ++axisIndex) {
      for (uint8_t cs = 0; cs < this->devAxisCSSize[axisIndex]; ++cs) {
        if (this->motors[axisIndex][cs] != nullptr) {
          this->motors[axisIndex][cs]->setCS(this->motorAxisCS[axisIndex][cs]);
          this->motors[axisIndex][cs]->initializeDevice(); // All CS of this motor axis will be initialized
        }
      }
    }
  }

  void Device::initializeDeviceProcessing(uint8_t axis, uint8_t cs) {
    if (this->motors[axis - 1][this->motorAxis[axis - 1][cs]] != nullptr) {
      this->motors[axis - 1][this->motorAxis[axis - 1][cs]]->setCS(this->motorAxisCS[axis - 1][cs]);
      this->motors[axis - 1][this->motorAxis[axis - 1][cs]]->initializeDevice(); // All CS of this motor axis will be initialized
    }
  }

  // (FAST) - Deallocate ALL dynamic memory
void Device::deallocateObjVars(void) {
  for (uint8_t controllerAxisIndex = 0; controllerAxisIndex < BOARD_SIZE; ++controllerAxisIndex) {
    // Free Dynamic Memory
    delete[] this->motorAxis[controllerAxisIndex];
    delete[] this->motorAxisCS[controllerAxisIndex];
    delete[] this->correspondingMotor[controllerAxisIndex];
    delete[] this->motors[controllerAxisIndex];

    // Pointers are no longer valid, set to nullptr
    this->motorAxis[controllerAxisIndex] = nullptr;
    this->motorAxisCS[controllerAxisIndex] = nullptr;
    this->correspondingMotor[controllerAxisIndex] = nullptr;
    this->motors[controllerAxisIndex] = nullptr;
  }
}

// Deallocate dynamic memory for specified device axis
void Device::deallocateObjVars(uint8_t devAxis) {
  if (devAxis > 0 && devAxis <= BOARD_SIZE) {
    // Free Dynamic Memory
    delete[] this->motorAxis[devAxis - 1];
    delete[] this->motorAxisCS[devAxis - 1];
    delete[] this->correspondingMotor[devAxis - 1];
    delete[] this->motors[devAxis - 1];

    // Pointers are no longer valid, set to nullptr
    this->motorAxis[devAxis - 1] = nullptr;
    this->motorAxisCS[devAxis - 1] = nullptr;
    this->correspondingMotor[devAxis - 1] = nullptr;
    this->motors[devAxis - 1] = nullptr;
  }
}

// Remove all information and memory associated for specified device axis
void Device::removeAxis(uint8_t axis) {
  uint8_t axisIndex = axis - 1;
  for (uint8_t cs = 0; cs < this->devAxisCSSize[axisIndex]; ++cs) {
    --this->numAxis[this->correspondingMotor[axisIndex][cs]]; // decrement motor instance counter for this specific dev axis-cs pair
  }
  --this->totalAxis;                                          // Decrement total # of axis
  this->deallocateObjVars(axis);                              // Deallocate for this axis
}



  void Device::linkDevVars(void) {
  // Set 'motor' pointers to their motor objs
  uint8_t motor;
  for (uint8_t axisIndex = 0; axisIndex < this->totalAxis; ++axisIndex) {
    for (uint8_t cs = 0; cs < this->devAxisCSSize[axisIndex]; ++cs) {
      motor = this->correspondingMotor[axisIndex][cs];

      // Indexing Validation
      if (this->motorAxis[axisIndex][cs] < 1 || this->motorAxis[axisIndex][cs] > BoardInfo::supportedBoardsAxisSize[this->boardConfiguration][motor]) {
        //#if (DEBUGGER_OVERRIDE)
          if (Serial) {
            Serial.println("Link Dev Vars for axis-cs pair incompatible.");
            
          }
       // #endif
        continue;
      }

      switch (motor) {
        #if NEMA17_SUPPORTED
          case BoardInfo::NEMA17 :
            this->motors[axisIndex][cs] = this->nema17Motors[this->motorAxis[axisIndex][cs] - 1];
          break;
        #endif
        #if STEPPER_ONLINE_SUPPORTED
          case BoardInfo::STEPPER_ONLINE :
            #if (DEBUGGER_OVERRIDE)
              Device::stepperOnlineMotors == nullptr ? Serial.println("nothing at all") : Serial.println("check 1 good");
              Device::stepperOnlineMotors[0] == nullptr ? Serial.println("nothing at all") : Serial.println("check 2 good");
              Device::stepperOnlineMotors[this->motorAxis[axisIndex][cs] - 1] == nullptr ? Serial.println("nothing at all") : Serial.println("check 3 good");
            #endif

            this->motors[axisIndex][cs] = Device::stepperOnlineMotors[this->motorAxis[axisIndex][cs] - 1];

            #if (DEBUGGER_OVERRIDE)
              (this->motors[axisIndex][cs] != nullptr) ? Serial.println("Successfull link") :Serial.println("dev motor link failed");
            #endif
          break;
        #endif
        #if G2_SUPPORTED
          case BoardInfo::G2 :
            this->motors[axisIndex][cs] = this->g2Motors[this->motorAxis[axisIndex][cs] - 1];     // motor axis cs is set independently using the api function [this->motorAxisCS[axisIndex][cs]];
            
              #if (DEBUGGER_OVERRIDE)
              (this->motors[axisIndex][cs] != nullptr) ? Serial.println("Successfull link") :Serial.println("dev motor link failed");
              #endif
          break;
        #endif
          default:
            this->motors[axisIndex][cs] = nullptr;
          break;
      }

      #if (DEBUGGER_OVERRIDE)
        if (this->motors[axisIndex][cs] != nullptr)
          Serial.print("Successfull motor obj storage!\r\n");
        else
          Serial.print("WARNING: Device axis motor pointer linked to nullptr!\r\n");
      #endif
    }
  }

  #if (DEBUGGER_OVERRIDE)
    Serial.println("Done linking dev vars succesfful");
  #endif
}

  // Function not used below... Needs updated for new architecture
  uint8_t Device::openMotor(uint8_t motorSpecifier) {
    uint8_t axisIndex = 0;
    uint8_t iter;
    bool needsInitialized = false;
    uint8_t axisSet;

    // Traverse through motorAxis arr until all elements of this same motor are identified
    uint8_t currElement = 0;
    uint8_t correspondingElements[BOARD_SIZE];
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
    for (axisSet = 1; iter < BOARD_SIZE; ++iter) {
      currElement = 0;
      
      duplicateFound = false;
      while (currElement < matchesFound && !duplicateFound ) {
        duplicateFound = axisSet == motorAxis[correspondingElements[currElement]];
        ++currElement;
      }
      if (!duplicateFound)
        break;
    }
    
    if (duplicateFound && axisSet > BOARD_SIZE)
      return 0;                                             // No more motors can be controlled
    
    this->motorAxis[destElement] = axisSet;                  // Axis
    this->correspondingMotor[destElement] = motorSpecifier;   // Store the motor specifier which identifies the type of motor this instance controls
    ++this->numAxis[motorSpecifier];                        // Increment # of axis of this specific motor
    ++this->totalAxis;                                      // Increment total # of axis
    return destElement + 1;                                 // Return axis of device obj for this new motor axis
  }
