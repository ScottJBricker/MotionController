#include "Device.h"

void Device::allocateDevVars(void) {
  //struct Motor* temp;
  uint8_t currMotor = 0;
  uint8_t i;
  uint8_t *currNumAxis = new uint8_t[Enumerators::NUM_MOTOR_MODELS];
  uint8_t tempMotor;
  for (i = 0; i < Enumerators::NUM_MOTOR_MODELS; ++i)
    currNumAxis[i] = 0;

  for (i = 0; i < this->totalAxis; ++i) {
    this->motors[i] = nullptr;
    tempMotor = this->correspondingMotor[i];
    switch (tempMotor) {
    #if NEMA17_SUPPORTED
      case Enumerators::NEMA17 :
        this->motors[i] = &(this->nema17Motors[currNumAxis[tempMotor]]);
      break;
    #endif
    #if NEMA23_SUPPORTED
      case Enumerators::NEMA23 :
        this->motors[i] = &(this->nema23Motors[currNumAxis[tempMotor]]);
      break;
    #endif
    #if G2_SUPPORTED
      case Enumerators::G2 :
        this->motors[i] = &(this->g2Motors[currNumAxis[tempMotor]]);
        this->extDriver[i] = &(this->g2Driver);
      break;
    #endif
    }

    ++currNumAxis[tempMotor];
    #if (DEBUGGER_OVERRIDE)
      if (this->motors[i])
        Serial.print("Successfull motor obj storage!\r\n");
      else
        Serial.print("WARNING: Device axis motor could not be stored to arr!\r\n");
    #endif
  }
  delete[] currNumAxis;

  for (i = 0; i < this->numUniqueMotors; ++i) {
    #if DEBUGGER_OVERRIDE
      Serial.print("Unique model ");  Serial.print(i + 1); Serial.print(" is ");  Serial.print(this->uniqueMotorAxis[i]); Serial.print(" and has ");  Serial.print(this->numAxis[this->uniqueMotorAxis[i]]); Serial.print(" num axis.\r\n");
    #endif
    switch (this->uniqueMotorAxis[i]) {   // There is at least one motor of the current motor class
      #if NEMA17_SUPPORTED
        case Enumerators::NEMA17 :
          for (uint8_t j = 0; j < this->numAxis[i]; ++j)
            this->motors[currMotor + j] = &(this->nema17Motors[j]);
        break;
      #endif
      #if NEMA23_SUPPORTED
        case Enumerators::NEMA23 :
          for (uint8_t j = 0; j < this->numAxis[i]; ++j)
            this->motors[currMotor + j] = &(this->nema23Motors[j]);
        break;
      #endif
      #if G2_SUPPORTED
        case Enumerators::G2 :
          for (uint8_t j = 0; j < this->numAxis[i]; ++j)
            this->motors[currMotor + j] = &(this->g2Motors[j]);
        break;
      #endif
    }
    currMotor+= this->numAxis[i];
  }
}

void Device::initializeDevice(void) {
  // Initialize axis' one or multiple at a time in order to satisfy board requirements 
  // which need both axis specifications during initialization. Do not change this implementation
  // until a solution can be found which works for the G2 actuators. They currently require 
  // an instance of each motor object to be supplied during device initialization.
  struct CommandProcessing cmdProcess;                 // Utility used to facilitate with device connection commands, as done in a setup routine
  bool awaitingInput = !USE_DEFAULT_LIST;                     // 1st input will use default parameter to specify motors
  bool isComplete = false;                        // Input will always need validated and complete constructor procedure
  uint8_t correspondingMotors[MAX_AXIS];
  uint8_t numAxis;
  uint8_t iter;

  if (!Serial) {       
    switch (MICRO_CONTROLLER) {                       // Select the baud rate based on the micro-controller the macros is programmed to execute on
    case LEONARDO:
      Serial.begin(LEONARDO_BAUD);                    // The specified baud rate will be used for serial communication across the USB cable
    break;
    case TEENSY:
      Serial.begin(TEENSY_BAUD);                      // "The baud rate is ignored, and only used for Arduino compatibility. USB serial communication always occurs at full USB speed" - pjrc (Teensy development)
    break;
    default:

    break;
    }
  }

  uint8_t actualNumDevs = 0;
  while (!Serial)
    delay(500);               // Allow time for Serial port to initialize, otherwise upcoming commands involving the Serial port may fail

  #if DEBUGGER_OVERRIDE
    Serial.print("Begin microcontroller setup\r\nWaiting for a valid device list cmd\r\n");
  #endif

  while (!isComplete) {                                // Prompt the user for the device list until there is at least one valid axis allocated for by the 'struct Device myController' object
    if (awaitingInput) {
      ProgMemFcns::displayProgramMemoryString(devListInputPrompt);  // Prompt the user to input a device list                                     
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

    // Input Validation
    ProgMemFcns::displayProgramMemoryString(validatingListPrompt);
    Serial.print("\r\n"); // Terminate the line
    if (awaitingInput)
      cmdProcess.processSerialPort();                 // Process the user input
    else
      cmdProcess.processParameters(DEFAULT_MOTORS);   // Hardcoded to prevent need to manually enter each time (use serial port otherwise)
    awaitingInput = true;                             // Assuming the current input is invalid

    #if (DEBUGGER_OVERRIDE)
      // Display Free Dynamic Memory (After cmd process)
      ProgMemFcns::displayProgramMemoryString(freeMemoryPrompt);
      Serial.print(CommandProcessing::freeMemory());
      Serial.print("\r\n"); // Terminate the line
    #endif

    numAxis = cmdProcess.getNumParameters();
    isComplete = numAxis > 0;                         // Now assume it is complete until we find otherwise

    if (isComplete) {
      iter = 0;
      while (isComplete && iter < numAxis) {
        #if (DEBUGGER_OVERRIDE)
          Serial.print(cmdProcess.getParameter(iter + 1));
          Serial.print("\r\n"); // Terminate the line
          delay(50);
        #endif
        isComplete = Enumerators::validateMotor(cmdProcess.getParameter(++iter));
      }
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Motor valid : "); Serial.print(uint8_t(isComplete));  Serial.print("\r\n"); // Terminate the line
      #endif
    }

    if (isComplete) {
      for (iter = 0; iter < numAxis; ++iter) {
        correspondingMotors[iter] = cmdProcess.computeCorrespondingMotor(iter + 1);// Store the motor index associated with this motor model
        #if (DEBUGGER_OVERRIDE)
          Serial.print(F("Corresponding motor (SEE Enumerators.h) : ")); Serial.print(correspondingMotors[iter]);   Serial.print("\r\n"); // Terminate the line
        #endif
      }
      this->loadMotors(correspondingMotors, numAxis);  // WARNING : MAX OF 4 AXIS
    
      #if DEBUGGER_OVERRIDE
        Serial.print(F("Allocating dev vars... Please wait.\r\n"));
      #endif

      this->allocateDevVars();
      
      #if DEBUGGER_OVERRIDE
        Serial.print(F("Initializing Controller... Please wait.\r\n"));
      #endif

      this->initializeDeviceProcessing();
      
      actualNumDevs = this->getNumDimensions();
      isComplete = actualNumDevs == numAxis;
    }
  }
  ProgMemFcns::displayProgramMemoryString(successfullInitializationPrompt);
  Serial.print("\r\n"); // Terminate the line
  
  #if DEBUGGER_OVERRIDE
    Serial.println("Done with Arduino setup()\r\n");
  #endif
}

void Device::initializeDeviceProcessing(void) {
  uint8_t deviceAxis = 0, temp = 0, index = 0;
  while (deviceAxis < this->totalAxis) {
    if (this->motors[deviceAxis]) {
      for (uint8_t currModelAxis = 0; currModelAxis < this->numAxis[this->correspondingMotor[deviceAxis]]; ++currModelAxis) {
        this->motors[index + currModelAxis]->initializeDevice(1);
      }
    }
    temp = this->numAxis[this->correspondingMotor[deviceAxis]];
    deviceAxis += (temp > 0) ? temp : 1;  // move to the next motor index (for each unique motor model, each of their axis are in sequential addressing order)
  }
}
