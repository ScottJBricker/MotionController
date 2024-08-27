#include "CommandProcessing.h"

unsigned long CommandProcessing::timeoutMS = 1000; // 1 second timeout for serial communication transmission

bool CommandProcessing::processSerialPort(void) {
  unsigned long startTime = millis();
  char currChar, prevChar = ' ';
  uint8_t index = 0;
  uint16_t maxCMDLength = MAX_PARAMETERS * MAX_PARAMETER_LENGTH + 1;  // Maxmimum # of characters in a full command + extra for terminating string.
  char inputBuffer[maxCMDLength];                                       // Allocate input buffer (locally to reduce mem usage) of the maximum statement length
  bool isDone = false;
  while (!isDone && index < (maxCMDLength - 2) && (Serial.available() > 0 || (millis() - startTime) < CommandProcessing::timeoutMS)) {
    // Only read data when it is available
    if (Serial.available()) {
      currChar = Serial.read();
      
      // Check if the character is part of the termination sequence
      if (prevChar == '\r') {
        if (currChar == '\n') {
          isDone = true;
        }
        else {
          // Carriage return sent but no new line -> reset the command buffer
          index = 0;    // Data in 'inputBuffer' remains intact but the counter indicates where the valid data stops
        }
      }
      else if (currChar != '\r') {
        inputBuffer[index] = currChar;
        index = index + 1;
      }
      prevChar = currChar;
    }
  }
  inputBuffer[index] = '\0';    // Terminate the input buffer
  if (isDone)                   // Only process the input buffer if the command was transmitted successfully
    this->processParameters(inputBuffer);
  return isDone;                // Return whether communication was successful or failed
}

// This fcn processes the final requirements when processing a command from another source
void CommandProcessing::processParameters(const char* srcParameter) {
  this->parseCMD(srcParameter);
  #if (DEBUGGER_OVERRIDE)
    Serial.print("CMD_# parameters :"); Serial.print(this->numParameters);  Serial.print("\r\n"); // Terminate the line
  #endif
}

void CommandProcessing::parseCMD(const char* cmd) {
  const char* delimiter = " ";
  char* tempCMD = strdup(cmd);                                // Copy our cmd to process locally
  char* tokenPtr = strtok(tempCMD, delimiter);
  char firstParam[MAX_PARAMETER_LENGTH + 1];
  if (tokenPtr != NULL) {
      strncpy(firstParam, tokenPtr, MAX_PARAMETER_LENGTH);
      firstParam[MAX_PARAMETER_LENGTH] = '\0';
  }
  else {
    if (tempCMD != NULL)
      free(tempCMD);   // Free the duplicated command string
    return;             // Early return if command is empty
  }
  bool isFirstParameterNumeric = isDigit(firstParam[0]);
  uint8_t firstParamValue = isFirstParameterNumeric ? atoi(firstParam) : -1;

  // Input Modes : Commands can be sent using the FULL Command or Individual Parameters at a time (done so to limit dynamic storage usage)
  // FULL Command Syntax : "<TASK> <OBJECTIVE> <AXIS> <VALUE>"
  // Partial Command Syntax : "<PARAMETER> <VALUE>", where : 1 <= PARAMETER <= 4 and PARAMETER is an integer
  // Using 'Partial Command Syntax', you will set your parameters starting with parameter = 1, then 2, then 3, then 4.
  // Setting parameter x will reset the values for parameters x + 1, x + 2, ..., 4
  

  // Clear parameters if first parameter is not numeric (ie. sending full command) or is numeric and equals 1 (Starting a new TASK CMD)
  if (!isFirstParameterNumeric || firstParamValue == 1) {
    clearParameters();
  }

  if (isFirstParameterNumeric) {
    if ((firstParamValue > this->numParameters && firstParamValue != 1)|| firstParamValue == 0) {
      free(tempCMD);   // Free the duplicated command string
      return;           // Early return if sequence is not followed properly
    }
    this->numParameters = firstParamValue - 1;  // The while loop will set the 'firstParamValue'th parameter and increment the 'numParameters'
  } else {
    strncpy(myParameters[0], firstParam, MAX_PARAMETER_LENGTH); // Copy the first parameter manually
    myParameters[0][MAX_PARAMETER_LENGTH] = '\0'; // Ensure null-termination
    this->numParameters = 1;
  }

  // Continue parsing the rest of the command
  tokenPtr = strtok(NULL, delimiter);            // identify location of the next delimiter, continuing from the previous 'strtok' call
  while (tokenPtr != NULL && this->numParameters < MAX_PARAMETERS) {
    strncpy(myParameters[this->numParameters], tokenPtr, MAX_PARAMETER_LENGTH);  // copy tokenPtr to parameters array
    myParameters[this->numParameters][MAX_PARAMETER_LENGTH] = '\0';               // Ensure null-termination
    ++this->numParameters;
    if (isFirstParameterNumeric) {
      free(tempCMD);   // Free the duplicated command string
      return; // only one value to set
    }
    tokenPtr = strtok(NULL, delimiter);
    
  }
  free(tempCMD);   // Free the duplicated command string
}


// Each supported sequence, if necessary, is compared against in order to determine if the parameter set is valid
uint8_t CommandProcessing::getParameterSequence(void) {
  bool matchFound = false;
  char taskParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter
  char objectiveParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter

  // TODO: need to check if sequence has less than 2 parameters (EX: HELP)
  if (this->numParameters < 2)
    return 0;

  uint8_t currSequence = 0; // 0-based indexing
  while (!matchFound && currSequence < NUM_CMD_SEQUENCES) {
    // Fetch the pointer to the sequence from program memory
    char* const* sequencePtr;
    memcpy_P(&sequencePtr, &cmdSequences[currSequence], sizeof(sequencePtr));

    const char* cmd;
    memcpy_P(&cmd, &sequencePtr[0], sizeof(cmd));
    strcpy_P(taskParameter, (const char*)cmd);// Retrieve a valid TASK string
    memcpy_P(&cmd, &sequencePtr[1], sizeof(cmd));
    strcpy_P(objectiveParameter, (const char*)cmd);// Retrieve a valid OBJECTIVE string

    matchFound = (strcmp(this->myParameters[0], taskParameter) == 0 && strcmp(this->myParameters[1], objectiveParameter) == 0);
    ++currSequence;
  }
  return matchFound ? currSequence : 0;
}

bool CommandProcessing::validateParameters(bool valueIsString) {                               // valueIsString is an optional parameter to satisfy the case when needing to open (need to specify) a device motor etc.
  bool isValid = this->numParameters > 0;     // It is only possible for a valid parameter is there is data to exist
  char *myParameter = nullptr;
  bool validParameter;
  uint8_t iter;
  char tempParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter
  uint8_t numChars;

  for (uint8_t parameter = 0; parameter < this->numParameters; ++parameter) {
    iter = 0;
    myParameter = this->myParameters[parameter];
    validParameter = false;

    switch (parameter) {
      case 0:   // Task
        while (!validParameter && iter < NUM_CMD_TASKS) {
          #if MICRO_CONTROLLER == LEONARDO
            strcpy_P(tempParameter, (char*)pgm_read_word(&(cmdTASKS[iter])));     // Retrieve a valid TASK string (WORKS for Arduino Leonardo)
          #else
            strcpy_P(tempParameter, (char*)&(cmdTASKS[iter]));                               // Teensy 4.0 method (TODO: test if it works for teensy and arduino)
          #endif
          validParameter = strcmp(myParameter, tempParameter) == 0;
          ++iter;
        }
        break;
      case 1: // Objective
        while (!validParameter && iter < NUM_CMD_OBJECTIVES) {
          #if MICRO_CONTROLLER == LEONARDO
            strcpy_P(tempParameter, (char*)pgm_read_word(&(cmdOBJECTIVES[iter])));      // Retrieve a valid TASK string (Works for Arduino Leonardo)
          #else
            strcpy_P(tempParameter, (char*)&(cmdOBJECTIVES[iter]));                               // Teensy 4.0 method (TODO: test if it works for teensy and arduino)
          #endif
          validParameter = strcmp(myParameter, tempParameter) == 0;
          ++iter;
        }
        break;
      case 2: // Axis
      case 3:
        
        numChars = 0;
        if (!valueIsString) {
          if (myParameter && ((*myParameter <= '9' && *myParameter >= '0') || *myParameter == '-' || *myParameter == '+')) {
            if (*myParameter == '-' || *myParameter == '+')
              ++myParameter;  // Traverse past the value sign

            
            while (myParameter && *myParameter != '\0' && *myParameter <= '9' && *myParameter >= '0') {
              ++numChars;
              ++myParameter;
            }
          }
          validParameter = (numChars && *myParameter == '\0');          // All of the previous characters were found to be numeric!!!
        }
        else {
          while (myParameter && ((*myParameter <= 'z' && *myParameter >= 'a') || (*myParameter <= 'Z' && *myParameter >= 'A'))) {
            ++numChars;
            ++myParameter;
          }
          validParameter = (numChars && *myParameter == '\0');          // All of the previous characters were found to be alphabetic!!!
        }
        break;
      default:
        return isValid;                                                     // short circuit to exit
    }
    isValid = isValid && validParameter;
  }
  return isValid;
}

uint8_t CommandProcessing::getSpecialParameterSequence(void) {
  if (this->numParameters == 0)
    return 0;                     // It is only possible for a valid parameter is there is data to exist
  
  uint8_t currParameter, numParameters, currSequence = 0;   // Counters (0-based indexing)
  char tempParameter[MAX_PARAMETER_LENGTH + 1];             // Large enough for the largest parameter
  bool matchFound = false;                                  // Required to enter while loop

  while (!matchFound && currSequence < NUM_CMD_SPECIAL_SEQUENCES) {
    numParameters = pgm_read_byte(&cmdSpecialSequenceParameterCount[currSequence]);
    if (this->numParameters != numParameters) {
      ++currSequence;
      continue; // continue to next loop of while loop
    }

    // Fetch the pointer to the sequence from program memory
    char* const* sequencePtr;
    memcpy_P(&sequencePtr, &specialCMDSequences[currSequence], sizeof(sequencePtr));
    const char* individualParameterPtr;
    
    currParameter = 0;
    matchFound = true;
    while (matchFound && currParameter < this->numParameters) {
      memcpy_P(&individualParameterPtr, &sequencePtr[currParameter], sizeof(individualParameterPtr));
      strcpy_P(tempParameter, (const char*)individualParameterPtr);// Retrieve a valid TASK string
      matchFound = strcmp(this->myParameters[currParameter++], tempParameter) == 0;
    }
    ++currSequence; // This works perfectly because the variables are referenced using 0-based indexing but the actual sequence being referred to is 1-based
  }
  return matchFound ? currSequence : 0;
}

// Function to get a parameter value by parameter (1-based)
const char* CommandProcessing::getParameter(uint8_t parameter) {
  if (parameter == 0 || parameter > this->numParameters)
    return "";  // Invalid index
  return myParameters[parameter - 1];
}

void CommandProcessing::displayCMD(void) {
  for (uint8_t iter = 0; iter < this->numParameters; ++iter) {
    Serial.print(this->getParameter(iter + 1));   
    Serial.print("\r\n"); // Terminate the line
  }
  Serial.print("\r\n"); // Terminate the line
}

bool CommandProcessing::inputValidation(void) {
  this->currSequence = this->getSpecialParameterSequence();  // Gets the sequence # corresponding to the current command. Equals 0 if it is not a special sequence.
  #if (DEBUGGER_OVERRIDE)
  Serial.print("Special Sequence : ");  Serial.print(this->currSequence); Serial.print("\r\n"); // Terminate the line
  #endif
  this->currSequence = (this->currSequence) ? this->currSequence + (1 << 7) : this->getParameterSequence();  // 'currSequence' is encoded at bit x7 to indicate whether the sequence is a 'special' sequence or not.
  #if (DEBUGGER_OVERRIDE)
    Serial.print("Reg Sequence : ");    Serial.print(this->currSequence); Serial.print("\r\n"); // Terminate the line
  #endif
  return this->currSequence != 0;
}

uint8_t CommandProcessing::computeCorrespondingMotor(uint8_t parameter) {
  char *currPtr = this->myParameters[parameter - 1];
  
  if (!Enumerators::validateMotor((const char*)currPtr))
    return 0 - 1;
  #if (G2_SUPPORTED)
    if (strcmp(currPtr, "G2_MOTOR") == 0) {
      #if (DEBUGGER_OVERRIDE)
        Serial.print("G2 Motor Detected!\r\n");
      #endif
      return Enumerators::G2;
    }
  #endif
  #if (NEMA17_SUPPORTED)
    if (strcmp(currPtr, "NEMA17_MOTOR") == 0) {
      #if (DEBUGGER_OVERRIDE)
        Serial.print("TMC2130 Motor Detected!\r\n");
      #endif
      return Enumerators::NEMA17;
    }
  #endif
  #if (NEMA23_SUPPORTED)
    if (strcmp(currPtr, "NEMA23_MOTOR") == 0) {
      #if (DEBUGGER_OVERRIDE)
        Serial.print("NEMA23 Motor Detected!\r\n");
      #endif
      return Enumerators::NEMA23;
    }
  #endif
  return 0 - 1; // Return the worst case scenario (overflow)
}