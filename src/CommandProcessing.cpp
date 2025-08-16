#include "CommandProcessing.h"

bool CommandProcessing::cmdValid = false;
uint16_t CommandProcessing::timeoutMS = 1000; // 1 second timeout for serial communication transmission
uint8_t CommandProcessing::numParameters = 0; 
uint8_t CommandProcessing::currSequence = 0;
uint8_t CommandProcessing::numLen = 0;       // 0 Chars are valid in the input buffer upon startup.
char CommandProcessing::inputBuffer[INPUT_BUFFER_SIZE] = "";
uint8_t CommandProcessing::indexOffset[MAX_PARAMETERS] = {};
uint8_t CommandProcessing::axis = 0;
int8_t CommandProcessing::cs = -1;
double CommandProcessing::value = 0;

bool CommandProcessing::processSerialPort(void) {
  const uint8_t maxCMDLength = INPUT_BUFFER_SIZE - 2;   // Maxmimum # of characters in a full command - extra for terminating string.
  const unsigned long startTime = millis();             // Current time, for communication timeout purposes.
  char recentChars[2];                                  // Store recent 2 chars to check for command terminator sequence.
  uint8_t index = 0;                                    // Index var and # chars counter.
  char inputBuffer[INPUT_BUFFER_SIZE];                  // Allocate input buffer of the input buffers size.
  bool isDone = false;                                  // Flag indicating if command terminator sequence was found.
  
  while (!isDone && index < maxCMDLength && (Serial.available() > 0 || (millis() - startTime) < CommandProcessing::timeoutMS)) {
    // Only read data when it is available
    if (Serial.available()) {
      recentChars[0] = Serial.read();
      
      // Check if the character is part of the termination sequence
      if (recentChars[1] == '\r') {
        if (recentChars[0] == '\n') {
          isDone = true;
        }
        else {
          // Carriage return sent but no new line -> reset the command buffer
          index = 0;                                    // Data in 'inputBuffer' remains intact but the counter indicates where the valid data stops
        }
      }
      else if (recentChars[0] != '\r') {
        inputBuffer[index] = recentChars[0];
        index = index + 1;
      }
      recentChars[1] = recentChars[0];
    }
  }
  inputBuffer[index] = '\0';                            // Terminate the input buffer
  CommandProcessing::numLen = index;               // Store the number of chars that are valid in the input buffer
  if (isDone)                                           // Only process the input buffer if the command was transmitted successfully
    CommandProcessing::parseCMD(inputBuffer);
  return isDone;                                        // Return whether communication was successful or failed
}

bool CommandProcessing::processString(const char* inputBuffer, const char *delimiter) {
  const uint8_t maxCMDLength = INPUT_BUFFER_SIZE - 2;   // Maxmimum # of characters in a full command - extra for terminating string.
  uint8_t index = 0;                                    // Index var and # chars counter.
  char* myPtr = inputBuffer;

  while (index < maxCMDLength && inputBuffer[index] != '\0')
    ++index;

  CommandProcessing::numLen = index;                    // Store the number of chars that are valid in the input buffer

  if (inputBuffer[index] == '\0')             // Only process the input buffer if the command was transmitted successfully
    CommandProcessing::parseCMD(inputBuffer, delimiter);
  else
    CommandProcessing::numLen = 0;
  return inputBuffer[index] == '\0';                                        // Return whether communication was successful or failed

}

// This fcn processes the final requirements when processing a command from another source
void CommandProcessing::parseCMD(const char* cmd, const char *delimiter) {
  uint8_t currParameter = 0;
  uint8_t currIndex = 0;
  uint8_t delimiterCount = 0;
  uint8_t delimiterLen = 0;
  uint8_t currLen = 0;
  uint8_t destIndex = 0;

  while (delimiter[delimiterCount++] != '\0')
    ++delimiterLen;

  // Reset the delimiter count
  char dest[INPUT_BUFFER_SIZE];
  char *start = cmd, *end;
  
  #if (DEBUGGER_OVERRIDE)
    Serial.println(cmd);
    Serial.print("Delimiter : (");Serial.print(delimiter);Serial.println(")");
    Serial.print("Delimiter length : ");Serial.println(delimiterLen);
  #endif
  
  // Copy cmd to input buffer and convert delimiter (multi-char) to string terminator ('\0').
  while (start < cmd + CommandProcessing::numLen) {
    end = strstr((const char*)start, delimiter);                                    // Search for the next delimiter sequence, returning nullptr if there is none.
    if (end == nullptr)
      end = cmd + CommandProcessing::numLen;
    
    currIndex = start - cmd;
    currLen = end - start;

    strncpy(&CommandProcessing::inputBuffer[destIndex], start, currLen);            // Copy the data to the input buffer
    CommandProcessing::inputBuffer[destIndex + currLen] = '\0';                     // Partition the input buffer via string terminations.
    CommandProcessing::indexOffset[currParameter++] = destIndex;
    destIndex += currLen + 1;                                                       // Move to next dest index (+1 extra for '\0')
    start = end + delimiterLen;                                                     // Move to start of next cmd
  }
  CommandProcessing::inputBuffer[INPUT_BUFFER_SIZE - 1] = '\0';                     // Ensure final char terminates cmd.
  CommandProcessing::numParameters = currParameter;
}

bool CommandProcessing::inputValidation(void) {
  // Fetch the "Special Parameter" Sequence #
  CommandProcessing::currSequence = CommandProcessing::getSpecialParameterSequence();  // Gets the sequence # corresponding to the current command. Equals 0 if it is not a special sequence.
  #if (DEBUGGER_OVERRIDE)
    Serial.print("Special Sequence : ");  Serial.print(CommandProcessing::currSequence); Serial.print("\r\n"); // Terminate the line
  #endif

  // Fetch the "Regular Parameter" Sequence #
  CommandProcessing::currSequence = (CommandProcessing::currSequence) ? CommandProcessing::currSequence + (1 << 7) : CommandProcessing::getParameterSequence();  // 'currSequence' is encoded at bit x7 to indicate whether the sequence is a 'special' sequence or not.
  #if (DEBUGGER_OVERRIDE)
    Serial.print("Reg Sequence : ");    Serial.print(CommandProcessing::currSequence); Serial.print("\r\n"); // Terminate the line
  #endif
  return CommandProcessing::currSequence != 0;
}

void CommandProcessing::processCMD(uint8_t prompt) {
  if (prompt == 1) {  // Motor List Initialization : List motor strings
      uint8_t iter;
      bool isComplete = CommandProcessing::numParameters > 0;                         // Now assume it is complete until we find otherwise
      uint8_t correspondingMotors[BOARD_SIZE];

      if (isComplete) {
        iter = 0;
        while (isComplete && iter < CommandProcessing::numParameters) {
          #if (DEBUGGER_OVERRIDE)
            Serial.print(CommandProcessing::getParameter(iter + 1));
            Serial.print("\r\n"); // Terminate the line
          #endif
          correspondingMotors[iter] = BoardInfo::computeMotorIndex(CommandProcessing::getParameter(iter + 1));
          isComplete = correspondingMotors[iter] >= 0;
          ++iter;
        }
        #if (DEBUGGER_OVERRIDE)
          Serial.print("Motor valid : "); Serial.print(uint8_t(isComplete));  Serial.print("\r\n"); // Terminate the line
        #endif
      }
  }
  else if (prompt == 2) {    // Normal operation
    uint8_t numParsed = (CommandProcessing::numParameters <= 2) ? 0 : sscanf(CommandProcessing::inputBuffer + CommandProcessing::indexOffset[2], "%hhu-%hhu", &CommandProcessing::axis, &CommandProcessing::cs);
    switch (numParsed) {
      case 0:
        CommandProcessing::axis = 0;  // CMD does not specify an axis
      case 1:
        CommandProcessing::cs = -1;   // CMD does not specify a chip select
        break;
    }
    CommandProcessing::value = 0;   // Reset the value var
    CommandProcessing::cmdValid = CommandProcessing::inputValidation();
  }
}

// Each supported sequence, if necessary, is compared against in order to determine if the parameter set is valid
uint8_t CommandProcessing::getParameterSequence(void) {
  bool matchFound = false;
  char taskParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter
  char objectiveParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter

  if (CommandProcessing::numParameters < 2)
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

    #if (DEBUGGER_OVERRIDE)
      Serial.print("Test ");Serial.print(taskParameter);Serial.print(" vs ");Serial.println(&CommandProcessing::inputBuffer[CommandProcessing::indexOffset[0]]);
      Serial.print("Test ");Serial.print(objectiveParameter);Serial.print(" vs ");Serial.println(&CommandProcessing::inputBuffer[CommandProcessing::indexOffset[1]]);
      delay(100);                   // give enough time for data to print before potential bugs in upcoming commands
    #endif
    matchFound = (strcmp(&CommandProcessing::inputBuffer[CommandProcessing::indexOffset[0]], taskParameter) == 0 && strcmp(&CommandProcessing::inputBuffer[CommandProcessing::indexOffset[1]], objectiveParameter) == 0);
    ++currSequence;
  }

  bool isAxisCMD = CommandProcessing::numParameters > 2;
  currSequence = (isAxisCMD && CommandProcessing::axis < 1 || CommandProcessing::axis > BOARD_SIZE) ? 0: currSequence;  // Invalidate sequence IFF the command requires an axis specifier and it is invalid
  return matchFound ? currSequence : 0;
}

bool CommandProcessing::validateParameters(bool valueIsString) {                               // valueIsString is an optional parameter to satisfy the case when needing to open (need to specify) a device motor etc.
  bool isValid = CommandProcessing::numParameters > 0;     // It is only possible for a valid parameter is there is data to exist
  char *myParameter = nullptr;
  bool validParameter;
  uint8_t iter;
  char tempParameter[MAX_PARAMETER_LENGTH + 1];    // Large enough for the largest parameter
  uint8_t numChars;
  uint8_t uint1, uint2;
  uint8_t loopSize = 0;
  uint8_t numParsed = 0;
  uint8_t parameterIndex = 0;

  while (isValid && parameterIndex < CommandProcessing::numParameters) {
    iter = 0;
    myParameter = CommandProcessing::inputBuffer[CommandProcessing::indexOffset[parameterIndex]];
    
    switch (parameterIndex) {
      case 0:   // Task
        validParameter = false;     // Assume the parameter is invalid, until we find that it matches the expected format
        while (!validParameter && iter < NUM_CMD_TASKS) {
          #if defined(IS_ARDUINO)
            strcpy_P(tempParameter, (char*)pgm_read_word(&(cmdTASKS[iter])));     // Retrieve a valid TASK string (WORKS for Arduino Leonardo)
            
          #else if defined(IS_TEENSY)
            strcpy_P(tempParameter, (char*)&(cmdTASKS[iter]));                    // Teensy 4.0 method (TODO: test if it works for teensy and arduino)
          #endif
          validParameter = strcmp(myParameter, tempParameter) == 0;               // The parameter matches the expected format if strcmp() == 0, that is, these strings are equal
          ++iter;
        }
        break;
      case 1: // Objective
        validParameter = false;     // Assume the parameter is invalid, until we find that it matches the expected format
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
        numParsed = sscanf(CommandProcessing::inputBuffer + CommandProcessing::indexOffset[2], "%hhu-%hhu", uint1, uint2);
        validParameter = numParsed > 0; // axis is always the first parameter, cs is the optional, second parameter.
        break;
      case 4:
        loopSize = CommandProcessing::getParameterLen(parameterIndex + 1);
         
        while (iter < loopSize && isdigit(CommandProcessing::inputBuffer[CommandProcessing::indexOffset[2] + iter]))
          ++iter;
        
        validParameter = iter >= loopSize; // Parameter is valid IFF each char was found to be numerical
        break;
      default:
        return isValid;                                                     // short circuit to exit
    }
    ++parameterIndex;
  }
  return isValid;
}

uint8_t CommandProcessing::getSpecialParameterSequence(void) {
  if (CommandProcessing::numParameters == 0)
    return 0;                     // It is only possible for a valid parameter is there is data to exist
  
  uint8_t currParameter, numParameters, currSequence = 0;   // Counters (0-based indexing)
  char tempParameter[MAX_PARAMETER_LENGTH + 1];             // Large enough for the largest parameter
  bool matchFound = false;                                  // Required to enter while loop

  while (!matchFound && currSequence < NUM_CMD_SPECIAL_SEQUENCES) {
    #if defined(IS_TEENSY)
      numParameters = cmdSpecialSequenceParameterCount[currSequence];
    #elif defined(IS_ARDUINO)
      numParameters = pgm_read_byte(&cmdSpecialSequenceParameterCount[currSequence]);
    #else
      numParameters = 1;
    #endif

    if (CommandProcessing::numParameters != numParameters) {
      ++currSequence;
      continue; // continue to next loop of while loop
    }

    // Fetch the pointer to the sequence from program memory
    char* const* sequencePtr;
    memcpy_P(&sequencePtr, &specialCMDSequences[currSequence], sizeof(sequencePtr));
    const char* individualParameterPtr;
    
    currParameter = 0;
    matchFound = true;
    while (matchFound && currParameter < CommandProcessing::numParameters) {
      memcpy_P(&individualParameterPtr, &sequencePtr[currParameter], sizeof(individualParameterPtr));
      strcpy_P(tempParameter, (const char*)individualParameterPtr);// Retrieve a valid TASK string
      matchFound = strcmp(&CommandProcessing::inputBuffer[CommandProcessing::indexOffset[currParameter++]], tempParameter) == 0;
    }
    ++currSequence; // This works perfectly because the variables are referenced using 0-based indexing but the actual sequence being referred to is 1-based
  }
  return matchFound ? currSequence : 0;
}

// Function to get a parameter value by parameter (1-based)
const char* CommandProcessing::getParameter(uint8_t parameter) {
  if (parameter == 0 || parameter > CommandProcessing::numParameters)
    return "";                                                    // Invalid index
  return CommandProcessing::inputBuffer + CommandProcessing::indexOffset[parameter - 1];
}

void CommandProcessing::displayCMD(void) {
  for (uint8_t parameter = 0; parameter < CommandProcessing::numParameters; ++parameter)
    Serial.println(CommandProcessing::inputBuffer + CommandProcessing::indexOffset[parameter]);
}
