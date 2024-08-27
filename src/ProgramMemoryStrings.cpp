#include "ProgramMemoryStrings.h"


// WARNING : This function is only valid for PROGRAM MEMORY stored strings. 
// RAM strings will not perform properly here.
void ProgMemFcns::displayProgramMemoryString(const char *programPtr) {
  char buffer[MAX_STATEMENT_LENGTH];
  strcpy_P(buffer, (const char *)programPtr);   // Copy the program-memory string into the buffer
  Serial.print(buffer);
}

void ProgMemFcns::displayTableSequence(const char* const* const* sequenceTable, uint8_t tableSequenceArray, uint8_t sequenceSize) {
  if (!Serial)
    return; // Invalid input
  
  // For command sequences, which are arrays of arrays of strings
  tableSequenceArray = tableSequenceArray - 1;  // convert to 0-based indexing
  #if MICRO_CONTROLLER == TEENSY
    // Retrieve the pointer to the selected sequence directly
    const char* const* selectedSequence = sequenceTable[tableSequenceArray];

    for (uint8_t iter = 0; iter < sequenceSize; ++iter) {
      Serial.print(selectedSequence[iter]);
      Serial.print(" ");
    }
  #else
    const char* const* cmdSequence = reinterpret_cast<const char* const*>(pgm_read_word(&(sequenceTable[tableSequenceArray])));
    for (uint8_t iter = 0; iter < sequenceSize; ++iter) {
      const char* cmd = reinterpret_cast<const char*>(pgm_read_word(&(cmdSequence[iter])));
      displayProgramMemoryString(cmd);
      Serial.print(" ");
    }
  #endif
}

void ProgMemFcns::displaySequences(void) {
  #if MICRO_CONTROLLER == TEENSY
    Serial.print(displaySequencesPrompt);   
  #else
    ProgMemFcns::displayProgramMemoryString(displaySequencesPrompt);
  #endif

  Serial.print("\r\n"); // Terminate the line
  for (uint8_t iter = 0; iter < NUM_CMD_SEQUENCES; ++iter) {
    ProgMemFcns::displayTableSequence((const char* const* const*)cmdSequences, iter + 1, 2);
    Serial.print("\r\n"); // Terminate the line
  }
}