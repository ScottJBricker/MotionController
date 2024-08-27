#pragma once
#ifndef _PROGRAMMEMORYSTRINGS_H_
#define _PROGRAMMEMORYSTRINGS_H_

#define MAX_STATEMENT_LENGTH 120
#define NUM_CMD_TASKS 8
#define NUM_CMD_OBJECTIVES 16
#define NUM_CMD_MOTORS  3
#define NUM_CMD_SPECIAL_SEQUENCES 1
#define NUM_CMD_SEQUENCES 26
#define NUM_HELP_LINES 27
#define MAX_PARAMETERS 4                // Store up to 4 parameters. Defined at compile time to minimize dynamic memory usage.
#define MAX_PARAMETER_LENGTH 20

#include <stdint.h>                     // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>             // Arduino built in header file (<SerialCommands.h>)... (local file here)
#include "BoardInfo.h"

// Macro to simplify reading strings from program memory
#define READ_STRING_FROM_PGM(pgm_ptr) (reinterpret_cast<const __FlashStringHelper *>(pgm_ptr))

  const char communicationTermination[] PROGMEM = "\r\n";                 // CR + LN

  // Define all commands individually first. Afterwards, we can initialize the cmdSequence and indicate the number of paramters for each.
  // Output Statements
  // This file stores output messages in program memory to reduce dynamic memory usage (aka : flash)
  const char devListInputPrompt[] PROGMEM = "Waiting for dev list...";
  const char validatingListPrompt[] PROGMEM = "Validating dev list. Please wait.";
  const char displaySequencesPrompt[] PROGMEM = "Display all regular command sequences : ";
  const char cmdSequenceNotProgrammed[] PROGMEM = "WARNING : Command was found to be valid, but there is no sequence associated with the resulting encoded sequence.";  // 115 chars, 4 AWAY FROM EXCEEDING THE MAXIMIUM (1 CHAR for null)
  const char loopBeginPrompt[] PROGMEM = "Send a command";
  const char noSerialDataPrompt[] PROGMEM = "No Serial Data to read.";
  const char freeMemoryPrompt[] PROGMEM = "Free memory: ";
  const char successfullInitializationPrompt[] PROGMEM = "Device controller initialization complete.";

  // Valid Command Strings
  // TASKS
  const char cmdTASK1[] PROGMEM = "GET";
  const char cmdTASK2[] PROGMEM = "SET";
  const char cmdTASK3[] PROGMEM = "HELP";
  const char cmdTASK4[] PROGMEM = "REMOVE";
  const char cmdTASK5[] PROGMEM = "OPEN";
  const char cmdTASK6[] PROGMEM = "INIT";
  const char cmdTASK7[] PROGMEM = "CLOSE";
  const char cmdTASK8[] PROGMEM = "DISPLAY";
  const char* const cmdTASKS[] PROGMEM = {  cmdTASK1, cmdTASK2, cmdTASK3, cmdTASK4, cmdTASK5, cmdTASK6, cmdTASK7, cmdTASK8 };

  // OBJECTIVES
  const char cmdMOTORS1[] PROGMEM = "NEMA17_MOTOR";     // These are motors, but they are an OBJECTIVE parameter
  const char cmdMOTORS2[] PROGMEM = "NEMA23_MOTOR";     // These are motors, but they are an OBJECTIVE parameter
  const char cmdMOTORS3[] PROGMEM = "G2_MOTOR";         // These are motors, but they are an OBJECTIVE parameter
  const char cmdOBJECTIVE1[] PROGMEM = "DIST";
  const char cmdOBJECTIVE2[] PROGMEM = "VEL_DIST";
  const char cmdOBJECTIVE3[] PROGMEM = "ACC_DIST";
  const char cmdOBJECTIVE4[] PROGMEM = "ORIGIN_DIST";
  const char cmdOBJECTIVE5[] PROGMEM = "ROT";
  const char cmdOBJECTIVE6[] PROGMEM = "VEL_ROT";
  const char cmdOBJECTIVE7[] PROGMEM = "ACC_ROT";
  const char cmdOBJECTIVE8[] PROGMEM = "ORIGIN_ROT";
  const char cmdOBJECTIVE9[] PROGMEM = "DEV";
  const char cmdOBJECTIVE10[] PROGMEM = "DEV_LIST";
  const char cmdOBJECTIVE11[] PROGMEM = "CLIENT";
  const char cmdOBJECTIVE12[] PROGMEM = "TIMEOUT";
  const char cmdOBJECTIVE13[] PROGMEM = "COMMANDS";
  const char* const cmdOBJECTIVES[] PROGMEM = {     // NOTE : THIS IS NOT A SEQUENCE, IT IS A LIST OF SUPPORTED OBJECTIVES
      cmdMOTORS1, cmdMOTORS2, cmdMOTORS3, 
      cmdOBJECTIVE1, cmdOBJECTIVE2, cmdOBJECTIVE3, cmdOBJECTIVE4, cmdOBJECTIVE5, 
      cmdOBJECTIVE6, cmdOBJECTIVE7, cmdOBJECTIVE8, cmdOBJECTIVE9, cmdOBJECTIVE10, 
      cmdOBJECTIVE11, cmdOBJECTIVE12, cmdOBJECTIVE13 
      };
  
  const char* const cmdMOTORS[] PROGMEM = {  cmdMOTORS1, cmdMOTORS2, cmdMOTORS3 };

  // Create Arrays for each command sequence
  // Custom Commands that do not fit the typical <TASK><OBJECTIVE><AXIS><VALUE> format.
  // array that indicates number of parameters in each sequence

  // Commands that fit the typical full command syntax (2 to 4 parameters)
  const char* const cmdSequence1[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE1   };    // GET DIST <AXIS>
  const char* const cmdSequence2[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE2   };    // GET VEL_DIST <AXIS>
  const char* const cmdSequence3[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE3   };    // GET ACC_DIST <AXIS>
  const char* const cmdSequence4[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE4   };    // GET ORIGIN_DIST <AXIS>
  const char* const cmdSequence5[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE5   };    // GET ROT <AXIS>
  const char* const cmdSequence6[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE6   };    // GET VEL_ROT <AXIS>
  const char* const cmdSequence7[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE7   };    // GET ACC_ROT <AXIS>
  const char* const cmdSequence8[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE8   };    // GET ORIGIN_ROT <AXIS>                    : Returns true if value is at origin state
  const char* const cmdSequence9[]  PROGMEM = { cmdTASK2, cmdOBJECTIVE1   };    // SET DIST <AXIS> <DISTANCE_MOTOR_UNITS>
  const char* const cmdSequence10[] PROGMEM = { cmdTASK2, cmdOBJECTIVE2   };    // SET VEL_DIST <AXIS> <ARB. LINEAR VEL>
  const char* const cmdSequence11[] PROGMEM = { cmdTASK2, cmdOBJECTIVE3   };    // SET ACC_DIST <AXIS> <ARB. LINEAR ACC>
  const char* const cmdSequence12[] PROGMEM = { cmdTASK2, cmdOBJECTIVE4   };    // SET ORIGIN_DIST <AXIS> <TO_MOVE>         : TO_MOVE = { true|Move to origin, false|set origin as current position }
  const char* const cmdSequence13[] PROGMEM = { cmdTASK2, cmdOBJECTIVE5   };    // SET ROT  <AXIS> <ROTATION_DEGREES>
  const char* const cmdSequence14[] PROGMEM = { cmdTASK2, cmdOBJECTIVE6   };    // SET VEL_ROT <AXIS> <ARB. ROTATIONAL VEL>
  const char* const cmdSequence15[] PROGMEM = { cmdTASK2, cmdOBJECTIVE7   };    // SET ACC_ROT <AXIS> <ARB. ROTATIONAL ACC>
  const char* const cmdSequence16[] PROGMEM = { cmdTASK2, cmdOBJECTIVE8   };    // SET ORIGIN_ROT <AXIS> <TO_MOVE>          : TO_MOVE = { true|Move to origin, false|set origin as current position }
  const char* const cmdSequence17[] PROGMEM = { cmdTASK5, cmdMOTORS1      };    // OPEN NEMA17_MOTOR <AXIS>
  const char* const cmdSequence18[] PROGMEM = { cmdTASK5, cmdMOTORS2      };    // OPEN NEMA23_MOTOR <AXIS>
  const char* const cmdSequence19[] PROGMEM = { cmdTASK5, cmdMOTORS3      };    // OPEN G2_MOTOR <AXIS>
  const char* const cmdSequence20[] PROGMEM = { cmdTASK6, cmdOBJECTIVE9   };    // INIT DEV <AXIS>
  const char* const cmdSequence21[] PROGMEM = { cmdTASK7, cmdOBJECTIVE9   };    // CLOSE DEV <AXIS>
  const char* const cmdSequence22[] PROGMEM = { cmdTASK6, cmdOBJECTIVE10  };    // INIT DEV_LIST
  const char* const cmdSequence23[] PROGMEM = { cmdTASK7, cmdOBJECTIVE10  };    // CLOSE DEV_LIST
  const char* const cmdSequence24[] PROGMEM = { cmdTASK4, cmdOBJECTIVE11  };    // REMOVE CLIENT
  const char* const cmdSequence25[] PROGMEM = { cmdTASK2, cmdOBJECTIVE12  };    // SET TIMEOUT
  const char* const cmdSequence26[] PROGMEM = { cmdTASK8, cmdOBJECTIVE13  };    // DISPLAY COMMANDS

  const char* const* const cmdSequences[] PROGMEM = 
      { 
        cmdSequence1, cmdSequence2, cmdSequence3, cmdSequence4, cmdSequence5, 
        cmdSequence6, cmdSequence7, cmdSequence8, cmdSequence9, cmdSequence10,
        cmdSequence11, cmdSequence12, cmdSequence13, cmdSequence14, cmdSequence15, 
        cmdSequence16, cmdSequence17, cmdSequence18, cmdSequence19, cmdSequence20,
        cmdSequence21, cmdSequence22, cmdSequence23, cmdSequence24, cmdSequence25,
        cmdSequence26
      };

  // SPECIAL SEQUENCES (AN INTEGER ARRAY IS USED TO INDICATE THE PARAMETER LENGTH)
  const char* const cmdSpecialSequence1[] PROGMEM = { cmdTASK3 };             // HELP
  const uint8_t cmdSpecialSequenceParameterCount[] PROGMEM = { 1 };    // # of parameters for each sequence
  const char* const* const specialCMDSequences[] PROGMEM = 
      { 
        cmdSpecialSequence1
      };

  // HELP MESSAGE LINES
  const char helpString1[] PROGMEM = "General Command Syntax : ";
  const char helpString2[] PROGMEM = "<TASK> <OBJECTIVE> <AXIS> <VALUE> : char *TASK, char *OBJECTIVE, uint8_t AXIS, float VALUE";
  const char helpString3[] PROGMEM = "";
  const char helpString4[] PROGMEM = "Primary Features ([3] or [4] arguments)";
  const char helpString5[] PROGMEM = "    1. Getters and Setters :";
  const char helpString6[] PROGMEM = "         1.1.a TASK = { \"GET\" [3], \"SET\" [4] }";
  const char helpString7[] PROGMEM = "         1.1.b OBJECTIVE = { <OBJ_SPEC>, \"VEL_<OBJ_SPEC>\", \"ACC_<OBJ_SPEC>\", \"ORIGIN_<OBJ_SPEC>\" }"; // Display the possible tasks
  const char helpString8[] PROGMEM = "              Where OBJ_SPEC = { \"DIST\" : Spatial Position, \"ROT\" : Radial Position }";
  const char helpString9[] PROGMEM = "         1.1.c Where : 1 <= AXIS <= 3";
  const char helpString10[] PROGMEM = "";
  const char helpString11[] PROGMEM = "";
  //  (Leave empty for now) // const char helpString11[] PROGMEM = "";
  const char helpString12[] PROGMEM = "Supplemental Features";
  const char helpString13[] PROGMEM = "    2. Device Initialization/Removal Commands :";
  const char helpString14[] PROGMEM = "        2.1.a [2/3] TASK = { \"CLOSE\" }";
  const char helpString15[] PROGMEM = "        2.1.b     OBJECTIVE = { \"DEV\", \"DEV_LIST\" }";
  const char helpString16[] PROGMEM = "";                                                               // Provide a line of space in-between new commands
  const char helpString17[] PROGMEM = "        2.2.a [4] TASK = { \"OPEN\", \"INIT\" }";
  const char helpString18[] PROGMEM = "        2.2.b     OBJECTIVE = { \"DEV\" }";
  const char helpString19[] PROGMEM = "        2.2.d     Value = { \"NEMA17_MOTOR\", \"NEMA23_MOTOR\", \"G2_MOTOR\" }";
  const char helpString20[] PROGMEM = "                  Warning : Duplicate axis specifiers invalidate the CMD.";
  const char helpString21[] PROGMEM = "    3. Special Commands : ";
  const char helpString22[] PROGMEM = "        3.1 [1] CMD = \"HELP\"          : Prinout high level command syntax and commands.";
  const char helpString23[] PROGMEM = "        3.2 [2] CMD = \"REMOVE CLIENT\" : Connection with the device will be terminated.";
  const char helpString24[] PROGMEM = "        3.3 [2] CMD = \"DISPLAY COMMANDS\"      : List all supported commands.";
  const char helpString25[] PROGMEM = "        3.3 [3] CMD = \"SET TIMEOUT <SECONDS>\" : Defines connection timeout of dev.";
  const char helpString26[] PROGMEM = "";
  const char helpString27[] PROGMEM = "";


  // HELP line sequences
  const char* const helpSequence1[]  PROGMEM = { helpString1 };
  const char* const helpSequence2[]  PROGMEM = { helpString2 };
  const char* const helpSequence3[]  PROGMEM = { helpString3 };
  const char* const helpSequence4[]  PROGMEM = { helpString4 };
  const char* const helpSequence5[]  PROGMEM = { helpString5 };
  const char* const helpSequence6[]  PROGMEM = { helpString6 };
  const char* const helpSequence7[]  PROGMEM = { helpString7 };
  const char* const helpSequence8[]  PROGMEM = { helpString8 };
  const char* const helpSequence9[]  PROGMEM = { helpString9 };
  const char* const helpSequence10[]  PROGMEM = { helpString10 };
  const char* const helpSequence11[]  PROGMEM = { helpString11 };
  const char* const helpSequence12[]  PROGMEM = { helpString12 };
  const char* const helpSequence13[]  PROGMEM = { helpString13 };
  const char* const helpSequence14[]  PROGMEM = { helpString14 };
  const char* const helpSequence15[]  PROGMEM = { helpString15 };
  const char* const helpSequence16[]  PROGMEM = { helpString16 };
  const char* const helpSequence17[]  PROGMEM = { helpString17 };
  const char* const helpSequence18[]  PROGMEM = { helpString18 };
  const char* const helpSequence19[]  PROGMEM = { helpString19 };
  const char* const helpSequence20[]  PROGMEM = { helpString20 };
  const char* const helpSequence21[]  PROGMEM = { helpString21 };
  const char* const helpSequence22[]  PROGMEM = { helpString22 };
  const char* const helpSequence23[]  PROGMEM = { helpString23 };
  const char* const helpSequence24[]  PROGMEM = { helpString24 };
  const char* const helpSequence25[]  PROGMEM = { helpString25 };
  const char* const helpSequence26[]  PROGMEM = { helpString26 };
  const char* const helpSequence27[]  PROGMEM = { helpString27 };
  
  const char* const* const helpTable[] PROGMEM = 
      { 
        helpSequence1, helpSequence2, helpSequence3, helpSequence4, helpSequence5, 
        helpSequence6, helpSequence7, helpSequence8, helpSequence9, helpSequence10,
        helpSequence11, helpSequence12, helpSequence13, helpSequence14, helpSequence15, 
        helpSequence16, helpSequence17, helpSequence18, helpSequence19, helpSequence20,
        helpSequence21, helpSequence22, helpSequence23, helpSequence24, helpSequence25,
        helpSequence26, helpSequence27
      };

/*
  const char* const* const helpTable[] PROGMEM = 
  {   helpString1, helpString2, helpString3, helpString4, helpString5,
      helpString6, helpString7, helpString8, helpString9, helpString10, 
                    helpString12, helpString13, helpString14, helpString15,
      helpString16, helpString17, helpString18, helpString19, helpString20,
      helpString21, helpString22, helpString23, helpString24, helpString25, 
      helpString26, helpString27 };

*/

  class ProgMemFcns {
  public:
    static void displayTableSequence(const char* const* const* sequenceTable, uint8_t sequence, uint8_t sequenceSize);
    static void displayProgramMemoryString(const char *programPtr);
    static void displaySequences(void);
  };

#endif