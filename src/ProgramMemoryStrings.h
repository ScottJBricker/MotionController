#pragma once
#ifndef _PROGRAMMEMORYSTRINGS_H_
#define _PROGRAMMEMORYSTRINGS_H_

#define MAX_STATEMENT_LENGTH 120
#define NUM_CMD_TASKS 12
#define NUM_CMD_OBJECTIVES 16
#define NUM_CMD_MOTORS  3
#define NUM_CMD_SPECIAL_SEQUENCES 1
#define NUM_CMD_SEQUENCES 21
#define NUM_HELP_LINES 28

#include <stdint.h>                     // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>             // Arduino built in header file (<SerialCommands.h>)... (local file here)
#include "BoardInfo.h"

const char communicationTermination[] PROGMEM = "\r\n";                 // CR + LN

// Define all commands individually first. Afterwards, we can initialize the cmdSequence and indicate the number of paramters for each.
// Output Statements
// This file stores output messages in program memory to reduce dynamic memory usage (aka : flash)
const char boardInputPrompt[] PROGMEM = "Waiting for board name...";
const char validatingBoardPrompt[] PROGMEM = "Validating board alias. Please wait.";
const char devListInputPrompt[] PROGMEM = "Waiting for motor list...";

const char validatingListPrompt[] PROGMEM = "Validating motor list. Please wait.";
const char displaySequencesPrompt[] PROGMEM = "Display all regular command sequences : ";
const char cmdSequenceNotProgrammed[] PROGMEM = "WARNING : Command was found to be valid, but there is no sequence associated with the resulting encoded sequence.";  // 115 chars, 4 AWAY FROM EXCEEDING THE MAXIMIUM (1 CHAR for null)
const char loopBeginPrompt[] PROGMEM = "Send a command";
const char noSerialDataPrompt[] PROGMEM = "No Serial Data to read.";
const char freeMemoryPrompt[] PROGMEM = "Free memory: ";
const char successfullInitializationPrompt[] PROGMEM = "Device controller initialization complete.";
const char deviceStatePrintoutHeader[] PROGMEM = "Device State Printout";

// Valid Command Strings
// TASKS
const char cmdTASK1[] PROGMEM = "GET";
const char cmdTASK2[] PROGMEM = "SET";
const char cmdTASK3[] PROGMEM = "MOVE";
const char cmdTASK4[] PROGMEM = "HELP";
const char cmdTASK5[] PROGMEM = "REMOVE";
const char cmdTASK6[] PROGMEM = "OPEN";
const char cmdTASK7[] PROGMEM = "INIT";
const char cmdTASK8[] PROGMEM = "CLOSE";
const char cmdTASK9[] PROGMEM = "DISPLAY";
const char cmdTASK10[] PROGMEM = "RESET";
const char cmdTASK11[] PROGMEM = "REDEFINE";
const char cmdTASK12[] PROGMEM = "SEARCH";
const char* const cmdTASKS[] PROGMEM = {  cmdTASK1, cmdTASK2, cmdTASK3, cmdTASK4, cmdTASK5, cmdTASK6, cmdTASK7, cmdTASK8, cmdTASK9, cmdTASK10, cmdTASK11, cmdTASK12 };

// OBJECTIVES
const char cmdMOTORS1[] PROGMEM = "NEMA17_MOTOR";     // These are motors, but they are an OBJECTIVE parameter
const char cmdMOTORS2[] PROGMEM = "STEPPER_ONLINE";   // These are motors, but they are an OBJECTIVE parameter
const char cmdMOTORS3[] PROGMEM = "G2_MOTOR";         // These are motors, but they are an OBJECTIVE parameter

const char cmdBoardConfig1[] PROGMEM = "Rotating Randy";
const char cmdBoardConfig2[] PROGMEM = "Precision Plunge";
const char cmdBoardConfig3[] PROGMEM = "Whirling William";


const char cmdOBJECTIVE1[] PROGMEM = "EN";            // Enable
const char cmdOBJECTIVE2[] PROGMEM = "POS";           // Position : Distance, Rotation, etc.
const char cmdOBJECTIVE3[] PROGMEM = "VEL";           // Velocity : Linear, rotational, etc.
const char cmdOBJECTIVE4[] PROGMEM = "ACC";           // Acceleration : Linear, rotational, etc.
const char cmdOBJECTIVE5[] PROGMEM = "ORIGIN";        // Origin
const char cmdOBJECTIVE6[] PROGMEM = "DEV";           // Single Device (will be used to dynamically allocate according to the board configuration specified)
const char cmdOBJECTIVE7[] PROGMEM = "DEV_LIST";      // Device List (REMOVE THIS COMMAND : Only 1 device so this would do nothing. It was previously used for individual axis'
const char cmdOBJECTIVE8[] PROGMEM = "CLIENT";       
const char cmdOBJECTIVE9[] PROGMEM = "TIMEOUT";
const char cmdOBJECTIVE10[] PROGMEM = "COMMANDS";
const char cmdOBJECTIVE11[] PROGMEM = "MOTOR";
const char cmdOBJECTIVE12[] PROGMEM = "CHANNEL";
const char cmdOBJECTIVE13[] PROGMEM = "ALL_CHANNELS";
const char cmdOBJECTIVE14[] PROGMEM = "STEPS";
const char cmdOBJECTIVE15[] PROGMEM = "DEGREES";
const char cmdOBJECTIVE16[] PROGMEM = "METERS";

const char* const cmdOBJECTIVES[] PROGMEM = {         // NOTE : THIS IS NOT A SEQUENCE, IT IS A LIST OF SUPPORTED OBJECTIVES
    cmdMOTORS1,     cmdMOTORS2,     cmdMOTORS3, 
    cmdOBJECTIVE1,  cmdOBJECTIVE2,  cmdOBJECTIVE3,  cmdOBJECTIVE4,  cmdOBJECTIVE5,  
    cmdOBJECTIVE6,  cmdOBJECTIVE7,  cmdOBJECTIVE8,  cmdOBJECTIVE9,  cmdOBJECTIVE10, 
    cmdOBJECTIVE11, cmdOBJECTIVE12, cmdOBJECTIVE13, cmdOBJECTIVE14, cmdOBJECTIVE15, 
    cmdOBJECTIVE16
    };

const char* const cmdMOTORS[] PROGMEM = {  cmdMOTORS1, cmdMOTORS2, cmdMOTORS3 };

// Create Arrays for each command sequence
// Custom Commands that do not fit the typical <TASK><OBJECTIVE><AXIS><VALUE> format.
// array that indicates number of parameters in each sequence

// Commands that fit the typical full command syntax (2 to 4 parameters)
const char* const cmdSequence1[]  PROGMEM = { cmdTASK5, cmdOBJECTIVE8   };      // REMOVE CLIENT
const char* const cmdSequence2[]  PROGMEM = { cmdTASK10,cmdOBJECTIVE8   };      // RESET CLIENT
const char* const cmdSequence3[]  PROGMEM = { cmdTASK8, cmdOBJECTIVE7   };      // CLOSE DEV_LIST
const char* const cmdSequence4[]  PROGMEM = { cmdTASK8, cmdOBJECTIVE6   };      // CLOSE DEV <AXIS> 
const char* const cmdSequence5[]  PROGMEM = { cmdTASK6, cmdOBJECTIVE11  };      // OPEN MOTOR <AXIS> <MOTOR_STRING>
const char* const cmdSequence6[]  PROGMEM = { cmdTASK2, cmdOBJECTIVE9   };      // SET TIMEOUT
const char* const cmdSequence7[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE1   };      // GET EN <AXIS> 
const char* const cmdSequence8[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE2   };      // GET POS <AXIS> (Position : Linear, rotational, etc.)
const char* const cmdSequence9[]  PROGMEM = { cmdTASK1, cmdOBJECTIVE3   };      // GET VEL <AXIS> (Velocity : Linear, rotational, etc.)
const char* const cmdSequence10[] PROGMEM = { cmdTASK1, cmdOBJECTIVE4   };      // GET ACC <AXIS> (Acceleration : Linear, rotation, etc.)
const char* const cmdSequence11[] PROGMEM = { cmdTASK2, cmdOBJECTIVE2   };      // SET POS  <AXIS> <ARB. LINEAR or ROTATIONAL>
const char* const cmdSequence12[] PROGMEM = { cmdTASK3, cmdOBJECTIVE14  };      // MOVE STEPS <AXIS> <# STEPS>
const char* const cmdSequence13[] PROGMEM = { cmdTASK1, cmdOBJECTIVE5   };      // GET ORIGIN <AXIS>        : Moves to origin
const char* const cmdSequence14[] PROGMEM = { cmdTASK12,cmdOBJECTIVE5   };      // SEARCH ORIGIN <AXIS>     : Searches for hardware origin
const char* const cmdSequence15[] PROGMEM = { cmdTASK2, cmdOBJECTIVE1   };      // SET EN <AXIS> <bool>
const char* const cmdSequence16[] PROGMEM = { cmdTASK7, cmdOBJECTIVE7   };      // INIT DEV_LIST
const char* const cmdSequence17[] PROGMEM = { cmdTASK7, cmdOBJECTIVE6   };      // INIT DEV <AXIS>
const char* const cmdSequence18[] PROGMEM = { cmdTASK9, cmdOBJECTIVE10  };      // DISPLAY COMMANDS
const char* const cmdSequence19[] PROGMEM = { cmdTASK11,cmdOBJECTIVE5   };      // REDEFINE ORIGIN <AXIS> 
const char* const cmdSequence20[] PROGMEM = { cmdTASK2, cmdOBJECTIVE3   };      // SET VEL <AXIS> <ARB. LINEAR or ROTATIONAL VEL>
const char* const cmdSequence21[] PROGMEM = { cmdTASK2, cmdOBJECTIVE4   };      // SET ACC <AXIS> <ARB. LINEAR or ROTATIONAL ACC>

const char* const* const cmdSequences[] PROGMEM = 
    { 
      cmdSequence1, cmdSequence2, cmdSequence3, cmdSequence4, cmdSequence5, 
      cmdSequence6, cmdSequence7, cmdSequence8, cmdSequence9, cmdSequence10,
      cmdSequence11, cmdSequence12, cmdSequence13, cmdSequence14, cmdSequence15, 
      cmdSequence16, cmdSequence17, cmdSequence18, cmdSequence19, cmdSequence20, 
      cmdSequence21
    };

const bool isAxisCMD[] PROGMEM = 
  {
    false, false, false, true, true, 
    false, true, true, true, true, 
    true,   true, true, true, true, 
    false, true, false, true, true, 
    true
  };

// SPECIAL SEQUENCES (AN INTEGER ARRAY IS USED TO INDICATE THE PARAMETER LENGTH)
const char* const cmdSpecialSequence1[] PROGMEM = { cmdTASK4 };             // HELP
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
const char helpString12[] PROGMEM = "Supplemental Features";
const char helpString13[] PROGMEM = "   2. Device Initialization/Removal Commands :";
const char helpString14[] PROGMEM = "        2.1.a [2/3] TASK = { \"CLOSE\" }";
const char helpString15[] PROGMEM = "        2.1.b     OBJECTIVE = { \"DEV\", \"DEV_LIST\" }";
const char helpString16[] PROGMEM = "";                                                               // Provide a line of space in-between new commands
const char helpString17[] PROGMEM = "        2.2.a [4] TASK = { \"OPEN\", \"INIT\" }";
const char helpString18[] PROGMEM = "        2.2.b     OBJECTIVE = { \"DEV\" }";
const char helpString19[] PROGMEM = "        2.2.d     Value = { \"NEMA17_MOTOR\", \"STEPPER_ONLINE\", \"G2_MOTOR\" }";
const char helpString20[] PROGMEM = "                  Warning : Duplicate axis specifiers invalidate the CMD.";
const char helpString21[] PROGMEM = "   3. Special Commands : ";
const char helpString22[] PROGMEM = "        3.1 [1] CMD = \"HELP\"          : Prinout high level command syntax and commands.";
const char helpString23[] PROGMEM = "        3.2 [2] CMD = \"RESET DEVICE\"  : Software reset of the device.";
const char helpString24[] PROGMEM = "        3.3 [2] CMD = \"REMOVE CLIENT\" : Device resets to startup state.";
const char helpString25[] PROGMEM = "        3.4 [2] CMD = \"DISPLAY COMMANDS\"      : List all supported commands.";
const char helpString26[] PROGMEM = "        3.5 [3] CMD = \"SET TIMEOUT <SECONDS>\" : Defines connection timeout of dev.";
const char helpString27[] PROGMEM = "";
const char helpString28[] PROGMEM = "";

// HELP line sequences
const char* const helpSequence1[]   PROGMEM = { helpString1 };
const char* const helpSequence2[]   PROGMEM = { helpString2 };
const char* const helpSequence3[]   PROGMEM = { helpString3 };
const char* const helpSequence4[]   PROGMEM = { helpString4 };
const char* const helpSequence5[]   PROGMEM = { helpString5 };
const char* const helpSequence6[]   PROGMEM = { helpString6 };
const char* const helpSequence7[]   PROGMEM = { helpString7 };
const char* const helpSequence8[]   PROGMEM = { helpString8 };
const char* const helpSequence9[]   PROGMEM = { helpString9 };
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
const char* const helpSequence28[]  PROGMEM = { helpString28 };

const char* const* const helpTable[] PROGMEM = 
    { 
      helpSequence1, helpSequence2, helpSequence3, helpSequence4, helpSequence5, 
      helpSequence6, helpSequence7, helpSequence8, helpSequence9, helpSequence10,
      helpSequence11, helpSequence12, helpSequence13, helpSequence14, helpSequence15, 
      helpSequence16, helpSequence17, helpSequence18, helpSequence19, helpSequence20,
      helpSequence21, helpSequence22, helpSequence23, helpSequence24, helpSequence25,
      helpSequence26, helpSequence27, helpSequence28
    };

class ProgMemFcns {
public:
  static void displayTableSequence(const char* const* const* sequenceTable, uint8_t sequence, uint8_t sequenceSize);
  static void displayProgramMemoryString(const char *programPtr);
  static void displaySequences(void);
};

#endif