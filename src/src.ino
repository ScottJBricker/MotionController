/*
File Name :     MotionController.ino (V.1.0)
Author    :     Scott Bricker MS, Department of Electrical Engineering, University of California, Davis (2024)
Description :   This program provides interfacing to control NEMA17, NEMA23 stepper motors and G2 linear actuators.
  Here, the main loop is a command handler which manages high level commands using serial protocol, via USB. 
  During run-time, motor models and their operating conditions are input via supported commands over the USB channel.
  The command handler is designed to terminate commands via a CR + LN (\r\n).

  This program implements three primary components to dynamically interface with a range of supported motors:
  1. Motor Class Definitions ('Motor.h')  : 'Motor', 'Motor_Stepper', and each derived motor specific to a model : 'Motor_Nema17', 'Motor_Nema23', 'Motor_G2'
  2. Device Class Definition ('Device.h') : Manages arrays of motor instances
  3. Main Script                          : Essentially a command handler  
*/

// ############   Scott Header Files  ##############
#include "BoardInfo.h"
#include "CommandProcessing.h"
#include "ProgramMemoryStrings.h"
#include "ComponentConnection.h"    // Basic class to store pinout details and perform pin mode assignments
#include "Device.h"                 // Supports interfacing with all of these devices..

// ##########   Built in Header Files   ############
#include <stdint.h>                               // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <math.h>                                 // Mathematical functions such as : pow
#include <SerialCommands.h>                       // Arduino custom header file (<SerialCommands.h>)... (local file here)

// Program Global Variables, function definitions, and macros
#define ENCODER_OPTIMIZE_INTERRUPTS
#define BUFFER_SIZE 100

const bool USE_DEFAULT_LIST = false;
const char DEFAULT_MOTORS[] = "NEMA17_MOTOR";         // TODO : Enter your desired default motor list here (up to 3 motors)
//const char DEFAULT_MOTORS[] = "G2_MOTOR G2_MOTOR";  // TODO : Enter your desired default motor list here (up to 3 motors)

// Overall controller class that is used to manage IO for each, possibly unique, individual axis motor
struct Device myController;                     // (MUST be global to maintain motor axis state over lifetime of program) 

void setup(void) {}                               // Setup routine that is performed upon micro-controller startup
void loop(void) {
  uint8_t iter = 0;
  struct CommandProcessing cmdProcess;
  uint8_t encodedSequence = 0;
  bool isSetter = false;
  uint8_t axisIndex = 0;
  bool successfullTransmission = false;
  uint8_t SIZE = 0;

  myController.initializeDevice();      // Initialize the device to its startup stand-by status. Here, it will await its motor axis specifications from its new master

  #if (DEBUGGER_OVERRIDE)
    if (Serial) {
      ProgMemFcns::displayProgramMemoryString(loopBeginPrompt);
      Serial.print("\r\n"); // Terminate the line
      myController.displayDeviceState();
    }
  #endif

  while (Serial) {
    #if (DEBUGGER_OVERRIDE)
      if (!Serial) {
        Serial.begin(9600);
        while (!Serial)
          delay(10000);

        // Make sure connection is made before communicating
        if (Serial) {
          Serial.print("CRITICAL WARNING : Serial Connection was LOST!!!\r\n");
          Serial.print("Execution will be halted to prevent further errors.\r\n");
        }

        // Halt execution to prevent future errors.
        while (1)   
          delay(10000);
      }
    #endif

    // Processes the serial port buffer and enters the loop if communication is successful.
    // See 'CommandProcessing.h' for Timeout control.
    if (Serial.available() && cmdProcess.processSerialPort()) {

      #if (DEBUGGER_OVERRIDE)
        cmdProcess.displayCMD();
      #endif

      if (!cmdProcess.inputValidation()) { // restart while(Serial) if input is invalid
        Serial.print("Invalid\r\n");
        continue;
      }
      encodedSequence = cmdProcess.getCurrSequence();                 // Commands are referred to using 1-based indexing (sequence = 0 -> invalid)
      
      if ((encodedSequence >> 7)) {                                   // SPECIAL COMMANDS
        #if (DEBUGGER_OVERRIDE)
          Serial.print((encodedSequence & 0b01111111));
          Serial.print("\r\n"); // Terminate the line
        #endif
        switch (encodedSequence & 0b01111111) {                         // Remove x7 from the sequence
        case 1:                                                         // HELP
          for (iter = 0; iter < NUM_HELP_LINES; ++iter) {        // Print each line
            ProgMemFcns::displayTableSequence((const char* const* const*)helpTable, iter + 1, 1);  // The help table is a single array - much different structure than normal args to this fct.
            Serial.print("\r\n");
          }
        break;
        default:
          ProgMemFcns::displayProgramMemoryString(cmdSequenceNotProgrammed);
          Serial.print("\r\n"); // Terminate the line
        break;
        }
      }

      else {                                                            // NORMAL COMMANDS
        #if (DEBUGGER_OVERRIDE)
          Serial.print("Normal commands\r\n");
        #endif

        axisIndex = (encodedSequence >= 1 && encodedSequence <= 21) ? myController.getMotorAxis(atoi(cmdProcess.getParameter(3))) - 1 : 0; // For each of these commands, an axis must be specified as the 3rd argument.
        if (!myController.motors[axisIndex])
          return; // short circuit to restart command handler (no motor obj instance when input validation said this axis should exist)

        //Serial.print("Axis index : ");  Serial.print(axisIndex); Serial.print(" : Motor : "); Serial.println(myController.motors[axisIndex]->getCorrespondingMotor());

        if (encodedSequence >= 1 && encodedSequence <= 16) {                                  // Getters and Setters
          isSetter = encodedSequence >= 9;                                                    // >= 9 and <= 16 -> Setter
          switch (encodedSequence % 4) { 
            case 0:                                                                           // ORIGIN_ROT OR ORIGIN_DIST
            if (isSetter) {
              myController.motors[axisIndex]->redefineOrigin();                               // Reset state counter (ie. set current position as the origin)
            }
            else {                                                                            // Perform an origin search
              myController.motors[axisIndex]->originSearch();                                 // Retract motor to origin (ie. get to the origin)
            }
            break;
            default :                                                                         // Position (1), Velocity (2), or Acceleration (3)
            if (isSetter)
              myController.motors[axisIndex]->setPositionRate(atof(cmdProcess.getParameter(4)), (encodedSequence % 4) - 1);
            else {
              Serial.print(myController.motors[axisIndex]->getPositionRate((encodedSequence % 4) - 1));
              Serial.print("\r\n");     // Terminate the communication transmission
            }
            break;
          }
        }
        else if (encodedSequence >= 17 && encodedSequence <= 19) {  // open motors
          axisIndex = myController.openMotor(cmdProcess.computeCorrespondingMotor(3));
          Serial.print(axisIndex);                                                          // Return the axis used to specify the motor under control
          Serial.print("\r\n");
        }
        else {
          switch (encodedSequence) {                                                          // the encoded sequence equals the 'currSequence' for 'non special' commands
            case 20: // INIT DEV
              myController.motors[axisIndex]->initializeDevice();
              break;
            case 21: // CLOSE DEV
              myController.removeAxis(axisIndex + 1);
              break;
            case 22: // INIT DEV_LIST
              SIZE = myController.getNumDimensions();
              for (iter = 0; iter < SIZE; ++iter)
                myController.motors[iter]->initializeDevice();
              break;
            
            case 23: // CLOSE DEV_LIST
              SIZE = myController.getNumDimensions();
              // Structure is optimal to remove the highest dimension (ie. axis) first
              for (iter = SIZE; iter > 0; --iter)
                myController.removeAxis(iter);
            break;
            case 24: // REMOVE CLIENT
              return;                           // restart, 'loop()', forcing local vars to be reallocated and initialized
            case 25: // SET TIMEOUT
              CommandProcessing::setTimeout(atof(cmdProcess.getParameter(3)));

            break;
            case 26: // DISPLAY COMMANDS
              ProgMemFcns::displaySequences();
            break;
            
            default:
              ProgMemFcns::displayProgramMemoryString(cmdSequenceNotProgrammed);
              Serial.print("\r\n"); // Terminate the line
              break;
          }
        }
      }
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Checking for memory leaks..\r\n");
        ProgMemFcns::displayProgramMemoryString(freeMemoryPrompt);
        Serial.print(CommandProcessing::freeMemory());
        Serial.print("\r\n"); // Terminate the line
      #endif
    }
    delay(200);
  }
}