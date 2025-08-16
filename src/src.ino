/*
File Name :     MotionController.ino (V.1.0)
Author    :     Scott Bricker MS, Department of Electrical Engineering, University of California, Davis (2025)
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

#if defined(IS_ARDUINO)
  #include <avr/wdt.h>                              // Watchdog Timer Library
#endif

// Program Global Variables, function definitions, and macros
#define ENCODER_OPTIMIZE_INTERRUPTS
#define BASIC_LED_MAX 20
#define LED_STRIP_MAX 165
//#define BUFFER_SIZE 100


const uint8_t startPrompt = Device::MC_NAME;
const bool USE_DEFAULT_LIST = true; // Use Board motor config list as input
const bool USE_DEFAULT_BOARD = false;
const char DEFAULT_BOARD[] = "Rotator Tester";
//const char DEFAULT_MOTORS[] = "1::0::STEPPER_ONLINE::1::0 1::1::STEPPER_ONLINE::1::1";        // TODO : Enter your desired default motor list here (up to 3 motors)

// Overall controller class that is used to manage IO for each, possibly unique, individual axis motor
struct Device myController;                             // (MUST be global to maintain motor axis state over lifetime of program) 


#if defined(IS_ARDUINO)
  // Disable Watchdog timer early in the hardware startup procedure.
  void disableWatchdog() __attribute__((naked)) __attribute__((section(".init3")));
  void disableWatchdog() {
    
      MCUSR = 0;        // Clear the reset cause flags
      wdt_disable();    // Disable the watchdog timer


  }
#endif



void setup(void) { 
  // Setup routine that is performed upon micro-controller startup
  if (!native_USB) {
    // On native USB boards, Serial is always evaluated to equal TRUE, 
    // so this comparison cannot be used to check for an open connection during run-time.
    // Therefore, we must establish connection on non-native board now.
    Serial.begin(BAUD_RATE);
  }
  else {
    Serial.begin(BAUD_RATE);
  }
  /*
  pinMode(11, OUTPUT);
  analogWrite(11, 0);
  pinMode(9, OUTPUT);
  analogWrite(9, BASIC_LED_MAX);
  */

  DeviceDynamics::resetStaticVars();
  MotorDriver::resetStaticVars();
  Device::resetStaticVars();

  // Initialize static vars starting with lowest level classes first
  DeviceDynamics::allocateStaticVars();
  #if (DEBUGGER_OVERRIDE)
    Serial.println("DeviceDynamics::allocateStaticVars() done");
    delay(100);
  #endif
  MotorDriver::allocateStaticVars();
  #if (DEBUGGER_OVERRIDE)
    Serial.println("MotorDriver::allocateStaticVars() done");
    delay(100);
  #endif
  Device::allocateStaticVars();
  #if (DEBUGGER_OVERRIDE)
    Serial.println("Device::allocateStaticVars() done");
    delay(100);
  #endif
  
  Serial.flush();
}

void fastISR(void) {
  if (Serial) {
    Serial.println("Tester");
  }
}


void loop(void) {
  uint8_t iter = 0;
  struct CommandProcessing cmdProcess;
  uint8_t controllerAxis = 0;
  bool temp = false;
  uint8_t SIZE = 0;
  uint8_t currSequence = 0;
  uint8_t pwmCounter = 0;
  uint8_t pwmDx = (uint8_t)ceil(LED_STRIP_MAX / 20); // Discretize LED into 20 voltage levels
  bool dir = true;
  int8_t motorAxis = -1;
  int8_t motorCS = -1;      // CS on the motor axis comms bus
  bool mainIsAxisCMD = false;
  uint8_t currPrintout;
  uint8_t axis = 0;
  int8_t cs = -1;
  
  myController.initializeDevice();      // Initialize the device to its startup stand-by status. Here, it will await its motor axis specifications from its new master
  
  // Enable the watchdog timer with a X-second timeout
  //wdt_enable(WDTO_8S);        // System has at most 60 seconds to complete a move command before timer triggers software reset
  // ^ Do not implement the active watchdog timer until motor movement is modularized so that movements that take too long to process.
  // For instance, if the movement requires more than 8 seconds, then the watchdog timer is guarenteed to trigger a software reset.


  #if (DEBUGGER_OVERRIDE)
    if (Serial) {
      ProgMemFcns::displayProgramMemoryString(loopBeginPrompt);
      Serial.print("\r\n"); // Terminate the line
      myController.displayDeviceState();
    }
  #endif

  /*
  analogWrite(9, 0);
  pinMode(11, OUTPUT);
  analogWrite(11, pwmCounter);
  */

  while (Serial) {
    #if defined(IS_ARDUINO)
      wdt_reset();              // Reset the watchdog timer, execution operating properly
    #endif
    
    /*
    if (dir && pwmCounter >= ceil(LED_STRIP_MAX / pwmDx) * pwmDx) {
      dir = false;
      pwmCounter = floor(LED_STRIP_MAX / pwmDx) * pwmDx;
    }
    else if (!dir && pwmCounter == 0) {
      dir = true;
      pwmCounter = pwmDx;
    }
    else if (dir)
      pwmCounter+= pwmDx;
    else
      pwmCounter-= pwmDx;

    analogWrite(11, pwmCounter);
    */
    
    #if (DEBUGGER_OVERRIDE)
      if (!Serial) {
        Serial.begin(BAUD_RATE);

        // Halt execution to prevent future errors.
        while (1)   
          delay(10000);
      }
    #endif

    // Processes the serial port buffer and enters the loop if communication is successful.
    // See 'CommandProcessing.h' for Timeout control.
    if (Serial.available() && cmdProcess.processSerialPort()) {
      currPrintout = 0;

      cmdProcess.processCMD();
      
      #if (DEBUGGER_OVERRIDE)
        cmdProcess.displayCMD();
      #endif

      currSequence = cmdProcess.getCurrSequence();
      
      // Process SPECIAL COMMANDS
      if ((currSequence >> 7)) {                                   // SPECIAL COMMANDS
        #if (DEBUGGER_OVERRIDE)
          Serial.print("Special Sequence : ");
          Serial.print((currSequence & 0b01111111));
          #if defined(IS_ARDUINO) 
            Serial.print(F("\r\n")); // Terminate the line
          #endif
        #endif
        switch (currSequence & 0b01111111) {                         // Remove x7 from the sequence
        case 1:                                                         // HELP
          for (iter = 0; iter < NUM_HELP_LINES; ++iter) {        // Print each line
            ProgMemFcns::displayTableSequence((const char* const* const*)helpTable, iter + 1, 1);  // The help table is a single array - much different structure than normal args to this fct.
            #if defined(IS_ARDUINO) 
              Serial.print(F("\r\n")); // Terminate the line
            #endif
          }
        break;
        default:
          ProgMemFcns::displayProgramMemoryString(cmdSequenceNotProgrammed);
          #if defined(IS_ARDUINO) 
            Serial.print(F("\r\n")); // Terminate the line
          #endif
        break;
        }
        continue;
      }

      // Process NORMAL COMMANDS
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Normal commands\r\n");
      #endif

      // Input Validation
      #if defined(IS_ARDUINO)
        mainIsAxisCMD = pgm_read_byte(isAxisCMD + (currSequence - 1) * sizeof(bool)) & 0x01;
      #else
        mainIsAxisCMD = isAxisCMD[currSequence - 1] & 0x01; // Read directly from program memory
      #endif
    
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Your cmd sequence ---> ");  Serial.println(currSequence);
        mainIsAxisCMD ? Serial.println("Axis cmd") : Serial.println("NOT AN Axis CMD");
        Serial.print("var : mainIsAxisCMD == ");  mainIsAxisCMD ? Serial.println("TRUE") : Serial.println("FALSE");
      #endif
      
      axis = CommandProcessing::axis;   // axis via serial port after command processing
      cs = CommandProcessing::cs;       // cs via serial port after command processing
      
      if (mainIsAxisCMD) {
        if (cs < 0) {
          cs = myController.getCurrDevCS(axis);   // cs not provided. fetch previous cs
        }
        else {
          myController.setDevCS(axis, cs);      // cs provided via serial port, store it for next cmd.
        }
        if (axis < 1 || (currSequence != 5 && axis > BOARD_SIZE) && myController.motors[axis - 1][cs]->isValid(motorCS) == false)
          continue;   //short circuit toj complete command. axis does not have an instance and there is no intent on opening the axis...
        motorCS = myController.getMotorCS(axis);
        myController.motors[axis - 1][cs]->setCS(motorCS); // Set cs on motor so that command will operate on the correct motor over the shared comms bus
      }

      #if (DEBUGGER_OVERRIDE)
        Serial.print("var : currSequence == ");  Serial.println(currSequence);
        myController.motors[axis - 1][cs] == nullptr ? Serial.println("no motor :(") : Serial.println("good");
      #endif

      // the encoded sequence equals the 'currSequence' for 'non special' commands
      if (currSequence >= 1 && currSequence <= 4) {
        switch (currSequence) {
          case 1:     // REMOVE CLIENT : Device resets to startup state
            // can do either to reset to startup -> myController.initializeDevice();
            return;                           // restart, 'loop()', forcing local vars to be reallocated and initialized
          case 2:     // RESET CLIENT : Software reset of the device
            #if defined(IS_ARDUINO)
              // Use Watchdog Timeout to Reset Microcontroller
              wdt_disable();            // Disable the active watchdog timer
              wdt_enable(WDTO_2S);      // Watchdog Timer Enabled for 2 seconds to trigger hardware reset.
              delay(2200);              // Trigger timeout of the watchdog timer
            #endif
          break;
          case 3:     // CLOSE DEV_LIST
            SIZE = myController.getNumDimensions();
            // Structure is optimal to remove the highest dimension (ie. axis) first
            for (iter = SIZE; iter > 0; --iter)
              myController.removeAxis(iter);
          break;
          case 4:     // CLOSE DEV <AXIS>
            myController.removeAxis(controllerAxis);
        }
        continue;
      }
      else if (currSequence >= 5 && currSequence <= 6) {
        switch (currSequence) {
          case 5: {   // OPEN MOTOR <int axis> <string motor>
            // TODO : PROGRAM ME
            
            int motor = BoardInfo::computeMotorIndex(cmdProcess.getParameter(4));  // Convert string motorDescription to motor index
            if (motor >= 0) {   // valid if motor >= 0
              // OPEN MOTOR <AXIS> <MOTOR_STRING>
              controllerAxis = myController.openMotor((uint8_t)motor);
              Serial.print(controllerAxis);                                                          // Return the axis used to specify the motor under control
              Serial.print("\r\n");
            }
            
            break;
          }
          case 6:     // SET TIMEOUT <seconds>
            CommandProcessing::setTimeout(atof(cmdProcess.getParameter(3)));
          break;
        }
        continue;
      }
      else if (currSequence >= 7 && currSequence <= 10) {
        switch (currSequence) {
          case 7:     // GET EN
          (myController.motors[axis - 1][cs]->getMotorEnable()) ? Serial.print("1\r\n") : Serial.print("0\r\n");
          break;
          case 8:     // GET POS
            Serial.print(myController.motors[axis - 1][cs]->getPositionRate(0), 3);   // Get position
            break;
          case 9:     // GET VEL
            Serial.print(myController.motors[axis - 1][cs]->getPositionRate(1));      // Get speed
          break;
          case 10:    // GET ACC
            Serial.print(myController.motors[axis - 1][cs]->getPositionRate(2));      // Get acceleration
          break;
        }
        Serial.print("\r\n");     // Terminate the communication transmission
        continue;
      }
      else if (currSequence >= 11 && currSequence <= 14) {
        switch (currSequence) {
          case 11:    // SET POS
            myController.motors[axis - 1][cs]->setPositionRate(atof(cmdProcess.getParameter(4)), 0);  // Set Position (0th derivative)
          break;
          case 12:    // MOVE STEPS
            myController.motors[axis - 1][cs]->move(atof(cmdProcess.getParameter(4)));
          break;
          case 13:    // GET ORIGIN 
          
          break;
          case 14:    // SEARCH ORIGIN 
            myController.motors[axis - 1][cs]->originSearch();                                 // Retract motor to origin (ie. get to the origin)
          break;
          
        }
        continue;
      }
      else if (currSequence >= 14 && currSequence <= 17) {
        switch (currSequence) {
          case 15: {   // SET EN <AXIS> <bool>
            if (cmdProcess.getNumParameters() >= 4) {
              bool setState = atoi(cmdProcess.getParameter(4)) != 0;
              myController.motors[axis - 1][cs]->driver.deviceDriver->setCS(motorCS);
              myController.motors[axis - 1][cs]->driver.deviceDriver->enableMotor(atoi(cmdProcess.getParameter(4)) != 0);
            }
          break;
          }
          case 16:    // INIT DEV_LIST -> Equivalent to selecting a new board or : CMD = REMOVE CLIENT
            myController.initializeDeviceProcessing();
          break;
          case 17:    // INIT DEV <AXIS>
            myController.initializeDeviceProcessing(axis, cs);
          break;
        }
        continue;
      }
      else if (currSequence == 18) {
        // DISPLAY COMMANDS
        ProgMemFcns::displaySequences();
        continue;
      }
      else if (currSequence >= 19 && currSequence <= 21) {
        switch (currSequence) {
          case 19:    // REDEFINE ORIGIN
            myController.motors[axis - 1][cs]->driver.deviceDriver->redefineOrigin();                               // Reset state counter (ie. set current position as the origin)
          break;
          case 20:    // SET VEL
            myController.motors[axis - 1][cs]->setPositionRate(atof(cmdProcess.getParameter(4)), 1);    // Set velocity
          break;
          case 21:    // SET ACC
            myController.motors[axis - 1][cs]->setPositionRate(atof(cmdProcess.getParameter(4)), 2);    // Set acceleration
          break;
        }
        continue;
      }
      else {
        ProgMemFcns::displayProgramMemoryString(cmdSequenceNotProgrammed);
        Serial.print("\r\n"); // Terminate the line
      }
      
      #if (DEBUGGER_OVERRIDE)
        Serial.print("Checking for memory leaks..\r\n");
        ProgMemFcns::displayProgramMemoryString(freeMemoryPrompt);
        Serial.print(CommandProcessing::freeMemory());
        Serial.print("\r\n"); // Terminate the line
      #endif
    }
    delay(1);
  }
}