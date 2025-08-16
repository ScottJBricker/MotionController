#pragma once
#ifndef _BOARDINFO_H_
#define _BOARDINFO_H_

// TODO : UPDATE ME
#define DEBUGGER_OVERRIDE false         // Display commands that are Critical when debugging
#define native_USB false
#define PINOUT_CONFIGURATION 2          // Configuration unique to the microcontroller for a specific pinout configuration

#include "CommandProcessing.h"

// Built-in Headers
#include <stdint.h>                         // Standard datatype sizing : (stdint.h for c, cstdint for c++)
#include <SerialCommands.h>                 

// Pre-processor directives : Used so compiler can remove unessessary code
// Microcontroller
#define MC_UNKNOWN 0
#define MC_LEONARDO 1
#define MC_TEENSY  2

#if defined(ARDUINO) // General Arduino check
  #if defined(__AVR__)  // Check for AVR-based Arduino (e.g., ATmega328, ATmega2560)
    #define IS_ARDUINO true
  #elif defined(__arm__) && defined(TEENSYDUINO)  // Teensy (ARM)
    #define IS_TEENSY true
  #endif
#endif

#if defined(IS_ARDUINO)
  // Custom commands for Arduino (ATmega328, ATmega2560, etc.)
  #define MICRO_CONTROLLER MC_LEONARDO
  #define BAUD_RATE 9600  // 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200                           // Selected from the available arduino baud rates

#elif defined(IS_TEENSY)
  // Custom commands for Teensy (Teensy 3.x, Teensy 4.x, etc.)
    #define MICRO_CONTROLLER MC_TEENSY
  #define BAUD_RATE     460800  // 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600   // USB is always 12 Mbit/sec or 480 Mbit/sec (Selected from the available Teensy4.0 baud rates)

#else
  // Default behavior for other MCUs
  #define MICRO_CONTROLLER MC_UNKNOWN
  #define BAUD_RATE     9600  // 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600   // USB is always 12 Mbit/sec or 480 Mbit/sec (Selected from the available Teensy4.0 baud rates)

#endif

#if (PINOUT_CONFIGURATION == 1)     //    ***   Rotating Randy    ***
  // THz Lab Prototype 1 :                Pololu A-Star 32U4 Prime LV Controlled Motorized Rotator Stage
  // 1 NEMA17 Stepper Motor using TM2130 Driver
  // 1 NEMA17 Stepper motor using STEPPER_ONLINE Driver
  #define G2_SUPPORTED false
  #define NEMA17_SUPPORTED true
  #define STEPPER_ONLINE_SUPPORTED true
#elif (PINOUT_CONFIGURATION == 2)   //    ***   Precision Plunge    ***
  // THz Lab Prototype 2 :                Teensy 4.0 Controlled 2D Linear Stage
  // 1 Pololu Dual G2 High-Power Motor Driver 24v14 Shield
  // 2 Firgelli Automations FA-BS16-11-12-60 Motorized Actuator Pen with Hall Effect Feedback
  
  // All motors supported for maximum capabilities. Teensy4.0 has plenty of memory, compared to the limited storage of the Arduino Leonardo
  #define G2_SUPPORTED true
  #define NEMA17_SUPPORTED false
  #define STEPPER_ONLINE_SUPPORTED false
#elif (PINOUT_CONFIGURATION == 3)   //    ***   Whirling William    ***
  // Prototype 3 :                        Arduino Leonardo Controlled, HIGH Resolution Rotator Stage
  // 1 Oriental Motors NEMA17 Stepper Motor using STEPPER_ONLINE Driver
  
  #define G2_SUPPORTED false
  #define NEMA17_SUPPORTED false
  #define STEPPER_ONLINE_SUPPORTED true
#elif (PINOUT_CONFIGURATION == 4)
  // Tester Configuration
  
  #define G2_SUPPORTED false
  #define NEMA17_SUPPORTED false
  #define STEPPER_ONLINE_SUPPORTED true
#elif (PINOUT_CONFIGURATION == 5)   //    ***   Turning Tony    ***
  // Tester Configuration

  #define G2_SUPPORTED false
  #define NEMA17_SUPPORTED false
  #define STEPPER_ONLINE_SUPPORTED true
#endif

#if G2_SUPPORTED == true
  #include <Teensy_PWM.h>               // For constants A0 (static const uint8_t 14) and A9 (static const uint8_t 23)
#endif

#if (PINOUT_CONFIGURATION == 1)     // THz Lab Prototype 1 : Arduino Leonardo Controlled Motorized Rotator Stage for IR and THz Polarizer control
  #if (NEMA17_SUPPORTED)
  #define NEMA17_SIZE 1
  #endif

  #if (STEPPER_ONLINE_SUPPORTED)
  #define STEPPER_ONLINE_SIZE 2             // Limit software to allocating 2 instances
  #endif

  #define BOARD_SIZE 3  // assumes both nema17 and stepper online supported
#elif (PINOUT_CONFIGURATION == 2)         // THz Lab Prototype 2 : Teensy 4.0 Controlled 2D Linear Stage
  #if (G2_SUPPORTED)
  #define G2_SIZE 2                                     // Limit software to allocating 2 instances
  #endif
  #define BOARD_SIZE 2
  #define DEBUGGER 0                    // (Extra Debugger Commands) Teensy4.0 has plenty of memory to store the debugger commands

#elif (PINOUT_CONFIGURATION == 3)         // Prototype 3 : Arduino Leonardo Controlled, HIGH Resolution Rotator Stage
  // 1 Oriental Motors NEMA17 Stepper Motor using STEPPER_ONLINE Driver
  #if (STEPPER_ONLINE_SUPPORTED)
  #define STEPPER_ONLINE_SIZE 2                         // Limit software to allocating 3 instances
  #endif
  #define BOARD_SIZE 2

#elif (PINOUT_CONFIGURATION == 4)         // TESTING
  #define STEPPER_ONLINE_SIZE 1
  #define BOARD_SIZE 1
#elif (PINOUT_CONFIGURATION == 5)         // Turning Tony
  #define STEPPER_ONLINE_SIZE 1
  #define BOARD_SIZE 1
#endif

// Ensure instances are not allocated when they are not needed...
#ifndef NEMA17_SIZE
  #define NEMA17_SIZE 0
#endif

#ifndef G2_SIZE
  #define G2_SIZE 0
#endif

#ifndef STEPPER_ONLINE_SIZE
  #define STEPPER_ONLINE_SIZE 0
#endif

#ifndef DEBUGGER
  #define DEBUGGER 0                    // (Extra Debugger Commands)
#endif

class BoardInfo {
  public:
    enum SupportedMotors { G2 = 0, NEMA17, STEPPER_ONLINE };
    enum pinoutTasks { CS = 0, EN, DIR, PUL, PWM, SLEEP, FAULT, C_SENSE, IN1, IN2, CLK };   // Chip-Select, Enable, Direction, Pulse, Pulse Width Modulation (PWM), Sleep, Fault, Current Sense, Feedback 1, Feedback 2
    static const uint8_t NUM_MOTOR_MODELS = 3;  // G2, NEMA17, STEPPER_MOTOR

    static const bool isSupported[];
    static const uint8_t boardAxisCSSize[];
    static const char *boardDescription[];
    static const uint8_t *supportedBoardsAxisSize[];
    static const uint8_t **supportedBoardsAxisCSSize[];
    static const char **supportedBoardsAxisDescription[];
    static const char *supportedMotorsDescription[];
    static const uint8_t supportedMotorsAxisSize[];
    static const uint8_t *motorConfigsAxisCSSize[];

    // Motors    
    static const uint8_t motorConfigsAxisSize[];
    static const uint8_t *nema17CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t *stepperOnlineCSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t *g2CSSize[];         // # of CS to select from on each comms bus for each axis
    
    
    // NEMA17 Components
    #if (NEMA17_SUPPORTED)
    static const uint8_t nema17NumPins = 4;
    static const char nema17Description[];
    static const uint8_t NEMA17Pins[];
    static uint8_t nema17M1Pinout[]; // The motor could be connected to axis 1
    static uint8_t nema17M2Pinout[]; // The motor could be connected to axis 2
    static uint8_t nema17M3Pinout[]; // The motor could be connected to axis 3
    static uint8_t *nema17_MotorPinouts[];
    #endif

    // STEPPER_MOTOR Components
    #if (STEPPER_ONLINE_SUPPORTED)
    static const uint8_t stepperOnlineNumPins = 2;
    static volatile const char stepperOnlineDescription[];
    static const uint8_t stepperOnlinePins[];
    static uint8_t stepperOnlineM1Pinout[];               // The motor could be connected to axis 1
    static uint8_t stepperOnlineM2Pinout[];               // The motor could be connected to axis 2
    static uint8_t stepperOnlineM3Pinout[];               // The motor could be connected to axis 3
    static uint8_t *stepperPinouts[];
    #endif

    // G2 Components
    #if (G2_SUPPORTED)
    static const uint8_t g2NumPins = 7;
    static const char g2Description[];
    static const uint8_t G2Pins[]; // The G2 Motor has 7 pins to use
    static uint8_t g2M1Pinout[]; // The motor could be connected to axis 1
    static uint8_t g2M2Pinout[]; // The motor could be connected to axis 2
    static uint8_t *g2_MotorPinouts[];
    #endif

    // Discrete Configurations
    static const char *boardConfig1AxisDescription[];
    static const char *boardConfig2AxisDescription[];
    static const char *boardConfig3AxisDescription[];
    static const char *boardConfig4AxisDescription[];
    static const char *boardConfig5AxisDescription[];

    static const uint8_t *motorConfig1AxisCSSize[];
    static const uint8_t *motorConfig2AxisCSSize[];
    static const uint8_t *motorConfig3AxisCSSize[];
    static const uint8_t *motorConfig4AxisCSSize[];
    static const uint8_t *motorConfig5AxisCSSize[];

    static const uint8_t motorConfig1AxisSize[];
    static const uint8_t motorConfig2AxisSize[];
    static const uint8_t motorConfig3AxisSize[];
    static const uint8_t motorConfig4AxisSize[];
    static const uint8_t motorConfig5AxisSize[];

    // Motor Configurations
    static const uint8_t nema17Config1CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t nema17Config2CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t nema17Config3CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t nema17Config4CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t nema17Config5CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t stepperOnlineConfig1CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t stepperOnlineConfig2CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t stepperOnlineConfig3CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t stepperOnlineConfig4CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t stepperOnlineConfig5CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t g2Config1CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t g2Config2CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t g2Config3CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t g2Config4CSSize[];         // # of CS to select from on each comms bus for each axis
    static const uint8_t g2Config5CSSize[];         // # of CS to select from on each comms bus for each axis
    
    static bool validateMotor(const char *motorString);
    static int8_t computeMotorIndex(const char *motorString);
    static int8_t computeBoardConfiguration(void);
};
#endif