#define LEONARDO 0
#define TEENSY  1
#define MICRO_CONTROLLER LEONARDO       // TODO: SPECIFY YOUR DEVICE HERE!!!
#define DEBUGGER_OVERRIDE false         // Display commands that are Critical when debugging

// Same baud rate is set for both chips to reduce possible errors. Higher baud rates may still work reliably
// if the current performance is not satisfactory.
#define LEONARDO_BAUD   9600  // 57600  // Selected from the available arduino baud rates list
#define TEENSY_BAUD     9600  // 19200 57600  // USB is always 12 Mbit/sec or 480 Mbit/sec (Selected from the available Teensy4.0 baud rates list)

#if MICRO_CONTROLLER == TEENSY
  #define DEBUGGER 0                    // (Extra Debugger Commands) Teensy4.0 has plenty of memory to store the debugger commands
  #include <Teensy_PWM.h>               // For constants A0 (static const uint8_t 14) and A9 (static const uint8_t 23)
  //#include 
  
  // All motors supported for maximum capabilities. Teensy4.0 has plenty of memory, compared to the limited storage of the Arduino Leonardo
  #define G2_SUPPORTED true
  #define NEMA17_SUPPORTED false
  #define NEMA23_SUPPORTED false
#else
  #define DEBUGGER 0                    // (Extra Debugger Commands) 
  #define G2_SUPPORTED false
  #define NEMA17_SUPPORTED true
  #define NEMA23_SUPPORTED true
#endif