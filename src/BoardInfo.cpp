#include "BoardInfo.h"

#if (NEMA17_SUPPORTED)
  const char BoardInfo::nema17Description[] = "NEMA17_MOTOR";
#endif
#if (STEPPER_ONLINE_SUPPORTED)
  volatile const char BoardInfo::stepperOnlineDescription[] = "STEPPER_ONLINE";
#endif
#if (G2_SUPPORTED)
  const char BoardInfo::g2Description[] = "G2_MOTOR";
#endif

// Boards
const char* BoardInfo::boardDescription[] = { "Rotating Randy", "Precision Plunge", "Whirling William", "Rotator Tester", "Turning Tony", nullptr };

// Link board config pointers to each board config
const uint8_t *BoardInfo::nema17CSSize[] = { BoardInfo::nema17Config1CSSize, BoardInfo::nema17Config2CSSize, BoardInfo::nema17Config3CSSize, BoardInfo::nema17Config4CSSize, BoardInfo::nema17Config5CSSize };            // # of CS to select from on each comms bus for each axis
const uint8_t *BoardInfo::stepperOnlineCSSize[] = { BoardInfo::stepperOnlineConfig1CSSize, BoardInfo::stepperOnlineConfig2CSSize, BoardInfo::stepperOnlineConfig3CSSize, BoardInfo::stepperOnlineConfig4CSSize, BoardInfo::stepperOnlineConfig5CSSize };     // # of CS to select from on each comms bus for each axis
const uint8_t *BoardInfo::g2CSSize[] = { BoardInfo::g2Config1CSSize, BoardInfo::g2Config2CSSize, BoardInfo::g2Config3CSSize, BoardInfo::g2Config4CSSize, BoardInfo::g2Config5CSSize };             // # of CS to select from on each comms bus for each axis

// Motors : enum SupportedMotors { G2 = 0, NEMA17, STEPPER_ONLINE };
const bool BoardInfo::isSupported[] = { G2_SUPPORTED, NEMA17_SUPPORTED, STEPPER_ONLINE_SUPPORTED };                   // MUST be same order as : BoardInfo::SupportedMotors
const char* BoardInfo::supportedMotorsDescription[] = { "G2_MOTOR", "NEMA17_MOTOR", "STEPPER_ONLINE", nullptr };  // MUST be same order as : BoardInfo::SupportedMotors
const uint8_t BoardInfo::supportedMotorsAxisSize[] = { G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE };         // MUST be same order as : BoardInfo::SupportedMotors

// Initialize vars for each config. Correct config will be set afterwards
const uint8_t BoardInfo::nema17Config1CSSize[] = { 1 };            // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::nema17Config2CSSize[] = { 0 };            // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::nema17Config3CSSize[] = { 0 };            // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::nema17Config4CSSize[] = { 0 };            // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::nema17Config5CSSize[] = { 0 };            // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::stepperOnlineConfig1CSSize[] = { 1 };     // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::stepperOnlineConfig2CSSize[] = { 0 };     // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::stepperOnlineConfig3CSSize[] = { 2 };     // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::stepperOnlineConfig4CSSize[] = { 2 };     // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::stepperOnlineConfig5CSSize[] = { 2 };     // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::g2Config1CSSize[] = { 0, 0 };             // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::g2Config2CSSize[] = { 1, 1 };             // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::g2Config3CSSize[] = { 0, 0 };             // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::g2Config4CSSize[] = { 0, 0 };             // # of CS to select from on each comms bus for each axis
const uint8_t BoardInfo::g2Config5CSSize[] = { 0, 0 };             // # of CS to select from on each comms bus for each axis

#if (PINOUT_CONFIGURATION == 1)       //    ***   Rotating Randy    ***
  // THz Lab Prototype 1 :                  Pololu A-Star 32U4 Prime LV Controlled Motorized Rotator Stage
  // 1 NEMA17 Stepper Motor using TM2130 Driver
  // 1 NEMA17 Stepper motor using STEPPER_ONLINE Driver

  // Define TM2130 Driver Pinout
  #if (NEMA17_SUPPORTED)
    const uint8_t BoardInfo::NEMA17Pins[] = { 3 /* CS */, 2 /* EN */, 0 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */, 0 /* Q */, 0 /* I */, 0 /* CLK */ };
    uint8_t BoardInfo::nema17M1Pinout[] = { 3, 5, 2, 4 };        // Direction, Pulse, Enable, and Chip Select pins, respectively
    uint8_t* BoardInfo::nema17_MotorPinouts[] = { BoardInfo::nema17M1Pinout };
  #endif

  // Define StepperOnline Driver Pinout
  #if (STEPPER_ONLINE_SUPPORTED)
    const uint8_t BoardInfo::stepperOnlinePins[] =   { 0 /* CS */, 3 /* EN */, 0 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 2 /* FAULT */, 0 /* C_SENSE */,0 /* Q */, 0 /* I */, 0 /* CLK */ };
    uint8_t BoardInfo::stepperOnlineM1Pinout[] =    { 8, 9, 10, 11 };      // Direction, Pulse, Fault/Alarm, and Enable pins, respectively
    uint8_t* BoardInfo::stepperPinouts[] = { BoardInfo::stepperOnlineM1Pinout };
  #endif
#elif (PINOUT_CONFIGURATION == 2)       //    ***   Precision Plunge    ***
  // THz Lab Prototype 2 :                    Teensy 4.0 Controlled 2D Linear Stage
  // 1 Pololu Dual G2 High-Power Motor Driver 24v14 Shield
  // 2 Firgelli Automations FA-BS16-11-12-60 Motorized Actuator Pen with Hall Effect Feedback
  
  #if (G2_SUPPORTED)
    const uint8_t BoardInfo::G2Pins[] =     { 0 /* CS */, 0 /* EN */, 0 /* DIR */, 0 /* PUL */, 4 /* PWM */, 2 /*SLEEP */, 3 /* FAULT */, 1 /* C_SENSE */, 5 /* Q */, 6 /* I */, 0 /* CLK */ };
    uint8_t BoardInfo::g2M1Pinout[] = { 2, A0, 1, A4, A1, 5, 6 };      // Direction, Current-Sense, Sleep, Fault, Pulse width modulation, In-phase Hall-Effect Sensor, quadrature Hall-Effect Sensor
    uint8_t BoardInfo::g2M2Pinout[] = { 4, A9, 3, A5, A8, 7, 8 };  // Direction, Current-Sense, Sleep, Fault, Pulse width modulation, In-phase Hall-Effect Sensor, quadrature Hall-Effect Sensor
    uint8_t* BoardInfo::g2_MotorPinouts[] = { BoardInfo::g2M1Pinout, BoardInfo::g2M2Pinout };
  #endif
#elif (PINOUT_CONFIGURATION == 3)         //    ***   Whirling William    ***
  // Prototype 3 :                              Arduino Leonardo Controlled, HIGH Resolution, Dual CS Rotator Stage
  // M1:CS0 1 NEMA 17 Stepper Motor using STEPPER_ONLINE Driver
  // M1:CS1 1 Oriental Motors High Resolution (0.05 degrees/step), High Torque, Low Speed, Stepper Motor : PK264A1A-SG36
  
  #if (STEPPER_ONLINE_SUPPORTED)
    const uint8_t BoardInfo::stepperOnlinePins[] =   { 3 /* CS */, 0 /* EN */, 2 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */,0 /* Q */, 0 /* I */, 4 /* CLK */ };
    uint8_t BoardInfo::stepperOnlineM1Pinout[] =    { 10, 11, 9, 8, 12 };       // Enable, Pulse, Direction, CS, and Clock pins, respectively
    uint8_t BoardInfo::stepperOnlineM2Pinout[] =    { 5, 2, 3, 0, 0 };          // Enable, Pulse, Direction, CS, and Clock pins, respectively
    uint8_t* BoardInfo::stepperPinouts[] = { BoardInfo::stepperOnlineM1Pinout, BoardInfo::stepperOnlineM2Pinout, nullptr };
  #endif

#elif (PINOUT_CONFIGURATION == 4) // TESTING

  const uint8_t BoardInfo::stepperOnlinePins[] =   { 3 /* CS */, 0 /* EN */, 2 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */,0 /* Q */, 0 /* I */, 4 /* CLK */ };
  uint8_t BoardInfo::stepperOnlineM1Pinout[] =    { 2, 3, 4, 5, 6 };      // Enable, Pulse, Direction, CS, and Clock pins, respectively
  uint8_t BoardInfo::stepperOnlineM2Pinout[] =    { 7, 7, 7, 7, 7};      // Enable, Pulse, Direction, CS, and Clock pins, respectively
  uint8_t BoardInfo::stepperOnlineM3Pinout[] =    { 8, 8, 8, 8, 8 };     // Enable, Pulse, Direction, CS, and Clock pins, respectively
  uint8_t* BoardInfo::stepperPinouts[] = { BoardInfo::stepperOnlineM1Pinout, BoardInfo::stepperOnlineM2Pinout, BoardInfo::stepperOnlineM3Pinout };
#elif (PINOUT_CONFIGURATION == 5)         //    ***   Turning Tony    ***
  // Prototype 5 :                              Arduino Micro Controlled, HIGH Resolution, Dual CS Rotator Stage
  // M1:CS0 1 NEMA 17 Stepper Motor using STEPPER_ONLINE Driver
  // M1:CS1 1 Oriental Motors High Resolution (0.05 degrees/step), High Torque, Low Speed, Stepper Motor : PK264A1A-SG36
  
  #if (STEPPER_ONLINE_SUPPORTED)
    const uint8_t BoardInfo::stepperOnlinePins[] =   { 3 /* CS */, 0 /* EN */, 2 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */,0 /* Q */, 0 /* I */, 4 /* CLK */ };
    uint8_t BoardInfo::stepperOnlineM1Pinout[] =    { A0, A2, A1, A3, A4 };       // Enable, Pulse, Direction, CS, and Clock pins, respectively
    uint8_t BoardInfo::stepperOnlineM2Pinout[] =    { 0, 0, 0, 0, 0 };          // Enable, Pulse, Direction, CS, and Clock pins, respectively
    uint8_t* BoardInfo::stepperPinouts[] = { BoardInfo::stepperOnlineM1Pinout, nullptr, nullptr };
  #endif  
#endif

const char* BoardInfo::boardConfig1AxisDescription[] = { "1::0::NEMA17_MOTOR::1::0", "1::0::STEPPER_ONLINE::1::0", nullptr };       // Each string : <Motor>::<Axis>::<CS>
const char* BoardInfo::boardConfig2AxisDescription[] = { "1::0::G2_MOTOR::1::0", "1::1::G2_MOTOR::2::0", nullptr };                  // Each string : <Motor>::<Axis>::<CS>
const char* BoardInfo::boardConfig3AxisDescription[] = { "1::0::STEPPER_ONLINE::1::0", "1::1::STEPPER_ONLINE::1::1", nullptr };     // Each string : <Motor>::<Axis>::<CS>
const char* BoardInfo::boardConfig4AxisDescription[] = { "1::0::STEPPER_ONLINE::1::0", "1::1::STEPPER_ONLINE::1::1", nullptr };     // Each string : <Motor>::<Axis>::<CS>
const char* BoardInfo::boardConfig5AxisDescription[] = { "1::0::STEPPER_ONLINE::1::0", "1::1::STEPPER_ONLINE::1::1", nullptr };     // Each string : <Motor>::<Axis>::<CS>

const uint8_t BoardInfo::motorConfig1AxisSize[] = { 0, 1, 1 };  // Specify motor config here instead of : G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE for each board config
const uint8_t BoardInfo::motorConfig2AxisSize[] = { 2, 0, 0 };  // Specify motor config here instead of : G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE for each board config
const uint8_t BoardInfo::motorConfig3AxisSize[] = { 0, 0, 1 };  // Specify motor config here instead of : G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE for each board config
const uint8_t BoardInfo::motorConfig4AxisSize[] = { 0, 0, 1 };  // Specify motor config here instead of : G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE for each board config
const uint8_t BoardInfo::motorConfig5AxisSize[] = { 0, 0, 1 };  // Specify motor config here instead of : G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE for each board config


const uint8_t *BoardInfo::motorConfig1AxisCSSize[] = { BoardInfo::g2CSSize[0], BoardInfo::nema17CSSize[0], BoardInfo::stepperOnlineCSSize[0], nullptr };// G2_CS, NEMA17_CS, STEPPER_ONLINE_CS
const uint8_t *BoardInfo::motorConfig2AxisCSSize[] = { BoardInfo::g2CSSize[1], BoardInfo::nema17CSSize[1], BoardInfo::stepperOnlineCSSize[1], nullptr };// G2_CS, NEMA17_CS, STEPPER_ONLINE_CS
const uint8_t *BoardInfo::motorConfig3AxisCSSize[] = { BoardInfo::g2CSSize[2], BoardInfo::nema17CSSize[2], BoardInfo::stepperOnlineCSSize[2], nullptr };// G2_CS, NEMA17_CS, STEPPER_ONLINE_CS
const uint8_t *BoardInfo::motorConfig4AxisCSSize[] = { BoardInfo::g2CSSize[3], BoardInfo::nema17CSSize[3], BoardInfo::stepperOnlineCSSize[3], nullptr };// G2_CS, NEMA17_CS, STEPPER_ONLINE_CS
const uint8_t *BoardInfo::motorConfig5AxisCSSize[] = { BoardInfo::g2CSSize[3], BoardInfo::nema17CSSize[3], BoardInfo::stepperOnlineCSSize[3], nullptr };// G2_CS, NEMA17_CS, STEPPER_ONLINE_CS

const uint8_t BoardInfo::motorConfigsAxisSize[] = { G2_SIZE, NEMA17_SIZE, STEPPER_ONLINE_SIZE };
const uint8_t *BoardInfo::motorConfigsAxisCSSize[] = { BoardInfo::g2CSSize[PINOUT_CONFIGURATION - 1], BoardInfo::nema17CSSize[PINOUT_CONFIGURATION - 1], BoardInfo::stepperOnlineCSSize[PINOUT_CONFIGURATION - 1], nullptr };
const uint8_t *BoardInfo::supportedBoardsAxisSize[] = { BoardInfo::motorConfig1AxisSize, BoardInfo::motorConfig2AxisSize, BoardInfo::motorConfig3AxisSize, BoardInfo::motorConfig4AxisSize, BoardInfo::motorConfig5AxisSize, nullptr };
const uint8_t **BoardInfo::supportedBoardsAxisCSSize[] = { BoardInfo::motorConfig1AxisCSSize, BoardInfo::motorConfig2AxisCSSize, BoardInfo::motorConfig3AxisCSSize, BoardInfo::motorConfig4AxisCSSize, BoardInfo::motorConfig5AxisCSSize, nullptr };
const char **BoardInfo::supportedBoardsAxisDescription[] = { BoardInfo::boardConfig1AxisDescription, BoardInfo::boardConfig2AxisDescription, BoardInfo::boardConfig3AxisDescription, BoardInfo::boardConfig4AxisDescription, BoardInfo::boardConfig5AxisDescription, nullptr };

int8_t BoardInfo::computeMotorIndex(const char *motorString) {//const char* processedCMD, uint8_t *endOfCMD, bool* isValid, const uint8_t SIZE) {
  #if (G2_SUPPORTED)
    if (strcmp(motorString, BoardInfo::g2Description) == 0)
      return (int8_t)BoardInfo::G2;
  #endif
  #if (STEPPER_ONLINE_SUPPORTED)
    if (strcmp(motorString, BoardInfo::stepperOnlineDescription) == 0)
      return (int8_t)BoardInfo::STEPPER_ONLINE;
  #endif
  #if (NEMA17_SUPPORTED)
    if (strcmp(motorString, BoardInfo::nema17Description) == 0)
      return (int8_t)BoardInfo::NEMA17;
  #endif
  return -1;
}

int8_t BoardInfo::computeBoardConfiguration(void) {
  const char *boardStringSequence = CommandProcessing::getParameter(1);
  const char *myPtr;                                                    // Ptr to a board description
  char testString[INPUT_BUFFER_SIZE];
  uint8_t *indexOffset = CommandProcessing::getStrLen();
  uint8_t numParameters = CommandProcessing::getNumParameters();
  uint8_t currParameter;                                                // counter to ensure each parameter of the sequence matches the board string sequence
  uint8_t numChars;

  uint8_t boardCounter = 0;                                             // board index to test
  bool keyFound = false;
  while (!keyFound && BoardInfo::boardDescription[boardCounter] != nullptr) {
    keyFound = true;
    myPtr = BoardInfo::boardDescription[boardCounter];
    
    // Search each string of the cmd and ensure they match the board description
    for (currParameter = 0; currParameter < numParameters - 1; ++currParameter) {
      numChars = indexOffset[currParameter + 1] - indexOffset[currParameter] - 1;
      strncpy(testString, &myPtr[indexOffset[currParameter]], numChars);
      testString[numChars] = '\0';  // terminate the string
      keyFound = strcmp(&boardStringSequence[indexOffset[currParameter]], testString) == 0;
      if (!keyFound)
        break;
    }

    // Check the final parsed string
    if (keyFound)
      keyFound = strcmp(&boardStringSequence[indexOffset[numParameters - 1]], &myPtr[indexOffset[numParameters - 1]]) == 0;  // Test the FINAL cmd parameter
      
    // Check if the sequence matches
    if (!keyFound)
      ++boardCounter;
  }
  return keyFound ? boardCounter : -1;
}

bool BoardInfo::validateMotor(const char *motorString) {//const char* processedCMD, uint8_t *endOfCMD, bool* isValid, const uint8_t SIZE) {
  int8_t motorIndex = BoardInfo::computeMotorIndex(motorString);
  return BoardInfo::isSupported[motorIndex];
}