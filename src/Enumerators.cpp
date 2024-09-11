#include "Enumerators.h"

// NEMA17 Stepper motor using the TM2130 driver.
#if (NEMA17_SUPPORTED)
const char Enumerators::nema17_description[] = "NEMA17_MOTOR";
const uint8_t Enumerators::NEMA17Pins[] = { 3 /* CS */, 2 /* EN */, 0 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */, 0 /* Q */, 0 /* I */ };
uint8_t Enumerators::nema17_m1Pinout[] = { 3, 5, 2, 4 };        // Direction, Pulse, Enable, and Chip Select pins, respectively
uint8_t* Enumerators::nema17_MotorPinouts[] = { Enumerators::nema17_m1Pinout };
#endif

// Nema23 Bipolar
#if (NEMA23_SUPPORTED)
const char Enumerators::nema23_description[] = "NEMA23_MOTOR";
const uint8_t Enumerators::NEMA23Pins[] =   { 0 /* CS */, 0 /* EN */, 0 /* DIR */, 1 /* PUL */, 0 /* PWM */, 0 /*SLEEP */, 0 /* FAULT */, 0 /* C_SENSE */,0 /* Q */, 0 /* I */ };   // Nema23 PINOUT : [0]Enable, [1]Chip-Select
uint8_t Enumerators::nema23_m1Pinout[] =    { 8, 9 };      // Direction and Pulse pins, respectively
uint8_t Enumerators::nema23_m2Pinout[] =    { 10, 11 };    // Direction and Pulse pins, respectively
uint8_t* Enumerators::nema23_MotorPinouts[] = { Enumerators::nema23_m1Pinout, Enumerators::nema23_m2Pinout };
#endif

// G2
#if (G2_SUPPORTED)
const char Enumerators::g2_description[] = "G2_MOTOR";
const uint8_t Enumerators::G2Pins[] =     { 0 /* CS */, 0 /* EN */, 0 /* DIR */, 0 /* PUL */, 4 /* PWM */, 2 /*SLEEP */, 3 /* FAULT */, 1 /* C_SENSE */, 5 /* Q */, 6 /* I */ };
uint8_t Enumerators::g2_m1Pinout[] = { 2, A9, 1, A5, A8, 5, 6 };      // Direction, Current-Sense, Sleep, Fault, Pulse width modulation, In-phase Hall-Effect Sensor, quadrature Hall-Effect Sensor
uint8_t Enumerators::g2_m2Pinout[] = { 4, A0, 3, A4, A1, 7, 8 };  // Direction, Current-Sense, Sleep, Fault, Pulse width modulation, In-phase Hall-Effect Sensor, quadrature Hall-Effect Sensor
uint8_t* Enumerators::g2_MotorPinouts[] = { Enumerators::g2_m1Pinout, Enumerators::g2_m2Pinout };
#endif

bool Enumerators::validateMotor(const char *motorString) {//const char* processedCMD, uint8_t *endOfCMD, bool* isValid, const uint8_t SIZE) {
  return (strcmp(motorString, "G2_MOTOR") == 0 || strcmp(motorString, "NEMA17_MOTOR") == 0 || strcmp(motorString, "NEMA23_MOTOR") == 0);
}