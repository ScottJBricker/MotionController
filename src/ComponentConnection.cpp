#include "ComponentConnection.h"

uint8_t *ComponentConnection::isrPins = nullptr;
bool *ComponentConnection::isrStartState = nullptr;
uint8_t ComponentConnection::isrSIZE = 0;
uint32_t ComponentConnection::interruptCount = 0;
uint32_t ComponentConnection::isrMax[2] = { 0, 0 }; 
bool ComponentConnection::isInitial = true;
uint32_t ComponentConnection::isrPeriodMicroseconds[2] = { 0, 0 };
uint8_t *ComponentConnection::resumePins = nullptr;
uint8_t *ComponentConnection::resumeState = nullptr;
uint8_t ComponentConnection::resumeSIZE = 0;
float ComponentConnection::errorPerStep = 0;
float ComponentConnection::accumulatedTimingError = 0;
uint8_t ComponentConnection::currISRStage = 0;
uint8_t ComponentConnection::isrPeriods = 0;
uint32_t ComponentConnection::netCount = 0;
uint32_t ComponentConnection::currInterruptCount = 0;


uint8_t ComponentConnection::getNumPins(void) {
  return this->numPins;
}

void ComponentConnection::begin(void) {
  switch (this->correspondingMotors[this->csIndex]) {
    #if (NEMA17_SUPPORTED)
      case BoardInfo::NEMA17 :
        pinMode(this->pinout[BoardInfo::NEMA17Pins[BoardInfo::EN]], OUTPUT);      // Set EN pin to Output
        pinMode(this->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], OUTPUT);      // Set CS pin to Output
        digitalWrite(this->pinout[BoardInfo::NEMA17Pins[BoardInfo::EN]], LOW);    // Set EN pin to LOW
        digitalWrite(this->pinout[BoardInfo::NEMA17Pins[BoardInfo::CS]], HIGH);   // Set CS pin to HIGH
        break;
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      case BoardInfo::STEPPER_ONLINE:
        /*
        Serial.print("PUL : ");Serial.println(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]]);
        Serial.print("DIR : ");Serial.println(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::DIR]]);
        Serial.print("EN : ");Serial.println(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::EN]]);
        Serial.print("CLK : ");Serial.println(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]]);
        Serial.print("CS : ");Serial.println(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]]);
        */
        
        // Assign pin modes
        pinMode(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], OUTPUT);    // Set PULSE pin to OUTPUT
        pinMode(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::DIR]], OUTPUT);    // Set DIRECTION pin to OUTPUT
        pinMode(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::EN]], OUTPUT);     // Set ENABLE pin to OUTPUT
        pinMode(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], OUTPUT);    // Set CLOCK pin to OUTPUT
        pinMode(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]], OUTPUT);    // Set CHIP SELECT pin to OUTPUT
        delay(1);

        // Initialize pin states
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);  // Disable asynchronous state changes
        delayMicroseconds(1);                                                           // Give time for gates to update (>59ns)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::PUL]], LOW);  // Pulses not desired (Timing Irrelevant Here) (>58ns)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::DIR]], LOW);  // Set CW (Timing Irrelevant Here) (>0ns)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]], LOW);   // Select CS Line 0, setup time (>36ns)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::EN]], LOW);   // Disable CS 0 Motor Driver, setup time (>25ns)
        delayMicroseconds(1);                                                           // Setup time before CLK (36ns)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], HIGH); // Trigger a state change on the enable flip-flop (EN = false stored)
        delayMicroseconds(1);                                                           // CLK Pulse duration (>20ns)

        // For each remaining CS Line (NOTE: Adjusting the CS Line can trigger a rising edge on the newly selected CS Line)
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CS]], HIGH);   // Select CS Line 1, setup time (>36ns)
        // Hardware Note : Rising edge is trigged on CS Line 1 when CS Line updated : setup time (+>20ns)
        delayMicroseconds(1);                                                           // Setup time before CLK (56ns)
        // Sequential CS Lines : output active low followed by inverter on each CLK Demux CS output keeps CLK LOW for ALL CS except the selected CS

        // Turn OFF CLK to prevent state changes.
        digitalWrite(this->pinout[BoardInfo::stepperOnlinePins[BoardInfo::CLK]], LOW);  // Disable asynchronous state changes
        delayMicroseconds(1);                                                           // Give time for gates to update (>59ns)
        break;
    #endif
    #if (G2_SUPPORTED)
      case BoardInfo::G2 :
        pinMode(this->pinout[BoardInfo::G2Pins[BoardInfo::IN1]], INPUT);
        pinMode(this->pinout[BoardInfo::G2Pins[BoardInfo::IN2]], INPUT);
        break;
    #endif
  }
}

void ComponentConnection::specifyConnection(uint8_t *motors, uint8_t *axis, uint8_t *cs, uint8_t SIZE) {
  if (motors == nullptr || axis == nullptr || cs == nullptr)
    return;
    
  if (this->SIZE > 0) {
    delete this->correspondingMotors;
    delete this->axis;
    delete this->axisCS;
  }

  this->SIZE = SIZE;
  this->correspondingMotors = new uint8_t[SIZE];
  this->axis = new uint8_t[SIZE];
  this->axisCS = new uint8_t[SIZE];
  
  for (uint8_t index = 0; index < SIZE; ++index) {
    this->correspondingMotors[index] = motors[index];
    this->axis[index] = axis[index];
    this->axisCS[index] = cs[index];
  }
}

void ComponentConnection::setCS(uint8_t cs) {
  this->csIndex = cs;
  this->selectPinout(this->correspondingMotors[cs], this->axis[cs]);
}

void ComponentConnection::selectPinout(uint8_t cs) {
  if (cs >= this->SIZE)
    return;

  this->csIndex = cs;
  
  uint8_t numPins;
  uint8_t **tempPinouts;                          // Each chip has pinouts pre-defined
  switch (this->correspondingMotors[this->csIndex]) {
    #if (NEMA17_SUPPORTED)
      case BoardInfo::NEMA17 :
        numPins = BoardInfo::nema17NumPins;
        tempPinouts = BoardInfo::nema17_MotorPinouts;
        break;
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      case BoardInfo::STEPPER_ONLINE:
      numPins = BoardInfo::stepperOnlineNumPins;
        tempPinouts = BoardInfo::stepperPinouts;
        break;
    #endif
    #if (G2_SUPPORTED)
      case BoardInfo::G2 :
        numPins = BoardInfo::g2NumPins;
        tempPinouts = BoardInfo::g2_MotorPinouts;
        break;
    #endif
    default:
      break;
  }
  
  this->numPins = numPins;
  this->pinout = tempPinouts[this->axis[this->csIndex] - 1];
}

void ComponentConnection::selectPinout(uint8_t correspondingMotor, uint8_t axis) {
  if (axis < 1)
    return;
    
  uint8_t numPins;
  uint8_t **tempPinouts;                          // Each chip has pinouts pre-defined
  switch (this->correspondingMotors[this->csIndex]) {
    #if (NEMA17_SUPPORTED)
      case BoardInfo::NEMA17 :
        numPins = BoardInfo::nema17NumPins;
        tempPinouts = BoardInfo::nema17_MotorPinouts;
        break;
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      case BoardInfo::STEPPER_ONLINE:
      numPins = BoardInfo::stepperOnlineNumPins;
        tempPinouts = BoardInfo::stepperPinouts;
        break;
    #endif
    #if (G2_SUPPORTED)
      case BoardInfo::G2 :
        numPins = BoardInfo::g2NumPins;
        tempPinouts = BoardInfo::g2_MotorPinouts;
        break;
    #endif
    default:
      break;
  }
  
  this->numPins = numPins;
  this->pinout = tempPinouts[axis - 1];
}

void ComponentConnection::sendPulses(const uint8_t *pins, const uint8_t *startState, int8_t pulseDurationExponent, float pulseDuration, int8_t offDurationExponent, float offDuration,  uint32_t numPulses, uint8_t SIZE) {
  #if (DEBUGGER_OVERRIDE)
    Serial.print("Send ");Serial.print(numPulses); Serial.println(" pulses");
    Serial.print("pulse duration = ");Serial.print(pulseDuration);Serial.print(" x 10^");Serial.println(pulseDurationExponent);
    Serial.print("off duration = ");Serial.print(offDuration);Serial.print(" x 10^");Serial.println(offDurationExponent);
  #endif

  // Convert to units of local function (microseconds)
  float onPulseMicroseconds = pulseDuration * pow(10, pulseDurationExponent + 6);  // convert to microseconds (This supports up to 4294 seconds)
  float offPulseMicroseconds = offDuration * pow(10, offDurationExponent + 6);     // convert to microseconds (This supports up to 4294 seconds)

  float clockDuration = pulseDuration * pow(10, pulseDurationExponent+6) + offDuration * pow(10, offDurationExponent+6);  // Clock duration in microseconds

  /*
  Solution using a timer from TimerOne library in Arduino (avr) based microcontrollers.
  Idea : Use a timer to reliably shape the pulse at set intervals
  if ((uint32_t)onPulseMicroseconds == (uint32_t)offPulseMicroseconds) {
    // Deallocate previous memory
    if (ComponentConnection::isrPins)
      delete[] ComponentConnection::isrPins;
    if (ComponentConnection::isrStartState)
      delete[] ComponentConnection::isrStartState;

    // Setup ISR to Perform Pin Toggling
    ComponentConnection::isrPins = new uint8_t[SIZE];
    ComponentConnection::isrStartState = new bool[SIZE];
    ComponentConnection::isrSIZE = SIZE;

    ComponentConnection::isrPeriodMicroseconds[0] = (uint32_t)onPulseMicroseconds;
    ComponentConnection::isrPeriodMicroseconds[1] = (uint32_t)(onPulseMicroseconds + 1);  // Round up

    for (uint8_t iter = 0; iter < SIZE; ++iter) {
      ComponentConnection::isrPins[iter] = pins[iter];
      ComponentConnection::isrStartState[iter] = startState[iter] != 0;
    }

    // Initialize timer period, Duty Cycle = 50%
    float percentageMissing = (clockDuration - 2 * (uint32_t)onPulseMicroseconds) / 1; // These will need to round up to compensate for the error accumulation
    ComponentConnection::isrPeriods = 2;  // 2 unique periods (ceil and floor to compensate for error accumulation)
    ComponentConnection::isrMax[0] = floor(numPulses * 2 * (1 - percentageMissing));  // Use two unique period to achieve the exact desired total duration. Note : shorter pulses (by 1us) are moved to begining of pulse train 
    ComponentConnection::isrMax[1] = ceil(numPulses * 2 * percentageMissing); 
    ComponentConnection::netCount = 2 * numPulses;

    //Serial.print("Percentage Missing : ");Serial.println(percentageMissing);
    //Serial.print("N0 : ");Serial.println(ComponentConnection::isrMax[0]);
    //Serial.print("N1 : ");Serial.println(ComponentConnection::isrMax[1]);



    // Initialize Timer ISR Vars
    ComponentConnection::currISRStage = 0;  // Restart the ISR processing
    ComponentConnection::isInitial = true;  // alternate between states and pulse duration times
    ComponentConnection::interruptCount = 0;
    ComponentConnection::currInterruptCount = 0;

    Timer1.initialize(ComponentConnection::isrPeriodMicroseconds[0]);
    Timer1.attachInterrupt(ComponentConnection::timerISR);
  }
  */
  //else {

    float processingDelay = 20; // 40 microseconds delay per loop shared equally between pulse on time and off time
    clockDuration -= processingDelay;
    
    //Serial.print("Clock duration (SEND PULSES) : ");Serial.print(clockDuration);Serial.println(" x 10^-6");
    
    onPulseMicroseconds -= onPulseMicroseconds > processingDelay / 2 ? processingDelay / 2 : 0;   // (Aggressively remove processing time to ensure clock remains synchronized)
    offPulseMicroseconds -= offPulseMicroseconds > processingDelay / 2 ? processingDelay / 2 : 0;   // (Aggressively remove processing time to ensure clock remains synchronized)

    // When timing error accumulates, delay a single microsecond to relax to the expected timing so the overall motion rate change will be more continuous
    float errorPerStep = clockDuration - (uint32_t)(pulseDuration * pow(10, pulseDurationExponent + 6) + offDuration * pow(10, offDurationExponent + 6)) + processingDelay + 1.4;   // Error per step (in terms of microseconds) + 9.5us of delay due to processing
    errorPerStep = 0;
    // Serial.print("error per step : ");Serial.println(errorPerStep);
    //uint32_t errorCorrectionSteps = 0.5 + (numPulses / errorPerStep);   // Round to nearest integer
    float accumulatedTimingError = 0;

    #if (DEBUGGER_OVERRIDE)
      Serial.print("pulse duration = ");Serial.print(onPulseMicroseconds);Serial.println("us");
      Serial.print("OFF duration = ");Serial.print(offPulseMicroseconds);Serial.println("us");
    #endif

    unsigned long startTime = millis(); // Record start time
    unsigned long onTime = 0;
    unsigned long offTime = 0;
    unsigned long t_on;
    unsigned long t_off;
    bool polarityDelay = true;

    // Note : Clock period is reduced to compensate for delays introduced by 
    // computational processes. As a result, this mechanism is only compatible
    // with positive delays.
    errorPerStep = errorPerStep > 0 ? errorPerStep : 0;

    if (onPulseMicroseconds < 10000 && offPulseMicroseconds < 10000) {
      // t < 10ms
      for (uint32_t pulse = 0; pulse < numPulses; ++pulse) {
        t_off = millis();

        polarityDelay = !polarityDelay;
        // Increment the timing error
        accumulatedTimingError += errorPerStep;

        for (uint8_t currPin = 0; currPin < SIZE; ++currPin) {
          digitalWrite(pins[currPin], startState[currPin] > 0 ? HIGH : LOW);
        }
        t_on = millis();
        offTime += t_on - t_off;

        // Delay small duration to produce a finite duration pulse
        delayMicroseconds((uint16_t)(onPulseMicroseconds + (polarityDelay ? accumulatedTimingError : 0)));

        // Now toggle the pin state
        for (uint8_t currPin = 0; currPin < SIZE; ++currPin) {
          digitalWrite(pins[currPin], startState[currPin] > 0 ? LOW : HIGH);
        }
        t_off = millis();
        onTime += t_off - t_on;

        // Delay small duration to produce a finite duration pulse
        // Assume error is in microseconds range so do not use delay for milli-seconds
        delayMicroseconds((uint16_t)(offPulseMicroseconds + (!polarityDelay ? accumulatedTimingError : 0)));
        accumulatedTimingError -= (uint16_t)accumulatedTimingError;   // This error was just accounted for by delaying in the above command

        offTime += millis() - t_off;
        
        
        
      }
      
    }
    else {
      for (uint32_t pulse = 0; pulse < numPulses; ++pulse) {
        for (uint8_t currPin = 0; currPin < SIZE; ++currPin) {
          digitalWrite(pins[currPin], startState[currPin] > 0 ? HIGH : LOW);
        }
        // Delay small duration to produce a finite duration pulse
        delay(onPulseMicroseconds / 1000);

        // Now toggle the pin state
        for (uint8_t currPin = 0; currPin < SIZE; ++currPin) {
          digitalWrite(pins[currPin], startState[currPin] > 0 ? LOW : HIGH);
        }

        // Delay small duration to produce a finite duration pulse
        delay(offPulseMicroseconds / 1000);

        accumulatedTimingError += errorPerStep;
        if (accumulatedTimingError > 1) {
          // Assume error is in microseconds range so do not use delay for milli-seconds
          delayMicroseconds((uint32_t)accumulatedTimingError);
          accumulatedTimingError -= (uint32_t)accumulatedTimingError;   // This error was just accounted for by delaying in the above command
        }
        
      }
    }
    
    unsigned long endTime = millis();   // Record end time
    // Serial.print("Your movement duration : ");Serial.print(endTime - startTime);Serial.println(" ms");
    // Serial.print("Duration on : ");Serial.println(onTime);
    // Serial.print("Duration off : ");Serial.println(offTime);
    


  //}
}

void ComponentConnection::displayPinout(void) {
  uint8_t *pinIndex;
  switch (this->correspondingMotors[this->csIndex]) {
    #if (NEMA17_SUPPORTED)
      case BoardInfo::NEMA17 :
        pinIndex = BoardInfo::NEMA17Pins;
        break;
    #endif
    #if (STEPPER_ONLINE_SUPPORTED)
      case BoardInfo::STEPPER_ONLINE:
        pinIndex = BoardInfo::stepperOnlinePins;
        break;
    #endif
    #if (G2_SUPPORTED)
      case BoardInfo::G2 :
        pinIndex = BoardInfo::G2Pins;
        break;
    #endif
    default:
      break;
  }
  Serial.println("Pinout Connection : ");
  Serial.print("EN = ");    Serial.println(this->pinout[pinIndex[BoardInfo::EN]]);
  Serial.print("CS = ");    Serial.println(this->pinout[pinIndex[BoardInfo::CS]]);
  Serial.print("CLK = ");   Serial.println(this->pinout[pinIndex[BoardInfo::CLK]]);
  Serial.print("DIR = ");   Serial.println(this->pinout[pinIndex[BoardInfo::DIR]]);
  Serial.print("PUL = ");   Serial.println(this->pinout[pinIndex[BoardInfo::PUL]]);
  Serial.print("PWM = ");   Serial.println(this->pinout[pinIndex[BoardInfo::PWM]]);
  Serial.print("SLEEP = "); Serial.println(this->pinout[pinIndex[BoardInfo::SLEEP]]);
  Serial.print("FLT = ");   Serial.println(this->pinout[pinIndex[BoardInfo::FAULT]]);
  Serial.print("C_SENSE = ");  Serial.println(this->pinout[pinIndex[BoardInfo::C_SENSE]]);
  Serial.print("IN1 = ");   Serial.println(this->pinout[pinIndex[BoardInfo::IN1]]);
  Serial.print("IN2 = ");   Serial.println(this->pinout[pinIndex[BoardInfo::IN2]]);
}