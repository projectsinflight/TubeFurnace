#include <RampSoakPID.h>

/*************************** INCLUDES ******************************/
#include "TubeFurnace.h"

/*************************** SETUP *********************************/
void setup() {

  initSerial();
  resetClock();
  initLcd();
  initLED();
  initEncoder();
  initTime();
  initThermocouple();
  initPwm();
  initPID();
  initLog();
}

/*************************** MAIN **********************************/
void loop() {

  getTime();
  measureTemp();
  displayData();
  updateLEDs();
  updateButton();
  recvSerial();
  evaluateCommand();
  updateHeater();
  // logData();
}