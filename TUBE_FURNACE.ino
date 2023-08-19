/*************************** INCLUDES ******************************/
#include "TubeFurnace.h"

/*************************** SETUP *********************************/
void setup() {

  initSerial();
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
  updateButton();
  recvSerial();
  evaluateCommand();
  updateHeater();
  // logData();
}