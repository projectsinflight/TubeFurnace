/*!
 * @file RampSoakPID.cpp
 *
 * @mainpage Ramp Soak PID Controller
 *
 * @section intro_sec Introduction
 *
 * This is a PID controller that allows you to set the maximum
 * ramp rate as well.
 *
 */

/*************************** Includes ****************************************/
#include "RampSoakPID.h"
#include <math.h>
#include <RunningAverage.h>


/*************************** Variables ***************************************/
float Kp = KP_DEFAULT;
float Ki = KI_DEFAULT;
float Kd = KD_DEFAULT;
float IMax = IMAX_DEFAULT;
float rampUpLimit = RAMP_UP_LIMIT_DEFAULT;
float rampDownLimit = RAMP_DOWN_LIMIT_DEFAULT;
float crossoverDistance = CROSSOVER_DISTANCE_DEFAULT;
float error = 0.0;
float PVal = 0.0;
float IVal = 0.0;
float DVal = 0.0;
float currentVal = 0.0;
float targetVal = 0.0;
unsigned long int currentStepTime;
bool currentStepTimeInitialized = false;
float previousVal = 0.0;
bool previousValInitialized = false;
float previousError = 0.0;
bool previousErrorInitialized = false;
unsigned long int previousStepTime = 0;
float rampRate = 0.0;
float desiredRampRate = 0.0;
RunningAverage rampRateAvg(N_RAMP_AVG);
bool debugPIDOn = false;
RunningAverage DValAvg(N_DVAL_AVG);


/*************************** Functions ***************************************/

// sets the coefficients of the PID controller
void setPidCoefficients(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

// sets the P coefficient of the PID controller
void setKp(float P) {
  Kp = P;
}

// sets the I coefficient of the PID controller
void setKi(float I) {
  Ki = I;
}

// sets the D coefficient of the PID controller
void setKd(float D) {
  Kd = D;
}

// sets the integrator windup limit (max value the integrator can reach)
void setIntegratorWindupLimit(float val) {
  IMax = val;
}

// sets the maximum ramp rate (both ramp up and ramp down)
void setRampLimit(float rampDown, float rampUp) {
  rampDownLimit = rampDown;
  rampUpLimit = rampUp;
}

// sets how far from the set point that the controller will transition
// from holding a constant ramp to holding a constant value
void setCrossoverDistance(float val) {
  crossoverDistance = val;
}

// set the target value for the controller to reach
void setTargetValue(float val) {
  targetVal = val;
}

// resets the PID loop
void resetPid()
{
  previousStepTime = 0;
  currentStepTime = 0;
  currentStepTimeInitialized = false;
  previousError = 0;
  previousErrorInitialized = false;
  previousVal = 0;
  // currentVal = 0;
  previousValInitialized = false;
  // IVal = 0;
  rampRateAvg.clear();
  rampRate = 0.0;
  DValAvg.clear();
}

// set debug on/off
void setDebugOnOff(bool onOff) {
  debugPIDOn = onOff;
}

// computes the next PID value (call this function every 100ms or so)
float pidStep(float val) {

  currentVal = val;
  float dt; // in seconds

  // compute the time since last PID step
  if (currentStepTimeInitialized == false) {
    dt = 0.0;
    rampRate = 0;
    currentStepTime = millis();
    currentStepTimeInitialized = true;
  } else {
    previousStepTime = currentStepTime;
    currentStepTime = millis();
    dt = (currentStepTime - previousStepTime) / 1000.0;
  }

  // compute the current rampRate
  if (previousValInitialized == false) {
    previousVal = currentVal;
    rampRate = 0.0;
    previousValInitialized = true;
  } else if (dt != 0.0) {;
    rampRateAvg.addValue(S_PER_MIN * (currentVal - previousVal) / dt);
    rampRate = rampRateAvg.getAverage();
    previousVal = currentVal;
  }

  // compute previousError
  previousError = error;

  // Adjust Ramp Limits if we are close to the set point

  if (currentVal < (targetVal - 0)) {
    // ramping up
    desiredRampRate = min(rampUpLimit, (rampUpLimit * abs(targetVal - currentVal) / crossoverDistance));
    error = desiredRampRate - rampRate;

  } else if (currentVal > (targetVal + 0)) {
    // ramping down
    desiredRampRate = max(rampDownLimit, (rampDownLimit * abs(targetVal - currentVal) / crossoverDistance));
    error = desiredRampRate - rampRate;
    
  } else {
    // holding steady
    error = targetVal - currentVal;
  }

  // Compute PID Values and PID Output

  PVal = Kp * error;
  IVal += Ki * error * dt;
  if (dt != 0) {
    DVal = Kd * (error - previousError) / dt;
  } else {
    DVal = 0;
  }
  
  // integrator windup prevention
  if (IVal > IMax) {
      IVal = IMax;
  }
  if (IVal < (-1 * IMax)) {
      IVal = -1 * IMax;
  }
  
  if (debugPIDOn) {
    debugPID();
  }

  DValAvg.addValue(DVal);
  return PVal + IVal + DValAvg.getAverage();

}

// prints PID values to terminal
void debugPID() {

  Serial.print("currentVal:"); Serial.print(currentVal);   Serial.print(",");
  Serial.print("targetVal:");  Serial.print(targetVal);    Serial.print(",");
  Serial.print("rampRate:");   Serial.print(rampRateAvg.getAverage());     Serial.print(",");
  Serial.print("desiredRampRate:");   Serial.print(desiredRampRate);     Serial.print(",");
  Serial.print("error:");     Serial.print(error);        Serial.print(",");
  Serial.print("P:");         Serial.print(PVal);         Serial.print(",");
  Serial.print("I:");         Serial.print(IVal);         Serial.print(",");
  Serial.print("D:");         Serial.print(DValAvg.getAverage());         Serial.print(",");
  Serial.print("Kp:");         Serial.print(Kp);         Serial.print(",");
  Serial.print("Ki:");         Serial.print(Ki);         Serial.print(",");
  Serial.print("Kd:");         Serial.print(Kd);         Serial.print(",");
  Serial.println("");
}


