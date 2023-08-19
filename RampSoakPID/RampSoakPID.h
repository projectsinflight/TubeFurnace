/*!
 * @file RampSoakPID.h
 *
 *
 */

#ifndef RAMPSOAKPID_H
#define RAMPSOAKPIDH

// default values

#define KP_DEFAULT 1.0
#define KI_DEFAULT 0.1
#define KD_DEFAULT 1.0
#define IMAX_DEFAULT 1000
#define RAMP_UP_LIMIT_DEFAULT 10
#define RAMP_DOWN_LIMIT_DEFAULT -10
#define CROSSOVER_DISTANCE_DEFAULT 10
#define N_RAMP_AVG 50
#define N_DVAL_AVG 10
#define S_PER_MIN 60

// function prototypes

// sets the coefficients of the PID controller
void setPidCoefficients(float P, float I, float D);
// sets the P coefficient of the PID controller
void setKp(float P);
// sets the I coefficient of the PID controller
void setKi(float I);
// sets the D coefficient of the PID controller
void setKd(float D);
// sets the integrator windup limit (max value the integrator can reach)
void setIntegratorWindupLimit(float val);
// sets the maximum ramp rate (both ramp up and ramp down)
void setRampLimit(float rampDown, float rampUp);
// sets how far from the set point that the controller will transition
// from holding a constant ramp to holding a constant value
void setCrossoverDistance(float val);
// set the target value for the controller to reach
void setTargetValue(float val);
// resets the PID loop
void resetPid();
// set debug on/off
void setDebugOnOff(bool onOff);
// computes the next PID value (call this function every 100ms or so)
float pidStep(float currentVal);
// prints PID values to terminal
void debugPID();

#endif
