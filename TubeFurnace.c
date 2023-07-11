/*************************** INCLUDES ******************************/
#include <Adafruit_MAX31856.h>
#include <math.h>
#include <LiquidCrystal_I2C.h> // Library for LCD

/*************************** DEFINES *******************************/

// pins
#define DRDY_PIN  5
#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define CS_PIN    10
#define RELAY_PIN 9  
#define CLK 2
#define DT 3
#define SW 4
#define DEBUGPIN 5

// registers
#define ICR1_VAL 15624  // Set the PWM frequency to 4Hz: 16MHz/(256 * 4Hz) - 1 = 15624

// system
#define STOP_MODE      0     // mode where the heater is off completely
#define MANUAL_MODE    1     // system mode for controlling pwm manually
#define RAMP_UP_MODE   2     // system mode for holding upward ramp rate steady
#define SOAK_MODE      3     // system mode for holding temperature steady
#define RAMP_DOWN_MODE 4     // system mode for holding downward ramp rate steady
#define NUM_MODES      5     //

// limits
#define MAXPWM 50       // max allowable PWM percentage for tube furnace
#define MINPWM 0        // min allowable PWM percentage for tube 
#define MAXTEMP 1100    // max allowable temperature for tube furnace
#define MINTEMP 0       // min allowable temperature for tube furnace
#define RAMP_LIMIT_POS 20   // maximum allowable ramp rate for heating (deg C per min)
#define RAMP_LIMIT_NEG -15  // maximum allowable ramp rate for heating (deg C per min)
#define MODE_SWITCH_CLEARANCE 10 // how many degrees within desired temperature before we switch from RAMP to SOAK mode

// time periods
#define LCD_PERIOD 100 // interval to update LCD
#define PID_PERIOD 250 // pid loop update interval in milliseconds
#define LOG_PERIOD 250 // log interval in milliseconds

// other
#define BUFSIZE 64
#define N_RAMP_AVG 5   // number of values to average to smooth ramp values
#define N_TEMP_AVG 5
#define N_DVAL_AVG 5
#define DEGREE_SYMBOL ((char)223)
#define DEGREES_PER_COUNT 10
#define DEGREES_START 1000


/*************************** FUNCTION PROTOTYPES *******************/
void setup();
void loop();
void initSerial();
void initLcd();
void initEncoder();
void initTime();
void initThermocouple();
void initPwm();
void resetPid();
void initLog();
void displayData();
void updateEncoder();
void updateButton();
void recvSerial();
void evaluateCommand();
void updateHeater();
void logData();
void getTemp();
float mapPidToPwm(float pid);
void calculateRampRate();
void updatePwm();
void pidStep(float currentVal, float setPoint);
void isrCLK();

/*************************** CLASSES *******************************/

class RunningAverage {

  public:
    float *vals = NULL;
    int vals_idx;
    int n_vals;
    int vals_filled;
    void init(int n);
    void add(float val);
    float avg();
    void reset();

};
    
    void RunningAverage::init(int n) {
      n_vals = n;
      vals = (float *)malloc(n_vals * sizeof(float));
      if (vals == NULL) {
        Serial.println("Could not allocate memory for RunningAverage!");
      }
      vals_idx = 0;
      vals_filled = 0;
    }

    void RunningAverage::add(float val) {
      vals[vals_idx] = val;
      vals_idx++;
      vals_filled++;
      if (vals_idx >= n_vals) {
        vals_idx = 0;
      }
      // Serial.print("vals: "); 
      // for (int i=0; i<n_vals; i++) {
      //   Serial.print(vals[i]); Serial.print(" ");
      // }
      // Serial.print(" vals_idx: "); Serial.print(vals_idx);
      // Serial.print(" vals_filled: "); Serial.print(vals_filled);
      // Serial.println("");
      
    }

    float RunningAverage::avg() {
      float vals_avg = 0;
      int stop = min(vals_filled, n_vals);
      if (stop == 0) {
        return 0.0;
      }
      for (int i=0; i<stop; i++) {
        vals_avg += vals[i];
      }
      vals_avg = vals_avg / stop;
      return vals_avg;
    }

    void RunningAverage::reset() {
      vals_idx = 0;
      vals_filled = 0;
    }

/*************************** GLOBALS *******************************/
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(CS_PIN);

// system control
int systemMode = STOP_MODE; // current system mode for furnace
char strbuf[BUFSIZE] = "";  // command buffer
float pwmPercent = 0.0;

// timers
unsigned long currentTime = 0;   // keeps track of current time
unsigned long lcdUpdateTime = 0; // when to update the LCD
unsigned long pidUpdateTime = 0; // keeps track of when to update PID
unsigned long nextLogTime = 0;   // keeps track of when to log data

// temperature values
float desiredTemp = 0.0; // temperature we want the furnace to reach
float currentTemp = 0.0; // whatever the current temperature is
float prevTemp = 0.0;    // previous temperature value
float rampRate = 0.0;    // difference between current and previous temp
bool rampInitialized = false; // whether or not we've calculated the ramp yet (prevent prevTemp == 0 problem)
float rampLimit = 0;

// pid variables
float pidCoefficientTable[NUM_MODES][4] = {   // {P, I, D, WINDUP_LIMIT}
  { 0.0,  0.0,   0.0,  0.0},  /* STOP MODE */
  { 0.0,  0.0,   0.0,  0.0},  /* MANUAL MODE */
  { 0.1, 0.05,   0.0, 50.0},  /* RAMP UP MODE */
  { 1.0, 0.05,  10.0, 50.0},  /* SOAK MODE */
  { 0.1, 0.05,   0.0, 50.0}   /* RAMP DOWN MODE */
};
float Kp = 0.0;     // 
float Ki = 0.0;     // 
float Kd = 0.0;     // 
float IVal = 0.0;
float windup = 0.0; // integrator windup limit
float pidVal = 0.0; // output of PID loop
float previousError = 0.0; // last PID error value (used to compute D term)
bool previousErrorInitialized = false; // whether or not we've computed the D term yet (to stop the prevError == 0 problem)

// lcd
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
int column_index = 0;
int row_index = 0;
#define NUM_LCD_CHARS 16
#define NUM_LCD_ROWS 2
char lcdBuffer[NUM_LCD_ROWS][NUM_LCD_CHARS] = {"TUBE FURNACE 1.0", "Nick Ogden"};

// Values to keep a running average of
RunningAverage tempAvg;
RunningAverage rampAvg;
RunningAverage dValAvg;

// rotary encoder
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;
unsigned long lastISR = 0;


/*************************** SETUP *********************************/
void setup() {

  // init running averagers
  tempAvg.init(N_TEMP_AVG);
  rampAvg.init(N_RAMP_AVG);
  dValAvg.init(N_DVAL_AVG);

  initSerial();
  initLcd();
  initEncoder();
  initTime();
  initThermocouple();
  initPwm();
  pidUpdateTime = currentTime;
  resetPid();
  initLog();

}


/*************************** MAIN **********************************/
void loop() {

  currentTime = millis();
  displayData();
  updateButton();
  recvSerial();
  evaluateCommand();
  updateHeater();
  // logData();

}


/*************************** FUNCTIONS *****************************/
// init serial
void initSerial() {
  // initialize serial
  Serial.begin(115200);
  while (!Serial) delay(10);
}

// init lcd
void initLcd() {
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight 
  row_index = 0; column_index = 0;
  lcd.setCursor(column_index, row_index);
}

// init rotary encoder
void initEncoder() {
  // Set encoder pins as inputs
	pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(SW, INPUT_PULLUP);
  pinMode(DEBUGPIN,OUTPUT);
  digitalWrite(DEBUGPIN,LOW);
  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
}

// init Time
void initTime() {
  currentTime = millis();
}

// init thermocouple
void initThermocouple() {
  pinMode(DRDY_PIN, INPUT);
  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
}

// init pwm
void initPwm() {
  pinMode(RELAY_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);                // Enable the PWM output OC1A on digital pins 9
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);     // Set fast PWM and prescaler of 256 on timer 1
  ICR1 = ICR1_VAL;                                  // Set the PWM frequency to 4Hz: 16MHz/(256 * 4Hz) - 1 = 15624
  OCR1A = 0;                                        // Set PWM output to 0 for now
}

// init pid loop
void resetPid()
{
  // initialize the coeficcients Kp, Ki, and Kd
  Kp = pidCoefficientTable[systemMode][0];
  Ki = pidCoefficientTable[systemMode][1];
  Kd = pidCoefficientTable[systemMode][2];
  windup = pidCoefficientTable[systemMode][3];
  // reset last pid value
  pidVal = 0;
  previousError = 0;
  previousErrorInitialized = false;
}

// init logging
void initLog() {
  // update log timers
  nextLogTime = currentTime;
}

// lcd display
// ROW1: Current temp, Ramp rate   "1234°C00°C/min"
// ROW2: Elapsed Time
void displayData() {

  if (currentTime < lcdUpdateTime) {
    return;
  }
  lcdUpdateTime = currentTime + LCD_PERIOD;

  char row1[17] = "";
  char row2[17] = "";
  char tmp1[7] = "";
  char tmp2[10] = "";
  char tmp3[9] = "";
  char tmp4[8] = "";
  int t = (int)tempAvg.avg();
  if (systemMode == STOP_MODE) {
    t = (counter * DEGREES_PER_COUNT) + DEGREES_START;
  }
  int r = (int)rampAvg.avg();
  int total_sec = currentTime / 1000;
  int m = total_sec / 60;
  int s = total_sec % 60;

  // check bounds
  if (r > 99) {
    r = 99;
  } else if (r < -99) {
    r = -99;
  }

  if (m > 999) {
    m = 999;
  }

  // create row strings
  sprintf(tmp1, "%d%cC", t, DEGREE_SYMBOL);
  sprintf(tmp2, "%+2d%cC/min", r, DEGREE_SYMBOL);
  sprintf(tmp3, "%dm:%02ds", m, s);
  switch (systemMode) {
    case STOP_MODE:
      sprintf(tmp4, "OFF");
      break;
    case MANUAL_MODE:
      sprintf(tmp4, "%d%%", pwmPercent);
      break;
    case RAMP_UP_MODE:
      sprintf(tmp4, "U%d%cC", (int)desiredTemp, DEGREE_SYMBOL);
      break;
    case SOAK_MODE:
      sprintf(tmp4, "H%d%cC", (int)desiredTemp, DEGREE_SYMBOL);
      break;
    case RAMP_DOWN_MODE:
      sprintf(tmp4, "D%d%cC", (int)desiredTemp, DEGREE_SYMBOL);
      break;
  }

  // finalize row strings
  sprintf(row1, "%-6s %9s", tmp1, tmp2);
  sprintf(row2, "%-8s %7s", tmp3, tmp4);

  // output to LCD
  lcd.setCursor(0,0);
  lcd.print(row1);
  lcd.setCursor(0,1);
  lcd.print(row2);
}

// keep track of position of rotary encoder
void updateEncoder() {

  // primitive debounce: ignore stuff that happened recently
  if ((millis() - lastISR) < 10) {
    return;
  }

  if (digitalRead(CLK) == HIGH) {
    if (digitalRead(DT) == LOW) {
      counter ++;
      currentDir ="CW";
    } else {
      counter --;
      currentDir ="CCW";
    }
  }
}

void updateButton() {
  // Read the button state
	int btnState = digitalRead(SW);

	//If we detect LOW signal, button is pressed
	if (btnState == LOW) {
		//if 100ms have passed since last LOW pulse, it means that the
		//button has been pressed, released and pressed again
		if (millis() - lastButtonPress > 100) {
      desiredTemp = (counter * DEGREES_PER_COUNT) + DEGREES_START;
      systemMode = SOAK_MODE;
		}

		// Remember last button press event
		lastButtonPress = millis();
  }
}

// Read data from serial line if available
void recvSerial() {
    char c;
    int i = 0;
    bool newData = false;
 
    while (Serial.available()) {
      newData = true;
      c = Serial.read();
      strbuf[i] = c;
      delay(1); // not sure why this is needed but it breaks without it :( 
      i++;
      if (i >= BUFSIZE) {
        i = BUFSIZE - 1;
      }
    }
    strbuf[i] = '\0'; // terminate string

    if (newData) {
      // Serial.print("strbuf: ");
      // Serial.print(strbuf);   
      ;
    }
    newData = false;
}

// execute new command if available
void evaluateCommand() {
  int newTemp = 0;
  int newPwm = 0;
  float val = 0.0;

  // no command available
  if (strbuf[0] == '\0') {
    return;

  // set pwm percentage
  } else if (strbuf[0] == 'S') {
    newPwm = atoi(strbuf + 1);
    if ((newPwm <= MAXPWM) && (newPwm >= 0)) {
      updatePwm(newPwm);
      if (newPwm == 0) {
        systemMode = STOP_MODE;
      } else {
        systemMode = MANUAL_MODE;
      }
      resetPid();
      Serial.print("Set PWM to ");
      Serial.print(newPwm);
      Serial.println("%");
    } else {
      Serial.print("Cannot set PWM to ");
      Serial.print(newPwm);
      Serial.println("%");
    }

  // set target temperature
  } else if (strbuf[0] == 'T') {
    newTemp = atoi(strbuf + 1);
    if ((newTemp <= MAXTEMP) && (newTemp >= MINTEMP)) {
      desiredTemp = newTemp;
      if (desiredTemp > (tempAvg.avg() + MODE_SWITCH_CLEARANCE)) {
        systemMode = RAMP_UP_MODE;
      } else if (desiredTemp < (tempAvg.avg() - MODE_SWITCH_CLEARANCE)) {
        systemMode = RAMP_DOWN_MODE;
      } else {
        systemMode = SOAK_MODE;
      }
      resetPid();
      Serial.print("Set temp to ");
      Serial.print(desiredTemp);
      Serial.println("C");
    } else {
      Serial.print("Cannot set temp to ");
      Serial.print(newTemp);
      Serial.println("C");
    }

  // set PID values
  } else if (strbuf[0] == 'P') {
    val = atof(strbuf + 1);
    pidCoefficientTable[systemMode][0] = val;
    resetPid();
    Serial.print("Set P value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'I') {
    val = atof(strbuf + 1);
    pidCoefficientTable[systemMode][1] = val;
    resetPid();
    Serial.print("Set I value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'D') {
    val = atof(strbuf + 1);
    pidCoefficientTable[systemMode][2] = val;
    resetPid();
    Serial.print("Set D value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'W') {
    val = atof(strbuf + 1);
    pidCoefficientTable[systemMode][3] = val;
    resetPid();
    Serial.print("Set W value to ");
    Serial.println(val);

  // Emergency Stop
  } else if (strbuf[0] == 'E') {
    Serial.println("EMERGENCY STOP");  // probably need to make this more foolproof like a hard shutdown
    systemMode = STOP_MODE;

  // command not recognized
  } else {
    Serial.print("Unknown Command: ");
    Serial.println(strbuf);
  }

  strbuf[0] = '\0'; // clear command
  
}

// check if we need to change heater setting based on PID
void updateHeater() {
  if (pidUpdateTime < currentTime) {
    pidUpdateTime += PID_PERIOD;
    getTemp();

    calculateRampRate();
    switch (systemMode) {
      case STOP_MODE :
        updatePwm(0);
        break;
      case MANUAL_MODE : 
        // keep constant PWM, dont run PID
        break;
      case RAMP_UP_MODE :
        // check if we are going up or down
        pidStep(rampAvg.avg(), RAMP_LIMIT_POS);
        updatePwm(mapPidToPwm(pidVal));
        if (tempAvg.avg() > (desiredTemp - MODE_SWITCH_CLEARANCE)) {
          systemMode = SOAK_MODE;
          Serial.println("switching to SOAK mode!");
          resetPid();
        }
        break;
      case SOAK_MODE :
        pidStep(tempAvg.avg(), desiredTemp);
        updatePwm(mapPidToPwm(pidVal));
        if (tempAvg.avg() < (desiredTemp - MODE_SWITCH_CLEARANCE)) {
          systemMode = RAMP_UP_MODE;
          Serial.println("switching to RAMP UP mode!");
          resetPid();
        } else if (tempAvg.avg() > (desiredTemp + MODE_SWITCH_CLEARANCE)) {
          systemMode = RAMP_DOWN_MODE;
          Serial.println("switching to RAMP DOWN mode!");
          resetPid();
        }
        break;
      case RAMP_DOWN_MODE :
        // check if we are going up or down
        pidStep(rampAvg.avg(), RAMP_LIMIT_NEG);
        updatePwm(mapPidToPwm(pidVal));
        if (tempAvg.avg() < (desiredTemp + MODE_SWITCH_CLEARANCE))  {
          systemMode = SOAK_MODE;
          Serial.println("switching to SOAK mode!");
          resetPid();
        }
        break;
    }
  }
}

// log new data
void logData() {

  if (nextLogTime < currentTime) {
    nextLogTime += LOG_PERIOD;
    // print logs
  Serial.print("tempAvg:"); Serial.print(tempAvg.avg()); Serial.print(",");
  Serial.print("rampAvg:"); Serial.print(rampAvg.avg()); Serial.print(",");
  Serial.print("pwmPercent:");  Serial.print(pwmPercent);    Serial.print(",");
  Serial.print("20:");  Serial.print(20);    Serial.print(",");
  Serial.print("-15:");  Serial.print(-15);    Serial.println("");
  }
}

// Read Thermocouple Temperature
void getTemp() {
  // The DRDY output goes low when a new conversion result is available
  while (digitalRead(DRDY_PIN));
  prevTemp = tempAvg.avg();
  currentTemp = maxthermo.readThermocoupleTemperature();  
  tempAvg.add(currentTemp);
}

  // control the OCR1A register directly to set PWM value
void updatePwm(float percent) {
  // check and enforce bounds
  if (percent > MAXPWM) {
    percent = MAXPWM;
  } else if (percent < 0) {
    percent = 0;
  }
  pwmPercent = percent;
  OCR1A = (percent * ICR1_VAL) / 100;
}

float mapPidToPwm(float pid) {
  float val = 0.0;
  if (pid < 0.0) {
    val = 0.0;
  } else if (pid > MAXPWM) {
    val = MAXPWM;
  } else {
    val = pid;
  }
  return val;
}

// checks the ramp rate and adjusts pid if necessary
void calculateRampRate() {

  // if we don't have a prev temp, rampRate = 0 
  if (!rampInitialized) {
    prevTemp = tempAvg.avg();
    rampInitialized = true;
  }

  // now add ramp rate to averager
  rampRate = 60 * (1000.0/PID_PERIOD) * (tempAvg.avg() - prevTemp);
  rampAvg.add(rampRate);
}

// we want to make the PID loop agnostic to systemMode, so we will achieve this
// by passing in the time 
void pidStep(float currentVal, float setPoint)
{
  float error = 0.0;
  float PVal = 0.0;
  float DVal = 0.0;
  float dt = PID_PERIOD/1000.0; // convert from ms to s

  error = setPoint - currentVal;
  PVal = Kp * error;
  IVal += Ki * error * dt;
  if (previousErrorInitialized) {
    DVal = Kd * (error - previousError)/dt;
  }
  dValAvg.add(DVal);
  previousError = error;
  previousErrorInitialized = true;
  // integrator windup prevention
  if (IVal > windup) {
      IVal = windup;
  } else if (IVal < (-1 * windup)) {
      IVal = -1 * windup;
  }
  pidVal = PVal + IVal + dValAvg.avg();

  // Serial.print("temp:"); Serial.print(currentTemp); Serial.print(",");
  Serial.print("tempAvg:"); Serial.print(tempAvg.avg()); Serial.print(",");
  // Serial.print("ramp:"); Serial.print(rampRate); Serial.print(",");
  Serial.print("rampAvg:"); Serial.print(rampAvg.avg()); Serial.print(",");
    Serial.print("pwmPercent:");  Serial.print(pwmPercent);    Serial.print(",");
  // Serial.print("d:"); Serial.print(DVal); Serial.print(",");
  // Serial.print("dValAvg:"); Serial.print(dValAvg.avg()); Serial.print(",");

  Serial.print("currentVal:");  Serial.print(currentVal);    Serial.print(",");
  Serial.print("error:");       Serial.print(error);         Serial.print(",");
  Serial.print("PVal:");        Serial.print(PVal);          Serial.print(",");
  Serial.print("IVal:");        Serial.print(IVal);          Serial.print(",");
  Serial.print("DVal:");        Serial.print(DVal);          Serial.print(",");
  Serial.print("DValAvg:");     Serial.print(dValAvg.avg()); Serial.print(",");
  Serial.print("pidVal:");      Serial.print(pidVal);        Serial.print(",");
  Serial.print("pwmPercent:");  Serial.print(pwmPercent);    Serial.print(",");
  Serial.print("currentTemp:");  Serial.print(currentTemp);  Serial.print(",");
  Serial.print("desiredTemp:"); Serial.print(desiredTemp);   Serial.print(",");
  Serial.print("setPoint:");    Serial.print(setPoint);      Serial.print(",");
  Serial.print("Kp:");          Serial.print(Kp);            Serial.print(",");
  Serial.print("Ki:");          Serial.print(Ki);            Serial.print(",");
  Serial.print("Kd:");          Serial.print(Kd);            Serial.println("");
}
