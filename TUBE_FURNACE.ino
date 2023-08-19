/*************************** INCLUDES ******************************/
#include <Adafruit_MAX31856.h>
#include <math.h>
#include <LiquidCrystal_I2C.h> // Library for LCD
#include <RunningAverage.h>
#include <RampSoakPID.h>

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
#define RED_LED A0
#define YELLOW_LED A1
#define GREEN_LED A2

// registers
#define ICR1_VAL 15624  // Set the PWM frequency to 4Hz: 16MHz/(256 * 4Hz) - 1 = 15624

// system mode
#define STOP_MODE      0     // mode where the heater is off completely
#define MANUAL_MODE    1     // system mode for controlling pwm manually
#define AUTO_MODE      2     // system mode for holding upward ramp rate steady
#define NUM_MODES      3     //

// limits
#define MAXPWM 60       // max allowable PWM percentage for tube furnace
#define MINPWM 0        // min allowable PWM percentage for tube 
#define MAXTEMP 1200    // max allowable temperature for tube furnace
#define MINTEMP 0       // min allowable temperature for tube furnace

// time periods
#define TEMP_PERIOD 100 // interval to measure temp value
#define LCD_PERIOD 100 // interval to update LCD
#define PID_PERIOD 100 // interval of time to update PID loop
#define LOG_PERIOD 100 // log interval in milliseconds

// other
#define BUFSIZE 64
#define N_RAMP_AVG 10   // number of values to average to smooth ramp values
#define N_TEMP_AVG 10
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
void initPID();
void initLog();
void measureTemp();
void displayData();
void updateEncoder();
void updateButton();
void recvSerial();
void evaluateCommand();
void updateHeater();
void logData();
float mapPidToPwm(float pid);
void calculateRampRate();
void updatePwm();
void isrCLK();

/*************************** GLOBALS *******************************/
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(CS_PIN);

// system control
int systemMode = STOP_MODE; // current system mode for furnace
char strbuf[BUFSIZE] = "";  // command buffer
float pwmPercent = 0.0;

// timers
unsigned long currentTime = 0;   // keeps track of current time
unsigned long nextTempTime = 0;  // keeps track of when to measure temp val
unsigned long lcdUpdateTime = 0; // when to update the LCD
unsigned long pidUpdateTime = 0; // keeps track of when to update PID
unsigned long nextLogTime = 0;   // keeps track of when to log data

// temperature values
float desiredTemp = 0.0; // temperature we want the furnace to reach
float currentTemp = 0.0; // whatever the current temperature is
float prevTemp = 0.0;    // previous temperature value
float rampRateF = 0.0;    // difference between current and previous temp
bool rampInitialized = false; // whether or not we've calculated the ramp yet (prevent prevTemp == 0 problem)
float rampLimit = 0;

// lcd
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
int column_index = 0;
int row_index = 0;
#define NUM_LCD_CHARS 16
#define NUM_LCD_ROWS 2
char lcdBuffer[NUM_LCD_ROWS][NUM_LCD_CHARS] = {"TUBE FURNACE 1.0", "Nick Ogden"};

// Values to keep a running average of
RunningAverage tempAvg(N_TEMP_AVG);
RunningAverage rampAvg(N_RAMP_AVG);

// rotary encoder
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;
unsigned long lastISR = 0;


float testVal = 0.0;

/*************************** SETUP *********************************/
void setup() {

  tempAvg.clear();
  rampAvg.clear();

  initSerial();
  initLcd();
  initEncoder();
  initTime();
  initThermocouple();
  initPwm();
  initPID();
  initLog();

  // init LEDS
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  delay(1000);
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

}


/*************************** MAIN **********************************/
void loop() {

  currentTime = millis();
  measureTemp();
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

// init pid
void initPID() {
  pidUpdateTime = currentTime;
  setPidCoefficients(0.5, 0.05, 1.0);
  setIntegratorWindupLimit(100);
  setRampLimit(-30, 30);
  setCrossoverDistance(10);
  setDebugOnOff(true);
}

// init logging
void initLog() {
  // update log timers
  nextLogTime = currentTime;
}

// measure the temperature and update running avg
void measureTemp() {
  if (currentTime < nextTempTime) {
    return;
  }
  nextTempTime += TEMP_PERIOD;
  while (digitalRead(DRDY_PIN)); 
  tempAvg.addValue(maxthermo.readThermocoupleTemperature());
  prevTemp = currentTemp;
  currentTemp = tempAvg.getAverage();

  // if we don't have a prev temp, rampRate = 0 
  if (!rampInitialized) {
    prevTemp = currentTemp;
    rampInitialized = true;
  }

  // now add ramp rate to averager
  rampRateF = 60 * (1000.0/TEMP_PERIOD) * (currentTemp - prevTemp);
  rampAvg.addValue(rampRateF);
    
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
  int t = (int)tempAvg.getAverage();
  if (systemMode == STOP_MODE) {
    t = (counter * DEGREES_PER_COUNT) + DEGREES_START;
  }
  int r = (int)rampAvg.getAverage();
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
    case AUTO_MODE:
      sprintf(tmp4, "H%d%cC", (int)desiredTemp, DEGREE_SYMBOL);
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
      systemMode = AUTO_MODE;
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
      setTargetValue(desiredTemp);
      systemMode = AUTO_MODE;
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
    setKp(val);
    // resetPid();
    Serial.print("Set P value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'I') {
    val = atof(strbuf + 1);
    setKi(val);
    // resetPid();
    Serial.print("Set I value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'D') {
    val = atof(strbuf + 1);
    setKd(val);
    // resetPid();
    Serial.print("Set D value to ");
    Serial.println(val);
  } else if (strbuf[0] == 'W') {
    val = atof(strbuf + 1);
    setIntegratorWindupLimit(val);
    // resetPid();
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
    switch (systemMode) {
      case STOP_MODE :
        updatePwm(0);
        break;
      case MANUAL_MODE : 
        // keep constant PWM, dont run PID
        break;
      case AUTO_MODE :
        float pidVal = pidStep(currentTemp);
        updatePwm(mapPidToPwm(pidVal));
        break;
    }
  }
}

// log new data
void logData() {

  if (nextLogTime < currentTime) {
    nextLogTime += LOG_PERIOD;
    // print logs
    Serial.print("temp:"); Serial.print(currentTemp); Serial.print(",");
    Serial.print("tempAvg:"); Serial.print(tempAvg.getAverage()); Serial.print(",");
    Serial.print("ramp:"); Serial.print(rampRateF); Serial.print(",");
    Serial.print("rampAvg:"); Serial.print(rampAvg.getAverage()); Serial.print(",");
    Serial.print("pwmPercent:");  Serial.print(pwmPercent);    Serial.print(",");
    Serial.print("pwmPercent:");  Serial.print(pwmPercent);    Serial.print(",");
    Serial.print("currentTemp:");  Serial.print(currentTemp);  Serial.print(",");
    Serial.print("desiredTemp:"); Serial.print(desiredTemp);   Serial.print(",");
  }
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


