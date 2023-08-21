

/*************************** DEFINES *******************************/

// Arduino Pins
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
void initLED();
void initEncoder();
void initTime();
void initThermocouple();
void initPwm();
void initPID();
void initLog();
void getTime();
void measureTemp();
void displayData();
void updateLEDs();
void updateEncoder();
void updateButton();
void recvSerial();
void evaluateCommand();
void updateHeater();
void logData();
float mapPidToPwm(float pid);
void calculateRampRate();
void updatePwm(float percent);
void isrCLK();

