#include <Tle493d.h>
#include <Tle493d_w2b6.h>
#include <Wire.h>
#include <MAX6675.h>
#include <PID_v1.h>

#define DEBUG
#define PIN_SCL   10
#define PIN_SDA   11
#define TRIGPIN   4
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
#define PID_SAMPLE_TIME 1000
#define SENSOR_SAMPLING_TIME 1000

typedef enum OVEN_STATUS
{
  OVEN_STATUS_OFF,
  OVEN_STATUS_ON
} ovenStatus_t;

const int thermoDO = 5;   // MISO
const int thermoCS = 3;   // CS
const int thermoCLK = 2;  // SCK

bool tle493_ini_ok = false;
byte address = Tle493d::TLE493D_A1;
unsigned long cmillis = millis();
unsigned long nextRead = millis();
float angle;
float zeroAngle = 0.0;
float lowAngle = 0.0;
float highAngle = 0.0;
float prevAngle = 0.0;
bool trigChg = false;
const int spMin = 50;
const int spMax = 250;

int calPoints = 0;
const int calComplete = 0b00000111;

const int SSR1_IN = 9;    //p0.5
const int SSR2_IN = 8;    //p0.0
bool SSR1 = false;
bool SSR2 = false;

float temperature = 0.0;  // Temperature output variable
float newT = 0.0;
// PID variables
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
ovenStatus_t ovenStatus = OVEN_STATUS_OFF;
int windowSize;
unsigned long windowStartTime;
bool spChange = false;

int printDelay = 5;

void(* resetFunc) (void) = 0;

// Thermocouple Sensor
MAX6675 temp(thermoCS, thermoDO, thermoCLK, 1);   // 1 means Celsius
// Magnetic Sensor
Tle493d_w2b6 Tle493dMagnetic3DSensor = Tle493d_w2b6(Tle493d::MASTERCONTROLLEDMODE, Tle493d::TLE493D_A1);
// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  // SSR OFF
  digitalWrite(SSR1_IN, LOW);
  pinMode(SSR1_IN, OUTPUT);
  digitalWrite(SSR2_IN, LOW);
  pinMode(SSR2_IN, OUTPUT);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);    // turn the LED on, otherwise I2C will not work.
  pinMode(TRIGPIN, OUTPUT);
  digitalWrite(TRIGPIN, LOW);
  delay(100);

  pinMode(PIN_SDA, INPUT);    // Let the pull up to rise the line high
  pinMode(PIN_SCL, INPUT);
  digitalWrite(PIN_SDA, LOW);
  digitalWrite(PIN_SCL, LOW);
  resetSensor();

  Wire.begin();
  Wire.setClock(10000);
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial);
#endif
  delay(500);
  Tle493dMagnetic3DSensor.begin();
  Wire.end();
  windowSize = 2000;
  cmillis = millis();
  nextRead = millis();
  interrupts();
#ifdef DEBUG
  Serial.println("Setup Done");
#endif
}

void loop() {
  // Current time
  unsigned long now;
  if ((millis() - nextRead) > SENSOR_SAMPLING_TIME) {
    nextRead = millis();
    temperature = temp.read_temp();
    if (temperature < 0) {
      ovenStatus = OVEN_STATUS_OFF;
      // If there is an error with the TC, temperature will be < 0
#ifdef DEBUG
      Serial.print("Thermocouple Error on CS");
      Serial.println( temperature );
#endif
    } else {
      input = temperature;
#ifdef DEBUG
    if (ovenStatus == OVEN_STATUS_ON) {
      printDelay--;
      if(!printDelay){
        printDelay = 5;
        Serial.print("Current Temperature: ");
        Serial.println( input );
        Serial.print("Current Setpoint: ");
        Serial.println( setpoint );
      }
    }
#endif
    }
  }
  if ((millis() - cmillis) > 200) {
    cmillis = millis();
    Wire.begin();
    Wire.setClock(10000);
    delay(10);
    int err = Tle493dMagnetic3DSensor.updateData();
    err = Tle493dMagnetic3DSensor.updateData();
    if (err != TLE493D_NO_ERROR) {
#ifdef DEBUG
      Serial.print("Error on Update: ");
      Serial.println(err);
#endif
      delay(1000);
      //      resetFunc();
    } else {
      angle = map(atan2(Tle493dMagnetic3DSensor.getY(), Tle493dMagnetic3DSensor.getX()) * (180 / M_PI), -180.00, 180.00, 0.00, 360.00);
      if (abs(prevAngle - angle) >= 5.0) {
        prevAngle = angle;
        trigChg = true;
#ifdef DEBUG
        Serial.print("angle = ");
        Serial.println(angle);
#endif
      }
    }
    Wire.end();
  }
  if (trigChg) {
    trigChg = false;
    // angle should increment when knob turns clockwise
    if (calPoints == calComplete) {
      if (highAngle > lowAngle) {
        if ((prevAngle > highAngle) || (prevAngle < lowAngle)) {
#ifdef DEBUG
          Serial.println("Knob is in OFF region");
          delay(1000);
#endif
          ovenStatus = OVEN_STATUS_OFF;
        } else {
          spChange = true;
          // this is the case when for example High=270, Low  = 60 , range = 210
          float m = (spMax - spMin) / (highAngle - lowAngle);
          newT = prevAngle * m + spMin;
#ifdef DEBUG
          Serial.print("Setpoint = ");
          Serial.println(newT);
#endif
        }
      } else {
        if ((prevAngle > highAngle) && (prevAngle < lowAngle)) {
#ifdef DEBUG
          Serial.println("Knob is in OFF region");
          delay(500);   // reduce debugging messages
#endif
          ovenStatus = OVEN_STATUS_OFF;
        } else {
          spChange = true;
          float range = highAngle + (360 - lowAngle);
          float m = (spMax - spMin) / range;
          float Alpha;
          if (prevAngle > lowAngle) {
            Alpha = prevAngle - lowAngle;
          } else {
            Alpha = (360 - lowAngle) + prevAngle;
          }
          newT = Alpha * m + spMin;
#ifdef DEBUG
          Serial.print("Setpoint = ");
          Serial.println(newT);
#endif
        }
      }
    }
  }

  if(spChange){
    spChange = false;
    if(ovenStatus == OVEN_STATUS_OFF){
      ovenStatus = OVEN_STATUS_ON;
      windowStartTime = millis();
      // Tell the PID to range between 0 and the full window size
      reflowOvenPID.SetOutputLimits(0, windowSize);
      reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
      // Turn the PID on
      reflowOvenPID.SetMode(AUTOMATIC);
    }
    setpoint = newT;
  }
  
#ifdef DEBUG
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    switch (incomingByte) {
      case 'z':
        if(calPoints != 0x6){
          Serial.println("Zero should be the last calibration - Avoid Turn On during calibration");
          break;
        }
        zeroAngle = angle;
        Serial.println("Calibrating Zero Position, angle = ");
        Serial.println(zeroAngle);
        calPoints |= 0x01;
        break;
      case 'l':
        lowAngle = angle;
        Serial.println("Calibrating Min Value Position, angle = ");
        Serial.println(lowAngle);
        calPoints |= 0x02;
        break;
      case 'h':
        highAngle = angle;
        Serial.println("Calibrating Max Value Position, angle = ");
        Serial.println(highAngle);
        calPoints |= 0x04;
        break;
      case '1':
        if (calPoints != calComplete) {   // If calibrartion is not done, we can manually change SSR
          Serial.println("Toggle SSR 1");
          SSR1 ^= 1;
          digitalWrite(SSR1_IN, SSR1);
        }else{
          Serial.println("SSR manual change not allowed");
        }
        break;
      case '2':
        if (calPoints != calComplete) {   // If calibrartion is not done, we can manually change SSR
          Serial.println("Toggle SSR 2");
          SSR2 ^= 1;
          digitalWrite(SSR2_IN, SSR2);
        }else{
          Serial.println("SSR manual change not allowed");
        }
        break;
      default:
        break;
    }
  }
#endif
//  // PID computation and SSR control
  if (calPoints == calComplete) { // proceed only if calibrationwas done
    if (ovenStatus == OVEN_STATUS_ON)
    {
      now = millis();
  
      reflowOvenPID.Compute();
  
      if ((now - windowStartTime) > windowSize)
      {
        // Time to shift the Relay Window
        windowStartTime += windowSize;
      }
      if (output > (now - windowStartTime)) 
        digitalWrite(SSR1_IN, HIGH);
      else 
        digitalWrite(SSR1_IN, LOW);
    }
    // Reflow oven process is off, ensure oven is off
    else
    {
      digitalWrite(SSR1_IN, LOW);
    }
  }
}

void i2c_Start() {
  delayMicroseconds(5);
  pinMode(PIN_SDA, OUTPUT);    // PIN_SDA LOW
  delayMicroseconds(5);
  pinMode(PIN_SCL, OUTPUT);    // PIN_SCL LOW
  delayMicroseconds(5);
}

void i2c_Stop() {
  pinMode(PIN_SDA, OUTPUT);
  delayMicroseconds(3);
  pinMode(PIN_SCL, INPUT);    // PIN_SCL HIGH
  delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT);    // PIN_SDA HIGH
  delayMicroseconds(5);
}

void i2c_Bit(int data) {
  if (data)
    pinMode(PIN_SDA, INPUT);
  //  else
  //    pinMode(PIN_SDA, OUTPUT);

  delayMicroseconds(3);
  pinMode(PIN_SCL, INPUT);
  delayMicroseconds(3);
  pinMode(PIN_SCL, OUTPUT);
}


void resetSensor() {
  // 0xFF
  i2c_Start();
  for (int i = 0; i < 8; ++i) {
    i2c_Bit(0x1);
  }
  i2c_Stop();
  delayMicroseconds(20);
  // 0xFF
  i2c_Start();
  for (int i = 0; i < 8; ++i) {
    i2c_Bit(0x1);
  }
  i2c_Stop();
  delayMicroseconds(20);
  // 0x00
  i2c_Start();
  pinMode(PIN_SDA, OUTPUT);
  for (int i = 0; i < 8; ++i) {
    i2c_Bit(0x0);
  }
  i2c_Stop();
  delayMicroseconds(20);
  // 0x00
  i2c_Start();
  pinMode(PIN_SDA, OUTPUT);
  for (int i = 0; i < 8; ++i) {
    i2c_Bit(0x0);
  }
  i2c_Stop();

  delayMicroseconds(30);
}


