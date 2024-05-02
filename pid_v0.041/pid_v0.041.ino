#include <QTRSensors.h>

//------------------------------------------------------------------------------------------------------------------
#define rightMotor2 12
#define rightMotor1 11
#define rightMotorPWM 10
#define leftMotorPWM 9
#define leftMotor1 8
#define leftMotor2 7

//------------------------------------------------------------------------------------------------------------------
#define Kp 0.14    //[0.14]  //0.35  //0.4  //0.7
#define Ki 0.0125  //[0.0125]  //0.15 //0.05 //0.15
#define Kd 0.01    //[0.01]  // //0.005   //0.4

//------------------------------------------------------------------------------------------------------------------
uint8_t rightMaxSpeed = 100;
uint8_t leftMaxSpeed = 100;

//------------------------------------------------------------------------------------------------------------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

//------------------------------------------------------------------------------------------------------------------
int lastError = 0;
int position = 0;
unsigned long cTime, pTime;
float eTime;
float P_error;
float I_error;
float D_error;
float PID_value;

//******************************************************************************************************************
void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A2, A1, A0, 2, 3, 4, 5, 6 }, SensorCount);

  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);

  analogWrite(rightMotorPWM, 0);
  analogWrite(leftMotorPWM, 0);

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
  digitalWrite(LED_BUILTIN, LOW);
}
//******************************************************************************************************************
void loop() {
  qtr.read(sensorValues);
  String pattern = getPattern("WhiteLine");
  Serial.println("White sensors pattern: " + pattern);
  if (pattern == "11100000" || pattern == "01110000") {  //sol ağır basarsa
    Stop();
    delay(10);
    turnRight();
    delay(10);
  } else if (pattern == "00000111" || pattern == "00001110") {  //sağ ağır basarsa
    Stop();
    delay(10);
    turnLeft();
    delay(10);
  } else if (pattern == "11111000" || pattern == "11110000") {  //sola doksan
    Stop();
    delay(10);
    turnRight();
    delay(50);
  } else if (pattern == "00011111" || pattern == "00001111") {  //sağa doksan
    Stop();
    delay(10);
    turnLeft();
    delay(50);
  } else {
    position = calculateError(sensorValues, "WhiteLine");
    PIDcalculate(position);
  }
  //SerialHaberlesme();
}

void SerialHaberlesme() {
  char pntX[100];
  char floX[10];
  Serial.print("  Position:");
  Serial.print(position);
  dtostrf(Kp * P_error, 9, 3, floX);
  sprintf(pntX, "    P: %s", floX);
  Serial.print(pntX);
  dtostrf(Ki * I_error, 9, 3, floX);
  sprintf(pntX, "    I: %s", floX);
  Serial.print(pntX);
  dtostrf(Kd * D_error, 9, 3, floX);
  sprintf(pntX, "    D: %s", floX);
  Serial.print(pntX);
  dtostrf(PID_value, 9, 3, floX);
  sprintf(pntX, "    PID: %s", floX);
  Serial.print(pntX);
}
double calculateError(unsigned int *sensors, const char *lineType) {  //BlackLine//WhiteLine
  double weightedSum = 0;
  double sum = 0;

  for (int i = 0; i < 4; i++) {
    if (strcmp(lineType, "WhiteLine") == 0) {
      weightedSum -= ((2500 - sensors[i]) * (7 - (2 * i)));
      sum += (2500 - sensors[i]);
    } else if (strcmp(lineType, "BlackLine") == 0) {
      weightedSum -= ((sensors[i]) * (7 - (2 * i)));
      sum += (sensors[i]);
    }
  }
  for (int i = 4; i < 8; i++) {
    if (strcmp(lineType, "WhiteLine") == 0) {
      weightedSum += ((2500 - sensors[i]) * ((2 * i) - 7));
      sum += (2500 - sensors[i]);
    } else if (strcmp(lineType, "BlackLine") == 0) {
      weightedSum += ((sensors[i]) * ((2 * i) - 7));
      sum += (sensors[i]);
    }
  }
  if (sum == 0)
    return 0;

  return ((weightedSum / sum) * 100);
}
String getPattern(const char *lineType) {  //BlackLine//WhiteLine
  String pattern = "";

  if (strcmp(lineType, "WhiteLine") == 0) {
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] > 1250) {
        pattern += "1";
      } else {
        pattern += "0";
      }
    }
  } else if (strcmp(lineType, "BlackLine") == 0) {
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] <= 1250) {
        pattern += "1";
      } else {
        pattern += "0";
      }
    }
  }
  return pattern;
}
void PIDcalculate(int pos) {
  int med_Speed_R;
  int med_Speed_L;
  P_error = pos;
  cTime = micros();
  eTime = (float)(cTime - pTime) / 1000000;
  I_error = I_error * 2 / 3 + P_error * eTime;
  D_error = (P_error - lastError) / eTime;
  PID_value = Kp * P_error + Ki * I_error + Kd * D_error;

  delayMicroseconds(10000);
  delayMicroseconds(10000);
  delayMicroseconds(1000);

  lastError = P_error;
  pTime = cTime;

  med_Speed_L = leftMaxSpeed - abs(PID_value);
  med_Speed_R = rightMaxSpeed - abs(PID_value);
  int leftMotorSpeed = med_Speed_L - PID_value;
  int rightMotorSpeed = med_Speed_R + PID_value;
  leftMotorSpeed = constrain(leftMotorSpeed, -leftMaxSpeed, leftMaxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -rightMaxSpeed, rightMaxSpeed);

  motorSetSpeed(leftMotorSpeed, rightMotorSpeed);
}
void motorSetSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed < 0) {
    // Sağa dön
    digitalWrite(leftMotor1, 1);
    digitalWrite(leftMotor2, 0);
    analogWrite(leftMotorPWM, abs(leftSpeed));

    digitalWrite(rightMotor1, 1);
    digitalWrite(rightMotor2, 0);
    analogWrite(rightMotorPWM, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    // Sola dön
    digitalWrite(leftMotor1, 0);
    digitalWrite(leftMotor2, 1);
    analogWrite(leftMotorPWM, abs(leftSpeed));

    digitalWrite(rightMotor1, 0);
    digitalWrite(rightMotor2, 1);
    analogWrite(rightMotorPWM, abs(rightSpeed));
  } else {
    // Both motors forward
    digitalWrite(leftMotor1, 0);
    digitalWrite(leftMotor2, 1);
    analogWrite(leftMotorPWM, abs(leftSpeed));

    digitalWrite(rightMotor1, 1);
    digitalWrite(rightMotor2, 0);
    analogWrite(rightMotorPWM, abs(rightSpeed));
  }
}
void turnRight() {
  // Sağa dön
  digitalWrite(leftMotor1, 1);
  digitalWrite(leftMotor2, 0);
  analogWrite(leftMotorPWM, leftMaxSpeed);

  digitalWrite(rightMotor1, 1);
  digitalWrite(rightMotor2, 0);
  analogWrite(rightMotorPWM, rightMaxSpeed);
}
void turnLeft() {
  // Sola dön
  digitalWrite(leftMotor1, 0);
  digitalWrite(leftMotor2, 1);
  analogWrite(leftMotorPWM, leftMaxSpeed);

  digitalWrite(rightMotor1, 0);
  digitalWrite(rightMotor2, 1);
  analogWrite(rightMotorPWM, rightMaxSpeed);
}
void Stop() {
  // Sola dön
  digitalWrite(leftMotor1, 1);
  digitalWrite(leftMotor2, 1);
  analogWrite(leftMotorPWM, 0);

  digitalWrite(rightMotor1, 1);
  digitalWrite(rightMotor2, 1);
  analogWrite(rightMotorPWM, 0);
}