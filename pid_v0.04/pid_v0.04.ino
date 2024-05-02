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
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }

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
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
//******************************************************************************************************************
void loop() {
  String pattern = getPattern("WhiteLine");

  if (pattern == "11100000" || pattern == "01110000") {  //sol ağır basarsa

  } else if (pattern == "00000111" || pattern == "00001110") {  //sağ ağır basarsa

  } else {
  }
  qtr.read(sensorValues);
  
  int med_Speed_R;
  int med_Speed_L;

  
  position = calculateError(sensorValues, "WhiteLine");  //BlackLine//WhiteLine

  P_error = position;
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


  /*
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
  Serial.print("  leftMotorSpeed:");
  Serial.print(leftMotorSpeed);
  Serial.print("  rightMotorSpeed:");
  Serial.println(rightMotorSpeed);
*/

  // Motorları kontrol et
  if (leftMotorSpeed < 0) {
    // Sağa dön
    digitalWrite(rightMotor1, 1);
    digitalWrite(rightMotor2, 0);
    analogWrite(rightMotorPWM, abs(rightMotorSpeed));

    digitalWrite(leftMotor1, 1);
    digitalWrite(leftMotor2, 0);
    analogWrite(leftMotorPWM, abs(leftMotorSpeed));
  } else if (rightMotorSpeed < 0) {
    // Sola dön
    digitalWrite(rightMotor1, 0);
    digitalWrite(rightMotor2, 1);
    analogWrite(rightMotorPWM, abs(rightMotorSpeed));

    digitalWrite(leftMotor1, 0);
    digitalWrite(leftMotor2, 1);
    analogWrite(leftMotorPWM, abs(leftMotorSpeed));
  } else {
    digitalWrite(rightMotor1, 1);  //ileri
    digitalWrite(rightMotor2, 0);
    analogWrite(rightMotorPWM, rightMotorSpeed);

    digitalWrite(leftMotor1, 0);  //ileri
    digitalWrite(leftMotor2, 1);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
}

double calculateError(unsigned int *sensors, const char *lineType) {
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
String getPattern(const char *lineType) {
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
