#include <QTRSensors.h>

//------------------------------------------------------------------------------------------------------------------
#define rightMotor2 12
#define rightMotor1 11
#define rightMotorPWM 10
#define leftMotorPWM 9
#define leftMotor1 8
#define leftMotor2 7

//------------------------------------------------------------------------------------------------------------------
#define Kp 0.085  // 255: 0.1     110: 0.2                //0.167
#define Ki 0.55   // 255: 0.05    110: 0.05
#define Kd 5      // 255: 0.003   110: 0.004

//------------------------------------------------------------------------------------------------------------------
uint8_t rightMaxSpeed = 110;
uint8_t leftMaxSpeed = 110;

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
  //------------------------------------------------------------------------------------------------------------------
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A2, A1, A0, 2, 3, 4, 5, 6 }, SensorCount);

  //------------------------------------------------------------------------------------------------------------------
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);

  //------------------------------------------------------------------------------------------------------------------
  analogWrite(rightMotorPWM, 0);
  analogWrite(leftMotorPWM, 0);
  //------------------------------------------------------------------------------------------------------------------
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
//******************************************************************************************************************
void loop() {
  char pntX[100];
  char floX[10];
  int med_Speed_R;
  int med_Speed_L;

  qtr.read(sensorValues);
  position = calculateError(sensorValues);

  P_error = position;
  cTime = millis();
  eTime = (float)(cTime - pTime) / 1000;
  I_error = I_error * 2 / 3 + P_error * eTime;
  D_error = (P_error - lastError) / eTime;
  PID_value = Kp * P_error + Ki * I_error + Kd * D_error;


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
  Serial.println(pntX);


  lastError = P_error;
  pTime = cTime;

  med_Speed_L = leftMaxSpeed - abs(PID_value);
  med_Speed_R = rightMaxSpeed - abs(PID_value);
  int leftMotorSpeed = med_Speed_L - PID_value;
  int rightMotorSpeed = med_Speed_R + PID_value;
  leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);

  Serial.print("  leftMotorSpeed:");
  Serial.print(leftMotorSpeed);
  Serial.print("  rightMotorSpeed:");
  Serial.print(rightMotorSpeed);

  delayMicroseconds(40);

  // Motorları kontrol et
  digitalWrite(rightMotor1, 1);
  digitalWrite(rightMotor2, 0);
  analogWrite(rightMotorPWM, rightMotorSpeed);

  digitalWrite(leftMotor1, 0);
  digitalWrite(leftMotor2, 1);
  analogWrite(leftMotorPWM, leftMotorSpeed);
}

double calculateError(unsigned int *sensors) {
  double weightedSum = 0;
  double sum = 0;

  for (int i = 0; i < 4; i++) {
    weightedSum -= (sensors[i] * (7 - (2 * i)));
    sum += (sensors[i]);
  }
  for (int i = 4; i < 8; i++) {
    weightedSum += (sensors[i] * ((2 * i) - 7));
    sum += (sensors[i]);
  }
  if (sum == 0)
    return 0;

  return ((weightedSum / sum) * 100);
}
