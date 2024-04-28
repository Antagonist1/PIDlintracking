#include <QTRSensors.h>

//------------------------------------------------------------------------------------------------------------------
#define rightMotor2 12
#define rightMotor1 11
#define rightMotorPWM 10
#define leftMotorPWM 9
#define leftMotor1 8
#define leftMotor2 7

//------------------------------------------------------------------------------------------------------------------
double Kp = 0.1;
double Kd = -0.05;
double Ki = 0.001;
uint8_t rightMaxSpeed = 250;
uint8_t leftMaxSpeed = 250;
uint8_t rightBaseSpeed = 120;
uint8_t leftBaseSpeed = 120;

//------------------------------------------------------------------------------------------------------------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

//------------------------------------------------------------------------------------------------------------------
double lastError = 0;
double integral = 0;
int position = 0;

//------------------------------------------------------------------------------------------------------------------
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

void loop() {
  SensorRead();
  double error = calculateError(sensorValues); 
  Serial.println(error);

  // PID Hesaplama
  double output = Kp * error + Ki * integral + Kd * (error - lastError);
  //Serial.println(output);
  lastError = error;
  integral += error;

  int rightMotorSpeed = rightBaseSpeed + constrain(output, -255, 255);
  int leftMotorSpeed = leftBaseSpeed - constrain(output, -255, 255);

  // Motor hızlarını sınırla
  if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

  // Motorları kontrol et
  digitalWrite(rightMotor1, 1);
  digitalWrite(rightMotor2, 0);
  analogWrite(rightMotorPWM, rightMotorSpeed);

  digitalWrite(leftMotor1, 0);
  digitalWrite(leftMotor2, 1);
  analogWrite(leftMotorPWM, leftMotorSpeed);
}
// Çizgiden sapma hesapla
double calculateError(unsigned int *sensors) {
  double weightedSum = 0;
  double sum = 0;

  for (int i = 0; i < 4; i++) {
    weightedSum -= (2500-sensors[i]) * (4 -i);
    sum += (2500-sensors[i]);
  }
  for (int i = 4; i < 8; i++) {
    weightedSum += (2500-sensors[i])* (i - 3);
    sum += (2500-sensors[i]);
  }
  if (weightedSum == 0)
    return 0;

  return ((weightedSum / sum)*100 );
}

void SensorRead() {
  qtr.read(sensorValues);
  for (int i = 0; i < 4; i++) {
    position += (-sensorValues[i]);
  }
  for (int i = 4; i < 8; i++) {
    position += (sensorValues[i]);
  }
  position /= 8;
  return position;
}