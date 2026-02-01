#include <QTRSensors.h>

#define ENA 5
#define IN1 2
#define IN2 3

#define ENB 6
#define IN3 4
#define IN4 7

//qtr setup
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint8_t qtrPins[SensorCount] = {8, 9, 10, 11, 12};

//pid vars
float Kp = 0.25;
float Ki = 0.0;
float Kd = 2.5;

int lastError = 0;
int integral = 0;
int baseSpeed = 150;

void setup() {
  Serial.begin(9600);

  // QTR DIGITAL
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(13); // optional

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Calibrate
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  digitalWrite(LED_BUILTIN, LOW);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Calibration done");
}

// --------------------------------------------------
void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues); // 0 → 4000
  int error = 2000 - position;

  // PID
  integral += error;
  integral = constrain(integral, -1000, 1000); 

  int derivative = error - lastError;
  lastError = error;

  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motorDrive(leftSpeed, rightSpeed);
}
void motorDrive(int left, int right) {

  // LEFT MOTOR
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, left);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left);
  }

  // RIGHT MOTOR
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
  }
}
