#include <QTRSensors.h>

//MOTOR PINS
// D0 D1 control m1 (Left)
// D2 D3 control m2 (Right)
#define D0 2  // Left motor forward 
#define D1 3  // Left motor reverse 
#define D2 5  // Right motor forward 
#define D3 6  // Right motor reverse

#define MIN_SPEED 60       // Minimum speed to overcome motor dead zone
#define LINE_THRESHOLD 800 //800 code for black

//QTR SETUP
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint8_t qtrPins[SensorCount] = {8, 9, 10, 11, 12};

//PID VARs
float Kp = 0.25;  // Proportional: current error (sharp turn strength)
                  // If robot is too slow to turn → increase Kp slightly
float Ki = 0.0;   // Integral: accumulated small errors (drift correction)
                  // If robot slowly drifts off line on straights → increase Ki slightly
float Kd = 1.8;   // Derivative: predicts error change (prevents overshoot/wobble)
                  // If it wobbles too much → increase Kd

int lastError = 0;     // For derivative calculation
int integral = 0;      // Accumulated error
int baseSpeed = 170;   // Base motor speed before PID correction

void setup() {
  Serial.begin(9600);

  //QTR DIGITAL INIT 
  qtr.setTypeRC();//digital RC sensors
  qtr.setSensorPins(qtrPins, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);//on during calibration

  //CALIBRATION (2sec)
  // 400 cycles × 5ms delay = 2s
  // Lower time → higher speed → less accuracy (and vice versa)
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  digitalWrite(LED_BUILTIN, LOW);//off after calibration

  //MOTOR PINS SETUP
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  // Initialize motors stopped
  analogWrite(D0, 0);
  digitalWrite(D1, LOW);
  analogWrite(D2, 0);
 digitalWrite(D3, LOW);

  Serial.println("Calibration done");
}

void loop() {
  //line position
  //0 (far left), 4000 (far right)
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // Error: negative = left of center, positive = right of center
  int error = 2000 - position;

  //COUNT ACTIVE SENSORS
  int activeSensors = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      activeSensors++;
    }
  }

  if (activeSensors >= 4) {
    baseSpeed = 120;   // intersection(multiple sensors) -> slow down
  } else {
    baseSpeed = 170;  
  }

  //PID CALCULATION
  // Integral: accumulate error over time
  integral += error;
  integral = constrain(integral, -1000, 1000);

  // Derivative: rate of error change (smooths response)
  int derivative = error - lastError;
  lastError = error;

  // PID correction value
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  //motor speed
  // If correction > 0 → turn right (left faster, right slower)
  // If correction < 0 → turn left (right faster, left slower)
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Constrain to valid PWM range
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  //avoid dead zone
  if (leftSpeed > 0 && leftSpeed < MIN_SPEED) leftSpeed = MIN_SPEED;
  if (leftSpeed < 0 && leftSpeed > -MIN_SPEED) leftSpeed = -MIN_SPEED;

  if (rightSpeed > 0 && rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;
  if (rightSpeed < 0 && rightSpeed > -MIN_SPEED) rightSpeed = -MIN_SPEED;

  //setup motor
  motorDrive(leftSpeed, rightSpeed);


void motorDrive(int left, int right) {
  
  //LEFT MOTOR (D0 PWM, D1 always LOW)
  analogWrite(D0, left);      // 0 = stopped, 255 = full speed
  // D1 stays LOW (already set in setup)

  //RIGHT MOTOR (D2 = PWM, D3 = always LOW)
  analogWrite(D2, right);     // 0 = stopped, 255 = full speed
  // D3 stays LOW (already set in setup)
}
