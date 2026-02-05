#include <QTRSensors.h>


//init motor
#define ENA 5 //pins for speed control
#define IN1 2 //left
#define IN2 3 //left

#define ENB 6 // kima ENA
#define IN3 4 //right
#define IN4 7 //right
#define MIN_SPEED 60
#define LINE_THRESHOLD 800

//qtr setup
QTRSensors qtr; //bib

const uint8_t SensorCount = 5; //obj for array
uint16_t sensorValues[SensorCount]; //[5]
uint8_t qtrPins[SensorCount] = {8, 9, 10, 11, 12}; //chaque sensor has a pin (qtrpins assigns pins to qtr)

//pid vars
float Kp = 0.25; //propo (current e) the further the car the stronger the propo (sharp turn)
//If robot is too slow to turn → increase Kp slightly
float Ki = 0.0; //integral / sum of smaller errors propo cant fix (low turn)
//If robot slowly drifts off the line on straight paths, increase Ki slightly
float Kd = 1.8; //derivee / predicts e (prevents overshoot)
//If it wobbles too much → increase Kd

int lastError = 0; //ll kd
int integral = 0; //accumelated error
int baseSpeed = 170; //for the motors before correction


void setup() {
  Serial.begin(9600);

  // QTR DIGITAL
  qtr.setTypeRC(); //telling it it's using digitals
  qtr.setSensorPins(qtrPins, SensorCount); //array, 5


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //turns built-in light when correcting


  // Calibrate
  for (int i = 0; i < 400; i++) { //400cycles*5delay=2s (lowertime->highspeed->less accurracy) and vice versa
    qtr.calibrate(); //built in to sense colors
    delay(5);
  }

  digitalWrite(LED_BUILTIN, LOW);

  // Motor pins setup connection
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
  int error = 2000 - position; //2000 c centre t3 line/ pos c sensor val

  int activeSensors = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      activeSensors++;
    }
  }


  // ----- SET BASE SPEED BASED ON LINE WIDTH -----
  if (activeSensors >= 4) {
    baseSpeed = 120;   // wide line / intersection
  } else {
    baseSpeed = 170;   // normal tracking
  }

  // PID
  integral += error; //cumulated error
  integral = constrain(integral, -1000, 1000); //limit

  int derivative = error - lastError; //error change 
  lastError = error;

  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative); //pid

  int leftSpeed  = baseSpeed - correction; 
  int rightSpeed = baseSpeed + correction; 
  //m3ntha it turns left ida correction>0 else right

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

 // prevent motor dead zone 
if (leftSpeed > 0 && leftSpeed < MIN_SPEED) leftSpeed = MIN_SPEED;
if (leftSpeed < 0 && leftSpeed > -MIN_SPEED) leftSpeed = -MIN_SPEED;

if (rightSpeed > 0 && rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;
if (rightSpeed < 0 && rightSpeed > -MIN_SPEED) rightSpeed = -MIN_SPEED;

  motorDrive(leftSpeed, rightSpeed); //calls function
  

}
void motorDrive(int left, int right) { 

  // LEFT MOTOR direction
  if (left >= 0) {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
    analogWrite(ENA, left);
    //digitsl -> firection, nalog -> speed
   // one wheel clockwise

  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left);
     // one wheel counterclockwise
  }

  // RIGHT MOTOR 
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
     // one wheel clockwise
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
     // one wheel counterclockwise

  }
}
