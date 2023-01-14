#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

int p = 0;
int i = 0;
int d = 0;

int error = 0;
int lastError1 = 0, lastError2 = 0, lastError3 = 0, lastError4 = 0;

const int maxSpeed = 225;
const int minSpeed = -225;
const int calibrationDuration = 6000;
int baseSpeed = 225;
int sense = 200;
int lastErrorCalib = 0;

QTRSensors qtr;
bool onLineStart = false;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
void setup() {

  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  Serial.begin(9600);
}


void loop() {

  if (millis() < calibrationDuration) {
    mainCalibrationDrive();
  } 
  else if (!onLineStart) {
      resetOnLine();
  }
  else 
    pidControl(40, 0, 4);
}


void resetOnLine() {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
  if (error < 0)
    setMotorSpeed(-200, 200);
  else if (error > 0)
    setMotorSpeed(200, -200);
  if (error == 0) {
    onLineStart = true;
    setMotorSpeed(0, 0);

  } 
}

void mainCalibrationDrive() {

  qtr.calibrate();
  
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  if ((error == -50 || error == 50) && lastErrorCalib != error) {
    m1Speed = -m1Speed;
    m2Speed = -m2Speed;
  }
  lastErrorCalib = error;

  setMotorSpeed(m1Speed, m2Speed);
}


void pidControl(float kp, float ki, float kd) {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -40, 40);

  p = error;
  d = error - lastError4;
  lastError4 = lastError3;
  lastError3 = lastError2;
  lastError2 = lastError1;
  lastError1 = error;

  int motorSpeed = kp * p + ki * i + kd * d;

  m1Speed = baseSpeed; 
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  }
  else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

  setMotorSpeed(m1Speed, m2Speed);

}


void setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}