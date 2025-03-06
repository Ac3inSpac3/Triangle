#include "MotorControl.h"
#include <Arduino.h>

/*
Functionality to be added:

Sending:
- sendMotorFeedback()
  - Send motor enable status (bool)
  - Send speed mode status (1, 2, 3)

Recieving:
- functionTBD()
  - Recieve command for enable and disable motors (bool motorsEnabled)
  - Recieve command for setting speed mode (int 1, 2, 3)
*/

/*
Max speed inputs for twist messages are 1.5 linear and 6.5 angular
*/

// Constructor: Initializes motor speeds to zero
MotorControl::MotorControl() {
  motorSpeed1 = 0;
  motorSpeed2 = 0;
  motorSpeed3 = 0;
}

void MotorControl::init() {
  // Configure motor control pins as input/output
  pinMode(m1En, OUTPUT);
  pinMode(m1Dir, OUTPUT);
  pinMode(m1Fault, INPUT);

  pinMode(m2En, OUTPUT);
  pinMode(m2Dir, OUTPUT);
  pinMode(m2Fault, INPUT);

  pinMode(m3En, OUTPUT);
  pinMode(m3Dir, OUTPUT);
  pinMode(m3Fault, INPUT);

  // Configure buttons as inputs with pull-up resistors
  pinMode(btnStop, INPUT);
  pinMode(btnReset, INPUT);
  //pinMode(btnMode, INPUT);
  pinMode(btnEnable, INPUT);

  // Set up PWM for each motor
  ledcAttach(m1PWM, pwmFrequency, pwmResolution);
  ledcAttach(m2PWM, pwmFrequency, pwmResolution);
  ledcAttach(m3PWM, pwmFrequency, pwmResolution);

}

void MotorControl::updateMotors(float Vx, float Vy, float omega) {
  // Checks if motors enabled flag is true, if false returns as calculating speeds is not needed
  if (!motorsEnabled) {
    enableMotors(false);    // Redundancy 
    return;
  }

  // If non zero motion command is received, enable motors and compute speeds
  if(Vx != 0 || Vy != 0 || omega != 0){
    enableMotors(true);

    // Compute the required motor speeds based on velocity inputs
    computeMotorSpeeds(Vx, Vy, omega);

    // Set motor PWM signals and direction
    setMotorPWM(motorSpeed1, m1PWM, m1En, m1Dir); // Motor 1
    setMotorPWM(motorSpeed2, m2PWM, m2En, m2Dir); // Motor 2
    setMotorPWM(motorSpeed3, m3PWM, m3En, m3Dir); // Motor 3

  }
  else{
    // If no movement command, disable motors
    enableMotors(false);
  }
}

void MotorControl::computeMotorSpeeds(float Vx, float Vy, float omega) {
    // Apply inversion settings
    if (invertX) Vx = -Vx;
    if (invertY) Vy = -Vy;
    if (invertOmega) omega = -omega;

    float r = 0.055;  // Wheel radius (meters)
    float d = 0.16;   // Distance from robot center to wheel (meters)

    // Compute motor speeds in radians per second
    motorSpeed1 = (1 / r) * (-0.5 * Vx + (sqrt(3) / 2) * Vy + d * omega);
    motorSpeed2 = (1 / r) * (-0.5 * Vx - (sqrt(3) / 2) * Vy + d * omega);
    motorSpeed3 = (1 / r) * (Vx + d * omega);
}


void MotorControl::enableMotors(bool en) {
  // Enable or disable all motors based on the command
  digitalWrite(m1En, en ? HIGH : LOW);
  digitalWrite(m2En, en ? HIGH : LOW);
  digitalWrite(m3En, en ? HIGH : LOW);
}


void MotorControl::setMotorPWM(float motorSpeed, int pwmPin, int enPin, int dirPin) {
  // Determine the direction of rotation
  if (motorSpeed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward direction
  } 
  else {
    digitalWrite(dirPin, LOW);  // Reverse direction
    motorSpeed = -motorSpeed;   // Convert speed to positive value
  }

  // Map motor speed to a PWM duty cycle (scaled to fit the range) 
  int pwmDutyCycle = map(motorSpeed, 0, maxRPS, minDutyCycle, maxDutyCycle);
  ledcWrite(pwmPin, pwmDutyCycle);
}

bool MotorControl::sendMotorFeedback() {
  // Check for motor faults (LOW indicates a fault)
  return (digitalRead(m2Fault) == LOW);
}

void MotorControl::flashLED(int i) {
  for (int count = 0; count < i; count++) {
    digitalWrite(ledPin, HIGH);   // Turn LED on
    delay(100);                   // Wait
    digitalWrite(ledPin, LOW);    // Turn LED off
    delay(100);                   // Wait
  }
}

void MotorControl::checkButtons() {
  // Stop button: Disable motors
  if (digitalRead(btnStop) == LOW) {
    motorsEnabled = false;
    enableMotors(false);
    flashLED(2);
  }

  // Enable button: Re-enable motors
  if (digitalRead(btnEnable) == LOW) {
    motorsEnabled = true;
    //enableMotors(true);
    flashLED(3);
  }

  // Reset button: Restart ESP32
  if (digitalRead(btnReset) == LOW) {
    flashLED(5);
    ESP.restart();
  }
}