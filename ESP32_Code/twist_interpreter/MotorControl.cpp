#include "MotorControl.h"
#include <Arduino.h>


MotorControl::MotorControl() {
  // Initialize the motor speeds to zero
  motorSpeed1 = 0;
  motorSpeed2 = 0;
  motorSpeed3 = 0;
}

void MotorControl::init() {
  // Initialize pins
  pinMode(m1En, OUTPUT);
  pinMode(m1Dir, OUTPUT);
  pinMode(m1Fault, INPUT);
  pinMode(m1Speed, INPUT);
  pinMode(m2En, OUTPUT);
  pinMode(m2Dir, OUTPUT);
  pinMode(m3En, OUTPUT);
  pinMode(m3Dir, OUTPUT);

  // Set up PWM for each motor
  ledcAttach(m1PWM, pwmFrequency, pwmResolution);
  //ledcSetup(m1Channel, pwmFrequency, pwmResolution);
  //ledcAttachPin(m1PWM, m1Channel);

  ledcAttach(m2PWM, pwmFrequency, pwmResolution);
  //ledcSetup(m2Channel, pwmFrequency, pwmResolution);
  //ledcAttachPin(m2PWM, m2Channel);

  ledcAttach(m3PWM, pwmFrequency, pwmResolution);
  //ledcSetup(m3Channel, pwmFrequency, pwmResolution);
  //ledcAttachPin(m3PWM, m3Channel);

}

void MotorControl::updateMotors(float Vx, float Vy, float omega) {
  if(Vx != 0 || Vy != 0 || omega != 0){
    enableMotors(true);
    // Compute motor speeds based on the input values
    computeMotorSpeeds(Vx, Vy, omega);

    // Set motor speeds with direction and PWM output
    setMotorPWM(motorSpeed1, m1PWM, m1En, m1Dir); // Motor 1
    setMotorPWM(motorSpeed2, m2PWM, m2En, m2Dir); // Motor 2
    setMotorPWM(motorSpeed3, m3PWM, m3En, m3Dir); // Motor 3

  }
  else{
    enableMotors(false);
  }
}

void MotorControl::computeMotorSpeeds(float Vx, float Vy, float omega) {
  // Example: Basic kinematic calculation for a 3-wheel holonomic robot
  float r = 0.1;  // Wheel radius
  float d = 0.1;  // Wheel distance from center of robot
  
  motorSpeed1 = -(Vx/2) + ((sqrt(3) * Vy )/ 2) + (omega * d);
  motorSpeed2 = -(Vx/2) - ((sqrt(3) * Vy )/ 2) + (omega * d);
  motorSpeed3 = Vx + (omega * d);
}

void MotorControl::enableMotors(bool en){
  if(en){
    digitalWrite(m1En, HIGH);
    digitalWrite(m2En, HIGH);
    digitalWrite(m3En, HIGH);
  }
  else{
    digitalWrite(m1En, LOW);
    digitalWrite(m2En, LOW);
    digitalWrite(m3En, LOW);
  }
}


void MotorControl::setMotorPWM(float motorSpeed, int pwmPin, int enPin, int dirPin) {
  // Set direction
  bool dir;
  if (motorSpeed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward
    dir = true;
  } else {
    digitalWrite(dirPin, LOW);  // Backward
    motorSpeed = -motorSpeed;   // Convert to positive for PWM
    dir = false;
  }

  // Ensure motor is enabled
  digitalWrite(enPin, HIGH);

  // Map motorSpeed to PWM duty cycle (adjust as needed)
  int pwmDutyCycle = map(motorSpeed, 0, 130, minDutyCycle, maxDutyCycle);  // Scale motor speed to 10% to 90% PWM range
  ledcWrite(pwmPin, pwmDutyCycle);
  ///Serial.print("Motor "); Serial.print(pwmChannel); 
  //Serial.print(" Duty "); Serial.print(pwmDutyCycle);
  //Serial.print(" Reversed "); Serial.println(dir);
}

bool MotorControl::sendMotorFeedback() {
  // Read fault signals (LOW means fault)
  //return (digitalRead(m1Fault) == LOW, digitalRead(m2Fault) == LOW, digitalRead(m3Fault) == LOW);
}
