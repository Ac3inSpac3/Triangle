#include "MotorControl.h"
#include <Arduino.h>
#include <math.h>

MotorControl::MotorControl() {
  motorSpeed1 = 0;
  motorSpeed2 = 0;
  motorSpeed3 = 0;
}

void MotorControl::init() {
  // Configure motor control pins as output (for enable and direction)
  pinMode(m1En, OUTPUT);
  pinMode(m1Dir, OUTPUT);
  pinMode(m1Speed, INPUT);
  
  pinMode(m2En, OUTPUT);
  pinMode(m2Dir, OUTPUT);
  pinMode(m2Speed, INPUT);
  
  pinMode(m3En, OUTPUT);
  pinMode(m3Dir, OUTPUT);
  pinMode(m3Speed, INPUT);
  
  // Configure buttons as inputs
  pinMode(btnStop, INPUT);
  pinMode(btnReset, INPUT);
  pinMode(btnEnable, INPUT);
  

  // Set up PWM for each motor
  //ledcAttach(m1PWM, pwmFrequency, pwmResolution);
  //ledcAttach(m2PWM, pwmFrequency, pwmResolution);
  //ledcAttach(m3PWM, pwmFrequency, pwmResolution);

  // Set up PWM for each motor
  ledcSetup(m1Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(m1PWM, m1Channel);

  ledcSetup(m2Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(m2PWM, m2Channel);

  ledcSetup(m3Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(m3PWM, m3Channel);
}

void MotorControl::updateMotors(float Vx, float Vy, float omega) {
  // If motors are not enabled, do nothing
  if (!motorsEnabled) {
    enableMotors(false);
    return;
  }
  
  enableMotors(true);
  computeMotorSpeeds(Vx, Vy, omega);
  setMotorPWM(motorSpeed1, m1En, m1Dir, m1Channel); // Motor 1
  setMotorPWM(motorSpeed2, m2En, m2Dir, m2Channel); // Motor 2
  setMotorPWM(motorSpeed3, m3En, m3Dir, m3Channel); // Motor 3
}

void MotorControl::computeMotorSpeeds(float Vx, float Vy, float omega) {
  // Apply inversion settings if required
  if (invertX) Vx = -Vx;
  if (invertY) Vy = -Vy;
  if (invertOmega) omega = -omega;
  
  // Robot parameters:
  float r = 0.055;    // Wheel radius in meters (110mm diameter)
  float d = 0.16;     // Distance from the robot center to each wheel (meters)
  float gearRatio = 26.0; // 26 motor rotations per one wheel rotation
  
  // Compute wheel speeds (in radians per second) via inverse kinematics
  float omega_wheel1 = (1 / r) * (-0.5 * Vx + (sqrt(3) / 2) * Vy + d * omega);
  float omega_wheel2 = (1 / r) * (-0.5 * Vx - (sqrt(3) / 2) * Vy + d * omega);
  float omega_wheel3 = (1 / r) * (Vx + d * omega);

    // Convert wheel speeds (rad/s) to wheel RPM
  float wheelRPM1 = (omega_wheel1 / (2 * PI)) * 60;
  float wheelRPM2 = (omega_wheel2 / (2 * PI)) * 60;
  float wheelRPM3 = (omega_wheel3 / (2 * PI)) * 60;
  
  // Convert wheel RPM to motor RPM (accounting for gearbox)
  motorSpeed1 = wheelRPM1 * gearRatio;
  motorSpeed2 = wheelRPM2 * gearRatio;
  motorSpeed3 = wheelRPM3 * gearRatio;
}

void MotorControl::enableMotors(bool en) {
  digitalWrite(m1En, en ? HIGH : LOW);
  digitalWrite(m2En, en ? HIGH : LOW);
  digitalWrite(m3En, en ? HIGH : LOW);
  motorsEnabled = en;
}

void MotorControl::setMotorPWM(float motorSpeed, int enPin, int dirPin, int pwmChannel) {
  // Set motor direction based on the sign of motorSpeed
  if (motorSpeed >= 0) {
    digitalWrite(dirPin, HIGH);  // Forward
  } else {
    digitalWrite(dirPin, LOW);   // Reverse
    motorSpeed = -motorSpeed;    // Use absolute value for PWM calculation
  }
  
  // Check that the commanded motor speed does not exceed the maximum allowed (setMotorMaxRPM)
  if (motorSpeed > setMotorMaxRPM) {
    return;
  }
  
  // Calculate the PWM duty cycle based on the desired motor speed (in RPM)
  float dutyCycle = (motorSpeed / setMotorMaxRPM) * (useableRange) + minDutyCycle;

  // Check if the calculated duty cycle is within the expected bounds
  if (dutyCycle < minDutyCycle || dutyCycle > maxDutyCycle) {
    return;
  }

  int pwmDutyCycle = (int)dutyCycle;
  ledcWrite(pwmChannel, pwmDutyCycle);
}

void MotorControl::flashLED(int i) {
  for (int count = 0; count < i; count++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void MotorControl::checkButtons() {
  // Stop button: disable motors
  if (digitalRead(btnStop) == LOW) {
    motorsEnabled = false;
    enableMotors(false);
    flashLED(2);
  }
  
  // Enable button: enable motors
  if (digitalRead(btnEnable) == LOW) {
    motorsEnabled = true;
    flashLED(3);
  }
  
  // Reset button: restart the ESP32
  if (digitalRead(btnReset) == LOW) {
    flashLED(5);
    ESP.restart();
  }
}

WheelSpeeds MotorControl::readWheelSpeeds() {
  WheelSpeeds speeds;
  
  // Read ADC values from the speed feedback pins (range: 0-4095)
  int raw1 = analogRead(m1Speed);
  int raw2 = analogRead(m2Speed);
  int raw3 = analogRead(m3Speed);

  speeds.adc1 = raw1;
  speeds.adc2 = raw2;
  speeds.adc3 = raw3;
  
  // Convert ADC readings to motor RPM using your calibration:
  // Assume that raw values from 1790 to 3085 map linearly to 0 to max motor RPM.
  float motorRPM1 = map(raw1, 1819, 3085, 0, setMotorMaxRPM);
  float motorRPM2 = map(raw2, 1819, 3085, 0, setMotorMaxRPM);
  float motorRPM3 = map(raw3, 1819, 3085, 0, setMotorMaxRPM);
  
  float gearRatio = 26.0;
  
  // Convert motor RPM to wheel RPM by dividing by the gear ratio
  float wheelRPM1 = motorRPM1 / gearRatio;
  float wheelRPM2 = motorRPM2 / gearRatio;
  float wheelRPM3 = motorRPM3 / gearRatio;
  
  // Convert wheel RPM to linear speed (m/s) using wheel circumference
  float wheelCircumference = 2 * PI * 0.06; // Wheel radius = 0.055 m
  float speedThreshold = 0.02; // Minimum speed threshold to filter jitter

  // Apply threshold filtering while preserving sign
  speeds.wheelSpeed1 = (std::abs(wheelRPM1 * wheelCircumference / 60.0) < speedThreshold) ? 0 : (wheelRPM1 * wheelCircumference / 60.0);
  speeds.wheelSpeed2 = (std::abs(wheelRPM2 * wheelCircumference / 60.0) < speedThreshold) ? 0 : (wheelRPM2 * wheelCircumference / 60.0);
  speeds.wheelSpeed3 = (std::abs(wheelRPM3 * wheelCircumference / 60.0) < speedThreshold) ? 0 : (wheelRPM3 * wheelCircumference / 60.0);
  
  return speeds;
}
