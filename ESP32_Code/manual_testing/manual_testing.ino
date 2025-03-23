#include "MotorControl.h"

MotorControl motorControl;

void setup() {
  Serial.begin(115200);
  motorControl.init();  // Initialize the motor control system
  Serial.println("MotorControl initialized.");
}

void loop() {
  // Send a speed command to the motors
  Serial.println("Sending speed command: Vx = 0.0, Vy = 1.0, omega = 0");
  motorControl.updateMotors(0.0, 1.0, 0);  // Move forward
  delay(1000);  // Wait for 2 seconds

  motorControl.readMotorSpeeds();
  Serial.print("Motor 1 Speed: "); Serial.println(motorControl.realMotorSpeed1);
  Serial.print("Motor 2 Speed: "); Serial.println(motorControl.realMotorSpeed2);
  Serial.print("Motor 3 Speed: "); Serial.println(motorControl.realMotorSpeed3);

  delay(1000);  // Wait for 2 seconds

  // Stop the motors
  Serial.println("Stopping motors");
  motorControl.updateMotors(0.0, 0.0, 0.0);  // Stop
  delay(1000);  // Wait for 1 second

  motorControl.readMotorSpeeds();
  Serial.print("Motor 1 Speed: "); Serial.println(motorControl.realMotorSpeed1);
  Serial.print("Motor 2 Speed: "); Serial.println(motorControl.realMotorSpeed2);
  Serial.print("Motor 3 Speed: "); Serial.println(motorControl.realMotorSpeed3);
  
  // Send a speed command to the motors
  Serial.println("Sending speed command: Vx = 0.0, Vy = 1.0, omega = 0");
  motorControl.updateMotors(0.0, -1.0, 0);  // Move forward
  delay(1000);  // Wait for 2 seconds

  motorControl.readMotorSpeeds();
  Serial.print("Motor 1 Speed: "); Serial.println(motorControl.realMotorSpeed1);
  Serial.print("Motor 2 Speed: "); Serial.println(motorControl.realMotorSpeed2);
  Serial.print("Motor 3 Speed: "); Serial.println(motorControl.realMotorSpeed3);

  delay(1000);  // Wait for 2 seconds

  // Stop the motors
  Serial.println("Stopping motors");
  motorControl.updateMotors(0.0, 0.0, 0.0);  // Stop
  delay(1000);  // Wait for 1 second

  motorControl.readMotorSpeeds();
  Serial.print("Motor 1 Speed: "); Serial.println(motorControl.realMotorSpeed1);
  Serial.print("Motor 2 Speed: "); Serial.println(motorControl.realMotorSpeed2);
  Serial.print("Motor 3 Speed: "); Serial.println(motorControl.realMotorSpeed3);

  // Check button states
  motorControl.checkButtons();

  // Repeat the loop
}