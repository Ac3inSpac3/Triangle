#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// Structure to hold measured wheel speeds (in m/s)
struct WheelSpeeds {
  float wheelSpeed1;
  float wheelSpeed2;
  float wheelSpeed3;
};

class MotorControl {
  public:
    MotorControl();
    void init();  // Initialize motors and pins
    // Update motor speeds based on velocity commands (Vx, Vy in m/s, omega in rad/s)
    void updateMotors(float Vx, float Vy, float omega);
    // Enable or disable motor drivers
    void enableMotors(bool en);
    // Check hardware buttons and perform actions (e.g., stop, enable, reset)
    void checkButtons();
    // Read wheel speeds (in m/s) using the analog speed feedback from each motor
    WheelSpeeds readWheelSpeeds();

    bool motorsEnabled = false;

    // float realMotorSpeed1;
    // float realMotorSpeed2;
    // float realMotorSpeed3;

  private:
    // Compute motor speeds from twist command using inverse kinematics (with gearbox factor)
    void computeMotorSpeeds(float Vx, float Vy, float omega);
    // Set the PWM output for one motor (based on computed motor speed in RPM)
    void setMotorPWM(float motorSpeed, int enPin, int dirPin, int pwmChannel);
    // Helper function to flash the LED a given number of times (for error or status indication)
    void flashLED(int i);

    const int ledPin = 2;

    // Configurable scaling: setRPM is the maximum allowed motor RPM
    float setRPM = 5000;
    float maxRPS = setRPM / 60;  // Maximum rotations per second

    // Inversion flags for axes
    bool invertX = true;
    bool invertY = true;
    bool invertOmega = false;

    // Button state and speed mode
    int speedMode = 1;

    // Computed motor speeds (in RPM) for each motor (after inverse kinematics & gearbox)
    int motorSpeed1;
    int motorSpeed2;
    int motorSpeed3;

    // Button pin definitions
    const int btnStop = 26;   // Immediately disables motor drivers
    const int btnReset = 5;   // Resets the board
    const int btnEnable = 14; // Enables the motors

    // Motor 1 pin definitions
    const int m1PWM = 21;     // PWM output pin for motor 1
    const int m1En = 4;       // Enable pin for motor 1
    const int m1Dir = 18;     // Direction pin for motor 1
    const int m1Speed = 34;   // Analog input for motor 1 speed feedback
    const int m1Channel = 0;  // PWM channel 0

    // Motor 2 pin definitions
    const int m2PWM = 17;     // PWM output pin for motor 2
    const int m2En = 12;      // Enable pin for motor 2
    const int m2Dir = 19;     // Direction pin for motor 2
    const int m2Speed = 35;   // Analog input for motor 2 speed feedback
    const int m2Channel = 1;  // PWM channel 1

    // Motor 3 pin definitions
    const int m3PWM = 16;     // PWM output pin for motor 3
    const int m3En = 32;      // Enable pin for motor 3
    const int m3Dir = 23;     // Direction pin for motor 3
    const int m3Speed = 39;   // Analog input for motor 3 speed feedback
    const int m3Channel = 2;  // PWM channel 2

    // PWM parameters: 1 kHz frequency and 10-bit resolution (0-1023), with a usable range
    const int pwmFrequency = 1000;
    const int pwmResolution = 10;
    const int minDutyCycle = 102;  // 10% of 1023
    const int maxDutyCycle = 921;  // 90% of 1023
    const int useableRange = maxDutyCycle - minDutyCycle;
};

#endif
