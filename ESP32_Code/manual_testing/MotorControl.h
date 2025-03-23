#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
  public:
    MotorControl();
    void init();  // Initialize motors and pins

    // Update motor speeds based on ROS2 velocity commands (forward/backward, lateral, and angular velocity)
    void updateMotors(float Vx, float Vy, float omega);

    // Read feedback from motor drivers and send it back to ROS2
    bool sendMotorFeedback();

    // Enables or disables all motors
    void enableMotors(bool en);

    // Reads button states and performs actions
    void checkButtons(); 

    void readMotorSpeeds();

    // Real motor speed values
    float realMotorSpeed1 = 0;
    float realMotorSpeed2 = 0;
    float realMotorSpeed3 = 0;

  private:
    // Compute motor speeds based on inverse kinematics for a holonomic robot
    void computeMotorSpeeds(float Vx, float Vy, float omega);

    // Output PWM signals and control direction based on motor speed
    void setMotorPWM(float motorSpeed, int pwmPin, int enPin, int dirPin);

    // Flashes an LED the number of times specified
    void flashLED(int i);

    const int ledPin = 2;

    // Configurable scaling factors
    float setRPM = 2000;
    float maxRPS = setRPM / 60;     // Default max motor RPS
    bool invertX = true;    // If true, reverses X-axis movement
    bool invertY = true;    // If true, reverses Y-axis movement
    bool invertOmega = false;// If true, reverses rotational direction

    // Button states
    bool motorsEnabled = false; // Tracks if motors can be enabled
    int speedMode = 1;         // Tracks current speed mode (1-3)

    // Motor speed values
    int motorSpeed1;
    int motorSpeed2;
    int motorSpeed3;

    // Button Pin definitions
    const int btnStop = 26;   // Disables motor drivers, cannot be re enabled without pressing btnEnable
    const int btnReset = 5;   // Allows for resetting firmware to exit micro ros error loop
    //const int btnMode = 9;    // Switching between modes e.g. Speed 1, 2 3 (not implemented yet)
    const int btnEnable = 14; // Allows for motors to be enabled

    // Motor 1 definitions
    const int m1PWM = 21;      // Pin outputting PWM signal for motor
    const int m1En = 4;        // High for enable
    const int m1Dir = 18;      // High for reverse direction 
    const int m1Fault = 22;    // Input High for motor driver fault 
    const int m1Speed = 34;    // Correct 

    // Motor 2 definitions
    const int m2PWM = 17;      // Pin outputting PWM signal for motor
    const int m2En = 12;       // High for enable
    const int m2Dir = 19;      // High for reverse direction 
    const int m2Fault = 27;    // Input High for motor driver fault 
    const int m2Speed = 35;    // Correct 

    // Motor 3 definitions
    const int m3PWM = 16;      // Pin outputting PWM signal for motor
    const int m3En = 32;       // High for enable
    const int m3Dir = 23;      // High for reverse direction 
    const int m3Fault = 25;    // Input High for motor driver fault 
    const int m3Speed = 39;    // Correct 

    // Define the PWM parameters
    const int pwmFrequency = 1000; // 1 kHz PWM frequency
    const int pwmResolution = 10;  // 10-bit resolution (0-1023, but we're using 800 steps)

    // Calculate duty cycles based on 800 steps
    const int minDutyCycle = 102;  // 10% of 1023
    const int maxDutyCycle = 921;  // 90% of 1023
    const int useableRange = maxDutyCycle - minDutyCycle;
};

#endif