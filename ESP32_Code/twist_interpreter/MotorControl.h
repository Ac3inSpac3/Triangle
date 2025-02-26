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
    //bool sendMotorFeedback(bool m1, bool m2, bool m3);

    void enableMotors(bool en);

  private:
    // Compute motor speeds based on inverse kinematics for a holonomic robot
    void computeMotorSpeeds(float Vx, float Vy, float omega);

  // Output PWM signals and control direction based on motor speed
    void setMotorPWM(float motorSpeed, int pwmPin, int enPin, int dirPin);

    // Motor speed values
    int motorSpeed1;
    int motorSpeed2;
    int motorSpeed3;

    // Motor 1 definitions
    const int m1PWM = 21;     // Correct
    const int m1En = 4;       // Correct
    const int m1Dir = 18;     // Correct 
    const int m1Fault = 22;   // Correct 
    const int m1Speed = 34;   // Correct 
    const int m1Channel = 0;  // PWM channel 1 

    // Motor 2 definitions
    const int m2PWM = 17;      // Correct
    const int m2En = 12;       // Correct
    const int m2Dir = 19;      // Correct
    const int m2Fault = 27;    // Correct
    const int m2Speed = 35;    // Correct
    const int m2Channel = 1;   // PWM channel 1

    // Motor 3 definitions
    const int m3PWM = 16;      // Correct
    const int m3En = 32;       // Correct
    const int m3Dir = 23;      // Correct
    const int m3Fault = 25;    // Correct
    const int m3Speed = 39;    // Correct
    const int m3Channel = 2;   // PWM channel 2

    // Define the PWM parameters
    const int pwmFrequency = 1000; // 1 kHz PWM frequency
    const int pwmResolution = 10;  // 10-bit resolution (0-1023, but we're using 800 steps)

    // Calculate duty cycles based on 800 steps
    const int minDutyCycle = 102;  // 10% of 1023
    const int maxDutyCycle = 921;  // 90% of 1023
    const int useableRange = maxDutyCycle - minDutyCycle;
};

#endif
