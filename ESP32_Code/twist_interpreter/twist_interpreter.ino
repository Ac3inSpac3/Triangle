
#include <micro_ros_arduino.h>
#include <Arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

#include "MotorControl.h"

// ROS2 Subscriber and Publisher declarations
rcl_subscription_t subscriber;  // Subscriber for velocity commands (cmd_vel)
rcl_publisher_t fault_publisher; // Publisher for motor fault status

// Message objects for received velocity commands and motor fault status
geometry_msgs__msg__Twist msg;
std_msgs__msg__Bool fault_msg;

// ROS2-related objects for managing the micro-ROS node and execution
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Motor control instance
MotorControl motorControl;

// LED pin for error indication
#define LED_PIN 2

// Macros for checking return values from ROS functions and handling errors
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Function to handle errors by blinking the LED indefinitely
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
    delay(100); // Brief delay before toggling again
  }
}

// Callback function for processing received Twist messages (cmd_vel)
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extract velocity components from the message
  float Vx = msg->linear.x;    // Forward/backward speed
  float Vy = msg->linear.y;    // Lateral speed (for holonomic robots)
  float omega = msg->angular.z; // Rotational speed

  // Send velocity commands to the motors
  motorControl.updateMotors(Vx, Vy, omega);
}

void setup() {
  // Initialize micro-ROS transport (sets up communication)
  set_microros_transports();

  // Configure the error LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn LED on initially

  delay(2000); // Allow time for initialization
  
  // Initialize motor control system
  motorControl.init();

  // Set up ROS2 memory allocator
  allocator = rcl_get_default_allocator();

  // Initialize ROS2 support structure
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create a ROS2 node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Set up a subscriber to receive velocity commands (cmd_vel)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Set up a publisher to send motor fault status
  RCCHECK(rclc_publisher_init_default(
    &fault_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_fault"));

  // Initialize the executor to manage callbacks
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Add the velocity command subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  //delay(100); // Brief delay between execution cycles

  // Check motor status and publish fault status
  //fault_msg.data = motorControl.sendMotorFeedback();
  //RCSOFTCHECK(rcl_publish(&fault_publisher, &fault_msg, NULL));

  // Process incoming messages and execute callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Check button status and implement commanded changes  
  motorControl.checkButtons();
}
