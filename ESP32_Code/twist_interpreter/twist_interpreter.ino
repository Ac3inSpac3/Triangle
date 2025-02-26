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

rcl_subscription_t subscriber;
rcl_publisher_t fault_publisher;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Bool fault_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

MotorControl motorControl;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Twist message callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extract linear and angular velocities
  float Vx = msg->linear.x;
  float Vy = msg->linear.y;
  float omega = msg->angular.z;

  // Update motors based on Twist message
  motorControl.updateMotors(Vx, Vy, omega);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  delay(2000);

  // Initialize motors
  motorControl.init();

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create publisher for motor fault status
  RCCHECK(rclc_publisher_init_default(
    &fault_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_fault"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);

  // Check for motor faults and publish status
  fault_msg.data = motorControl.sendMotorFeedback();
  RCSOFTCHECK(rcl_publish(&fault_publisher, &fault_msg, NULL));

  // Process micro-ROS callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
