#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS2 message types
#include <std_msgs/msg/int32.h>       // For motor enable subscriber
#include <nav_msgs/msg/odometry.h>    // For odometry publisher
#include <geometry_msgs/msg/twist.h>  // For twist subscriber

#include "MotorControl.h"

// ----- ROS2 Node and Allocator -----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// ----- Publisher for Odometry -----
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t odom_timer; // Timer for publishing odometry

// ----- Subscriber for Motor Enable Control -----
rcl_subscription_t enb_subscriber;
std_msgs__msg__Int32 msg_enb;

// ----- Subscriber for Twist Messages (cmd_vel) -----
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist msg_twist;

// ----- Single Executor -----
rclc_executor_t executor;

// Motor control instance
MotorControl motorControl;

// LED pin for status indication
#define LED_PIN 2  

// ----- Macros for Error Handling -----
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

/**
 * @brief Function to indicate error by blinking LED indefinitely.
 */
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/**
 * @brief Timer callback function that publishes dummy odometry data.
 * 
 * @param timer Timer instance (unused)
 * @param last_call_time Last time the timer was executed (unused)
 */
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Set dummy values for odometry message
    msg_odom.header.stamp.sec = millis() / 1000; // Convert to seconds
    msg_odom.header.stamp.nanosec = (millis() % 1000) * 1000000; // Convert to nanoseconds
    msg_odom.header.frame_id.data = (char *)"odom";
    msg_odom.child_frame_id.data = (char *)"base_link";

    // Dummy position (robot stays at origin)
    msg_odom.pose.pose.position.x = 0.0;
    msg_odom.pose.pose.position.y = 0.0;
    msg_odom.pose.pose.position.z = 0.0;

    // Dummy orientation (no rotation)
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = 0.0;
    msg_odom.pose.pose.orientation.w = 1.0; // Represents no rotation (quaternion identity)

    // Dummy velocity (robot not moving)
    msg_odom.twist.twist.linear.x = 0.0;
    msg_odom.twist.twist.linear.y = 0.0;
    msg_odom.twist.twist.angular.z = 0.0;

    // Publish odometry message
    RCSOFTCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL));
  }
}

/**
 * @brief Callback function for motor enable state subscriber.
 *        This will be used to enable/disable motor drivers.
 *
 * @param msgin Incoming ROS2 message.
 */
void enb_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg_enb = (const std_msgs__msg__Int32 *)msgin;

  // TODO: Add motor enable/disable logic here
  // For now, this will just toggle the LED instead of controlling motors
  
  if(msg_enb->data == 1){
    motorControl.motorsEnabled = true;
    motorControl.enableMotors(true);
    digitalWrite(LED_PIN, HIGH);
  }
  else{ 
    motorControl.motorsEnabled = false;
    motorControl.enableMotors(false);
    digitalWrite(LED_PIN, LOW);
  }
  
}

/**
 * @brief Callback function for processing Twist (cmd_vel) messages.
 *        This function will be used to control motor movement.
 *
 * @param msgin Incoming ROS2 message.
 */
void twist_subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg_twist = (const geometry_msgs__msg__Twist *)msgin;

  // Extract velocity components from the message
  float Vx = msg_twist->linear.x;    // Forward/backward speed
  float Vy = msg_twist->linear.y;    // Lateral speed (for holonomic robots)
  float omega = msg_twist->angular.z; // Rotational speed

  // Send velocity commands to the motors
  motorControl.updateMotors(Vx, Vy, omega);
}

/**
 * @brief Setup function for micro-ROS node.
 */
void setup()
{
  // Initialize micro-ROS transport (e.g., Serial, WiFi)
  set_microros_transports();

  // Configure LED as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000); // Allow time for initialization

  // Initialize motor control system
  motorControl.init();

  // Set up ROS2 allocator
  allocator = rcl_get_default_allocator();

  // Initialize ROS2 support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_odom_test_node", "", &support));

  // ----- Initialize Motor Enable State Subscriber (/enb_state) -----
  RCCHECK(rclc_subscription_init_default(
    &enb_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "enb_state"));

  // ----- Initialize Twist Subscriber (/cmd_vel) -----
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // ----- Initialize Odometry Publisher (/odom) -----
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // ----- Initialize Odometry Timer -----
  const unsigned int odom_publish_rate = 200; // Publish every 100ms
  RCCHECK(rclc_timer_init_default(
    &odom_timer,
    &support,
    RCL_MS_TO_NS(odom_publish_rate),
    odom_timer_callback));

  // ----- Initialize Executor (single executor for all callbacks) -----
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); // Handles: odom timer + 2 subscribers

  // Add the odometry timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  // Add both subscribers to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &enb_subscriber, &msg_enb, &enb_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &msg_twist, &twist_subscription_callback, ON_NEW_DATA));

  // Start Serial for debugging
  Serial.begin(115200);
  Serial.println("micro-ROS Odom Node Initialized.");
}

/**
 * @brief Main loop - Spins the executor to process messages.
 */
void loop()
{
  // Process ROS2 messages and execute callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Check button status and implement commanded changes  
  motorControl.checkButtons();
}
