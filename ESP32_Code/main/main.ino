#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2
#define BUTTON_PIN 27  // Change this to your actual button pin

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscription callback: Controls LED based on received data
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

// Timer callback to publish button state at 5Hz (every 200ms)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data = (digitalRead(BUTTON_PIN) == LOW) ? 1 : 0;  // Button Pressed = 1, Not Pressed = 0
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Enable internal pull-up (button should connect to GND)

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // Create timer (5 Hz = every 200ms)
  const unsigned int timer_timeout = 200;  // 200ms = 5Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor (handles both publisher and subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));  // Add timer to executor
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));  // Reduce to 50ms for faster processing
}
