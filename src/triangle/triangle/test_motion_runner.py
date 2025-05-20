import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class TestMotionRunner(Node):
    def __init__(self):
        super().__init__('test_motion_runner')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Int32, '/enb_state', 10)

        self.last_test_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.cooldown = 2  # seconds between test triggers
        self.square_repeats = 2
        self.move_speed = 0.2  # m/s
        self.turn_speed = 0.5  # rad/s

        self.get_logger().info("TestMotionRunner ready. Waiting for button press.")

    def joy_callback(self, msg):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.last_test_time < self.cooldown:
            return  # Debounce

        if msg.buttons[13]:  # Button 13 (up): Square Test
            self.last_test_time = now
            self.run_square_test(self.square_repeats)

        elif msg.buttons[14]:  # Button 14 (down): Forward/Back
            self.last_test_time = now
            self.run_forward_back_test()

        elif msg.buttons[15]:  # Button 15 (left): Left/Right
            self.last_test_time = now
            self.run_left_right_test()

        elif msg.buttons[16]:  # Button 16 (right): Angular Z Test
            self.last_test_time = now
            self.run_angular_test()

    def publish_twist(self, x=0.0, y=0.0, z=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.cmd_vel_pub.publish(twist)
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        end_time = start_time + int(duration)
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Stop

    def toggle_logging(self, enable: bool):
        msg = Int32()
        msg.data = 1 if enable else 0
        self.enable_pub.publish(msg)
        self.get_logger().info(f"Logging {'started' if enable else 'stopped'}")
        time.sleep(1.0)  # Wait for logger to sync

    def run_square_test(self, repeats):
        self.get_logger().info(f"Running square test ({repeats} loops)")
        self.toggle_logging(True)
        side_length = 0.25  # meters
        duration = side_length / self.move_speed

        for i in range(repeats):
            self.publish_twist(x=self.move_speed, duration=duration)   # forward
            self.publish_twist(y=self.move_speed, duration=duration)   # left
            self.publish_twist(x=-self.move_speed, duration=duration)  # backward
            self.publish_twist(y=-self.move_speed, duration=duration)  # right

        self.toggle_logging(False)

    def run_forward_back_test(self):
        self.get_logger().info("Running forward/back test")
        self.toggle_logging(True)
        duration = 0.5 / self.move_speed
        self.publish_twist(x=self.move_speed, duration=duration)
        self.publish_twist(x=-self.move_speed, duration=duration)
        self.toggle_logging(False)

    def run_left_right_test(self):
        self.get_logger().info("Running left/right test")
        self.toggle_logging(True)
        duration = 0.5 / self.move_speed
        self.publish_twist(y=self.move_speed, duration=duration)
        self.publish_twist(y=-self.move_speed, duration=duration)
        self.toggle_logging(False)

    def run_angular_test(self):
        self.get_logger().info("Running angular test (left/right)")
        self.toggle_logging(True)
        duration = 90.0 / (self.turn_speed * 180.0 / 3.1416)  # ~90 deg turn each way
        self.publish_twist(z=self.turn_speed, duration=duration)
        self.publish_twist(z=-self.turn_speed, duration=duration)
        self.toggle_logging(False)

def main(args=None):
    rclpy.init(args=args)
    node = TestMotionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test runner stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
