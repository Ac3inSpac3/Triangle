import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np

class TeleopJoyNode(Node):
    def __init__(self):
        super().__init__('triangle_teleop_joy')
        
        # Parameters for speed control
        self.declare_parameter('linear_speed_scale', 0.5)
        self.declare_parameter('angular_speed_scale', 1.0)
        self.declare_parameter('deadzone', 0.1)
        
        self.linear_speed_scale = self.get_parameter('linear_speed_scale').value
        self.angular_speed_scale = self.get_parameter('angular_speed_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Create subscriber for joy messages
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10)
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        self.enable_pub = self.create_publisher(
            Int32, 
            '/enb_state', 
            10)

        # Button and axis mapping for PS3 controller - adjust if needed
        self.BTN_ENABLE = 0  # X button
        self.BTN_DISABLE = 1  # Circle button
        self.AXIS_LINEAR_X = 1  # Left stick Y (up/down)
        self.AXIS_LINEAR_Y = 0  # Left stick X (left/right)
        self.AXIS_ANGULAR_Z = 3  # Right stick X (left/right)

        # Initialize state
        self.motors_enabled = False
        self.last_twist = Twist()

        # Timer to publish latest twist at fixed rate (2 Hz)
        self.timer = self.create_timer(0.25, self.publish_latest_twist)

        self.get_logger().info('Teleop Joy node initialized - With Movement Control')

    def joy_callback(self, msg):
        # Check for enable/disable motor buttons
        if len(msg.buttons) > max(self.BTN_ENABLE, self.BTN_DISABLE):
            if msg.buttons[self.BTN_ENABLE] == 1 and not self.motors_enabled:
                self.motors_enabled = True
                self.publish_enable_state(True)
                self.get_logger().info('Motors enabled')
            elif msg.buttons[self.BTN_DISABLE] == 1 and self.motors_enabled:
                self.motors_enabled = False
                self.publish_enable_state(False)
                self.get_logger().info('Motors disabled')
        
        # Only process movement if motors are enabled
        if self.motors_enabled and len(msg.axes) > max(self.AXIS_LINEAR_X, self.AXIS_LINEAR_Y, self.AXIS_ANGULAR_Z):
            self.update_latest_twist(msg)

    def update_latest_twist(self, joy_msg):
        twist = Twist()
        
        # Apply deadzone and scale to joystick inputs
        linear_x = self.apply_deadzone(joy_msg.axes[self.AXIS_LINEAR_X])
        linear_y = self.apply_deadzone(joy_msg.axes[self.AXIS_LINEAR_Y])
        angular_z = self.apply_deadzone(joy_msg.axes[self.AXIS_ANGULAR_Z])
        
        twist.linear.x = linear_x * self.linear_speed_scale
        twist.linear.y = linear_y * self.linear_speed_scale
        twist.angular.z = angular_z * self.angular_speed_scale
        
        self.last_twist = twist

    def publish_latest_twist(self):
        if self.motors_enabled:
            self.cmd_vel_pub.publish(self.last_twist)

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        else:
            return (value - np.sign(value) * self.deadzone) / (1.0 - self.deadzone)

    def publish_enable_state(self, state):
        enable_msg = Int32()
        enable_msg.data = int(state)
        self.enable_pub.publish(enable_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we disable motors when shutting down
        node.publish_enable_state(False)
        if node.motors_enabled:
            zero_twist = Twist()
            node.cmd_vel_pub.publish(zero_twist)
        node.get_logger().info('Shutting down and disabling motors')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
