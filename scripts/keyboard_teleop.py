#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import select


# Define the speed and turning increments
# THIS NEEDS TO BE CHANGED WHEN UPDATING ARDUINO CODE

LINEAR_SPEED_STEP = 50.0
ANGULAR_SPEED_STEP = 100.0

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher for geometry_msgs/Twist
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Current speeds
        self.linear_speed = 0.0
        self.linear_strafe = 0.0
        self.angular_speed = 0.0

        self.get_logger().info("Keyboard teleop node started.\n"
                               "Controls:\n"
                               "  W/S : Move forward/backward\n"
                               "  A/D : Strafe left/right\n"
                               "  Q/E : Rotate left/right\n"
                               "  X   : Stop all motion\n"
                               "  Ctrl-C to quit.")

    def run(self):
        try:
            while rclpy.ok():
                key = get_key()
                if key == 'w':
                    self.linear_speed += LINEAR_SPEED_STEP
                elif key == 's':
                    self.linear_speed -= LINEAR_SPEED_STEP
                elif key == 'a':
                    self.linear_strafe += LINEAR_SPEED_STEP
                elif key == 'd':
                    self.linear_strafe -= LINEAR_SPEED_STEP
                elif key == 'q':
                    self.angular_speed += ANGULAR_SPEED_STEP
                elif key == 'e':
                    self.angular_speed -= ANGULAR_SPEED_STEP
                elif key == 'x':
                    # Stop
                    self.linear_speed = 0.0
                    self.linear_strafe = 0.0
                    self.angular_speed = 0.0
                elif key == '\x03':  # Ctrl-C
                    break

                # Publish the Twist
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.linear.y = self.linear_strafe
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)

        except KeyboardInterrupt:
            pass

def get_key():
    # Utility function to read one character from terminal
    # without waiting for newline
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
