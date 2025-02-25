#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Changed from String to Int32
import sys
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Int32, 'micro_ros_arduino_subscriber', 10)  # Match the Arduino topic name
        self.get_logger().info("Press W to turn LED ON, S to turn LED OFF, Q to quit.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            msg = Int32()  # Use Int32 message type

            if key.lower() == "w":
                msg.data = 1  # Turn LED ON
            elif key.lower() == "s":
                msg.data = 0  # Turn LED OFF
            elif key.lower() == "q":
                self.get_logger().info("Exiting keyboard control.")
                break
            else:
                continue

            self.publisher.publish(msg)
            self.get_logger().info(f"Sent: {msg.data}")

def main():
    rclpy.init()
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
