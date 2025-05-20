import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime
from math import atan2
import numpy as np

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        # Save logs to ~/ros2_ws/logs/
        ros_ws_path = os.path.expanduser('~/ros2_ws')
        logs_dir = os.path.join(ros_ws_path, 'testing_logs')
        os.makedirs(logs_dir, exist_ok=True)

        self.log_filename = f'odometry_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.log_path = os.path.join(logs_dir, self.log_filename)
        self.file = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'yaw_deg'])

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.writerow([f'{t:.3f}', f'{x:.3f}', f'{y:.3f}', f'{np.degrees(yaw):.2f}'])

    def quaternion_to_yaw(self, x, y, z, w):
        return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odometry logger...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
