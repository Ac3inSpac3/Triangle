import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from datetime import datetime
import os
import csv

class CmdOdomLogger(Node):
    def __init__(self):
        super().__init__('cmd_odom_logger')
        self.enabled = False
        self.file = None
        self.writer = None
        self.latest_cmd = Twist()

        self.logs_dir = os.path.expanduser('~/ros2_ws/testing_logs')
        os.makedirs(self.logs_dir, exist_ok=True)

        self.create_subscription(Int32, '/enb_state', self.enable_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.get_logger().info('Cmd/Odom logger initialized. Waiting for /enb_state to start logging...')

    def enable_callback(self, msg):
        if msg.data == 1 and not self.enabled:
            self.enabled = True
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'cmd_vs_odom_log_{timestamp}.csv'
            self.log_path = os.path.join(self.logs_dir, filename)
            self.file = open(self.log_path, 'w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                'time',
                'cmd_linear_x', 'cmd_linear_y', 'cmd_angular_z',
                'odom_linear_x', 'odom_linear_y', 'odom_angular_z'
            ])
            self.get_logger().info(f"Logging enabled. Writing to {self.log_path}")

        elif msg.data == 0 and self.enabled:
            self.enabled = False
            if self.file:
                self.file.close()
                self.get_logger().info(f"Logging stopped. File saved to {self.log_path}")
                self.file = None
                self.writer = None

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def odom_callback(self, msg):
        if self.enabled and self.writer:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cmd = self.latest_cmd
            odom = msg.twist.twist

            self.writer.writerow([
                f'{t:.3f}',
                f'{cmd.linear.x:.3f}', f'{cmd.linear.y:.3f}', f'{cmd.angular.z:.3f}',
                f'{odom.linear.x:.3f}', f'{odom.linear.y:.3f}', f'{odom.angular.z:.3f}'
            ])

    def destroy_node(self):
        if self.enabled and self.file:
            self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logger interrupted. Closing any open files...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
