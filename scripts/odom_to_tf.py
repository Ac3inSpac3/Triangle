#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"  # usually "odom"
        t.child_frame_id = "odom"    # usually "base_link"

        # Position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Orientation
        t.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
