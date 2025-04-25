import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import State

def generate_launch_description():
    triangle_dir = os.path.expanduser('~/ros2_ws/src/triangle')
    config_dir = os.path.join(triangle_dir, 'config')

    ekf_config = os.path.join(config_dir, 'ekf.yaml')
    slam_config = os.path.join(config_dir, 'mapper_params_online_async.yaml')
    return LaunchDescription([
        # 1) Micro‑ROS agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),

        # 2) BNO055 IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.expanduser('~/ros2_ws/src/triangle/launch/bno055.launch.py')
            )
        ),

        # 3) SLLidar A1
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #    os.path.expanduser('~/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py')
        #    )
        #),

        # 7) Foxglove Bridge
        Node(package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge'
        ),
    ])
