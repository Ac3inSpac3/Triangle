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
    launch_dir = os.path.join(triangle_dir, 'launch')

    imu_params = os.path.join(config_dir, 'bno055_params_i2c.yaml')
    ekf_params = os.path.join(config_dir, 'ekf_params.yaml')

    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')
        )
    )

    return LaunchDescription([
        # 1) Micro‑ROS agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),

        # 2) BNO055 IMU Node
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            #output='screen',
            parameters=[imu_params],
        ),

        # 3) SLLIDAR A1 Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),

        

        # 5) Add Robot Localization Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # 4) Include SLAM launch file
        slam_launch,
        
        # Other nodes to be added here

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),

        Node(
            package='ps_ros2_common',
            executable='joy_test',
            name='joy_test',
        ),

        # Teleop node for enable/disable functionality
        Node(
            package='triangle',
            executable='teleop_joy_node',  # This matches the entry_point in setup.py
            name='triangle_teleop_joy',
            output='screen',  # Add output='screen' to see log messages for debugging
            parameters=[{
                'linear_speed_scale': 0.2,
                'angular_speed_scale': 0.75,
                'deadzone': 0.1,
            }]
        ),

        Node(
            package='triangle',
            executable='log_odometry',  # This matches the entry_point in setup.py
            name='log_odometry',
            output='screen',  # Add output='screen' to see log messages for debugging
        ),

        Node(
            package='triangle',
            executable='run_motion_test',  # This matches the entry_point in setup.py
            name='run_motion_test',
            output='screen',  # Add output='screen' to see log messages for debugging
        ),

        # 7) Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        ),
    ])
