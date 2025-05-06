import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    triangle_dir = os.path.expanduser('~/ros2_ws/src/triangle')
    config_dir = os.path.join(triangle_dir, 'config')
    launch_dir = os.path.join(triangle_dir, 'launch')
    
    # Config files
    imu_params = os.path.join(config_dir, 'bno055_params_i2c.yaml')
    ekf_params = os.path.join(config_dir, 'ekf_params.yaml')
    
    # Map file path
    map_file = os.path.expanduser('~/maps/my_map.yaml')
    
    # Parameters for Nav2
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')
    
    # Robot description
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_state_publisher.launch.py')
        )
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    
    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Nav2 controllers, planners, and behaviors
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]}
        ]
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
                'angle_compensate': True,
                'min_distance': 0.2,  # Filter out readings closer than 20cm
                'max_distance': 8.0
            }]
        ),

        # 4) Robot State Publisher
        robot_state_publisher_cmd,

        # 5) Robot Localization Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # 6) Navigation2 Stack
        map_server,
        amcl,
        nav2_controller,
        nav2_planner,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,

        # 7) Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        ),
    ])