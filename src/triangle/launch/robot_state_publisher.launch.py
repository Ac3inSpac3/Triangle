import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    triangle_dir = os.path.expanduser('~/ros2_ws/src/triangle/description/')
    urdf_file_path = os.path.join(triangle_dir, 'urdf', 'Digital.urdf')
    
    # Read URDF from file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }],
        ),
        
        # Optional: Static transform publisher for any fixed transforms not in URDF
        # Replace with appropriate values for your robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
    ])