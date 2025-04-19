from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      # 1. LiDAR driver (adjust serial port as needed)
      Node(
        package='rplidar_ros',
        executable='rplidarNode',
        name='rplidar',
        parameters=[{'serial_port': '/dev/ttyUSB0',
                     'serial_baudrate': 115200}],
        output='screen'
      ),

      # 2. Static TFs
      Node(package='tf2_ros', executable='static_transform_publisher',
           arguments=['0.10','0.00','0.20','0','0','0','base_link','laser_frame']),
      Node(package='tf2_ros', executable='static_transform_publisher',
           arguments=['0','0','0','0','0','0','base_link','imu_link']),

      # 3. EKF for odom + IMU
      Node(package='robot_localization', executable='ekf_node',
           name='ekf_filter_node',
           parameters=['config/ekf.yaml'],

           # You can set use_sim_time if you ever bag playback—
           # but here we use real time
           output='screen'
      ),

      # 4. SLAM Toolbox (online async)
      Node(package='slam_toolbox', executable='async_slam_toolbox_node',
           name='slam_toolbox',
           parameters=['config/mapper_params_online_async.yaml'],
           remappings=[('scan','/scan'),
                       ('odom','/odometry/filtered')],
           output='screen'
      ),
    ])
