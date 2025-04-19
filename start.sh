#!/bin/bash

# Navigate to base folder
cd

# Source the ROS 2 workspace
echo "Setup environment..."
source /opt/ros/jazzy/setup.bash

# Navigate to ROS 2 workspace
cd ~/ros2_ws || { echo "ros2_ws not found!"; exit 1; }

# Source the ROS 2 workspace
echo "Sourcing ROS 2 workspace..."
source install/setup.bash

# Build the workspace
echo "Building the workspace..."
#rm -rf build/ install/ log/
colcon build --symlink-install


# Source the workspace again after building
source install/setup.bash

# Launch the Foxglove Bridge
#echo "Launching Foxglove Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

# Start the Micro-ROS Agent
# Ensure that /dev/ttyUSB0 is the correct device (e.g., the motor driver)
echo "Starting Micro-ROS Agent..."
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &

# Start the ROS2 bno055 imu node
echo "Starting bno055 IMU..."
ros2 launch bno055 bno055.launch.py &

# Start the LiDAR
echo "Starting RPLiDAR..."
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py &

# Start the TF
echo "Starting Static transforms..."
python3 src/triangle/launch/static_tf.launch.py &