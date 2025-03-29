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
colcon build --symlink-install

# Source the workspace again after building
source install/setup.bash

# Launch the Foxglove Bridge
echo "Launching Foxglove Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

# Start the Micro-ROS Agent
# Ensure that /dev/ttyUSB0 is the correct device (e.g., the motor driver)
echo "Starting Micro-ROS Agent..."
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &

# Start the ROS2 Razor imu node
#echo "Starting Razor IMU..."
#ros2 launch ros2_razor_imu razor-pub.launch.py &
