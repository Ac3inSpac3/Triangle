#!/bin/bash

# Navigate to your ROS 2 workspace
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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 &
