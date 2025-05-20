#!/bin/bash

# Navigate to workspace
cd ~/ros2_ws

# Source ROS 2 distro
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Build packages
echo "Building packages..."
colcon build --symlink-install

# Source the workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Display menu
echo "========================================"
echo "        TRIANGLE ROBOT LAUNCHER         "
echo "========================================"
echo "Please select an option:"
echo "1. Start Mapping Mode (SLAM)"
echo "2. Start Navigation Mode (with saved map)"
echo "3. Manual Control"
echo "4. Manual Control With Logging"
echo "5. Exit"
echo "----------------------------------------"

# Get user choice
read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        echo "Starting Mapping Mode..."
        # Configure and activate the SLAM node first
        ros2 launch triangle robot.launch.py &
        echo "Waiting for ODOM to stabilise"
        # Wait for SLAM to be available
        sleep 30
        # Configure and activate SLAM
        ros2 lifecycle set /slam_toolbox configure
        ros2 lifecycle set /slam_toolbox activate
        
        # Keep the script running until Ctrl+C
        echo "Robot mapping is active. Press Ctrl+C to stop."
        wait
        ;;
        
    2)
        echo "Starting Navigation Mode with saved map..."
        ros2 launch triangle nav.launch.py
        ;;

    3)
        echo "Starting Manual control"
        ros2 launch triangle manual_robot.launch.py
        ;;
        
    4)
        echo "Starting Manual control with logging"
        ros2 launch triangle manual_robot_logging.launch.py
        ;;

    5)
        echo "Exiting..."
        exit 0
        ;;
        
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac