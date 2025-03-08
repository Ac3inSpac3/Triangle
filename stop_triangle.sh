#!/bin/bash

echo "Current ROS 2 Processes..."
ps aux | grep [r]os2

# Confirm before stopping
read -p "Are you sure you want to stop all ROS 2 processes? (y/n): " confirm
if [[ "$confirm" != "y" ]]; then
    echo "Aborting..."
    exit 1
fi

echo "Stopping Foxglove Bridge..."
if pgrep -f foxglove_bridge > /dev/null; then
    pkill -f foxglove_bridge
    echo "Foxglove Bridge stopped."
else
    echo "Foxglove Bridge was not running."
fi

echo "Stopping Micro-ROS Agent..."
if pgrep -f micro_ros_agent > /dev/null; then
    pkill -f micro_ros_agent
    echo "Micro-ROS Agent stopped."
else
    echo "Micro-ROS Agent was not running."
fi

echo "Stopping All Remaining ROS 2 Processes..."
pkill -TERM -f ros2

# Wait for shutdown
sleep 2

# Check if any ROS 2 processes are still running
if pgrep -f ros2 > /dev/null; then
    echo "Some ROS 2 processes are still running. Forcing shutdown..."
    pkill -KILL -f ros2
fi

echo "Processes After Stopping..."
ps aux | grep [r]os2
