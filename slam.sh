cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
echo "Configuring SLAM"
ros2 lifecycle set /slam_toolbox configure
echo "Activating SLAM"
ros2 lifecycle set /slam_toolbox activate