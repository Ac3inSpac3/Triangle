cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
# source ROS 2 distro
source /opt/ros/jazzy/setup.bash
# build your new package
colcon build --symlink-install
# source your workspace
source install/setup.bash
# launch the entire stack
ros2 launch triangle robot.launch.py
