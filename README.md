# Triangle
**T**hree-wheeled **R**obotics platform for **I**ntelligent, **A**utonomous **N**avigation, **G**uidance and **L**ocalisation in unknown **E**nvironments

## Project Summary
This repository supports the base platform for a three-wheeled holonomic drive robot developed in EGH400 at QUT, originally designed by Josh Roe. The platform is designed to enable dynamic vision and LiDAR based navigation for bimanual manipulation tasks, focusing on real-time path planning and obstacle avoidance in complex environments.

The project addresses limitations in traditional mobile robots by integrating an RGB-D camera and LiDAR for enhanced perception and navigation. The holonomic drive system allows for omnidirectional movement, providing greater flexibility in confined spaces.

## Key Features:
- **Modular Base Platform:** Designed for expansion and upgrades, using carbon fiber rods and 3D-printed components.
- **ESP32 Motor Control:** Processes ROS2 movement commands into motor signals using a custom control library.
- **Raspberry Pi 5 Compute Module:** Runs ROS2 for SLAM and high-level navigation tasks.
- **Custom Motor Driver PCB:** Integrates power protection features (TVS diodes, capacitors, fuses) and improves motor control.
- **Sensor Suite:** Uses LiDAR, RGB-D camera, and IMU for real-time mapping, obstacle detection, and SLAM.
- **ROS2 Integration:** Designed for autonomous navigation, with real-time obstacle avoidance and remote control capabilities.

## Project Goal
The Triangle Robot Platform is designed to be a modular and adaptable base for research in autonomous navigation and robotic manipulation. It integrates a holonomic drive system, vision-based perception, and ROS2 compatibility to enable real-time path planning, obstacle avoidance, and scalable robotic applications.

## Table of Contents
- [Project Structure](#project-structure)
- [Installation & Setup](#installation--setup)
  - [Initial Steps](#initial-steps)
  - [Installing ROS2 Jazzy](#installing-ros2-jazzy)
  - [Setting up Triangle Project](#setting-up-triangle-project)
  - [Motor Driver Microcontroller Flashing](#motor-driver-microcontroller-flashing)
  - [Running Robot in ROS2](#running-robot-in-ros2)
- [Tips and Tricks](#tips-and-tricks)
  - [Working with GitHub through Terminal on the Pi5](#working-with-github-through-terminal-on-the-pi5)
  - [General ROS2 Practices](#general-ros2-practices)
- [License](#license)
- [Troubleshooting](#troubleshooting)

## Project Structure
```
ros2_ws/
│── build/          (Generated after colcon build)
│── install/        (Generated after colcon build)
│── src/            (Generated after colcon build)
│── log/            (Generated after colcon build)
├── scripts/        # Stand alone scripts made for TRIANGLE
│   ├── example.py  # Send keyboard commands to ESP32
├── ESP32_Code/     # Code developed for the ESP32 Motor driver
│   ├── main        # Main ESP32 code, current code is basic pub/sub
│── README.md       # This file
│── .gitignore      # Files to be ignored when commiting to GitHub
```

## Features being worked on

- [ ] Remote control / remote monitoring (Game controller or Web server)
- [ ] LiDAR integration
- [ ] Autonomous navigation
- [ ] Kinect 360 integration
- [ ] Advanced navigation

## Installation & Setup

### Initial Steps

Ensure Ubuntu 24.04 server is installed on the Raspberry Pi 5 and configured for remote access via SSH.

### Installing ROS2 Jazzy

Follow the official [ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) or use the steps below.

1. Set locale:
```sh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

2. Enable required repositories:
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

3. Add ROS2 GPG key:
```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. Add ROS2 repository to sources list:
```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. Install development tools (optional):
```sh
sudo apt update && sudo apt install ros-dev-tools
```

6. Update repo caches
```sh
sudo apt update
```

7. Upgrade system before installing new packages:
```sh
sudo apt upgrade
```

8. Install ROS2 Jazzy Desktop:
```sh
sudo apt install ros-jazzy-ros-base
```


9. Setup environment:
```sh
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```

10. Verify installation:
TBD

### Setting up Triangle Project
1. Clone Triangle repository:
```sh
git clone https://github.com/Ac3inSpac3/Triangle.git ~/ros2_ws
cd ~/ros2_ws
```

2. Install required packages:
```sh
mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent -b jazzy
git clone https://github.com/micro-ROS/micro_ros_msgs -b jazzy

// More to be added as project progresses
```

3. Installing dependencies:
https://wiki.ros.org/rosdep
```sh
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the ROS2 Workspace:
```sh
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Motor Driver microcontroller flashing
1. Flash ESP32
Flash ESP32 with Arduino IDE or preferred method, ESP cannot be flashed while micro_ros_agent is running

### Running robot in ROS2
Steps TBD as project progresses

1. Connect the motor driver microcontroller to the raspberry pi

2. Run the micro-ROS Agent
Connect your ESP32 and start the micro-ROS agent:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Tips and Tricks

### Working with GitHub through Terminal on the Pi5

#### Understanding `git pull` vs `git fetch`
- Use `git fetch` when you want to check for updates without modifying your local files. This is useful if you want to review changes before merging.
- Use `git pull` when you are ready to update your local branch with the latest changes from the remote repository.

**When to use `git fetch`**
```sh
git fetch origin  # Retrieve changes from remote
git status  # Check what has changed before merging
git merge origin/main  # Merge if needed
```

**When to use `git pull`**
```sh
git pull origin main  # Fetch and automatically merge changes
```

#### Switching Between Branches
```sh
git checkout -b feature-branch  # Create and switch to a new branch
git checkout main  # Switch back to the main branch
git merge feature-branch  # Merge feature-branch into main
```

#### Handling Merge Conflicts
```sh
git add .  # Stage resolved files
git commit -m "Resolved merge conflict"
git push origin main  # Push resolved changes
```

#### Resetting Unwanted Changes
```sh
git checkout -- filename  # Discard changes in a file
git reset --hard HEAD  # Discard all uncommitted changes
```

#### Stashing Temporary Changes
```sh
git stash  # Save uncommitted changes
git checkout main  # Switch branches
git stash pop  # Restore saved changes
```

#### Viewing Log History
```sh
git log --oneline --graph --all --decorate  # View commit history
```

### General ROS2 Practices

#### Sourcing the Environment
Every time you build or modify your ROS2 workspace, you need to source the setup file:
```sh
source install/setup.bash  # Source after building
```

#### Building the Workspace
After adding new packages or modifying code, always rebuild your workspace:
```sh
colcon build --symlink-install
source install/setup.bash
```

#### Checking Available ROS2 Nodes and Topics
To list all running nodes:
```sh
ros2 node list
```
To check active topics:
```sh
ros2 topic list
```
To see messages from a topic:
```sh
ros2 topic echo /your_topic_name
```

#### Running ROS2 Packages
To launch a ROS2 package with a launch file:
```sh
ros2 launch package_name launch_file.py
```
To run a single node:
```sh
ros2 run package_name node_name
```
To run a single file:
```sh
python filename.py
```

#### Manually send ROS2 messages through terminal
##### Sending a Twist message to control a robot:
```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" --once
```
Remove the `--once` flag to send on loop
##### Sending an std_msgs/String message
```sh
ros2 topic pub /status std_msgs/msg/String "{data: 'Robot is active'}" -r 0.5
```
`-r 0.5` sends messages on a loop at 0.5Hz, Ctrl+C to stop

## Troubleshooting
- TBD
