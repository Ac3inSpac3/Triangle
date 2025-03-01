# Triangle

This repository contains ROS2 scripts for controlling an ESP32 using micro-ROS. The ESP32 receives ROS2 messages to control an LED and sends button press events back to ROS2.

## Project Structure
```
ros2_ws/
│── build/    (Generated after colcon build)
│── install/  (Generated after colcon build)
│── src/      (Generated after colcon build)
│── log/      (Generated after colcon build)
├── scripts/
│   ├── keyboard_control.py   # Send keyboard commands to ESP32
├── ESP32_Code/
│   ├── main   # Main ESP32 code, current code is basic pub/sub
│── README.md  # This file
│── .gitignore # Files to be ignored when commiting to GitHub
```

## Tips and Tricks

### Working with GitHub through terminal on the Pi5
#### Pulling changes to the Pi
Before making any changes, it is best to pull changes from GitHub to the Pi to prevent errors:
```sh
git add .
git commit -m "Updated scripts and README"
git push origin main
```

#### Pushing Updates to GitHub
After making changes, commit and push your updates:
```sh
git add .
git commit -m "Updated scripts and README"
```

### Viewing ROS2 Topics via Termminal
For example, viewing messages on the topic 'micro_ros_arduino_node_publisher':
```sh
ros2 topic echo /micro_ros_arduino_node_publisher 
```

## Installation & Setup

### Initial Steps

Ensure Ubuntu 24.04 is installed on the Raspberry Pi 5 and configured for remote access via SSH or VNC.

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
sudo apt install ros-jazzy-desktop
```


9. Setup environment:
```sh
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```

10. Verify installation:
Open a terminal and run a C++ talker:
```sh
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal, run a Python listener:
```sh
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

### Setting up Triangle Project
1. Clone Triangle repository:
```sh
git clone https://github.com/Ac3inSpac3/Triangle.git ~/ros2_ws
cd ~/ros2_ws
```

2. Install dependencies:
```sh
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent -b humble
git clone https://github.com/micro-ROS/micro_ros_msgs -b humble

// More to be added as project progresses
```

3. Handling errors:
Any encountered errors and their resolutions will be documented here.
```sh
Errors TBD
```

4. Build the ROS2 Workspace:
```sh
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Motordriver microcontroller flashing
1. Flash ESP32
Flash ESP32 with `main.ino` file with Arduino IDE or preferred method, ESP cannot be flashed while micro_ros_agent is running

### Running robot in ROS2
Steps TBD as project progresses

1. Connect the motor driver microcontroller to the raspberry pi

2. Run the micro-ROS Agent
Connect your ESP32 and start the micro-ROS agent:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## License
This project is licensed under the MIT License.

## Troubleshooting
- TBD

