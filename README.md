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

## Installation & Setup

### Clone repository
```sh
git clone https://github.com/Ac3inSpac3/Triangle.git ~/ros2_ws
cd ~/ros2_ws
```

### Install dependencies
```sh
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent -b humble
git clone https://github.com/micro-ROS/micro_ros_msgs -b humble
```

### Deal with any errors here
```sh
idk what the errors are, TBD
```

### Build Your ROS2 Workspace
```sh
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run the micro-ROS Agent
Connect your ESP32 and start the micro-ROS agent:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Keyboard Control
The `keyboard_control.py` script allows you to send commands from your keyboard to control the ESP32's LED.

### Run the Script
```sh
python3 ~/ros2_ws/scripts/keyboard_control.py
```

### Controls
| Key  | Action       |
|------|------------|
| `W`  | Turn LED ON |
| `S`  | Turn LED OFF |
| `Q`  | Quit the script |

## Pushing Updates to GitHub
After making changes, commit and push your updates:
```sh
git add .
git commit -m "Updated scripts and README"
git push origin main
```

## License
This project is licensed under the MIT License.

## Troubleshooting
- TBD

