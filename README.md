# Simulation, ROS2 and Gazebo

![Screenshot](Sim_Models/screenshots/hexapod.png)

## Table of Contents
1. [Introduction](#introduction)  
2. [Features](#features)  
3. [Installation](#installation)  
4. [Usage](#usage)  
5. [Contributing](#contributing)  

## Introduction  
ROS2 Jazzy + Gazebo Harmonic simulation environment for a 6-legged hexapod robot. Features realistic physics, collision detection, and hardware integration capabilities.

## Features
- **18-DOF Hexapod Robot** - 6 legs with 3 joints each
- **Multiple Gait Patterns** - Wave, bipod, tripod, ripple gaits for different terrains
- **Contact Sensor Integration** - Foot contact detection for adaptive walking
- **IMU Stabilization** - Body orientation control based on inertial feedback
- **Dual Mode Operation** - Seamless switching between simulation and real hardware
- **Mobile App Control** - Wireless control via WebSocket interface
- **MoveIt Integration** - Advanced motion planning and trajectory control
- **Real-time Physics** - Gazebo simulation with accurate collision detection
- **ROS2 Native** - Full ros2_control integration with joint trajectory controllers
- **ESP32 Hardware Interface** - Direct serial communication with servo controllers

## Requirements  
1. Ubuntu 24.04 (recommended, but other Ubuntu 24 releases should also work).  
2. ROS2 Jazzy Jalisco  
   [Link to installation tutorial](https://docs.ros.org/en/jazzy/Installation.html)  
3. Gazebo Harmonic  
   [Link to installation tutorial](https://gazebosim.org/docs/harmonic/install/)  
4. MoveIt  
   [Link to installation tutorial](https://moveit.ai/install-moveit2/binary/)  

## Installation & Build
```bash
# Clone repository
git clone https://github.com/Siedmiu/Hexapod-ROS2-System.git
cd Hexapod/sim_and_real/ros2_ws_hex

# Build packages
colcon build

# Source workspace
source install/setup.bash
```
If the packages build correctly, you should be able to run all necessary launch files and programs.

## Usage

### Running Gazebo simulation
```bash
ros2 launch hex_gz gazebo.launch.py
```
This should open the Gazebo simulator with a world containing the robot model and an inclined plane.

### Launching controllers for hardware  
To run controllers on the real robot, you need to check which port the ESP32 microcontroller is connected to. You can view all active ports with:
```bash
ls /dev/tty{USB,ACM}*
```
The ESP32 should appear on one of these ports.

Launching hardware controllers:  
```bash
# If ESP32 is on USB0
ros2 launch hex_gz robot.launch.py port_name:=/dev/ttyUSB0

# If ESP32 is on ACM0
ros2 launch hex_gz robot.launch.py port_name:=/dev/ttyACM0
```

### Controlling the robot (through bash)  
After launching the controllers (either in simulation or on hardware), you can activate the robot by running one of the movement algorithms. General syntax looks like this:
```bash
ros2 run hex_gz [name_of_algorithm]
```

List of available executables:
```bash
# Basic movement algorithms
ros2 run hex_gz wave_gate_square.py
ros2 run hex_gz bi_gate.py
ros2 run hex_gz tri_gate.py
ros2 run hex_gz ripple_gate.py
ros2 run hex_gz rotation.py

# Algorithms using contact sensors
ros2 run hex_gz wave_gate_contact_sensors.py
ros2 run hex_gz bi_gate_contact_sensors.py
ros2 run hex_gz tri_gate_contact_sensor.py

# Other executables
ros2 run hex_gz hi.py
ros2 run hex_gz dance.py
ros2 run hex_gz dance1.py
ros2 run hex_gz move_body_based_on_imu.py
```

### Controlling the robot (through mobile application)  
You can control the robot via a mobile application, but first you need to configure it properly.

1. Connect the ROS system to the same Wi-Fi network as the mobile device.
2. Check the IP address of the ROS system using:
   ```bash
   ifconfig
   ```
3. Insert this address into the application.
4. Launch either the Gazebo simulation or the hardware controllers as described earlier.
5. In another terminal, run:
   ```bash
   ros2 run hex_gz ws_run.py
   ```

### Controlling the robot (via MoveIt motion planning GUI)  
The MoveIt GUI allows you to control individual joints, move individual robot legs to specific positions, or configure custom trajectory points.

First, launch the [hardware controllers](#launching-controllers-for-hardware) or [Gazebo simulation](#running-gazebo-simulation):
```bash
ros2 launch hexapod_moveit_config demo.launch.py
```

### Other functionalities  
Viewing the robot model in the ROS2 visualization tool, Rviz:
```bash
ros2 launch hex_gz rviz.launch.py
```

Viewing the collision model specifically:
```bash
ros2 launch hex_gz collision_view.launch.py
```

## Acknowledgments  
This project uses the ROS2 framework and follows the standard ROS package structure. The initial setup and configuration patterns were inspired by [ROBOTIS open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator), although the implementation has been completely redesigned for hexapod robotics, with original kinematics, hardware interfaces, and control algorithms.
