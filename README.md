# README for hil_odrive_ros2_control

## Overview
This repository provides a ROS 2 package for controlling ODrive motors.

## Prerequisites
### SocketCAN Setup
To enable communication with the ODrive via CAN, ensure that SocketCAN is configured properly:
1. Connect your USB-CAN adapter.
2. Run the following commands to set up the CAN interface:
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set can0 up
   ```
3. Verify that your CAN interface is up:
   ```bash
   ifconfig can0
   ```

### rosdep
Make sure you have rosdep installed and initialized. Install dependencies for the package:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Workspace Setup
1. Create a workspace directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. Clone the repository:
   ```bash
   git clone https://github.com/salhus/hil_odrive_ros2_control.git
   ```
3. Install all dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Build Steps
1. Source the ROS 2 environment:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the local workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Repository Structure
- **root package:** Contains the main package which interfaces with ODrive.
- **odrive_velocity_pid:** A package for PID control of the ODrive motor.
- **vendor/ros_odrive:** Contains vendor-specific files and dependencies for the ODrive.

## Launch Motor Control
To launch the motor control:
```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

## Controllers Management
### Spawn/List Controllers
To spawn a controller, run:
```bash
ros2 control load_controller <controller_name>
```
To list all controllers:
```bash
ros2 control list_controllers
```

## Running velocity_pid_node
You can run the velocity PID node with the following command:
```bash
ros2 run hil_odrive_ros2_control velocity_pid_node --ros-args -p gains:={"p":1.0,"i":0.0,"d":0.01} -p command:={"velocity":1.0}
```

### Example Commands
- To set a specific velocity command:
   ```bash
   ros2 topic pub /odrive/velocity_cmd std_msgs/msg/Float64 "data: 2.0"
   ```

## Verifying the Setup
To verify the setup:
1. Check hardware interfaces:
   ```bash
   ros2 control list_hardware_interfaces
   ```
2. Monitor joint states:
   ```bash
   ros2 topic echo /joint_states
   ```

## Tuning PID
Adjust PID parameters according to the response:
1. Increase/decrease `p` to tune proportional control.
2. Tune `i` to reduce steady-state error.
3. Adjust `d` for damping.

## Common Troubleshooting
- **CAN interface down**: Ensure SocketCAN is properly configured and interface is up.
- **Wrong node_id**: Verify that the correct node ID is set for the ODrive device.
- **Controller not loaded**: Ensure that you're using the correct controller name.
- **Missing effort_controllers**: Ensure required packages are installed and sourced correctly.

## Conclusion
This README has been expanded to provide detailed instructions for setup and usage of the hil_odrive_ros2_control package. For further assistance, consult the ODrive documentation and ROS 2 resources.
