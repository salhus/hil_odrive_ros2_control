# colcon Workspace for ODrive ROS2 Control

## Build Instructions

1. Make sure you have installed the required dependencies for ROS2 and ODrive.
2. Create a workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
3. Clone this repository into the workspace:
   ```bash
   git clone https://github.com/salhus/hil_odrive_ros2_control.git
   ```
4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
5. Source the local setup file:
   ```bash
   source install/setup.bash
   ```
6. Launch the example:
   ```bash
   ros2 launch hil_odrive_ros2_control launch/motor_control.launch.py
   ```
