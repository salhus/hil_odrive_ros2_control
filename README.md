# hil_odrive_ros2_control

Standalone ROS 2 Jazzy workspace repo containing:
- `odrive_velocity_pid`: velocity PID node that tracks a sine velocity reference and outputs **torque** commands.
- vendored `odrive_ros2_control` + `odrive_base` from https://github.com/odriverobotics/ros_odrive

## Build (Jazzy)

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/salhus/hil_odrive_ros2_control.git
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Run (single motor)

### 1) Bring up ros2_control + ODrive hardware
Edit the ODrive settings in `description/urdf/motor.urdf.xacro`:
- CAN interface name (default `can0`)
- ODrive `node_id` (default `0`)

Then:

```bash
source ~/ws/install/setup.bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

### 2) Run the PID node
In another terminal:

```bash
source ~/ws/install/setup.bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_name:=motor_joint \
  -p kp:=1.0 -p ki:=0.0 -p kd:=0.0 \
  -p amplitude_rad_s:=2.0 \
  -p omega_rad_s:=6.283185307 \
  -p torque_limit_nm:=1.0 \
  -p rate_hz:=200.0
```

## Notes
- `odrive_velocity_pid` reads measured velocity from `/joint_states` and publishes torque to
  `/motor_effort_controller/commands`.
- Ensure SocketCAN is set up (e.g. `can0`) and your ODrive is configured/calibrated.
