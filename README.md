# hil_odrive_ros2_control

ROS 2 Jazzy package for controlling ODrive motors via CAN using ros2_control.

## Architecture

```
/joint_states  (sensor_msgs/JointState)
      |
      v
 velocity_pid_node  (odrive_velocity_pid)
      |
      v  std_msgs/Float64
/motor_effort_controller/commands
      |
      v
 motor_effort_controller  (effort_controllers/JointGroupEffortController)
      |
      v  ros2_control hardware interface
 ODriveHardwareInterface  (odrive_ros2_control plugin)
      |
      v  SocketCAN
   can0  -->  ODrive axis (node_id)
```

The `velocity_pid_node` subscribes to `/joint_states`, computes a torque command
using a PID loop tracking a sinusoidal velocity reference, and publishes to the
effort controller's command topic. The `ODriveHardwareInterface` hardware plugin
forwards the effort command to the ODrive over the CAN bus and reports back
position and velocity as joint states.

## Repository Structure

```
hil_odrive_ros2_control/       # Root package: launch, config, URDF description
  config/controllers.yaml      # Controller manager and controller parameters
  description/urdf/motor.urdf.xacro  # Hardware description; set can and node_id here
  launch/motor_control.launch.py

odrive_velocity_pid/           # Velocity PID node package
  src/velocity_pid_node.cpp

vendor/ros_odrive/             # Vendored ODrive ros2_control packages (MIT)
  odrive_ros2_control/         # ros2_control hardware plugin
  odrive_base/                 # Header-only library used by the plugin
```

## Prerequisites

- **ROS 2 Jazzy** installed on Ubuntu 24.04 (`ros-jazzy-desktop` or `ros-jazzy-ros-base`)
- **ros2_control stack:**
  ```bash
  sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  ```
- **SocketCAN kernel modules** (included in mainline Linux; no extra installation needed on Ubuntu 24.04)
- **can-utils** (for verification):
  ```bash
  sudo apt install can-utils
  ```
- **xacro** and **robot_state_publisher:**
  ```bash
  sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher
  ```

## SocketCAN Setup

Bring up the CAN interface before launching any ROS nodes. Adjust the interface
name (`can0`) and bitrate to match your hardware.

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

Verify the interface is up:

```bash
ip link show can0
```

Verify that the ODrive is sending frames (run in a separate terminal while the
ODrive is powered on):

```bash
candump can0
```

You should see periodic heartbeat frames from the ODrive. If `candump` shows
nothing, check physical connections and the ODrive power state.

To bring the interface down:

```bash
sudo ip link set can0 down
```

## Workspace Setup

```bash
# 1. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Create a workspace
mkdir -p ~/ws/src
cd ~/ws/src

# 3. Clone the repository
git clone https://github.com/salhus/hil_odrive_ros2_control.git

# 4. Install ROS dependencies
cd ~/ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build

# 6. Source the install overlay
source ~/ws/install/setup.bash
```

## URDF Configuration

Before launching, configure `description/urdf/motor.urdf.xacro` to match your
hardware:

```xml
<ros2_control name="ODriveSystem" type="system">
  <hardware>
    <plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>
    <!-- CAN interface name as shown by `ip link` -->
    <param name="can">can0</param>
  </hardware>

  <joint name="motor_joint">
    <!-- ODrive node_id configured on the axis (0–63) -->
    <param name="node_id">0</param>
  </joint>
</ros2_control>
```

- `can`: the SocketCAN interface name (e.g., `can0`, `can1`).
- `node_id`: the CAN node ID configured on the ODrive axis. Each axis on the
  bus must have a unique node ID.

## Launching Motor Control

```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

This launch file starts:
- `ros2_control_node` — loads the hardware plugin and controller manager.
- `robot_state_publisher` — publishes TF transforms from the URDF.
- `joint_state_broadcaster` spawner — activates the broadcaster that publishes
  `/joint_states`.
- `motor_effort_controller` spawner — activates the effort controller that
  accepts torque commands.

An optional `controllers_file` argument allows overriding the default
`config/controllers.yaml`:

```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py \
  controllers_file:=/path/to/custom_controllers.yaml
```

## Verifying Controllers and Interfaces

After launching, confirm that all controllers are active:

```bash
ros2 control list_controllers
```

Expected output:

```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
motor_effort_controller[effort_controllers/JointGroupEffortController] active
```

List the available hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

Monitor joint states published by the broadcaster:

```bash
ros2 topic echo /joint_states
```

You should see `motor_joint` with `position`, `velocity`, and `effort` fields.

## Running the Velocity PID Node

The `velocity_pid_node` is part of the `odrive_velocity_pid` package. It tracks
a sinusoidal velocity reference using a PID controller and publishes torque
commands to the effort controller.

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_state_topic:=/joint_states \
  -p command_topic:=/motor_effort_controller/commands \
  -p joint_name:=motor_joint \
  -p kp:=2.0 \
  -p ki:=0.1 \
  -p kd:=0.05 \
  -p amplitude_rad_s:=3.0 \
  -p omega_rad_s:=1.0 \
  -p torque_limit_nm:=5.0 \
  -p rate_hz:=100.0
```

### Parameter Reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | string | `/joint_states` | Topic to subscribe for joint state feedback |
| `command_topic` | string | `/motor_effort_controller/commands` | Topic to publish torque commands |
| `joint_name` | string | `motor_joint` | Name of the joint to control (must match URDF) |
| `kp` | double | `1.0` | Proportional gain |
| `ki` | double | `0.0` | Integral gain |
| `kd` | double | `0.0` | Derivative gain |
| `amplitude_rad_s` | double | `1.0` | Amplitude of the sinusoidal velocity setpoint (rad/s) |
| `omega_rad_s` | double | `1.0` | Angular frequency of the sinusoidal setpoint (rad/s) |
| `torque_limit_nm` | double | `10.0` | Output torque clamp (Nm); applied symmetrically |
| `integral_limit` | double | `5.0` | Anti-windup clamp for the integral term |
| `deadband_rad_s` | double | `0.0` | Velocity error deadband (rad/s) |
| `rate_hz` | double | `100.0` | Control loop rate (Hz) |

## Troubleshooting

**No `/joint_states` messages**

- Verify that `joint_state_broadcaster` is in `active` state:
  ```bash
  ros2 control list_controllers
  ```
- Confirm the hardware plugin loaded without errors. Check `ros2_control_node`
  output for `[ERROR]` or `[WARN]` lines.
- Ensure the CAN interface is up (`ip link show can0`) and the ODrive is powered.

**Velocity or effort fields show `NaN`**

- The hardware plugin may not have received a valid frame from the ODrive yet.
  Run `candump can0` and confirm heartbeat frames are present.
- Check that `node_id` in the URDF matches the ODrive axis configuration.

**Controller not in `active` state**

- Re-run the spawner manually:
  ```bash
  ros2 run controller_manager spawner motor_effort_controller \
    --controller-manager /controller_manager
  ```
- Confirm `effort_controllers` is installed:
  ```bash
  sudo apt install ros-jazzy-ros2-controllers
  ```

**CAN interface is down**

- Bring it back up:
  ```bash
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
  ```
- Check kernel module: `lsmod | grep can_raw`. If missing:
  ```bash
  sudo modprobe can_raw
  sudo modprobe can
  ```

**Wrong `node_id`**

- Each ODrive axis has a configurable node ID (default `0`). Verify it in
  ODrive Tool or the ODrive firmware configuration, then update `node_id` in
  `description/urdf/motor.urdf.xacro` and rebuild.

**Missing `effort_controllers/JointGroupEffortController` type**

- Install the controllers package:
  ```bash
  sudo apt install ros-jazzy-ros2-controllers
  ```
- Re-source your workspace after installation:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/ws/install/setup.bash
  ```

## Vendored Code

The `vendor/ros_odrive/` directory contains a snapshot of two packages from
[odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive)
vendored at commit `6386bf7871ec495c74f9a6d37de1fd6fa3bfdece`:

- `odrive_ros2_control/` — the ros2_control hardware plugin.
- `odrive_base/` — header-only support library compiled directly by the plugin.

These packages are licensed under the MIT license. See
[`vendor/ros_odrive/LICENSE`](vendor/ros_odrive/LICENSE) for the full license
text and [`VENDORED.md`](VENDORED.md) for details on the vendored snapshot and
update instructions.
