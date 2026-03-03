# hil_odrive_ros2_control (ROS 2 Jazzy)

This repository is a self-contained ROS 2 Jazzy workspace repo for controlling an **ODrive** over **SocketCAN (CAN bus)** using **ros2_control**, plus a companion node that performs **velocity PID control** (tracking a sine-wave velocity reference) and outputs **torque/effort commands**.

It is designed so you can:

- clone this repo into a fresh ROS 2 Jazzy workspace (`~/ws/src`)
- `colcon build`
- launch `ros2_control_node` + controllers
- run the velocity PID node and command the motor

---

## What’s in this repo

### Packages

- **`hil_odrive_ros2_control`** (root package)
  - Purpose: installs the launch file, controller YAML, and a minimal URDF/Xacro for a single joint named `motor_joint`
  - Key paths:
    - `launch/motor_control.launch.py`
    - `config/controllers.yaml`
    - `description/urdf/motor.urdf.xacro`

- **`odrive_velocity_pid`** (package)
  - Purpose: reads measured velocity from `/joint_states`, tracks a sine-wave reference velocity, runs a PID loop, and publishes torque (effort) commands

### Vendored upstream code

Under `vendor/ros_odrive/` you have the minimal required parts of the upstream repository:

- `vendor/ros_odrive/odrive_ros2_control` (ros2_control hardware interface plugin)
- `vendor/ros_odrive/odrive_base` (base code used by the hardware plugin)

See `VENDORED.md` and `vendor/ros_odrive/LICENSE` for provenance and licensing.

---

## High-level architecture / data flow

### Runtime components

1. **`ros2_control_node`** (from `controller_manager`)
   - loads the ODrive hardware plugin declared in the URDF
   - exposes ros2_control state and command interfaces for `motor_joint`

2. **`joint_state_broadcaster`**
   - publishes `sensor_msgs/msg/JointState` on `/joint_states`
   - this provides measured velocity feedback used by the PID node

3. **`motor_effort_controller`**
   - accepts `std_msgs/msg/Float64` commands on `/motor_effort_controller/commands`
   - forwards them to the ros2_control effort command interface for `motor_joint`

4. **`odrive_velocity_pid/velocity_pid_node`**
   - subscribes to `/joint_states`
   - extracts measured velocity for `motor_joint`
   - generates desired velocity:  
     **v_des(t) = amplitude_rad_s · sin(omega_rad_s · t)**
   - computes PID on velocity error
   - publishes torque command as `std_msgs/msg/Float64` to `/motor_effort_controller/commands`

### Data flow summary

`/joint_states (measured velocity)` → **velocity PID node** → `/motor_effort_controller/commands (effort)` → **effort controller** → **ODrive ros2_control hardware plugin** → **CAN** → ODrive

---

## Prerequisites

### ROS 2 Jazzy
You must have ROS 2 Jazzy installed. Make sure you source it in every terminal before building/running:

```bash
source /opt/ros/jazzy/setup.bash
```

### ros2_control + controllers
You need the controller manager and controllers installed (commonly via `ros-jazzy-ros2-control` and `ros-jazzy-ros2-controllers`).

In particular, this repo expects an effort controller type:

- `effort_controllers/JointGroupEffortController`

If you see controller-type load failures, install the appropriate Jazzy packages and/or check available controller types (see troubleshooting).

### SocketCAN + can-utils
You need a Linux SocketCAN interface (e.g. `can0`). Helpful tools:

```bash
sudo apt-get install can-utils
```

---

## SocketCAN setup (example)

ODrive CAN bitrate depends on your configuration. Example with `500000` (500k):

```bash
# Bring interface down if it exists
sudo ip link set can0 down 2>/dev/null || true

# Configure bitrate and bring it up
sudo ip link set can0 up type can bitrate 500000

# Inspect interface state/details
ip -details link show can0
```

Verify traffic:

```bash
candump can0
```

If `candump` shows nothing, common causes include: wrong bitrate, wiring/termination issues, ODrive not transmitting, or wrong interface name.

---

## Configuration you MUST check (CAN + node_id)

The ODrive hardware plugin configuration is in:

- `description/urdf/motor.urdf.xacro`

Key parameters:

### CAN interface name
```xml
<param name="can">can0</param>
```

### ODrive CAN node ID mapped to `motor_joint`
```xml
<joint name="motor_joint">
  <param name="node_id">0</param>
</joint>
```

Make sure:
- your SocketCAN interface is actually `can0` (or change it)
- your ODrive node ID is actually `0` (or change it)

---

## Build (fresh workspace)

```bash
# Terminal
source /opt/ros/jazzy/setup.bash

mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/salhus/hil_odrive_ros2_control.git
cd ..

# Install dependencies
rosdep install --from-paths src -y --ignore-src

# Build
colcon build --symlink-install

# Source overlay
source install/setup.bash
```

### Confirm packages are discoverable by colcon
```bash
colcon list
```

You should see packages like:
- `hil_odrive_ros2_control`
- `odrive_velocity_pid`
- `odrive_ros2_control` (vendored)
- `odrive_base` (vendored)

---

## Run: bring up ros2_control + controllers

Launch the included single-motor setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash

ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

This launch file should:
- start `ros2_control_node`
- start `robot_state_publisher`
- spawn/activate:
  - `joint_state_broadcaster`
  - `motor_effort_controller`

---

## Verify: controllers, interfaces, feedback

### Check controllers
```bash
ros2 control list_controllers
```

You want (at minimum) both of these to show as `active`:
- `joint_state_broadcaster`
- `motor_effort_controller`

### Check hardware interfaces
```bash
ros2 control list_hardware_interfaces
```

Look for `motor_joint` interfaces. You should see an effort command interface and state interfaces (e.g., velocity).

### Check feedback stream
```bash
ros2 topic echo /joint_states --once
```

Confirm:
- `motor_joint` appears in `name: [...]`
- `velocity: [...]` has a sensible value (not NaN)

---

## Run: velocity PID node (sine velocity → torque)

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash

ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_name:=motor_joint \
  -p joint_state_topic:=/joint_states \
  -p command_topic:=/motor_effort_controller/commands \
  -p kp:=1.0 -p ki:=0.0 -p kd:=0.0 \
  -p amplitude_rad_s:=2.0 \
  -p omega_rad_s:=6.283185307 \
  -p torque_limit_nm:=1.0 \
  -p integral_limit:=5.0 \
  -p deadband_rad_s:=0.0 \
  -p rate_hz:=200.0
```

### PID node parameters (`odrive_velocity_pid`)
| Parameter | Type | Default | Description |
|---|---:|---:|---|
| `joint_state_topic` | string | `/joint_states` | JointState feedback topic |
| `command_topic` | string | `/motor_effort_controller/commands` | Effort command output topic (`std_msgs/Float64`) |
| `joint_name` | string | `motor_joint` | Joint name inside `/joint_states.name[]` |
| `amplitude_rad_s` | double | `1.0` | Sine velocity amplitude (rad/s) |
| `omega_rad_s` | double | `1.0` | Sine angular frequency (rad/s). For 1 Hz use `2*pi ≈ 6.283` |
| `kp` | double | `1.0` | Proportional gain |
| `ki` | double | `0.0` | Integral gain |
| `kd` | double | `0.0` | Derivative gain |
| `torque_limit_nm` | double | `10.0` | Output torque saturation limit |
| `integral_limit` | double | `5.0` | Integral accumulator clamp |
| `deadband_rad_s` | double | `0.0` | Error deadband on velocity error |
| `rate_hz` | double | `100.0` | Control-loop frequency |

### Practical tuning guidance
- Start with small `amplitude_rad_s` and small `torque_limit_nm`.
- Start with `ki=0`, `kd=0`. Increase `kp` until tracking is acceptable without oscillation.
- Add a small `kd` if you get oscillation/overshoot.
- Add `ki` last to remove steady-state error (watch for windup).

---

## Troubleshooting

### 1) CAN interface down / no traffic
Symptoms:
- No `/joint_states` updates (or they remain zero/NaN)
- `candump can0` shows nothing

Checks/fixes:
- `ip -details link show can0`
- confirm bitrate matches the bus
- check wiring and termination
- confirm the ODrive is powered and connected

### 2) Wrong ODrive node ID
Symptoms:
- CAN is up, but hardware plugin never gets valid feedback / joint stays NaN

Fix:
- Update `node_id` in `description/urdf/motor.urdf.xacro` to match the ODrive CAN node id

### 3) Controllers not active
Symptoms:
- PID publishes torque commands but motor doesn’t respond
- `/motor_effort_controller/commands` exists but effort interface not claimed

Checks:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Fix:
- make sure `motor_effort_controller` is `active` (spawner should do this)
- verify controller manager is running and reachable at `/controller_manager`

### 4) Effort controller type not available
Symptoms:
- controller fails to load with a type-not-found error

Checks:
```bash
ros2 control list_controller_types
```

Fix:
- install `ros-jazzy-ros2-controllers` (or equivalent)
- update `config/controllers.yaml` to use an effort controller type that exists on your system

### 5) No `motor_joint` in `/joint_states`
Symptoms:
- `/joint_states` publishes but `motor_joint` isn’t present

Fix:
- confirm the URDF joint name is exactly `motor_joint`
- confirm the controllers are spawned successfully
- check `ros2_control_node` logs for hardware/plugin load errors

### 6) ODrive not in a state that accepts torque commands
Symptoms:
- feedback exists but motor does not move/respond

Fix:
- ensure the ODrive is calibrated/configured for closed-loop control
- confirm it accepts CAN setpoints as expected for your ODrive firmware/config

---

## Licensing / vendored code

Vendored upstream code is under `vendor/ros_odrive/` and retains the upstream license:

- `vendor/ros_odrive/LICENSE`

Provenance and update notes are documented in:

- `VENDORED.md`
