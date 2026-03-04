# hil_odrive_ros2_control

A self-contained **ROS 2 Jazzy** workspace for controlling an **ODrive** motor over **SocketCAN (CAN bus)** using **ros2_control**. Includes a companion velocity PID node that tracks a sinusoidal velocity reference and outputs torque commands using a **feedforward + PID** architecture for smooth closed-loop velocity control.

---

## Repository structure

```
hil_odrive_ros2_control/
└── src/
    ├── hil_odrive_ros2_control/   # Launch files, controller YAML, URDF/Xacro for motor_joint
    ├── odrive_base/               # ODrive base library (from upstream odriverobotics/ros_odrive)
    ├── odrive_ros2_control/       # ODrive ros2_control hardware interface plugin (from upstream odriverobotics/ros_odrive)
    └── odrive_velocity_pid/       # Velocity PID + feedforward node
```

- **`src/hil_odrive_ros2_control/`** — launch file (`motor_control.launch.py`), controller YAML (`config/controllers.yaml`), and URDF/Xacro (`description/urdf/motor.urdf.xacro`) for `motor_joint`
- **`src/odrive_velocity_pid/`** — velocity PID + feedforward node that reads `/joint_states`, generates a sinusoidal velocity reference, and publishes torque commands
- **`src/odrive_base/`** and **`src/odrive_ros2_control/`** — ODrive `ros2_control` hardware interface plugin and its base library, sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository

---

## Prerequisites

- **ROS 2 Jazzy** — source `/opt/ros/jazzy/setup.bash` in every terminal
- **ros2_control + ros2_controllers** — `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`
- **SocketCAN + can-utils** — `sudo apt-get install can-utils`
- **ODrive** configured for CAN communication (correct node ID, CAN bitrate set to 250 kbps)

---

## CAN setup

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 250000
candump can0
```

> **Note:** bitrate must be **250000** — this matches the hardware configuration used here.

If `candump` shows no frames, check wiring/termination, verify the ODrive is powered, and confirm the bitrate matches what is configured in ODrive Tool.

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/hil_odrive_ros2_control   # root of this cloned repository
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package after making changes:

```bash
colcon build --packages-select odrive_velocity_pid
source install/setup.bash
```

---

## Launch the hardware stack

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

This starts `ros2_control_node`, `robot_state_publisher`, `joint_state_broadcaster`, and `motor_effort_controller`.

---

## Verify

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Both `joint_state_broadcaster` and `motor_effort_controller` should show as **active**. The `/joint_states` message should contain `motor_joint` with a valid (non-NaN) velocity.

---

## Run the velocity PID node

> **Tested working command** — copy and run this directly:

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p kp:=0.025 \
  -p ki:=0.02 \
  -p kd:=0.0 \
  -p kff:=1.0 \
  -p amplitude_rad_s:=0.5 \
  -p omega_rad_s:=3.14 \
  -p torque_limit_nm:=15.0 \
  -p filter_alpha:=0.85 \
  -p rate_hz:=50.0 \
  -p invert_output:=false
```

### Tuning philosophy

| Parameter | Value | Rationale |
|---|---|---|
| `kff` | `1.0` | **Feedforward does the heavy lifting** — provides smooth anticipatory torque proportional to desired acceleration |
| `kp` | `0.025` | Small proportional feedback corrects residual tracking errors without causing jitter |
| `ki` | `0.02` | Removes steady-state drift once `kff` and `kp` are stable |
| `kd` | `0.0` | Not needed — feedforward already handles the dynamics |
| `filter_alpha` | `0.85` | Heavy smoothing on velocity feedback reduces CAN noise (closer to 1.0 = more smoothing) |
| `rate_hz` | `50.0` | 50 Hz control loop |
| `torque_limit_nm` | `15.0` | Enough headroom for the motor; start lower and increase once stable |
| `invert_output` | `false` | Motor has normal sign convention — positive torque → positive velocity |

---

## Data flow

```
/joint_states → velocity_pid_node → /motor_effort_controller/commands → effort_controller → ODrive HW plugin → CAN → ODrive
```

---

## Full parameter reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | `string` | `/joint_states` | Topic for `sensor_msgs/JointState` feedback |
| `command_topic` | `string` | `/motor_effort_controller/commands` | Topic for `std_msgs/Float64MultiArray` torque output |
| `joint_name` | `string` | `motor_joint` | Joint name inside `JointState.name[]` |
| `amplitude_rad_s` | `double` | `1.0` | Sine velocity amplitude (rad/s) |
| `omega_rad_s` | `double` | `1.0` | Sine angular frequency (rad/s) — use `2π ≈ 6.283` for 1 Hz |
| `kp` | `double` | `1.0` | Proportional gain |
| `ki` | `double` | `0.0` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.0` | Feedforward gain — scales desired acceleration to produce anticipatory torque |
| `torque_limit_nm` | `double` | `10.0` | Output torque saturation limit (N·m) |
| `integral_limit` | `double` | `5.0` | Integral accumulator clamp (rad/s · s) |
| `deadband_rad_s` | `double` | `0.0` | Error deadband — velocity errors smaller than this are treated as zero |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |
| `filter_alpha` | `double` | `0.3` | Exponential moving average on velocity feedback (`0.0` = off, closer to `1.0` = heavier smoothing) |
| `invert_output` | `bool` | `false` | Negate the torque command — set `true` if positive torque produces negative velocity |

---

## Tuning tips

1. **Tune `kff` first** — it provides the bulk of the required torque; with `kff=1.0` the loop roughly pre-compensates for inertia.
2. **Keep `kp` small** — high `kp` combined with CAN latency causes oscillation. Start at `0.01`–`0.05`.
3. **Add `ki` last** — only after `kff` and `kp` are stable; small values (`0.01`–`0.05`) remove steady-state drift.
4. **`filter_alpha` closer to `1.0`** = heavier smoothing (reduces noise but adds phase lag; `0.85` is a good starting point).
5. **Start with low `torque_limit_nm`** for safety; increase once the loop is stable.
6. **Motor sign convention** — on this hardware: +0.5 Nm torque → +30 rad/s velocity, so `invert_output:=false`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `candump can0` shows nothing | CAN interface down or wrong bitrate | `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 250000` |
| "Failed to send CAN frame" errors | CAN bus error state | Reset the interface (command above), check wiring/termination |
| Motor runaway | PID gains too high or wrong sign | Kill the node, set ODrive to IDLE (`odrivetool` → `odrv0.axis0.requested_state = 1`), clear errors |
| Motor not moving | `torque_limit_nm` too low to overcome friction | Increase `torque_limit_nm` |
| `/joint_states` has NaN velocity | Wrong ODrive node ID in URDF | Update `node_id` in `description/urdf/motor.urdf.xacro` |
| Controller type not found | `ros-jazzy-ros2-controllers` not installed | `sudo apt-get install ros-jazzy-ros2-controllers` |

---

## Sub-package documentation

- [`src/hil_odrive_ros2_control/README.md`](src/hil_odrive_ros2_control/README.md) — hardware launch, URDF configuration, CAN node ID setup, and detailed controller bring-up steps
- [`src/odrive_velocity_pid/README.md`](src/odrive_velocity_pid/README.md) — velocity PID node build, run, and parameter reference

---

## Licensing

This repository's own code is original work. The `src/odrive_base/` and `src/odrive_ros2_control/` packages are sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository and retain its MIT license. Provenance details (upstream repository, commit SHA, and which packages were included) are documented in [`src/hil_odrive_ros2_control/VENDORED.md`](src/hil_odrive_ros2_control/VENDORED.md).
