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

## Set up visual feedback
'''bash
source install/setup.bash
ros2 run plotjuggler plotjuggler
'''

This opens plot juggler where you can start streaming data. Drag and drop /velocity_pid_node/desired_velocity/data and velocity_pid_node/measured_velocity/data to get started.

'''bash
source install/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
'''
This opens rqt_reconfigure, where you can refresh the parameter list and select velocity_pid_node to access and change most pid parameters.

---

## Verify

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Both `joint_state_broadcaster` and `motor_effort_controller` should show as **active**. The `/joint_states` message should contain `motor_joint` with a valid (non-NaN) velocity.

---

## Run the velocity PID node

The defaults are hardware-tuned — no parameters needed for basic operation:

```bash
ros2 run odrive_velocity_pid velocity_pid_node
```

Override specific parameters with `--ros-args -p key:=value`. For example:

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args -p amplitude_rad_s:=5.0 -p torque_limit_nm:=0.3
```

### Tuning philosophy

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `kff` | `0.015` | Velocity feedforward — compensates viscous friction / back-EMF. Estimated from hardware: motor reaches ~2× desired speed with `kff=0.03`, so `0.015` gives correct steady-state torque |
| `kaff` | `0.003` | Acceleration feedforward — compensates rotor inertia (`kaff ≈ J ≈ 0.003 kg·m²`). Produces 0.14 Nm peak at `ω=3.14` |
| `kp` | `0.03` | Small proportional feedback for residual error correction. Higher values (>0.05) amplify CAN velocity noise (±5-10 rad/s) causing oscillation |
| `ki` | `0.1` | Integral term removes steady-state drift. Directional anti-windup prevents lockup during saturation |
| `kd` | `0.0` | Not needed — feedforward handles dynamics, and CAN noise makes derivative unreliable |
| `filter_alpha` | `0.7` | 70% old + 30% new. Balances noise rejection vs. phase lag. `0.85` was too laggy, `0.5` too noisy |
| `rate_hz` | `100.0` | 100 Hz control loop — doubled from 50 Hz for tighter tracking (200 samples per sine cycle) |
| `torque_limit_nm` | `0.5` | Safe torque limit — motor trips at ±1.0 Nm during fast reversals. `0.5` provides margin |
| `integral_limit` | `0.3` | Caps integrator contribution to `ki × 0.3 = 0.03 Nm` max |
| `invert_output` | `false` | Normal sign convention: positive torque → positive velocity |

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
| `amplitude_rad_s` | `double` | `15.0` | Sine velocity amplitude (rad/s) |
| `omega_rad_s` | `double` | `3.14` | Sine angular frequency (rad/s) — use `2π ≈ 6.283` for 1 Hz |
| `kp` | `double` | `0.03` | Proportional gain |
| `ki` | `double` | `0.1` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.015` | Velocity feedforward gain — scales desired velocity to produce anticipatory torque (compensates viscous friction / back-EMF) |
| `kaff` | `double` | `0.003` | Acceleration feedforward gain — scales desired acceleration to produce anticipatory torque (compensates rotor inertia, `kaff ≈ J`) |
| `torque_limit_nm` | `double` | `0.5` | Output torque saturation limit (N·m) |
| `integral_limit` | `double` | `0.3` | Integral accumulator clamp (rad/s · s) |
| `deadband_rad_s` | `double` | `0.0` | Error deadband — velocity errors smaller than this are treated as zero |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |
| `filter_alpha` | `double` | `0.7` | Exponential moving average on velocity feedback (`0.0` = off, closer to `1.0` = heavier smoothing) |
| `invert_output` | `bool` | `false` | Negate the torque command — set `true` if positive torque produces negative velocity |

---

## Tuning tips

1. **Start with low `torque_limit_nm`** (0.3-0.5 Nm) and low `omega_rad_s` (0.5-1.0) for safety
2. **Tune `kff` first** — it provides the bulk of the required torque without noise amplification. If the motor overshoots to 2× desired speed, halve `kff`
3. **Add `kaff` for inertia compensation** — set `kaff ≈ J` (rotor inertia in kg·m²). Start with `0.003`
4. **Keep `kp` small** (0.01-0.05) — CAN velocity noise (±5-10 rad/s) gets amplified by `kp`, causing oscillation at higher values
5. **Add `ki` last** — small values (0.05-0.2) remove steady-state drift. The directional anti-windup prevents integrator lockup
6. **`filter_alpha=0.7`** is a good starting point — `0.5` lets too much CAN noise through, `0.85` adds too much phase lag
7. **Increase `rate_hz` to 100** — 100 Hz gave noticeably tighter tracking than 50 Hz on this hardware
8. **Motor sign convention** — on this hardware: +0.5 Nm torque → +30 rad/s velocity, so `invert_output:=false`

---

## Motor characterization (from hardware tuning)

| Property | Estimated value | How determined |
|----------|----------------|----------------|
| Viscous friction coefficient | ~0.015 Nm/(rad/s) | `kff` that gives best tracking without overshoot |
| Rotor inertia J | ~0.003 kg·m² | `kaff` value; matches `τ = J·α` at observed accelerations |
| Max safe torque reversal rate | ~0.5 Nm | Motor trips (overcurrent) at ±1.0 Nm during fast sinusoidal reversals |
| CAN velocity noise floor | ±5-10 rad/s | Observed measurement scatter at zero command |
| Usable `kp` range | 0.01-0.05 | Higher values amplify CAN noise into oscillation |
| Optimal filter alpha | 0.7 | `0.85` too laggy (overshoots), `0.5` too noisy (jitters) |

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
