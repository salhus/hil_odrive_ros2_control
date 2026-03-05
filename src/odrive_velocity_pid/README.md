# odrive_velocity_pid

A ROS 2 Jazzy C++ package that implements a **velocity PID loop** for a single joint (`motor_joint` by default). It subscribes to `/joint_states`, tracks a sinusoidal reference velocity, and publishes a torque (effort) command.

---

## Prerequisites

- ROS 2 Jazzy desktop installed and sourced
- A working `ros2_control` stack with `effort_controllers` (for the hardware side)

---

## Build

```bash
# From your ROS 2 workspace root (e.g. ~/ros2_ws)
colcon build --packages-select odrive_velocity_pid
source install/setup.bash
```

---

## Run

The defaults are hardware-tuned — no parameters needed for basic operation:

```bash
ros2 run odrive_velocity_pid velocity_pid_node
```

Override specific parameters with `--ros-args -p key:=value`. For example:

```bash
ros2 run odrive_velocity_pid velocity_pid_node \
  --ros-args \
  -p amplitude_rad_s:=5.0 \
  -p torque_limit_nm:=0.3
```

To load the bundled controller configuration when using `ros2_control`:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active motor_effort_controller
```

Or pass the YAML file to the controller manager:

```bash
ros2 launch <your_hardware_launch> \
  controller_params_file:=$(ros2 pkg prefix odrive_velocity_pid)/share/odrive_velocity_pid/config/controllers.yaml
```

---

## ROS Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | `string` | `/joint_states` | Topic for `sensor_msgs/JointState` feedback |
| `command_topic` | `string` | `/motor_effort_controller/commands` | Topic for `std_msgs/Float64MultiArray` torque output |
| `joint_name` | `string` | `motor_joint` | Name of the joint inside `JointState.name[]` |
| `amplitude_rad_s` | `double` | `15.0` | Amplitude of the sine reference in rad/s |
| `omega_rad_s` | `double` | `3.14` | Angular frequency of the sine reference in rad/s |
| `kp` | `double` | `0.03` | Proportional gain |
| `ki` | `double` | `0.1` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.015` | Velocity feedforward gain — scales desired velocity to produce anticipatory torque (compensates viscous friction / back-EMF) |
| `kaff` | `double` | `0.003` | Acceleration feedforward gain — scales desired acceleration to produce anticipatory torque (compensates rotor inertia, `kaff ≈ J`) |
| `torque_limit_nm` | `double` | `0.5` | Output torque saturation limit (N·m) |
| `integral_limit` | `double` | `0.3` | Integral term clamp (rad/s · s) |
| `deadband_rad_s` | `double` | `0.0` | Error deadband — errors smaller than this are treated as zero |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |
| `filter_alpha` | `double` | `0.7` | Exponential moving average coefficient for velocity smoothing (0.0 = no filter, closer to 1.0 = heavier smoothing) |

---

## Hardware-tuned defaults

The default parameter values above were tuned on real hardware (ODrive motor over CAN bus) with 15+ second stable runs, zero saturation, and ±3-6 rad/s tracking error on a ±15 rad/s sine at ω=3.14 rad/s. They are safe to use out of the box for a similar motor setup.

Key results from hardware tuning:
- **`kff=0.015`**: motor reached ~2× desired speed with `kff=0.03`, so `0.015` gives correct steady-state torque
- **`kaff=0.003`**: matches rotor inertia `J ≈ 0.003 kg·m²` from `τ = J·α` at observed accelerations
- **`kp=0.03`**: higher values (>0.05) amplify CAN velocity noise (±5-10 rad/s) causing oscillation
- **`ki=0.1`**: removes steady-state drift; directional anti-windup prevents integrator lockup
- **`filter_alpha=0.7`**: `0.85` was too laggy (overshoot), `0.5` was too noisy (jitter)
- **`torque_limit_nm=0.5`**: motor trips (overcurrent) at ±1.0 Nm during fast reversals; `0.5` provides margin

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `command_topic` (default `/motor_effort_controller/commands`) | `std_msgs/Float64MultiArray` | Torque command sent to the effort controller |
| `~/desired_velocity` | `std_msgs/Float64` | Sine-wave reference velocity (rad/s) |
| `~/measured_velocity` | `std_msgs/Float64` | Filtered encoder feedback velocity (rad/s) |
| `~/velocity_error` | `std_msgs/Float64` | Tracking error: desired − measured (rad/s) |

The `~/` prefix scopes topics under the node name (e.g. `/velocity_pid_node/desired_velocity`). These topics are useful for plotting in tools like PlotJuggler or `rqt_plot`.

---

## Console Debug Logging

The control loop emits a throttled `INFO` log at ~10 Hz (100 ms interval) that summarises each PID iteration:

```
[PID] des=0.300 meas=0.012 err=0.288 out=0.029 sat=0
```

| Field | Description |
|---|---|
| `des` | Desired (reference) velocity in rad/s |
| `meas` | Measured (filtered) velocity in rad/s |
| `err` | Tracking error (desired − measured) in rad/s |
| `out` | PID output torque (after saturation) in N·m |
| `sat` | `1` if the output is currently saturated, `0` otherwise |

---

## Safety Features

- **Torque saturation** – output is clamped to `[-torque_limit_nm, torque_limit_nm]`.
- **Integral clamp** – integral accumulator is clamped to `[-integral_limit, integral_limit]`.
- **Anti-windup** – directional (conditional) anti-windup: when the output is saturated, integration is only allowed in the direction that *unwinds* the integrator (i.e. when the error has the opposite sign from the current integral). This prevents permanent integrator lockup by ensuring the integral can always recover when the error reverses, while still preventing further wind-up in the saturation direction.
- **Error deadband** – small velocity errors (< `deadband_rad_s`) are ignored to reduce chatter.

---

## `config/controllers.yaml`

Declares the `ros2_control` controllers used with this package:

| Controller | Type | Notes |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes joint feedback |
| `motor_effort_controller` | `effort_controllers/JointGroupEffortController` | Accepts effort commands for `motor_joint` |
| `controller_manager` | — | `update_rate: 200` Hz |
