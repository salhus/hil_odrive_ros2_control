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

```bash
ros2 run odrive_velocity_pid velocity_pid_node \
  --ros-args \
  -p kp:=2.0 \
  -p ki:=0.5 \
  -p kd:=0.05 \
  -p amplitude_rad_s:=1.5 \
  -p omega_rad_s:=0.5
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
| `amplitude_rad_s` | `double` | `1.0` | Amplitude of the sine reference in rad/s |
| `omega_rad_s` | `double` | `1.0` | Angular frequency of the sine reference in rad/s |
| `kp` | `double` | `1.0` | Proportional gain |
| `ki` | `double` | `0.0` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.0` | Feedforward gain — scales desired acceleration to produce anticipatory torque |
| `torque_limit_nm` | `double` | `10.0` | Output torque saturation limit (N·m) |
| `integral_limit` | `double` | `5.0` | Integral term clamp (rad/s · s) |
| `deadband_rad_s` | `double` | `0.0` | Error deadband — errors smaller than this are treated as zero |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |
| `filter_alpha` | `double` | `0.3` | Exponential moving average coefficient for velocity smoothing (0.0 = no filter, closer to 1.0 = heavier smoothing) |

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

## Safety Features

- **Torque saturation** – output is clamped to `[-torque_limit_nm, torque_limit_nm]`.
- **Integral clamp** – integral accumulator is clamped to `[-integral_limit, integral_limit]`.
- **Anti-windup** – integrator is frozen while the output is saturated.
- **Error deadband** – small velocity errors (< `deadband_rad_s`) are ignored to reduce chatter.

---

## `config/controllers.yaml`

Declares the `ros2_control` controllers used with this package:

| Controller | Type | Notes |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes joint feedback |
| `motor_effort_controller` | `effort_controllers/JointGroupEffortController` | Accepts effort commands for `motor_joint` |
| `controller_manager` | — | `update_rate: 200` Hz |
