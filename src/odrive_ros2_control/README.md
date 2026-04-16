# odrive_ros2_control

> **Vendored package** — sourced from the upstream
> [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository.
> See [`../hil_odrive_ros2_control/VENDORED.md`](../hil_odrive_ros2_control/VENDORED.md) for
> the specific commit SHA and provenance details.

This package provides the **ODrive `ros2_control` hardware interface plugin**. It is loaded by
`ros2_control_node` at runtime and exposes position, velocity, and effort state/command interfaces
for each ODrive axis declared in the URDF.

## Role in this project

The plugin is declared in `src/hil_odrive_ros2_control/description/urdf/motor.urdf.xacro` and
bridges the `ros2_control` framework to the ODrive hardware over SocketCAN:

```
ros2_control_node
  └── ODriveHardwareInterface (this plugin)
        ├── motor_joint (axis0, node_id=0)  ← effort command interface
        └── pto_joint   (axis1, node_id=1)  ← effort command interface
```

The `joint_state_broadcaster` reads position and velocity state interfaces from this plugin and
publishes them on `/joint_states`, which the `velocity_pid_node` subscribes to.

## URDF configuration

Each joint in the URDF specifies:

```xml
<joint name="motor_joint">
  <param name="node_id">0</param>   <!-- ODrive CAN node ID -->
</joint>
```

The CAN interface name is set at the hardware level:

```xml
<hardware>
  <plugin>odrive_ros2_control/ODriveHardwareInterface</plugin>
  <param name="can">can0</param>
</hardware>
```

## License

MIT — same as the upstream `odriverobotics/ros_odrive` repository.
