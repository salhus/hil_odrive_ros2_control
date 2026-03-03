# Vendored Dependencies

## ros_odrive

**Upstream repository:** https://github.com/odriverobotics/ros_odrive  
**Upstream commit SHA:** `6386bf7871ec495c74f9a6d37de1fd6fa3bfdece`  
**License:** MIT (see `vendor/ros_odrive/LICENSE`)

### Directories copied

| Vendored path | Upstream path |
|---|---|
| `vendor/ros_odrive/odrive_base/` | `odrive_base/` |
| `vendor/ros_odrive/odrive_ros2_control/` | `odrive_ros2_control/` |

The following upstream packages were intentionally **not** vendored:
- `odrive_node/`
- `odrive_botwheel_explorer/`

### Why these packages

- **`odrive_ros2_control`** — the ros2_control hardware plugin that exposes ODrive axes as `hardware_interface::SystemInterface`.
- **`odrive_base`** — header-only + source library (no separate `package.xml`) used directly by `odrive_ros2_control` via relative CMake paths (`../odrive_base/include` and `../odrive_base/src/`).

### Build integration

Both vendored directories live inside this repository under `vendor/ros_odrive/`.  
Because this repository itself is placed under `src/` in a colcon workspace, colcon will
automatically discover the `odrive_ros2_control` package (it contains a `package.xml`).  
`odrive_base` is not a standalone ROS package; its sources are compiled directly by
`odrive_ros2_control/CMakeLists.txt` using the relative paths above.

### How to update from upstream

1. Identify the new upstream commit SHA on https://github.com/odriverobotics/ros_odrive.
2. Replace the contents of `vendor/ros_odrive/odrive_base/` and
   `vendor/ros_odrive/odrive_ros2_control/` with the new upstream versions.
3. Update the **Upstream commit SHA** field in this file.
4. Review any CMakeLists.txt path changes if the upstream directory layout changed.
