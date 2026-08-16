# piper_hardware_interface

`ros2_control` hardware plugin package for the Agilex Piper arm.

This package is intentionally hardware-focused:
- hardware plugin library and plugin XML
- `ros2_control` `SystemInterface` implementations for the arm and optional gripper
- hardware-parameter parsing and command-mode switching

Robot model assets, controller YAML, and bringup launch files should live in
separate packages such as `piper_description` and `piper_bringup`.

## Plugin

| Item | Value |
|------|--------|
| Pluginlib type | `piper_hardware_interface/PiperHardwareInterface` |
| URDF `<hardware>` parameters | See below |

## Hardware Parameters

These are `ros2_control` hardware parameters, declared in the robot
description under:

```xml
<ros2_control>
  <hardware>
    <param name="...">...</param>
  </hardware>
</ros2_control>
```

They are read by `PiperHardwareInterface::on_init()` from
`hardware_interface::HardwareInfo::hardware_parameters`. They are not
controller YAML parameters.

| Name | Required | Default | Description |
|------|----------|---------|-------------|
| `can_interface` | yes | - | SocketCAN interface used by the arm, for example `piper0`. |
| `mit_kd_effort_damping` | no | `0.0` | MIT effort-mode velocity damping. `0.0` keeps the ROS effort interface torque-transparent; `0.05` to `0.3` can be used as optional bottom-level safety damping; higher values add hidden damping and affect impedance tuning. |

SocketCAN link setup is intentionally outside both hardware plugins. The named
interface must already exist and be UP before controller manager configures the
hardware. This keeps bus bitrate, restart policy, and shared-bus ownership in
the host deployment layer.

The xacro entry point is `piper_description/urdf/parts/piper_arm.ros2_control.xacro`.
Launch files should pass hardware-specific values through robot-description
xacro arguments, following the same pattern used by Franka's `robot_ip` and
interface-version parameters.

For a no-gripper robot, keep the arm hardware block enabled and omit the
`PiperGripperInterface` block in the robot description.

The independent `PiperGripperInterface` component accepts:

| Name | Required | Default | Description |
|------|----------|---------|-------------|
| `can_interface` | yes | - | SocketCAN interface shared with the matching Piper arm. |
| `home_on_activate` | no | `true` | Run the blocking one-shot homing operation during lifecycle activation. |

During controller updates the plugin uses forward-position semantics: unchanged
targets are not resent, and every changed target is passed once to
`Gripper::commandWidth()` with the SDK-compatible 1 N clamp-force default. The
ROS joint is one finger's prismatic travel; libpiper uses full opening width, so
the hardware boundary converts `opening_width = 2 * gripper_joint1`.

## Dependencies

This package depends on `libpiper`, which is maintained separately and is not a
ROS package. In the pixi/robostack workflow, install it from:

```text
https://prefix.dev/channels/gabriel-robotics/packages/libpiper
```

The hardware interface requires libpiper `0.5.x`. The arm and gripper plugins
use non-blocking `readLatest()` feedback snapshots; the gripper additionally
uses `commandWidth()` for forward-position streaming without waiting for target
arrival. Feedback freshness uses an internal 100 ms hardware safety constant.

The arm plugin selects libpiper scheduling from the host kernel: it enforces
SCHED_FIFO priority 80 when PREEMPT_RT is detected and leaves scheduling
unchanged on a non-RT kernel. This is not exposed as a hardware parameter. On
an RT host, the process must have permission to use realtime scheduling or
starting an arm control mode fails explicitly. libpiper's realtime setting does
not change SocketCAN RX or gripper/teaching-pendant scheduling.

The package is expected to provide a CMake package config and target:

```cmake
find_package(libpiper CONFIG REQUIRED)
target_link_libraries(<target> libpiper::libpiper)
```

This is sufficient for local pixi/conda-based builds. Publishing through the
ROS build farm would require a rosdep key or a ROS vendor package for
`libpiper`.

## Build

```bash
colcon build --packages-select piper_description piper_hardware_interface --symlink-install
```

## Known Issues / Design Notes

`piper_description/config/joint_limits.yaml` expresses one-finger travel, not
the SDK's full opening width. For example, a `0.04 m` upper joint limit
represents an approximately `0.08 m` opening. Any future travel calibration
must preserve this factor-of-two boundary.
