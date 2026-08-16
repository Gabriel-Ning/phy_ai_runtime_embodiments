# marvin_hardware_interface

ros2_control `SystemInterface` plugin for the Marvin CCS M6 bimanual arm (14 DOF).

## Overview

Provides real-time joint state reading and position-command dispatch to the
Marvin controller via the TJ/FX vendor SDK (`libMarvinSDK` / `libmarvin`).

```text
ros2_control lifecycle
  → marvin_sdk_bridge (C++ wrappers)
    → libMarvinSDK C API
      → UDP @ ~1 kHz → Controller
```

The SDK runs its own internal 1 kHz timer thread for UDP I/O. `OnGetBuf`
(read) is a non-blocking memcpy of the latest frame and `OnSetSend` (write)
just stages a command for that thread — both are microsecond-level, so the
achievable `controller_manager` update rate is bounded by Linux scheduling,
not by SDK I/O.

## Requirements

- ROS 2 Jazzy (`hardware_interface`, `rclcpp`, `rclcpp_lifecycle`, `pluginlib`)
- `libmarvin >= 0.1.0` — packaged Marvin CCS SDK
- `CMPL_LIN` compile definition for Linux platform types

## Build

```bash
colcon build --symlink-install --packages-select marvin_hardware_interface
```

Requires `libmarvin::libmarvin` CMake target (conda package).

## Test

```bash
colcon test --packages-select marvin_hardware_interface
```

32 offline tests covering:

- SDK bridge utilities (deg/rad, IP validation, joint limits, canonical names)
- Write-path guards (initial/per-cycle delta, NaN/Inf, joint limits, per-arm scoping)
- SDK transaction logic (bimanual dispatch, partial-send rejection, single-arm isolation)

## ros2_control Contract

|          | Count | Interfaces                            |
|----------|-------|---------------------------------------|
| Command  | 14    | `position`                            |
| State    | 42    | `position`, `velocity`, `effort`      |

The hardware contract exposes only the command interface implemented and
validated on the robot. Velocity and effort remain state-only.

Joint naming: `Joint1_L..Joint7_L`, `Joint1_R..Joint7_R`.

Per-arm command dispatch: left arm claims 7 position interfaces → `OnSetJointCmdPos_A`;
right arm → `OnSetJointCmdPos_B`; both → A and B in one `OnClearSet/OnSetSend` cycle.

## Safety Guards

Safety thresholds are **time-based**, so their physical meaning does not change
when `update_rate` changes (all configurable via URDF `hardware_parameters`):

| Guard                                     | Threshold                                                              | Param                                                   |
|-------------------------------------------|------------------------------------------------------------------------|---------------------------------------------------------|
| First-write delta (vs activation baseline)| 0.01 rad                                                               | —                                                       |
| Per-cycle step                            | `max_joint_velocity × period`, backstop 0.2 rad                        | `max_joint_velocity` (default 6.2832 rad/s = 360°/s)   |
| Joint limits                              | ±170° (J1/J3/J5), ±120° (J2), -145°/+60° (J4), ±60° (J6), ±90° (J7) | —                                                       |
| Stale frame (per-arm)                     | warn 20 ms / error 100 ms of no fresh serial                           | `stale_warn_ms`, `stale_error_ms`                       |
| Arm error                                 | rejects write if `m_ERRCode != 0`                                      | —                                                       |

Position-command dispatch is encapsulated in `marvin_sdk_bridge::dispatch_position_commands()` —
staging failures discard the transaction without partial send.

## Mode Switching

- Arm position interfaces transition `inactive → active` → enters SDK position state 1
- `active → inactive` → exits to idle state 0, verified via `wait_for_arm_state(ARM_STATE_IDLE)`
- Bimanual stop uses single `exit_position_modes(true, true)` transaction
- Bimanual start: if one arm fails after the other succeeded, the activated arm is rolled back

## Integration Boundary

This package exports only the ros2_control plugin and SDK bridge. The canonical
ros2_control xacro lives in `marvin_description`; controller configuration,
launch files, GUI sources, and real-hardware diagnostics live in
`marvin_manipulation_controller_bringup`.

## Performance / Real-Time

The SDK ceiling is **~1 kHz (UDP)**; the practical `update_rate` is set by
Linux scheduling. Key rules:

- `update_rate` **must be <= the SDK frame rate**. **500 Hz** target,
  **200 Hz** stock-kernel fallback.
- To reach 500 Hz: PREEMPT_RT kernel + `isolcpus` + pin the CM thread to an
  isolated core (`cpu_affinity`) so it stops contending with the SDK's 1 kHz
  thread. Raising priority *without* core separation makes it worse.

The workspace `marvin_manipulation_controller_bringup` package owns the
frame-rate probe and real-time deployment runbook.

## License

Apache-2.0
