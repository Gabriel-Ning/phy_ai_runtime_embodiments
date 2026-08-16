# phy_ai_runtime_embodiments

Source ROS 2 Jazzy packages for the robot and end-effector embodiments used by
Physical AI Runtime.

## Components

- `robots/marvin/marvin_description`
- `robots/marvin/marvin_hardware_interface`
- `robots/piper/piper_description`
- `robots/piper/piper_hardware_interface`
- `end_effectors/pika_gripper/pika_gripper_description`
- `end_effectors/pika_gripper/pika_gripper_hardware_interface`
- `end_effectors/pika_gripper/pika_gripper_bringup`
- `end_effectors/pika_gripper/pika_sensor_description`

Camera drivers, application orchestration, manipulation controllers, and RT
profile bringup packages are intentionally outside this repository.

## Build

Place this repository in a ROS 2 workspace and build the required packages:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Hardware plugins depend on separately packaged vendor-facing libraries:

- Marvin: `libmarvin`
- Piper: `libpiper`

SocketCAN setup, serial-device permissions, controller configuration, and
controller-manager scheduling belong to the host deployment/RT bringup layer.

## Validation status

The Marvin, Piper, and Pika hardware paths have been exercised on their real
devices. The source release additionally validates xacro/config contracts,
Release builds, fake-hardware composition, and available offline unit tests.

## License

Repository-authored code is generally Apache-2.0. Individual packages and
assets may carry additional licenses; package metadata, per-file SPDX headers,
and nested license/source-attribution files are authoritative.
