# phy_ai_runtime_embodiments

Owned robot and Pika gripper packages for Physical AI Runtime,
plus pinned Franka vendor checkouts. Each entry is a git submodule
over HTTPS.

```text
robots/marvin/marvin_description          # branch dev (site-calibrated)
robots/marvin/marvin_hardware_interface   # branch main
robots/piper/piper_description            # branch main
robots/piper/piper_hardware_interface     # branch main
robots/franka/franka_ros2                 # frankarobotics/franka_ros2, branch jazzy
robots/franka/franka_description          # frankarobotics/franka_description, tag 2.8.1
end_effectors/pika_gripper/pika_gripper_description
end_effectors/pika_gripper/pika_gripper_hardware_interface
```

Franka pins the versions used by Physical AI Runtime (`franka_ros2`
`73a1501` / 3.4.1, `franka_description` `2.8.1`). Do not fast-forward
`franka_ros2` to latest `jazzy` without an explicit upgrade.

Hikvision, `pika_gripper_bringup`, and `pika_sensor_description`
are not in this repository.

## Clone

```bash
git clone --recurse-submodules https://github.com/Gabriel-Ning/phy_ai_runtime_embodiments.git
```

Already cloned:

```bash
git submodule update --init --recursive
```

## Build

Place the checkout under a ROS 2 workspace `src/` tree (this repository's
layout matches `src/embodiments/`) and build the packages you need.

Hardware plugins expect separately packaged SDKs: `libmarvin`, `libpiper`,
`libfranka`.
