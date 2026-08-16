# phy_ai_runtime_embodiments

Owned robot and Pika gripper packages for Physical AI Runtime.
Each package is a git submodule over HTTPS.

```text
robots/marvin/marvin_description          # branch dev (site-calibrated)
robots/marvin/marvin_hardware_interface   # branch main
robots/piper/piper_description            # branch main
robots/piper/piper_hardware_interface     # branch main
end_effectors/pika_gripper/pika_gripper_description
end_effectors/pika_gripper/pika_gripper_hardware_interface
```

Franka, Hikvision, `pika_gripper_bringup`, and `pika_sensor_description`
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

Hardware plugins expect separately packaged SDKs: `libmarvin`, `libpiper`.
