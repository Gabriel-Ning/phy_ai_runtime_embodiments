# pika_gripper_description

ROS 2 description for the AgileX Pika gripper with FinRay fingers.
Validated kinematics: base + left/right prismatic fingers (1 DOF via mimic).

ROS joint units: **single-finger travel** `[0, 0.045]` m (see
`config/joint_limits.yaml`). Hardware converts to full opening ≈ 2×.

## Layout

```text
pika_gripper_description/
  config/
    cameras.yaml          # fisheye + D405 TF placeholders
    gripper_tcp.yaml      # TCP pose placeholder
    joint_limits.yaml     # gripper_left_joint [0, 0.045]
  docs/MESH_SOURCES.md
  launch/visualize_pika_gripper.launch.py
  meshes/
    gripper_base_link.STL
    pika_finray_link.STL
    pika_adaptor/pika_franka_adaptor.STL
    pika_adaptor/pika_marvin_adaptor.STL
  rviz/visualize_pika_gripper.rviz
  urdf/
    franka_pika_adaptor.xacro          # FR3 *_link8 -> pika_adaptor_link
    marvin_pika_adaptor.xacro          # Marvin flange_* -> pika_adaptor_link
    pika_gripper.xacro                 # mountable macro (+ prefix)
    pika_gripper.ros2_control.xacro
    pika_gripper_standalone.urdf.xacro # world-fixed bench entry
```

## Visualize

```bash
ros2 launch pika_gripper_description visualize_pika_gripper.launch.py
```

Drag `gripper_left_joint`; `gripper_right_joint` follows (mimic ×1).

## Mount on a host (single)

```xml
<xacro:include filename="$(find pika_gripper_description)/urdf/pika_gripper.xacro"/>
<xacro:pika_gripper prefix="left_" parent="flange_L">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:pika_gripper>
```

Actuated joint: `${prefix}gripper_left_joint`.

For FR3 flange (`*_link8`), include the adaptor first (full assembly entry:
`franka_manipulation_controller_bringup/urdf/fr3_manipulation.urdf.xacro`):

```xml
<xacro:include filename="$(find pika_gripper_description)/urdf/franka_pika_adaptor.xacro"/>
<xacro:franka_pika_adaptor prefix="" parent="fr3_link8"/>
<xacro:pika_gripper prefix="" parent="pika_adaptor_link">
  <origin xyz="0 0 0.004" rpy="0 0 ${pi/4}"/>
</xacro:pika_gripper>
```

For Marvin flanges, use `marvin_pika_adaptor.xacro` (requires non-empty `prefix` on dual mounts).

## Dual grippers (required for bimanual)

Always pass a non-empty `prefix` so link/joint/TF names do not collide.
`marvin_description/urdf/marvin_manipulation.urdf.xacro` mounts both
sides this way:

```xml
<xacro:pika_gripper prefix="left_" parent="flange_L">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:pika_gripper>
<xacro:pika_gripper prefix="right_" parent="flange_R">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:pika_gripper>

<xacro:pika_gripper_ros2_control
  name="LeftPikaGripperHardware"
  prefix="left_"
  serial_port="$(arg left_gripper_serial_port)"
  max_width="0.045"/>
<xacro:pika_gripper_ros2_control
  name="RightPikaGripperHardware"
  prefix="right_"
  serial_port="$(arg right_gripper_serial_port)"
  max_width="0.045"/>
```

Produces e.g. `left_gripper_left_joint` / `right_gripper_left_joint`,
`left_gripper_base_link`, `left_pika_gripper_tcp`, …
