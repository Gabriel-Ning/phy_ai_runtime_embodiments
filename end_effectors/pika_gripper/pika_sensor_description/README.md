# pika_sensor_description

ROS 2 port of AgileX
[pika_ros](https://github.com/agilexrobotics/pika_ros) package
`pika_sensor_description`: SolidWorks URDF of the **teleop / data-collection**
device (not the robot-mounted gripper actuator).

Upstream license: BSD. Packaging here mirrors that URDF for RViz reference.

## Visualize

```bash
ros2 launch pika_sensor_description visualize_pika_sensor.launch.py
```

Fixed Frame: `base_link`. Use the joint GUI to move revolute / prismatic joints.

## Tree (summary)

```text
base_link
  ├─ center_link          (revolute center_joint)
  │    ├─ left_link1 / left_link2
  │    └─ right_link1 / right_link2
  ├─ left_gripper_add_*   (prismatic) → left_hand_link
  └─ right_gripper_add_*  (prismatic) → right_hand_link
```

No camera / hand-eye frames in upstream URDF.
