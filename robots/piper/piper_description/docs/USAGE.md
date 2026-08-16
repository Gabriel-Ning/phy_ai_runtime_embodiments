# piper_description usage

## Layout

```text
piper_description/
  config/                 joint_limits, kinematics, inertials, gripper_tcp
  launch/                 RViz / Foxglove visualization
  meshes/                 vendored visual + collision assets
  urdf/
    piper.urdf.xacro
    piper_with_gripper.urdf.xacro
    piper_with_teach.urdf.xacro
    piper_bimanual_manipulation.urdf.xacro
    parts/                arm, gripper, teach, experiment_table, ros2_control
```

Arm kinematics / limits / inertials are loaded from YAML by `piper_arm.xacro`.
TCP pose is `config/gripper_tcp.yaml`.

## Single arm

```bash
xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper.urdf.xacro" \
  can_interface:=can0

xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_with_gripper.urdf.xacro" \
  can_interface:=can0 load_gripper:=true

xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper.urdf.xacro" \
  prefix:=left_ connected_to:=world xyz:='0 0 0' rpy:='0 0 0'
```

Arm and gripper are two independent `ros2_control` systems and may share one
SocketCAN interface. This package does not bring the CAN link up.

`enable_gripper_control:=false` keeps the gripper geometry without a hardware
component.

`gripper_joint1` is one finger's prismatic travel; `gripper_joint2` mimics it
with multiplier `-1`. libpiper reports/commands full opening width;
`PiperGripperInterface` applies the factor-of-two conversion.

## Leader model

```bash
xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_with_teach.urdf.xacro"
```

Private dynamics model for each leader teleop node. Unprefixed libpiper
joint/frame names, no `ros2_control` block.

## Bimanual cell

`piper_bimanual_manipulation.urdf.xacro` always includes the experiment table
(visual mesh only; [experiment_table_models](https://github.com/Marco6-3/experiment_table_models)).
Collision for the table belongs in the planner world, not this URDF.

Default mounts (rear crossbeam, yaw toward the tabletop):

| Arm | `xyz` / m | `rpy` / rad |
| --- | --- | --- |
| left | `-0.38 0.32 0.71` | `0 0 -π/2` |
| right | `0.38 0.32 0.71` | `0 0 -π/2` |

Default end effectors are native Piper grippers. Bringup maps
`<side>_end_effector` (`none` / `piper_gripper` / `pika_gripper`) onto:

- `enable_<side>_gripper`
- `enable_<side>_pika_gripper`

Pika xacro is included only when a Pika flag is true, so default visualization
does not need `pika_gripper_description`.

```bash
ros2 launch piper_description visualize_piper_bimanual.launch.py

xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_bimanual_manipulation.urdf.xacro"
```

This emits four `ros2_control` components (left/right arm and gripper). Gripper
components reuse the matching arm CAN name. All components assume SocketCAN is
already ready.

## Visualize in RViz

Model-only: `ros2_control:=false`, no CAN or hardware.

```bash
ros2 launch piper_description visualize_piper.launch.py
ros2 launch piper_description visualize_piper.launch.py with_gripper:=true
ros2 launch piper_description visualize_piper.launch.py joint_states_topic:=/joint_states
```

Default joint-state topic is `/piper_description/joint_states` so unrelated
`/joint_states` publishers do not move RViz.

Under pixi, `visualize_piper_bimanual.launch.py` sets `CONDA_PREFIX` for
`joint_state_publisher_gui` (`python_qt_binding` expects it).

| Parameter | Default | Description |
| --- | --- | --- |
| `with_gripper` | `false` | Load `piper_with_gripper.urdf.xacro`. |
| `prefix` | `""` | Prefix for link/joint names. |
| `connected_to` | `world` | Parent of the fixed base mount. |
| `xyz` / `rpy` | `0 0 0` | Base mount pose relative to `connected_to`. |
| `xyz_ee` / `rpy_ee` | `0 0 0` | Gripper mount relative to `flange_link`. |
| `tcp_xyz` / `tcp_rpy` | `config/gripper_tcp.yaml` | TCP relative to `gripper_base`. |
| `use_joint_state_gui` | `false` (`true` on bimanual) | GUI sliders vs headless publisher. |
| `use_rviz` | `true` | Start RViz with `rviz/visualize_piper.rviz`. |
| `joint_states_topic` | `/piper_description/joint_states` | Joint state topic. |
| `enable_grippers` | `true` | Bimanual launch only: native grippers. |

## Visualize in Foxglove

```bash
pixi add ros-humble-foxglove-bridge   # if needed
ros2 launch piper_description visualize_piper_foxglove.launch.py
ros2 launch piper_description visualize_piper_foxglove.launch.py with_gripper:=true
```

Connect to `ws://localhost:8765`. Import `config/foxglove/display_robot.json`
for a 3D layout that reads `/robot_description`.

| Parameter | Default | Description |
| --- | --- | --- |
| `with_gripper` | `false` | Load the gripper model. |
| `prefix` | `""` | Prefix for link/joint names. |
| `connected_to` | `world` | Parent of the fixed base mount. |
| `xyz` / `rpy` | `0 0 0` | Base mount pose. |
| `xyz_ee` / `rpy_ee` | `0 0 0` | Gripper mount relative to `flange_link`. |
| `tcp_xyz` / `tcp_rpy` | `config/gripper_tcp.yaml` | TCP relative to `gripper_base`. |
| `joint_states_topic` | `/piper_description/joint_states` | Joint state topic. |
| `start_bridge` | `true` | Include `foxglove_bridge`. Set `false` if one is already running. |

## Meshes

```text
meshes/piper_arm/{visual,collision}/
meshes/piper_gripper/{visual,collision}/
meshes/piper_teach/{visual,collision}/
meshes/table/                 visual STL for the experiment table
```
