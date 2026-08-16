# piper_description

URDF/xacro and meshes for the Agilex Piper arm. Meshes are vendored here.
Hardware plugins live in `piper_hardware_interface`.

## Models

| Xacro | Role |
| --- | --- |
| `urdf/piper.urdf.xacro` | Single arm |
| `urdf/piper_with_gripper.urdf.xacro` | Arm + native gripper |
| `urdf/piper_with_teach.urdf.xacro` | Leader gravity model (no `ros2_control`) |
| `urdf/piper_bimanual_manipulation.urdf.xacro` | Experiment table + dual arms + native grippers |

The bimanual table is visual-only (no collision). Pika is optional and is not required unless enabled.

## Visualize

Single arm:

```bash
ros2 launch piper_description visualize_piper.launch.py
ros2 launch piper_description visualize_piper.launch.py with_gripper:=true
ros2 launch piper_description visualize_piper_foxglove.launch.py
```

Bimanual:

```bash
ros2 launch piper_description visualize_piper_bimanual.launch.py
```

Xacro arguments, launch parameters, CAN notes, and mesh layout: [docs/USAGE.md](docs/USAGE.md).
