# pika_gripper_bringup

Bench-only bring-up for the AgileX Pika gripper. Host-robot (Marvin)
composition lives in `marvin_bringup`.

## Validation gates

Per `.codex/skills/physical-ai-incremental-validation`. Status reflects the
last verified run; update as gates pass.

### Gate 1 — Serial sniff (read-only) ✅ 2026-07-22

```bash
stty -F /dev/ttyUSB0 460800 raw -echo && timeout 3 head -c 1200 /dev/ttyUSB0 | od -c | head
```

- Expected: continuous `{"motor":...,"motorstatus":...}` frames.
- Verified: device streams EMPTY-value frames (`"motor":,`) while the motor
  is disabled — this is normal quiescent output.

### Gate 2 — Mock hardware rehearsal (no hardware) ✅ 2026-07-22

```bash
ros2 launch pika_gripper_bringup bench_bringup.launch.py use_fake_hardware:=true
```

- Expected log lines:
  - `Loaded hardware 'PikaGripperSystem' from plugin 'mock_components/GenericSystem'`
  - `Configured and activated gripper_controller`
  - `Configured and activated joint_state_broadcaster`
- `ros2 topic echo /joint_states --once` shows `gripper_left_joint`.

### Gate 3 — Real hardware, feedback only ✅ 2026-07-22

Hardware: real gripper on the desk, fingers clear, **24V connected to the
XT30(PB) port** (pin 1 = 24V+, pin 2 = 24V−; see the Pika user manual).
USB Type-C only powers the MCU and cameras — without 24V the device streams
empty `"motor":,` frames forever, ENABLE is ignored, and activation fails
with `No valid motor feedback within ... s after ENABLE`.
Activation sends ENABLE (motor energizes and holds) but no motion command;
the controller syncs its target to the measured width.

```bash
ros2 launch pika_gripper_bringup bench_bringup.launch.py \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

- Expected: `Connected to Pika gripper on ...`, then
  `Activated; measured finger travel X m` with a plausible X in
  roughly `[0, 0.045]`.
- `ros2 topic echo /joint_states` tracks the joint when you move the fingers
  BY HAND (motor disabled test: relaunch with the gripper unpowered to
  confirm activation fails cleanly with the timeout error).
- Manually verify travel; if tips differ, update `max_width` and
  `joint_limits.yaml` together.

### Gate 4 — Low-speed motion ✅ 2026-07-22

GUI (slider + presets + live travel readback):

```bash
ros2 run pika_gripper_bringup gripper_slider.py
```

or CLI:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper_left_joint], position: [0.0225]}}"
```

- Expected: smooth motion toward mid travel (plugin rate-limits steps
  internally); large jumps ramp instead of snapping.
- Then command `0.0` (open) and `0.045` (closed); watch `/joint_states` converge.

### Controllers

| Controller | Role | Default |
|---|---|---|
| `gripper_controller` | `parallel_gripper_action_controller` — task-level open/close | active |
| `gripper_forward_controller` | `forward_command_controller` — streaming servo for teleop/policy | inactive |

Switch:

```bash
ros2 control switch_controllers \
  --deactivate gripper_controller --activate gripper_forward_controller
```

### Gate 5 — Cameras + host integration ⏳

```bash
ros2 launch pika_gripper_bringup cameras.launch.py
```

- D405 via `realsense2_camera` (workspace dependency already present).
- Fisheye via `usb_cam` — requires `ros-jazzy-usb-cam` in `pixi.toml`
  (not added yet; add on first use).
- Calibrate fisheye/D405 extrinsic origins in `pika_gripper.xacro`
  (`fisheye_xyz/rpy`, `d405_xyz/rpy` are placeholders).
- Finally: mount on Marvin, compose in `marvin_bringup`, confirm the episode
  recorder picks up `gripper_left_joint` from `/joint_states` unchanged.
