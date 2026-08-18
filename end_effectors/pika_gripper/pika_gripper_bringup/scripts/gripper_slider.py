#!/usr/bin/env python3
# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""Pika gripper commissioning GUI (bench gate 4).

Sends control_msgs/action/ParallelGripperCommand to
/gripper_controller/gripper_cmd and shows measured finger travel from
/joint_states.

Usage (after sourcing workspace, bench_bringup running):
  ros2 run pika_gripper_bringup gripper_slider.py
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState

GRIPPER_JOINT = "gripper_left_joint"
# Finger travel [m]: open=0, closed=0.045
GRIPPER_MIN_MM = 0.0
GRIPPER_MAX_MM = 45.0
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
JOINT_STATES_TOPIC = "/joint_states"


class PikaCommissionNode(Node):

    def __init__(self) -> None:
        super().__init__("pika_gripper_commissioning")
        self._client = ActionClient(self, ParallelGripperCommand, GRIPPER_ACTION)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.actual_pos = 0.0

        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._js_cb, 10)

        if self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info("Gripper action server ready.")
        else:
            self.get_logger().warn(
                "Gripper action server not available — is bench_bringup running?"
            )

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == GRIPPER_JOINT:
                self.actual_pos = pos

    def send_position(self, position_m: float) -> None:
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [float(position_m)]
        with self._goal_lock:
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal()
                except Exception:
                    pass
                self._goal_handle = None
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._resp_cb)

    def _resp_cb(self, future) -> None:
        handle = future.result()
        if handle.accepted:
            with self._goal_lock:
                self._goal_handle = handle


def build_gui(node: PikaCommissionNode) -> None:
    root = tk.Tk()
    root.title("Pika Gripper Commissioning")
    root.resizable(False, False)

    pad = 10
    frame = ttk.Frame(root, padding=pad)
    frame.pack(fill="both", expand=True)

    cmd_var = tk.DoubleVar(value=0.0)
    track_var = tk.BooleanVar(value=False)

    ttk.Label(
        frame,
        text="Pika Finger Travel (gripper_left_joint)",
        font=("Helvetica", 12, "bold"),
    ).grid(row=0, column=0, columnspan=3, pady=(0, pad))

    ttk.Label(frame, text="Command").grid(row=1, column=0, sticky="w")
    cmd_lbl = ttk.Label(frame, text="0.0 mm", width=9, anchor="e")
    cmd_lbl.grid(row=1, column=2, sticky="e")

    def _on_drag(_event=None):
        if track_var.get():
            node.send_position(cmd_var.get() / 1000.0)

    scale = ttk.Scale(
        frame,
        from_=GRIPPER_MIN_MM,
        to=GRIPPER_MAX_MM,
        orient="horizontal",
        length=320,
        variable=cmd_var,
    )
    scale.grid(row=2, column=0, columnspan=3, pady=(2, 6))
    scale.bind("<B1-Motion>", _on_drag)

    ttk.Label(frame, text="Actual").grid(row=3, column=0, sticky="w")
    act_lbl = ttk.Label(frame, text="-- mm", width=9, anchor="e", foreground="#0055cc")
    act_lbl.grid(row=3, column=2, sticky="e")
    act_bar = ttk.Progressbar(
        frame,
        orient="horizontal",
        length=320,
        maximum=GRIPPER_MAX_MM,
        mode="determinate",
    )
    act_bar.grid(row=4, column=0, columnspan=3, pady=(2, pad))

    ttk.Separator(frame, orient="horizontal").grid(
        row=5, column=0, columnspan=3, sticky="ew", pady=(0, pad)
    )

    presets = ttk.Frame(frame)
    presets.grid(row=6, column=0, columnspan=3)

    def send_preset(position_m: float) -> None:
        cmd_var.set(position_m * 1000.0)
        node.send_position(position_m)

    ttk.Button(
        presets, text="Open  0 mm", width=12, command=lambda: send_preset(0.0)
    ).grid(row=0, column=0, padx=3)
    ttk.Button(
        presets, text="Half  25 mm", width=12, command=lambda: send_preset(0.025)
    ).grid(row=0, column=1, padx=3)
    ttk.Button(
        presets, text="Close 50 mm", width=12, command=lambda: send_preset(0.05)
    ).grid(row=0, column=2, padx=3)

    bottom = ttk.Frame(frame)
    bottom.grid(row=7, column=0, columnspan=3, pady=(pad, 0))
    ttk.Button(
        bottom,
        text="Send Command",
        command=lambda: node.send_position(cmd_var.get() / 1000.0),
    ).grid(row=0, column=0, ipadx=16, ipady=2)
    ttk.Checkbutton(bottom, text="Drag-to-track", variable=track_var).grid(
        row=0, column=1, padx=(12, 0)
    )

    def update_ui() -> None:
        cmd_lbl.config(text=f"{cmd_var.get():.1f} mm")
        actual_mm = node.actual_pos * 1000.0
        act_lbl.config(text=f"{actual_mm:.1f} mm")
        act_bar["value"] = actual_mm
        root.after(100, update_ui)

    update_ui()
    root.mainloop()


def main() -> None:
    rclpy.init(args=sys.argv)
    node = PikaCommissionNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        build_gui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
