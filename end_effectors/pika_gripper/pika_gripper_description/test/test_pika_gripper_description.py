# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Contract checks for pika_gripper_description."""

from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
STANDALONE = SOURCE_ROOT / "urdf" / "pika_gripper_standalone.urdf.xacro"
BASE_MESH = SOURCE_ROOT / "meshes" / "gripper_base_link.STL"
FINRAY_MESH = SOURCE_ROOT / "meshes" / "pika_finray_link.STL"

REQUIRED_PATHS = [
    "config/cameras.yaml",
    "config/gripper_tcp.yaml",
    "config/joint_limits.yaml",
    "docs/MESH_SOURCES.md",
    "launch/visualize_pika_gripper.launch.py",
    "rviz/visualize_pika_gripper.rviz",
    "urdf/franka_pika_adaptor.xacro",
    "urdf/marvin_pika_adaptor.xacro",
    "urdf/pika_gripper.xacro",
    "urdf/pika_gripper.ros2_control.xacro",
    "urdf/pika_gripper_standalone.urdf.xacro",
    "meshes/gripper_base_link.STL",
    "meshes/pika_finray_link.STL",
    "meshes/pika_adaptor/pika_franka_adaptor.STL",
    "meshes/pika_adaptor/pika_marvin_adaptor.STL",
    "README.md",
]


def test_package_layout_is_complete() -> None:
    for relative in REQUIRED_PATHS:
        assert (SOURCE_ROOT / relative).is_file(), relative


def test_meshes_are_present() -> None:
    assert BASE_MESH.is_file()
    assert FINRAY_MESH.is_file()
    assert BASE_MESH.stat().st_size > 100_000
    assert FINRAY_MESH.stat().st_size > 100_000


def test_standalone_matches_limits_and_mimic() -> None:
    limits = yaml.safe_load(
        (SOURCE_ROOT / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
    )["gripper_left_joint"]["limit"]
    tcp = yaml.safe_load(
        (SOURCE_ROOT / "config" / "gripper_tcp.yaml").read_text(encoding="utf-8")
    )["origin"]

    result = subprocess.run(
        ["xacro", str(STANDALONE), "use_fake_hardware:=true"],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)

    joint = next(
        j for j in root.findall("joint") if j.get("name") == "gripper_left_joint"
    )
    assert joint.get("type") == "prismatic"
    limit = joint.find("limit")
    assert float(limit.get("lower")) == float(limits["lower"])
    assert float(limit.get("upper")) == float(limits["upper"])
    assert float(limit.get("lower")) == 0.0
    assert float(limit.get("upper")) > 0.0
    assert joint.find("axis").get("xyz") == "-1 0 0"

    right = next(
        j for j in root.findall("joint") if j.get("name") == "gripper_right_joint"
    )
    mimic = right.find("mimic")
    assert mimic is not None
    assert mimic.get("joint") == "gripper_left_joint"
    assert float(mimic.get("multiplier")) == 1.0
    assert right.find("axis").get("xyz") == "-1 0 0"

    tcp_joint = next(
        j for j in root.findall("joint") if j.get("name") == "pika_gripper_tcp_joint"
    )
    assert tcp_joint.find("origin").get("xyz") == str(tcp["xyz"])

    mesh_names = [m.get("filename") or "" for m in root.findall(".//mesh")]
    assert any("meshes/gripper_base_link.STL" in name for name in mesh_names)
    assert any("meshes/pika_finray_link.STL" in name for name in mesh_names)

    links = {link.get("name") for link in root.findall("link")}
    assert {
        "world",
        "gripper_base_link",
        "gripper_left_link",
        "gripper_right_link",
        "pika_gripper_tcp",
        "pika_fisheye_link",
        "pika_d405_link",
    } <= links


def test_prefix_scopes_links_and_joints() -> None:
    """Dual-gripper hosts need left_/right_ prefixes without name collisions."""
    import tempfile

    snippet = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dual_pika">
  <xacro:include filename="{SOURCE_ROOT / 'urdf' / 'pika_gripper.xacro'}"/>
  <link name="flange_L"/>
  <link name="flange_R"/>
  <xacro:pika_gripper prefix="left_" parent="flange_L">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:pika_gripper>
  <xacro:pika_gripper prefix="right_" parent="flange_R">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:pika_gripper>
</robot>
"""
    with tempfile.NamedTemporaryFile("w", suffix=".urdf.xacro", delete=False) as handle:
        handle.write(snippet)
        path = handle.name
    try:
        result = subprocess.run(
            ["xacro", path],
            check=True,
            capture_output=True,
            text=True,
        )
    finally:
        Path(path).unlink(missing_ok=True)

    root = ET.fromstring(result.stdout)
    joint_names = {j.get("name") for j in root.findall("joint")}
    link_names = {link.get("name") for link in root.findall("link")}
    assert "left_gripper_left_joint" in joint_names
    assert "right_gripper_left_joint" in joint_names
    assert "left_gripper_base_link" in link_names
    assert "right_gripper_base_link" in link_names
    assert "left_pika_gripper_tcp" in link_names
    assert "right_pika_gripper_tcp" in link_names
    right_mimic = next(
        j for j in root.findall("joint") if j.get("name") == "right_gripper_right_joint"
    ).find("mimic")
    assert right_mimic.get("joint") == "right_gripper_left_joint"
