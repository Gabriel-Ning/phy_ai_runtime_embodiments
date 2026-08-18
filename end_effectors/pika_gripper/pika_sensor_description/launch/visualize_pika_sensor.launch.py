# Copyright 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Official AgileX pika_sensor_description RViz reference (ROS 2 port).

Mirrors upstream Noetic ``display.launch`` from
https://github.com/agilexrobotics/pika_ros
(package ``pika_sensor_description``).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("pika_sensor_description"))
    urdf_path = share / "urdf" / "pika_sensor_description.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    rviz_config = PathJoinSubstitution(
        [str(share), "rviz", "visualize_pika_sensor.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/pika_sensor_description/joint_states",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", joint_states_topic)],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_gui),
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", joint_states_topic)],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=UnlessCondition(use_joint_state_gui),
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", joint_states_topic)],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
