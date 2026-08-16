import os

import xacro
import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_description_from_xacro(context: LaunchContext) -> str:
    share = get_package_share_directory("piper_description")

    with_gripper = context.perform_substitution(LaunchConfiguration("with_gripper"))
    xacro_file = os.path.join(
        share,
        "urdf",
        "piper_with_gripper.urdf.xacro"
        if with_gripper == "true"
        else "piper.urdf.xacro",
    )

    mappings = {
        "prefix": context.perform_substitution(LaunchConfiguration("prefix")),
        "connected_to": context.perform_substitution(
            LaunchConfiguration("connected_to")
        ),
        "xyz": context.perform_substitution(LaunchConfiguration("xyz")),
        "rpy": context.perform_substitution(LaunchConfiguration("rpy")),
        "ros2_control": "false",
        "use_fake_hardware": "true",
    }

    if with_gripper == "true":
        mappings.update(
            {
                "load_gripper": "true",
                "xyz_ee": context.perform_substitution(LaunchConfiguration("xyz_ee")),
                "rpy_ee": context.perform_substitution(LaunchConfiguration("rpy_ee")),
                "tcp_xyz": context.perform_substitution(LaunchConfiguration("tcp_xyz")),
                "tcp_rpy": context.perform_substitution(LaunchConfiguration("tcp_rpy")),
            }
        )

    return xacro.process_file(xacro_file, mappings=mappings).toprettyxml(indent="  ")


def launch_setup(context: LaunchContext):
    robot_description = robot_description_from_xacro(context)
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    start_bridge = context.perform_substitution(LaunchConfiguration("start_bridge"))

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_states_topic)],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_states_topic)],
        ),
    ]

    if start_bridge == "true":
        try:
            foxglove_bridge_share = get_package_share_directory("foxglove_bridge")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "foxglove_bridge is not installed. Install it first, for example: "
                "pixi add ros-humble-foxglove-bridge"
            ) from exc

        foxglove_bridge_launch = os.path.join(
            foxglove_bridge_share,
            "launch",
            "foxglove_bridge_launch.xml",
        )
        nodes.append(
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_bridge_launch)
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("piper_description")
    gripper_tcp_yaml_path = os.path.join(share, "config", "gripper_tcp.yaml")
    with open(gripper_tcp_yaml_path, "r", encoding="utf-8") as f:
        gripper_tcp_cfg = yaml.safe_load(f)

    tcp_xyz_default = str(gripper_tcp_cfg["origin"]["xyz"])
    tcp_rpy_default = str(gripper_tcp_cfg["origin"]["rpy"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("with_gripper", default_value="false"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("connected_to", default_value="world"),
            DeclareLaunchArgument("xyz", default_value="0 0 0"),
            DeclareLaunchArgument("rpy", default_value="0 0 0"),
            DeclareLaunchArgument("xyz_ee", default_value="0 0 0"),
            DeclareLaunchArgument("rpy_ee", default_value="0 0 0"),
            DeclareLaunchArgument("tcp_xyz", default_value=tcp_xyz_default),
            DeclareLaunchArgument("tcp_rpy", default_value=tcp_rpy_default),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/piper_description/joint_states"
            ),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
