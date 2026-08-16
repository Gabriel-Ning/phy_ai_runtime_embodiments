# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
#
# Bench-only bring-up of a world-fixed Pika gripper.
#
#   Gate 2 (no hardware):  ros2 launch pika_gripper_bringup bench_bringup.launch.py use_fake_hardware:=true
#   Gate 3/4 (real):       ros2 launch pika_gripper_bringup bench_bringup.launch.py serial_port:=/dev/ttyUSB0
#
# Host-robot (Marvin) composition does NOT use this file; it lives in
# marvin_bringup per docs/END_EFFECTOR_CONVENTIONS.md.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    max_width = LaunchConfiguration('max_width')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('pika_gripper_description'),
                'urdf', 'pika_gripper_standalone.urdf.xacro',
            ]),
            ' serial_port:=', serial_port,
            ' max_width:=', max_width,
            ' use_fake_hardware:=', use_fake_hardware,
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('pika_gripper_bringup'), 'config', 'controllers.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'max_width',
            default_value='0.045',
            description='ROS finger-travel upper clamp [m]; opening on wire ≈ 2×.',
        ),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controllers_yaml],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller',
                       '--controller-manager', '/controller_manager',
                       '--param-file', controllers_yaml],
        ),
        # Streaming servo path, loaded but inactive; switch in for teleop/policy.
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_forward_controller', '--inactive',
                       '--controller-manager', '/controller_manager',
                       '--param-file', controllers_yaml],
        ),
    ])
