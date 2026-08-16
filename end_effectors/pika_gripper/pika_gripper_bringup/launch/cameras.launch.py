# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
#
# Integrated cameras of the Pika gripper (perception path, separate from the
# ros2_control actuation path — docs/END_EFFECTOR_CONVENTIONS.md §1.3).
#
#   - RealSense D405: realsense2_camera (already a workspace dependency)
#   - Fisheye (Sunplus UVC): usb_cam
#
# Frame names match pika_gripper_description (pika_d405_link,
# pika_fisheye_link); TF comes from robot_state_publisher.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fisheye_device', default_value='/dev/video0'),
        DeclareLaunchArgument('d405_serial', default_value="''",
                              description='RealSense serial number; empty = first device'),
        DeclareLaunchArgument('start_fisheye', default_value='true'),
        DeclareLaunchArgument('start_d405', default_value='true'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='pika_d405',
            condition=IfCondition(LaunchConfiguration('start_d405')),
            parameters=[{
                'serial_no': LaunchConfiguration('d405_serial'),
                'base_frame_id': 'pika_d405_link',
                'enable_depth': True,
                'enable_color': True,
            }],
            output='screen',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='pika_fisheye',
            condition=IfCondition(LaunchConfiguration('start_fisheye')),
            parameters=[{
                'video_device': LaunchConfiguration('fisheye_device'),
                'camera_frame_id': 'pika_fisheye_link',
                'image_width': 1280,
                'image_height': 720,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
            }],
            output='screen',
        ),
    ])
