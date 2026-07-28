#!/usr/bin/env python3
"""Launch the browser UI with its embedded M0609 monitor backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    control_enabled = LaunchConfiguration("control_enabled")

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="dsr01"),
        DeclareLaunchArgument("control_enabled", default_value="false"),
        Node(
            package="monitor_sys",
            executable="web_ui",
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "control_enabled": control_enabled,
            }],
        ),
    ])
