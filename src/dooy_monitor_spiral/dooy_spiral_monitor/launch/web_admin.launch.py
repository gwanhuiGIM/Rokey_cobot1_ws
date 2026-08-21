#!/usr/bin/env python3
"""Launch the M0609 monitor backend and its browser-based administrator UI."""

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
            package="rokey",
            executable="system_monitor",
            name="system_monitor",
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "control_enabled": control_enabled,
            }],
        ),
        Node(
            package="rokey",
            executable="web_ui",
            name="coffee_webui_bridge",
            output="screen",
        ),
    ])
