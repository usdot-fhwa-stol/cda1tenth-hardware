#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "publish_rate",
                default_value="20.0",
                description="CmdVel filter publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/cmd_vel",
                description="Input cmd_vel topic",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/cmd_vel_filtered",
                description="Output filtered cmd_vel topic",
            ),
            # CmdVel filter node
            Node(
                package="car_odometry",
                executable="cmd_vel_filter.py",
                name="cmd_vel_filter",
                output="screen",
                parameters=[
                    {
                        "publish_rate": LaunchConfiguration("publish_rate"),
                    }
                ],
                remappings=[
                    ("/cmd_vel", LaunchConfiguration("input_topic")),
                    ("/cmd_vel_filtered", LaunchConfiguration("output_topic")),
                ],
            ),
        ]
    )
