#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments for odometry
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.0325",
                description="Wheel radius in meters",
            ),
            DeclareLaunchArgument(
                "wheelbase", default_value="0.185", description="Wheelbase in meters"
            ),
            DeclareLaunchArgument(
                "track_width", default_value="0.15", description="Track width in meters"
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="50.0",
                description="Odometry publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "frame_id", default_value="odom", description="Odometry frame ID"
            ),
            DeclareLaunchArgument(
                "child_frame_id",
                default_value="base_link",
                description="Base link frame ID",
            ),
            # Declare launch arguments for cmd_vel filter
            DeclareLaunchArgument(
                "cmd_vel_filter_rate",
                default_value="20.0",
                description="CmdVel filter publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "input_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Input cmd_vel topic",
            ),
            DeclareLaunchArgument(
                "output_cmd_vel_topic",
                default_value="/cmd_vel_filtered",
                description="Output filtered cmd_vel topic",
            ),
            # Odometry node
            Node(
                package="car_odometry",
                executable="odometry_node.py",
                name="car_odometry_node",
                output="screen",
                parameters=[
                    {
                        "wheel_radius": LaunchConfiguration("wheel_radius"),
                        "wheelbase": LaunchConfiguration("wheelbase"),
                        "track_width": LaunchConfiguration("track_width"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "child_frame_id": LaunchConfiguration("child_frame_id"),
                    }
                ],
                remappings=[
                    ("car_state", "/car/car_state"),
                    ("car_config", "/car/config"),
                    ("odom", "/car/odom"),
                    ("odom_twist", "/car/odom_twist"),
                ],
            ),
            # CmdVel filter node
            Node(
                package="car_odometry",
                executable="cmd_vel_filter.py",
                name="cmd_vel_filter",
                output="screen",
                parameters=[
                    {
                        "publish_rate": LaunchConfiguration("cmd_vel_filter_rate"),
                    }
                ],
                remappings=[
                    ("/cmd_vel", LaunchConfiguration("input_cmd_vel_topic")),
                    ("/cmd_vel_filtered", LaunchConfiguration("output_cmd_vel_topic")),
                ],
            ),
        ]
    )
