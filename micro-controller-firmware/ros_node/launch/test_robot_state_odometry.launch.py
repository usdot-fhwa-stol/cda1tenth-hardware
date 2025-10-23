#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.03',
            description='Wheel radius in meters'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='0.185',
            description='Wheelbase in meters'
        ),
        DeclareLaunchArgument(
            'track_width',
            default_value='0.15',
            description='Track width in meters'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Odometry publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='odom',
            description='Odometry frame ID'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='base_link',
            description='Base link frame ID'
        ),
        
        # Odometry node (Python version)
        Node(
            package='car_odometry',
            executable='odometry_node.py',
            name='car_odometry_node',
            output='screen',
            parameters=[{
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'track_width': LaunchConfiguration('track_width'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
            }],
            remappings=[
                ('robot_state', '/car_controller/robot_state'),
                ('odom', '/car_controller/odom'),
                ('odom_twist', '/car_controller/odom_twist'),
            ]
        ),
        
        # Test robot state publisher
        Node(
            package='car_odometry',
            executable='test_robot_state_odometry.py',
            name='robot_state_tester',
            output='screen'
        ),
    ])
