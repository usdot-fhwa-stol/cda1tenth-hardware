#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/cda1t_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
