#!/bin/bash
source /ros/humble/setup.bash
source ~/cda_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 921600
