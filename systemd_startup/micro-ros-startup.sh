#!/bin/bash

echo "source /opt/vulcanexus/humble/setup.bash" >> ~/.bashrc
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 921600 -v6
