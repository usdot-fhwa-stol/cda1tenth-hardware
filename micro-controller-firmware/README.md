## Building and Running

1. Build the project:
   ```bash
   pio run
   ```

2. Upload to your ESP32:
   ```bash
   pio run --target upload
   ```

3. Monitor serial output:
   ```bash
   pio device monitor
   ```

## Installing and Running the Micro-Ros Agent

1. Installing the Micro-Ros Agent
```bash
source /opt/ros/humble/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash
```

2. Running the Micro-Ros Agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev $DEVICE -b $BAUD
```

