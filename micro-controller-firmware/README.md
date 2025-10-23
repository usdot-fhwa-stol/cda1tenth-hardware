## Building and Running

0. Install Platform IO using the following installation [guide](https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation)

1. Build the project:
   ```bash
   pio run
   ```

2. Use a USB cable to connect your laptop to the micro-controller board's USB-C port

3. Put the ESP32 into the manual bootloader mode ([docs by EXPRESSIF](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html#manual-bootloader)).

4. Upload to your ESP32:
   ```bash
   pio run --target upload
   ```

5. Monitor serial output:
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

3. Testing Odometry Publisher
```bash
ros2 topic list

ros2 topic echo /odom
```

4. Testing ROS Parameters
```bash
ros2 param set  /car_controller wheel_radius <value>
ros2 param set  /car_controller wheelbase <value>
ros2 param set  /car_controller odom_period_ms <value>

ros2 param get /car_controller wheel_radius
ros2 param get  /car_controller wheelbase 
ros2 param get  /car_controller odom_period_ms 
```

https://github.com/micro-ROS/micro_ros_platformio

```bash
pio run --target clean_microros
```


add robot_state_msgs to the ROS node by doing the following
1. navigate to your ROS 2 workspace
   - if you don't have one make it with the following commands:
       mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
2. resolve any required dependencies:
  cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y

3. install the package:
colcon build

4. source your workspace:
source install/local_setup.bash
