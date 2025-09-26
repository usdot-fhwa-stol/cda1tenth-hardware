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
