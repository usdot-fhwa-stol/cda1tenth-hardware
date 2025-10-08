# Car Odometry Node

This ROS2 node calculates odometry from raw sensor data published by the ESP32 firmware.

## Overview

The odometry node subscribes to:
- `/imu/data` - IMU data (accelerometer, gyroscope, orientation)
- `/motor_data` - Motor RPM and current data

And publishes:
- `/odom` - Standard ROS2 odometry message
- `/odom_twist` - Twist message for debugging
- TF transforms from `odom` to `base_link`

## Features

- **Dual odometry calculation**: Uses both wheel encoders and IMU data
- **Configurable parameters**: Wheel radius, wheelbase, track width
- **Covariance estimation**: Proper uncertainty modeling
- **TF broadcasting**: Standard ROS2 transform tree
- **Python and C++ implementations**: Choose your preferred language

## Installation

### Prerequisites

```bash
# Install ROS2 dependencies
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

### Build the package

```bash
cd /path/to/your/workspace
colcon build --packages-select car_odometry
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Run the odometry node
ros2 run car_odometry odometry_node

# Or use the Python version
ros2 run car_odometry odometry_node.py
```

### With Launch File

```bash
# Launch with default parameters
ros2 launch car_odometry car_odometry.launch.py

# Launch with custom parameters
ros2 launch car_odometry car_odometry.launch.py \
    wheel_radius:=0.03 \
    wheelbase:=0.185 \
    track_width:=0.15
```

### With Configuration File

```bash
# Use configuration file
ros2 run car_odometry odometry_node --ros-args --params-file config/car_params.yaml
```

## Parameters

| Parameter | Default | Description |
|-----------|----------|-------------|
| `wheel_radius` | 0.03 | Wheel radius in meters |
| `wheelbase` | 0.185 | Distance between front and rear axles |
| `track_width` | 0.15 | Distance between left and right wheels |
| `publish_rate` | 50.0 | Odometry publishing rate in Hz |
| `frame_id` | "odom" | Odometry frame ID |
| `child_frame_id` | "base_link" | Base link frame ID |

## Topics

### Subscribed Topics

- `/imu/data` (sensor_msgs/Imu): IMU data from ESP32
- `/motor_data` (std_msgs/Float32MultiArray): Motor RPM data from ESP32

### Published Topics

- `/odom` (nav_msgs/Odometry): Calculated odometry
- `/odom_twist` (geometry_msgs/Twist): Linear and angular velocities
- TF transforms: `odom` → `base_link`

## Odometry Calculation

The node uses a combination of wheel encoder and IMU data:

1. **Linear velocity**: Calculated from average of left and right wheel RPM
2. **Angular velocity**: Uses IMU gyroscope data (more accurate than wheel difference)
3. **Position**: Integrated from linear velocity and heading
4. **Heading**: Integrated from angular velocity

### Mathematical Model

```
Linear velocity = (right_velocity + left_velocity) / 2
Angular velocity = IMU.gyro.z  (preferred)
                 = (right_velocity - left_velocity) / track_width  (fallback)

Position update:
x += linear_velocity * cos(theta) * dt
y += linear_velocity * sin(theta) * dt
theta += angular_velocity * dt
```

## Debugging

### View Odometry Data

```bash
# View odometry messages
ros2 topic echo /odom

# View twist data
ros2 topic echo /odom_twist

# View TF tree
ros2 run tf2_tools view_frames
```

### Monitor Sensor Data

```bash
# Check IMU data
ros2 topic echo /imu/data

# Check motor data
ros2 topic echo /motor_data

# List all topics
ros2 topic list
```

### Visualize in RViz

```bash
# Launch RViz with odometry visualization
ros2 run rviz2 rviz2 -d config/odometry.rviz
```

## Troubleshooting

### Common Issues

1. **No odometry published**: Check if sensor data is being received
   ```bash
   ros2 topic hz /imu/data
   ros2 topic hz /motor_data
   ```

2. **Incorrect odometry**: Verify robot parameters
   ```bash
   ros2 param get /car_odometry_node wheel_radius
   ros2 param get /car_odometry_node wheelbase
   ```

3. **TF errors**: Check frame IDs
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Performance Tuning

- Adjust `publish_rate` based on your needs (10-100 Hz)
- Modify covariance values based on sensor quality
- Use IMU for angular velocity (more accurate than wheel difference)

## Customization

### Adding New Sensors

To add additional sensors (e.g., GPS, wheel encoders):

1. Add new subscribers in the node
2. Modify the odometry calculation in `publish_odometry()`
3. Update covariance matrices accordingly

### Different Robot Configurations

Create new parameter files for different robots:

```yaml
# config/different_robot.yaml
car_odometry_node:
  ros__parameters:
    wheel_radius: 0.05
    wheelbase: 0.3
    track_width: 0.2
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

Apache 2.0
