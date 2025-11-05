# Car Odometry Node

This ROS2 package provides odometry calculation and cmd_vel filtering for car-like robots using the unified CarState message format.

## Quick Start

```bash
# 1. Install dependencies
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-ament-cmake-python

# 2. Build dependencies first
cd /path/to/your/workspace/extra_packages
colcon build --packages-select car_state_msg car_config_msg
source install/setup.bash

# 3. Build the odometry package
cd ../ros_node
colcon build --packages-select car_odometry
source install/setup.bash

# 4. Run the nodes
ros2 run car_odometry odometry_node.py
ros2 run car_odometry cmd_vel_filter.py

# Or launch both together
ros2 launch car_odometry car_system.launch.py
```

**⚠️ Important**: Always build `car_state_msg` and `car_config_msg` before `car_odometry` to avoid build errors.

## Overview

This package provides two main nodes:

### Odometry Node
The odometry node subscribes to:
- `/car/car_state` - Unified car state message containing IMU and motor data

And publishes:
- `/car/odom` - Standard ROS2 odometry message
- `/car/odom_twist` - Twist message for debugging
- `/car/config` - Car configuration message (wheelbase, track width, encoder offset, etc.)
- TF transforms from `odom` to `base_link`

### CmdVel Filter Node
The cmd_vel filter node subscribes to:
- `/cmd_vel` - Input velocity commands

And publishes:
- `/cmd_vel_filtered` - Filtered velocity commands at constant rate

## CarState Message Integration

This node uses the unified `CarState` message for synchronized sensor data and `CarConfig` message for vehicle configuration. This provides:

- **Simplified Architecture**: Single message type instead of multiple separate topics
- **Synchronized Data**: All sensor data arrives in a single, synchronized message
- **Reduced Network Overhead**: Single message instead of multiple separate messages
- **Easier Integration**: Simpler subscriber logic and data management
- **Better Performance**: Reduced message passing overhead
- **Dynamic Configuration**: Car parameters can be updated at runtime via `CarConfig` message

### CarState Message Structure

The `CarState` message contains:
- **Header**: Standard ROS header with timestamp and frame_id
- **IMU Data**: `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z`
- **Motor Data**: `speed`, `steering_angle`, `right_motor_rpm`, `left_motor_rpm`

### CarConfig Message Structure

The `CarConfig` message contains:
- **Physical Parameters**: `wheelbase`, `track_width`, `wheel_radius`
- **Control Parameters**: `encoder_offset`, `max_steering_angle`, `max_rpm`

The odometry node publishes this configuration at 1 Hz based on ROS parameters, allowing the ESP32 microcontroller to receive dynamic configuration updates.

## Features

### Odometry Node
- **Dual odometry calculation**: Uses both wheel encoders and IMU data
- **Configurable parameters**: Wheel radius, wheelbase, track width, encoder offset, control limits
- **Dynamic configuration**: Publishes car configuration to ESP32 via `CarConfig` message
- **Covariance estimation**: Proper uncertainty modeling
- **TF broadcasting**: Standard ROS2 transform tree

### CmdVel Filter Node
- **Constant rate publishing**: Maintains consistent 20Hz output regardless of input frequency
- **Zero velocity fallback**: Publishes zero velocities when no input received
- **Configurable rate**: Adjustable publish rate via parameters
- **Smooth operation**: Prevents motor stuttering from irregular cmd_vel messages

### General
- **Python implementation**: Clean, maintainable Python code
- **Unified message format**: Uses CarState message for synchronized sensor data
- **Parameter-based configuration**: All vehicle parameters configurable via ROS parameters

## Installation

### Prerequisites

```bash
# Install ROS2 dependencies
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-ament-cmake-python
```

### Build the package

**⚠️ Important: Build Dependencies First**

This package depends on the custom `car_state_msg` and `car_config_msg` packages, which must be built first:

```bash
# 1. Build the custom message packages first
cd /path/to/your/workspace/extra_packages
colcon build --packages-select car_state_msg car_config_msg
source install/setup.bash

# 2. Build the car_odometry package
cd ../ros_node
colcon build --packages-select car_odometry
source install/setup.bash
```

**Alternative: Build all packages at once**
```bash
# From the workspace root, build all packages
cd /path/to/your/workspace
colcon build
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Run individual nodes
ros2 run car_odometry odometry_node.py
ros2 run car_odometry cmd_vel_filter.py

# Or run both together
ros2 launch car_odometry car_system.launch.py
```

### With Launch File

```bash
# Launch odometry node only
ros2 launch car_odometry car_odometry.launch.py

# Launch cmd_vel filter only
ros2 launch car_odometry cmd_vel_filter.launch.py

# Launch both nodes together
ros2 launch car_odometry car_system.launch.py

# Launch with custom parameters
ros2 launch car_odometry car_system.launch.py \
    wheel_radius:=0.0325 \
    wheelbase:=0.185 \
    track_width:=0.15 \
    cmd_vel_filter_rate:=20.0

# Test with robot state messages
ros2 launch car_odometry test_robot_state_odometry.launch.py
```

### With Configuration File

```bash
# Use configuration file
ros2 run car_odometry odometry_node --ros-args --params-file config/car_params.yaml
```

## Parameters

| Parameter | Default | Description |
|-----------|----------|-------------|
| `wheel_radius` | 0.0325 | Wheel radius in meters |
| `wheelbase` | 0.185 | Distance between front and rear axles (meters) |
| `track_width` | 0.15 | Distance between left and right wheels (meters) |
| `encoder_offset` | 187.5 | Steering encoder offset in degrees (0-360) |
| `max_steering_angle` | 30.0 | Maximum steering angle in degrees |
| `max_rpm` | 300.0 | Maximum motor RPM |
| `velocity_threshold` | 0.001 | Minimum linear velocity (m/s) - below this is treated as 0 |
| `angular_threshold` | 0.001 | Minimum angular velocity (rad/s) - below this is treated as 0 |
| `publish_rate` | 50.0 | Odometry publishing rate in Hz |
| `frame_id` | "odom" | Odometry frame ID |
| `child_frame_id` | "base_link" | Base link frame ID |
| `debug_log_frequency` | 50 | Log every Nth message (set to 0 to disable) |

## Topics

### Subscribed Topics

- `/car/car_state` (car_state_msg/CarState): Unified car state message containing IMU and motor data
- `/cmd_vel` (geometry_msgs/Twist): Input velocity commands (for cmd_vel_filter)

### Published Topics

- `/car/odom` (nav_msgs/Odometry): Calculated odometry
- `/car/odom_twist` (geometry_msgs/Twist): Linear and angular velocities
- `/car/config` (car_config_msg/CarConfig): Car configuration message (published at 1 Hz)
- `/cmd_vel_filtered` (geometry_msgs/Twist): Filtered velocity commands at constant rate
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
# Check car state data
ros2 topic echo /car/car_state

# Check car configuration
ros2 topic echo /car/config

# List all topics
ros2 topic list
```

### Visualize in RViz

```bash
# Launch RViz with odometry visualization
ros2 run rviz2 rviz2 -d config/odometry.rviz
```

## Testing

### Test Scripts

The package includes test scripts to verify both odometry and cmd_vel filter functionality:

```bash
# Test odometry with CarState messages
ros2 run car_odometry test_robot_state_odometry.py
ros2 launch car_odometry test_robot_state_odometry.launch.py

# Test cmd_vel filter
ros2 run car_odometry test_cmd_vel_filter.py
```

The test scripts verify that:
- Odometry messages are published correctly
- Twist messages are published correctly
- Position and orientation calculations are working
- Motor RPM data is properly converted to velocities
- CmdVel filter maintains constant publish rate
- CmdVel filter handles missing input gracefully

### Topic Mapping

| Component | Topic | Full Path |
|-----------|-------|-----------|
| Car State (ESP32 → ROS) | `car_state` | `/car/car_state` |
| Car Config (ROS → ESP32) | `car_config` | `/car/config` |
| Odometry | `odom` | `/car/odom` |
| Odometry Twist | `odom_twist` | `/car/odom_twist` |
| Debug Data | `debug_data` | `/car/debug_data` |
| Cmd Vel Filtered | `cmd_vel_filtered` | `/cmd_vel_filtered` |

## Troubleshooting

### Common Build Issues

1. **CMake Error: "Could not find ament_python"**
   ```
   CMake Error: By not providing "Findament_python.cmake" in CMAKE_MODULE_PATH...
   ```
   **Solution**: The CMakeLists.txt has been fixed to use `ament_cmake_python` instead of `ament_python`. If you encounter this error, ensure you have the latest version of the code.

2. **CMake Error: "Could not find car_state_msg" or "Could not find car_config_msg"**
   ```
   CMake Error: Could not find a package configuration file provided by "car_state_msg"
   ```
   **Solution**: Build the `car_state_msg` and `car_config_msg` packages first:
   ```bash
   cd /path/to/your/workspace/extra_packages
   colcon build --packages-select car_state_msg car_config_msg
   source install/setup.bash
   cd ../ros_node
   colcon build --packages-select car_odometry
   ```

3. **Package not found after building**
   **Solution**: Always source the workspace after building:
   ```bash
   source install/setup.bash
   ```

### Runtime Issues

1. **No odometry published**: Check if sensor data is being received
   ```bash
   ros2 topic hz /car/car_state
   ros2 topic echo /car/car_state
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

### Build Order Reference

For reference, here's the correct build order for all packages in this workspace:

```bash
# 1. Build custom message packages first
cd /path/to/your/workspace/extra_packages
colcon build
source install/setup.bash

# 2. Build ROS nodes
cd ../ros_node
colcon build
source install/setup.bash
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
