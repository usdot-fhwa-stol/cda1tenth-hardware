# Micro-controller Firmware

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

## Configuring the Car Controller

The `car_controller` node on the ESP32 receives its configuration from the `car_odometry` node via the `/car/config` topic. To configure parameters such as `encoder_offset`, `wheel_radius`, `wheelbase`, `track_width`, `max_steering_angle`, and `max_rpm`, you need to set them in the `car_odometry` node.

For detailed instructions on how to configure the `car_odometry` node and set parameters like `encoder_offset`, please refer to the [car_odometry README](https://github.com/Michael7371/cda1tenth-bringup/blob/develop/car_odom_node/README.md).

The `car_odometry` node publishes configuration messages to `/car/config` at 1 Hz, which the `car_controller` node subscribes to and uses to update its internal parameters.

### Quick Reference

The `encoder_offset` parameter is particularly important for accurate steering control. It compensates for the initial encoder position and should be calibrated for your specific hardware setup. Other key parameters include:

- `wheel_radius`: Wheel radius in meters (default: 0.0325)
- `wheelbase`: Distance between front and rear axles in meters (default: 0.185)
- `track_width`: Distance between left and right wheels in meters (default: 0.15)
- `encoder_offset`: Steering encoder offset in degrees (default: 187.5)
- `max_steering_angle`: Maximum steering angle in degrees (default: 30.0)
- `max_rpm`: Maximum motor RPM (default: 300.0)

See the [car_odometry README](https://github.com/Michael7371/cda1tenth-bringup/blob/develop/car_odom_node/README.md) for complete parameter documentation and configuration instructions.

## Installing Custom Message Packages

The firmware uses custom ROS 2 message packages (`car_state_msg` and `car_config_msg`) that need to be built in your ROS 2 workspace.

1. Navigate to your ROS 2 workspace:

   ```bash
   # If you don't have a workspace, create one:
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Copy the message packages from `extra_packages/` to your workspace:

   ```bash
   # Copy car_state_msg and car_config_msg to your workspace src directory
   ```

3. Resolve dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the packages:

   ```bash
   colcon build --packages-select car_state_msg car_config_msg
   ```

5. Source your workspace:

   ```bash
   source install/local_setup.bash
   ```

## Additional Resources

- [micro-ROS PlatformIO](https://github.com/micro-ROS/micro_ros_platformio)
- Clean micro-ROS build: `pio run --target clean_microros`
