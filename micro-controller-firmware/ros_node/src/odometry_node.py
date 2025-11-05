#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from car_state_msg.msg import CarState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Pose
from car_config_msg.msg import CarConfig
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CarOdometryNode(Node):
    def __init__(self):
        super().__init__("car_odometry_node")

        # Declare parameters - Physical robot parameters
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheelbase", 0.185)
        self.declare_parameter("track_width", 0.15)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # Velocity filtering parameters
        self.declare_parameter("velocity_threshold", 0.001)  # 1 mm/s
        self.declare_parameter("angular_threshold", 0.001)  # ~0.06 degrees/s

        # Control parameters
        self.declare_parameter(
            "encoder_offset", 187.5
        )  # Steering encoder offset in degrees (0-360)
        self.declare_parameter(
            "max_steering_angle", 30.0
        )  # Maximum steering angle in degrees
        self.declare_parameter("max_rpm", 300.0)  # Maximum motor RPM

        # Debug parameters
        self.declare_parameter("debug_log_frequency", 50)  # Log every Nth message

        # Get parameters
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.track_width = self.get_parameter("track_width").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # Velocity filtering parameters
        self.velocity_threshold = self.get_parameter("velocity_threshold").value
        self.angular_threshold = self.get_parameter("angular_threshold").value

        # Control parameters
        self.encoder_offset = self.get_parameter("encoder_offset").value
        self.max_steering_angle = self.get_parameter("max_steering_angle").value
        self.max_rpm = self.get_parameter("max_rpm").value

        # Debug parameters
        self.debug_log_frequency = self.get_parameter("debug_log_frequency").value

        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Latest sensor data
        self.latest_car_state = None
        self.car_state_received = False

        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Create publishers
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.twist_publisher = self.create_publisher(Twist, "odom_twist", 10)
        self.car_config_publisher = self.create_publisher(CarConfig, "car_config", 10)

        # Create subscribers
        self.car_state_subscription = self.create_subscription(
            CarState, "car_state", self.car_state_callback, qos_profile
        )

        # Create timer for odometry publishing (fallback if no data received)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        # Create timer for car config publishing at 1 Hz
        self.config_timer = self.create_timer(1.0, self.publish_car_config)

        # Add parameter change callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.frame_id
        self.odom_msg.child_frame_id = self.child_frame_id

        # Set covariance matrices
        self.set_covariance_matrices()

        self.get_logger().info("Car Odometry Node started")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius:.3f} m")
        self.get_logger().info(f"Wheelbase: {self.wheelbase:.3f} m")
        self.get_logger().info(f"Track width: {self.track_width:.3f} m")
        self.get_logger().info(f"Encoder offset: {self.encoder_offset:.1f} deg")
        self.get_logger().info(f"Max steering angle: {self.max_steering_angle:.1f} deg")
        self.get_logger().info(f"Max RPM: {self.max_rpm:.1f}")

    def parameter_change_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            param_name = param.name
            param_value = param.value

            try:
                # Update cached parameter values
                if param_name == "wheel_radius":
                    self.wheel_radius = float(param_value)
                    self.get_logger().info(
                        f"Updated wheel_radius to {self.wheel_radius:.3f} m"
                    )
                elif param_name == "wheelbase":
                    self.wheelbase = float(param_value)
                    self.get_logger().info(
                        f"Updated wheelbase to {self.wheelbase:.3f} m"
                    )
                elif param_name == "track_width":
                    self.track_width = float(param_value)
                    self.get_logger().info(
                        f"Updated track_width to {self.track_width:.3f} m"
                    )
                elif param_name == "encoder_offset":
                    self.encoder_offset = float(param_value)
                    self.get_logger().info(
                        f"Updated encoder_offset to {self.encoder_offset:.1f} deg"
                    )
                elif param_name == "max_steering_angle":
                    self.max_steering_angle = float(param_value)
                    self.get_logger().info(
                        f"Updated max_steering_angle to {self.max_steering_angle:.1f} deg"
                    )
                elif param_name == "max_rpm":
                    self.max_rpm = float(param_value)
                    self.get_logger().info(f"Updated max_rpm to {self.max_rpm:.1f}")
                elif param_name == "velocity_threshold":
                    self.velocity_threshold = float(param_value)
                    self.get_logger().info(
                        f"Updated velocity_threshold to {self.velocity_threshold:.6f} m/s"
                    )
                elif param_name == "angular_threshold":
                    self.angular_threshold = float(param_value)
                    self.get_logger().info(
                        f"Updated angular_threshold to {self.angular_threshold:.6f} rad/s"
                    )
                elif param_name == "publish_rate":
                    old_rate = self.publish_rate
                    new_rate = float(param_value)
                    if new_rate <= 0:
                        return SetParametersResult(
                            successful=False,
                            reason="publish_rate must be greater than 0",
                        )
                    self.publish_rate = new_rate
                    # Update timer period if publish_rate changes
                    timer_period = 1.0 / self.publish_rate
                    self.timer.cancel()
                    self.timer = self.create_timer(timer_period, self.publish_odometry)
                    self.get_logger().info(
                        f"Updated publish_rate from {old_rate:.1f} to {self.publish_rate:.1f} Hz"
                    )
                elif param_name == "frame_id":
                    self.frame_id = str(param_value)
                    self.odom_msg.header.frame_id = self.frame_id
                    self.get_logger().info(f"Updated frame_id to {self.frame_id}")
                elif param_name == "child_frame_id":
                    self.child_frame_id = str(param_value)
                    self.odom_msg.child_frame_id = self.child_frame_id
                    self.get_logger().info(
                        f"Updated child_frame_id to {self.child_frame_id}"
                    )
                elif param_name == "debug_log_frequency":
                    self.debug_log_frequency = int(param_value)
                    self.get_logger().info(
                        f"Updated debug_log_frequency to {self.debug_log_frequency}"
                    )
                else:
                    # Unknown parameter, reject it
                    self.get_logger().warn(f"Unknown parameter: {param_name}")
                    return SetParametersResult(
                        successful=False, reason=f"Unknown parameter: {param_name}"
                    )
            except (ValueError, TypeError) as e:
                self.get_logger().error(
                    f"Invalid value for parameter {param_name}: {e}"
                )
                return SetParametersResult(
                    successful=False, reason=f"Invalid value: {e}"
                )

        # All parameters processed successfully
        return SetParametersResult(successful=True)

    def car_state_callback(self, msg):
        self.latest_car_state = msg
        self.car_state_received = True
        # Publish odometry immediately when new data arrives
        self.publish_odometry()

    def set_covariance_matrices(self):
        # Position covariance (6x6 matrix)
        # [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        self.odom_msg.pose.covariance[0] = 0.1  # xx
        self.odom_msg.pose.covariance[7] = 0.1  # yy
        self.odom_msg.pose.covariance[14] = 0.1  # zz
        self.odom_msg.pose.covariance[21] = 0.1  # roll
        self.odom_msg.pose.covariance[28] = 0.1  # pitch
        self.odom_msg.pose.covariance[35] = 0.1  # yaw

        # Velocity covariance (6x6 matrix)
        self.odom_msg.twist.covariance[0] = 0.1  # vx
        self.odom_msg.twist.covariance[7] = 0.1  # vy
        self.odom_msg.twist.covariance[14] = 0.1  # vz
        self.odom_msg.twist.covariance[21] = 0.1  # wx
        self.odom_msg.twist.covariance[28] = 0.1  # wy
        self.odom_msg.twist.covariance[35] = 0.1  # wz

    def publish_odometry(self):
        if not self.car_state_received:
            return  # Don't publish if we don't have sensor data

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0 or dt > 1.0:  # Avoid division by zero and large time jumps
            self.last_time = current_time
            return

        # Calculate velocities from motor data
        right_rpm = self.latest_car_state.right_motor_rpm
        left_rpm = self.latest_car_state.left_motor_rpm

        # Convert RPM to linear velocities (m/s)
        right_velocity = (right_rpm / 60.0) * 2.0 * math.pi * self.wheel_radius
        left_velocity = (left_rpm / 60.0) * 2.0 * math.pi * self.wheel_radius

        # Calculate linear and angular velocities
        linear_velocity = (right_velocity + left_velocity) / 2.0

        # Use IMU for angular velocity (more accurate than wheel difference)
        angular_velocity = self.latest_car_state.gyro_z

        # Apply velocity thresholds to prevent drift (from parameters)
        if abs(linear_velocity) < self.velocity_threshold:
            linear_velocity = 0.0
        if abs(angular_velocity) < self.angular_threshold:
            angular_velocity = 0.0

        # Update position using integration
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt

        # Normalize angle
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        # Fill odometry message
        self.odom_msg.header.stamp = current_time.to_msg()

        # Position
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert from yaw to quaternion)
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        self.odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Linear velocity
        self.odom_msg.twist.twist.linear.x = linear_velocity
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0

        # Angular velocity
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = angular_velocity

        # Publish odometry
        self.odom_publisher.publish(self.odom_msg)

        # Publish twist for debugging
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.twist_publisher.publish(twist_msg)

        # Publish transform
        self.publish_transform(current_time)

        # Debug logging (frequency from parameter)
        if hasattr(self, "debug_counter"):
            self.debug_counter += 1
        else:
            self.debug_counter = 0

        if self.debug_counter % self.debug_log_frequency == 0:
            self.get_logger().info(
                f"Odometry: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}, "
                f"vx={linear_velocity:.3f}, wz={angular_velocity:.3f}, dt={dt:.3f}, "
                f"RPM: R={right_rpm:.1f}, L={left_rpm:.1f}"
            )

        self.last_time = current_time

    def publish_transform(self, time):
        transform = TransformStamped()

        transform.header.stamp = time.to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

    def publish_car_config(self):
        """Publish car configuration at 1 Hz based on parameters"""
        config_msg = CarConfig()

        # Physical parameters
        config_msg.wheelbase = self.wheelbase
        config_msg.track_width = self.track_width
        config_msg.wheel_radius = self.wheel_radius

        # Encoder configuration and control limits from parameters
        config_msg.encoder_offset = self.encoder_offset
        config_msg.max_steering_angle = self.max_steering_angle
        config_msg.max_rpm = self.max_rpm

        self.car_config_publisher.publish(config_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
