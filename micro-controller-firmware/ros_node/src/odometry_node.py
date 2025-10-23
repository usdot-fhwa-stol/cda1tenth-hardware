#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_state_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class CarOdometryNode(Node):
    def __init__(self):
        super().__init__('car_odometry_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheelbase', 0.185)
        self.declare_parameter('track_width', 0.15)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Latest sensor data
        self.latest_robot_state = None
        self.robot_state_received = False
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.twist_publisher = self.create_publisher(Twist, 'odom_twist', 10)
        
        # Create subscribers
        self.robot_state_subscription = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_callback, qos_profile)
        
        # Create timer for odometry publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.frame_id
        self.odom_msg.child_frame_id = self.child_frame_id
        
        # Set covariance matrices
        self.set_covariance_matrices()
        
        self.get_logger().info('Car Odometry Node started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.3f} m')
        self.get_logger().info(f'Track width: {self.track_width:.3f} m')
    
    def robot_state_callback(self, msg):
        self.latest_robot_state = msg
        self.robot_state_received = True
    
    def set_covariance_matrices(self):
        # Position covariance (6x6 matrix)
        # [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        self.odom_msg.pose.covariance[0] = 0.1   # xx
        self.odom_msg.pose.covariance[7] = 0.1   # yy
        self.odom_msg.pose.covariance[14] = 0.1   # zz
        self.odom_msg.pose.covariance[21] = 0.1   # roll
        self.odom_msg.pose.covariance[28] = 0.1   # pitch
        self.odom_msg.pose.covariance[35] = 0.1   # yaw
        
        # Velocity covariance (6x6 matrix)
        self.odom_msg.twist.covariance[0] = 0.1   # vx
        self.odom_msg.twist.covariance[7] = 0.1   # vy
        self.odom_msg.twist.covariance[14] = 0.1  # vz
        self.odom_msg.twist.covariance[21] = 0.1  # wx
        self.odom_msg.twist.covariance[28] = 0.1  # wy
        self.odom_msg.twist.covariance[35] = 0.1  # wz
    
    def publish_odometry(self):
        if not self.robot_state_received:
            return  # Don't publish if we don't have sensor data
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0.0:
            return  # Avoid division by zero
        
        # Calculate velocities from motor data
        right_rpm = self.latest_robot_state.right_motor_rpm
        left_rpm = self.latest_robot_state.left_motor_rpm
        
        # Convert RPM to linear velocities
        right_velocity = (right_rpm / 60.0) * 2.0 * math.pi * self.wheel_radius
        left_velocity = (left_rpm / 60.0) * 2.0 * math.pi * self.wheel_radius
        
        # Calculate linear and angular velocities
        linear_velocity = (right_velocity + left_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / self.track_width
        
        # Alternative: Use IMU for angular velocity (more accurate)
        if self.robot_state_received:
            angular_velocity = self.latest_robot_state.gyro_z
        
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

if __name__ == '__main__':
    main()
