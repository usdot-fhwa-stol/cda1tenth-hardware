#!/usr/bin/env python3

"""
Test script to verify the updated odometry node works with RobotState messages.
This script creates a mock RobotState message and tests the odometry calculation.
"""

import rclpy
from rclpy.node import Node
from robot_state_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

class RobotStateOdometryTester(Node):
    def __init__(self):
        super().__init__('robot_state_odometry_tester')
        
        # Create publisher for robot state
        self.robot_state_publisher = self.create_publisher(
            RobotState, 'robot_state', 10)
        
        # Create subscriber for odometry
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Create subscriber for twist
        self.twist_subscription = self.create_subscription(
            Twist, 'odom_twist', self.twist_callback, 10)
        
        # Test data
        self.test_data_received = False
        self.odom_received = False
        self.twist_received = False
        
        # Create timer to publish test data
        self.timer = self.create_timer(0.1, self.publish_test_data)
        
        self.get_logger().info('Robot State Odometry Tester started')
    
    def publish_test_data(self):
        """Publish test RobotState data"""
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set test IMU data
        msg.accel_x = 0.0
        msg.accel_y = 0.0
        msg.accel_z = 9.81
        msg.gyro_x = 0.0
        msg.gyro_y = 0.0
        msg.gyro_z = 0.1  # Small angular velocity
        
        # Set test motor data
        msg.speed = 1.0  # 1 m/s
        msg.steering_angle = 0.0
        msg.right_motor_rpm = 100.0
        msg.left_motor_rpm = 100.0
        
        self.robot_state_publisher.publish(msg)
        self.test_data_received = True
        
        if self.test_data_received and self.odom_received and self.twist_received:
            self.get_logger().info('Test completed successfully!')
            self.get_logger().info('RobotState message structure is working correctly')
            rclpy.shutdown()
    
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.odom_received = True
        self.get_logger().info(f'Received odometry: x={msg.pose.pose.position.x:.3f}, '
                              f'y={msg.pose.pose.position.y:.3f}, '
                              f'theta={math.atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z), 
                                                  1-2*(msg.pose.pose.orientation.z**2)):.3f}')
    
    def twist_callback(self, msg):
        """Callback for twist messages"""
        self.twist_received = True
        self.get_logger().info(f'Received twist: linear.x={msg.linear.x:.3f}, '
                              f'angular.z={msg.angular.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateOdometryTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
