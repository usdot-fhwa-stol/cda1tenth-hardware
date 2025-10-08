#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math
import time

class OdometryTestNode(Node):
    def __init__(self):
        super().__init__('odometry_test_node')
        
        # Create publishers to simulate sensor data
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.motor_publisher = self.create_publisher(Float32MultiArray, 'motor_data', 10)
        
        # Create subscriber to monitor odometry
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Test parameters
        self.test_duration = 10.0  # seconds
        self.start_time = time.time()
        self.test_phase = 0
        self.phase_start_time = 0
        
        # Create timer for test execution
        self.timer = self.create_timer(0.1, self.run_test)
        
        self.get_logger().info('Odometry Test Node started')
        self.get_logger().info('Running odometry tests...')
    
    def run_test(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > self.test_duration:
            self.get_logger().info('Test completed')
            return
        
        # Test phases
        if elapsed < 2.0:
            # Phase 1: Straight line forward
            self.publish_straight_forward()
        elif elapsed < 4.0:
            # Phase 2: Turn in place
            self.publish_turn_in_place()
        elif elapsed < 6.0:
            # Phase 3: Straight line backward
            self.publish_straight_backward()
        elif elapsed < 8.0:
            # Phase 4: Turn in opposite direction
            self.publish_turn_opposite()
        else:
            # Phase 5: Stop
            self.publish_stop()
    
    def publish_straight_forward(self):
        # Simulate forward motion
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = 0.0  # No rotation
        self.imu_publisher.publish(imu_msg)
        
        motor_msg = Float32MultiArray()
        motor_msg.data = [100.0, 100.0, 0.0, 0.0]  # Both wheels same speed
        self.motor_publisher.publish(motor_msg)
    
    def publish_turn_in_place(self):
        # Simulate turning in place
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = 0.5  # 0.5 rad/s rotation
        self.imu_publisher.publish(imu_msg)
        
        motor_msg = Float32MultiArray()
        motor_msg.data = [50.0, -50.0, 0.0, 0.0]  # Opposite wheel speeds
        self.motor_publisher.publish(motor_msg)
    
    def publish_straight_backward(self):
        # Simulate backward motion
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = 0.0  # No rotation
        self.imu_publisher.publish(imu_msg)
        
        motor_msg = Float32MultiArray()
        motor_msg.data = [-100.0, -100.0, 0.0, 0.0]  # Both wheels reverse
        self.motor_publisher.publish(motor_msg)
    
    def publish_turn_opposite(self):
        # Simulate turning in opposite direction
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = -0.5  # -0.5 rad/s rotation
        self.imu_publisher.publish(imu_msg)
        
        motor_msg = Float32MultiArray()
        motor_msg.data = [-50.0, 50.0, 0.0, 0.0]  # Opposite wheel speeds
        self.motor_publisher.publish(motor_msg)
    
    def publish_stop(self):
        # Simulate stopped
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = 0.0  # No rotation
        self.imu_publisher.publish(imu_msg)
        
        motor_msg = Float32MultiArray()
        motor_msg.data = [0.0, 0.0, 0.0, 0.0]  # No motion
        self.motor_publisher.publish(motor_msg)
    
    def odom_callback(self, msg):
        # Log odometry data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Calculate yaw from quaternion
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        self.get_logger().info(
            f'Odometry: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°, '
            f'v={linear_vel:.3f}, ω={angular_vel:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
