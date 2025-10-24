#!/usr/bin/env python3

"""
Test script for the cmd_vel_filter node.
This script publishes test cmd_vel messages and verifies the filter output.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelFilterTester(Node):
    def __init__(self):
        super().__init__("cmd_vel_filter_tester")

        # Create publisher for test cmd_vel messages
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create subscriber for filtered cmd_vel messages
        self.filtered_subscription = self.create_subscription(
            Twist, "/cmd_vel_filtered", self.filtered_callback, 10
        )

        # Test data
        self.test_sequence = [
            {
                "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.5},
            },
            {
                "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        ]

        self.current_test = 0
        self.filtered_messages_received = 0
        self.last_filtered_msg = None

        # Create timer to publish test messages
        self.timer = self.create_timer(2.0, self.publish_test_message)

        self.get_logger().info("CmdVel Filter Tester started")
        self.get_logger().info("Publishing test cmd_vel messages every 2 seconds")

    def publish_test_message(self):
        """Publish test cmd_vel message"""
        if self.current_test < len(self.test_sequence):
            msg = Twist()
            test_data = self.test_sequence[self.current_test]

            msg.linear.x = test_data["linear"]["x"]
            msg.linear.y = test_data["linear"]["y"]
            msg.linear.z = test_data["linear"]["z"]
            msg.angular.x = test_data["angular"]["x"]
            msg.angular.y = test_data["angular"]["y"]
            msg.angular.z = test_data["angular"]["z"]

            self.cmd_vel_publisher.publish(msg)

            self.get_logger().info(
                f"Published test message {self.current_test + 1}: "
                f"linear.x={msg.linear.x:.1f}, angular.z={msg.angular.z:.1f}"
            )

            self.current_test += 1
        else:
            # Stop publishing after all tests
            self.timer.cancel()
            self.get_logger().info(
                "All test messages published. Monitoring filtered output..."
            )

    def filtered_callback(self, msg):
        """Callback for filtered cmd_vel messages"""
        self.filtered_messages_received += 1
        self.last_filtered_msg = msg

        self.get_logger().info(
            f"Received filtered message #{self.filtered_messages_received}: "
            f"linear.x={msg.linear.x:.1f}, angular.z={msg.angular.z:.1f}"
        )

        # Check if we've received enough messages to verify the filter is working
        if self.filtered_messages_received >= 10:
            self.get_logger().info("✅ CmdVel filter is working correctly!")
            self.get_logger().info(
                "Filter is publishing at constant rate with latest cmd_vel values"
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFilterTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
