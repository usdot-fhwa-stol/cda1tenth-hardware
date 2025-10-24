#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelFilter(Node):
    def __init__(self):
        super().__init__("cmd_vel_filter")

        # Set fixed publish rate (Hz)
        self.publish_rate_hz = 20.0  # Constant 20 Hz
        self.publish_period = 1.0 / self.publish_rate_hz

        # Initialize latest command (None means no cmd_vel received yet)
        self.latest_cmd = None

        # Subscriber and publisher
        self.subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel_filtered", 10)

        # Timer for constant publishing at 20 Hz
        self.timer = self.create_timer(self.publish_period, self.publish_loop)

        self.get_logger().info("CmdVelFilter running at constant 20 Hz")

    def cmd_vel_callback(self, msg: Twist):
        """Callback for incoming /cmd_vel messages."""
        self.latest_cmd = msg

    def publish_loop(self):
        """Publishes latest command or zero if nothing received."""
        msg = Twist()
        if self.latest_cmd is not None:
            msg.linear.x = self.latest_cmd.linear.x
            msg.linear.y = self.latest_cmd.linear.y
            msg.linear.z = self.latest_cmd.linear.z
            msg.angular.x = self.latest_cmd.angular.x
            msg.angular.y = self.latest_cmd.angular.y
            msg.angular.z = self.latest_cmd.angular.z
        else:
            # Publish zero velocities if no message received
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
