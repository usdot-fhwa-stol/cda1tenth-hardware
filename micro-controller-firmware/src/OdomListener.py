import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, '/odom', self.callback, qos)

    def callback(self, msg: Odometry):
        self.get_logger().info(
            f"Pose=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}), "
            f"Yaw={msg.pose.pose.orientation.z:.2f}, "
            f"V={msg.twist.twist.linear.x:.2f} m/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
