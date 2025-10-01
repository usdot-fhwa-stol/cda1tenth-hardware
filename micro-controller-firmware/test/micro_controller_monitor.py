#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class BoardMonitor(Node):
    def __init__(self):
        super().__init__('board_monitor')
        
        # Subscribe to performance topics
        self.health_sub = self.create_subscription(
            Float32MultiArray, '/system_health', self.health_callback, 10)
        self.metrics_sub = self.create_subscription(
            Float32MultiArray, '/system_metrics', self.metrics_callback, 10)
        self.performance_sub = self.create_subscription(
            Float32MultiArray, '/system_performance', self.performance_callback, 10)
            
        # Create timer for periodic reporting
        self.timer = self.create_timer(5.0, self.print_summary)
        
        self.latest_health = None
        self.latest_metrics = None
        self.latest_performance = None
        
    def health_callback(self, msg):
        self.latest_health = msg.data
        
    def metrics_callback(self, msg):
        self.latest_metrics = msg.data
        
    def performance_callback(self, msg):
        self.latest_performance = msg.data
        
    def print_summary(self):
        print("\n" + "="*60)
        print("ESP32 BOARD PERFORMANCE SUMMARY")
        print("="*60)
        
        if self.latest_health:
            print(f"Overall Status: {self.status_to_string(int(self.latest_health[0]))}")
            print(f"Health Score: {self.latest_health[7]:.1%}")
            print(f"Uptime: {int(self.latest_health[8])}s")
            
        if self.latest_performance:
            print(f"CPU Usage: {self.latest_performance[0]:.1f}%")
            print(f"Free Memory: {self.latest_performance[1]:.1f} KB")
            print(f"Loop Time: {self.latest_performance[2]:.1f}ms (max: {self.latest_performance[3]:.1f}ms)")
            print(f"Loop Overruns: {int(self.latest_performance[4])}")
            
        if self.latest_metrics:
            print(f"Twist Rate: {int(self.latest_metrics[0])} Hz")
            print(f"Odom Rate: {int(self.latest_metrics[1])} Hz")
            print(f"Control Freq: {int(self.latest_metrics[2])} Hz")
            print(f"SPI Errors: {int(self.latest_metrics[6])}")
            print(f"Connection Drops: {int(self.latest_metrics[7])}")
            
            # Odometry-specific metrics
            if len(self.latest_metrics) >= 20:
                print(f"Odom Stale Data: {int(self.latest_metrics[15])}")
                print(f"Odom Invalid Data: {int(self.latest_metrics[16])}")
                print(f"Odom Calc Time: {self.latest_metrics[18]:.2f}ms")
                
    def status_to_string(self, status):
        status_map = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
        return status_map.get(status, "UNKNOWN")

def main():
    rclpy.init()
    monitor = BoardMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
