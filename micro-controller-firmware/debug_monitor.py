#!/usr/bin/env python3
"""
Enhanced Debug Monitor for ESP32 micro-ROS system
Monitors debug_log topic with best effort QoS and provides detailed analysis
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import statistics

class DebugMonitor(Node):
    def __init__(self):
        super().__init__('debug_monitor')
        
        # Create subscription with best effort QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'debug_log',
            self.debug_callback,
            qos_profile
        )
        
        # Debug data storage
        self.debug_count = 0
        self.last_debug_time = time.time()
        self.loop_times = []
        self.ros_spin_times = []
        self.steering_times = []
        self.drive_times = []
        
        # Performance thresholds
        self.LOOP_TIME_WARNING = 20.0  # ms
        self.LOOP_TIME_CRITICAL = 50.0  # ms
        self.ROS_SPIN_WARNING = 10.0   # ms
        self.ROS_SPIN_CRITICAL = 20.0  # ms
        
        self.get_logger().info("Debug Monitor started - monitoring with best effort QoS")
        
    def debug_callback(self, msg):
        now = time.time()
        self.debug_count += 1
        
        # Extract debug data
        data = msg.data
        
        # Basic metrics
        twist_callbacks = data[0]
        odom_publishes = data[1]
        state = data[2]
        free_heap = data[3]
        total_heap = data[4]
        
        # Motor/sensor data
        right_rpm = data[5]
        left_rpm = data[6]
        gyro_z = data[7]
        odom_x = data[8]
        odom_y = data[9]
        
        # Performance metrics
        max_loop_time = data[10]
        avg_loop_time = data[11]
        cpu_freq = data[12]
        cpu_cycles = data[13]
        free_psram = data[14]
        total_psram = data[15]
        temperature = data[16]
        reset_reason = data[17]
        
        # Timing breakdown
        max_ros_spin_time = data[18]
        max_sensor_time = data[19]
        max_control_time = data[20]
        
        # Odometry status
        odom_success = data[21]
        odom_failures = data[22]
        publisher_init = data[23]
        
        # Detailed timing (new metrics)
        max_steering_time = data[24] if len(data) > 24 else 0
        max_drive_time = data[25] if len(data) > 25 else 0
        max_odom_time = data[26] if len(data) > 26 else 0
        max_debug_publish_time = data[27] if len(data) > 27 else 0
        max_led_update_time = data[28] if len(data) > 28 else 0
        
        # State tracking (new metrics)
        state_transitions = data[29] if len(data) > 29 else 0
        connection_failures = data[30] if len(data) > 30 else 0
        executor_timeouts = data[31] if len(data) > 31 else 0
        
        # System metrics (new metrics)
        num_tasks = data[32] if len(data) > 32 else 0
        stack_high_water = data[33] if len(data) > 33 else 0
        uptime = data[34] if len(data) > 34 else 0
        
        # Store for analysis
        self.loop_times.append(max_loop_time)
        self.ros_spin_times.append(max_ros_spin_time)
        self.steering_times.append(max_steering_time)
        self.drive_times.append(max_drive_time)
        
        # Keep only last 100 samples
        if len(self.loop_times) > 100:
            self.loop_times.pop(0)
            self.ros_spin_times.pop(0)
            self.steering_times.pop(0)
            self.drive_times.pop(0)
        
        # Print every 5th message to avoid spam
        if self.debug_count % 5 == 0:
            self.print_status(data)
            
        # Check for critical issues
        self.check_critical_issues(max_loop_time, max_ros_spin_time, executor_timeouts, connection_failures)
        
        self.last_debug_time = now
    
    def print_status(self, data):
        """Print formatted debug status"""
        print(f"\n=== Debug Status #{self.debug_count} ===")
        
        # System state
        state_names = {0: "WAITING_AGENT", 1: "AGENT_AVAILABLE", 2: "AGENT_CONNECTED", 3: "AGENT_DISCONNECTED"}
        state_name = state_names.get(int(data[2]), f"UNKNOWN({data[2]})")
        print(f"State: {state_name}")
        
        # Performance metrics
        print(f"Loop Time: {data[10]:.1f}ms (max), {data[11]:.1f}ms (avg)")
        print(f"ROS Spin: {data[18]:.1f}ms")
        print(f"Steering: {data[24]:.1f}ms, Drive: {data[25]:.1f}ms")
        
        # Odometry status
        print(f"Odometry: {data[21]:.0f} success, {data[22]:.0f} failures")
        
        # System health
        heap_usage = ((data[4] - data[3]) / data[4]) * 100 if data[4] > 0 else 0
        print(f"Memory: {data[3]:.0f}/{data[4]:.0f} bytes ({heap_usage:.1f}% used)")
        
        # New metrics
        print(f"State Transitions: {data[29]:.0f}, Failures: {data[30]:.0f}")
        print(f"Executor Timeouts: {data[31]:.0f}")
        print(f"Tasks: {data[32]:.0f}, Stack: {data[33]:.0f}")
        print(f"Uptime: {data[34]:.0f}s")
        
        # Performance trends
        if len(self.loop_times) >= 10:
            avg_loop = statistics.mean(self.loop_times[-10:])
            avg_ros = statistics.mean(self.ros_spin_times[-10:])
            print(f"Trends: Loop {avg_loop:.1f}ms, ROS {avg_ros:.1f}ms (last 10)")
        
        print("=" * 40)
    
    def check_critical_issues(self, loop_time, ros_spin_time, executor_timeouts, connection_failures):
        """Check for critical performance issues"""
        issues = []
        
        # Loop time issues
        if loop_time > self.LOOP_TIME_CRITICAL:
            issues.append(f"CRITICAL: Loop time {loop_time:.1f}ms > {self.LOOP_TIME_CRITICAL}ms")
        elif loop_time > self.LOOP_TIME_WARNING:
            issues.append(f"WARNING: Loop time {loop_time:.1f}ms > {self.LOOP_TIME_WARNING}ms")
        
        # ROS spin time issues
        if ros_spin_time > self.ROS_SPIN_CRITICAL:
            issues.append(f"CRITICAL: ROS spin {ros_spin_time:.1f}ms > {self.ROS_SPIN_CRITICAL}ms")
        elif ros_spin_time > self.ROS_SPIN_WARNING:
            issues.append(f"WARNING: ROS spin {ros_spin_time:.1f}ms > {self.ROS_SPIN_WARNING}ms")
        
        # Executor timeout issues
        if executor_timeouts > 0:
            issues.append(f"ISSUE: {executor_timeouts} executor timeouts")
        
        # Connection issues
        if connection_failures > 0:
            issues.append(f"ISSUE: {connection_failures} connection failures")
        
        # Print critical issues
        if issues:
            print(f"\n🚨 CRITICAL ISSUES DETECTED:")
            for issue in issues:
                print(f"  - {issue}")
            print()
    
    def get_performance_summary(self):
        """Get performance summary statistics"""
        if not self.loop_times:
            return "No data available"
        
        return {
            'loop_time': {
                'current': self.loop_times[-1] if self.loop_times else 0,
                'avg': statistics.mean(self.loop_times),
                'max': max(self.loop_times),
                'min': min(self.loop_times)
            },
            'ros_spin_time': {
                'current': self.ros_spin_times[-1] if self.ros_spin_times else 0,
                'avg': statistics.mean(self.ros_spin_times),
                'max': max(self.ros_spin_times),
                'min': min(self.ros_spin_times)
            },
            'samples': len(self.loop_times)
        }

def main():
    rclpy.init()
    
    try:
        monitor = DebugMonitor()
        
        print("🔍 Enhanced Debug Monitor for ESP32 micro-ROS")
        print("Monitoring debug_log topic with best effort QoS")
        print("Press Ctrl+C to stop")
        print()
        
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\n🛑 Debug monitor stopped")
        
        # Print final summary
        if hasattr(monitor, 'get_performance_summary'):
            summary = monitor.get_performance_summary()
            if isinstance(summary, dict):
                print("\n📊 Performance Summary:")
                print(f"  Loop Time: {summary['loop_time']['avg']:.1f}ms avg, {summary['loop_time']['max']:.1f}ms max")
                print(f"  ROS Spin: {summary['ros_spin_time']['avg']:.1f}ms avg, {summary['ros_spin_time']['max']:.1f}ms max")
                print(f"  Samples: {summary['samples']}")
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
