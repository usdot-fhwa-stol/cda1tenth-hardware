#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>

// Odometry state variables
typedef struct {
    float x;
    float y;
    float yaw;
    float wheel_radius;
    uint32_t last_update_ms;
    uint32_t publish_count;
} OdometryState;

// Odometry configuration
typedef struct {
    unsigned int period_ms;
    float wheel_radius;
} OdometryConfig;

// Global odometry state
extern OdometryState odom_state;
extern OdometryConfig odom_config;

// ROS2 objects
extern rcl_publisher_t odom_publisher;
extern rcl_timer_t odom_timer;

// Function declarations
bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
void odometry_fini(rcl_node_t* node);
void odometry_reset(float x, float y, float yaw_rad);
bool odometry_set_period_ms(unsigned int period_ms);
bool odometry_set_rate_hz(float hz);
void odometry_set_wheel_radius(float radius);
void odometry_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

// Helper functions
geometry_msgs__msg__Quaternion yaw_to_quaternion(float yaw_rad);
void send_test_odometry_message();

#endif // ODOMETRY_H
