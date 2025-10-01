#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

// Performance metrics structure for odometry monitoring
struct OdometryPerformanceMetrics {
    uint32_t total_publishes;
    uint32_t stale_data_events;
    uint32_t invalid_data_events;
    uint32_t calculation_errors;
    float last_calculation_time_ms;
    float max_calculation_time_ms;
    float publish_rate_hz;
    float stale_data_rate;
    float invalid_data_rate;
    float error_rate;
    float data_quality_score;  // 0.0 to 1.0, higher is better
};

// Core odometry functions
bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
void odometry_fini(rcl_node_t* node);
void odometry_reset(float x, float y, float yaw_rad);
void odometry_set_period_ms(unsigned int period_ms);
void odometry_set_wheel_radius(float r);

// Performance monitoring functions
void odometry_get_performance_metrics(OdometryPerformanceMetrics* metrics);
void odometry_reset_performance_metrics();
float calculate_odometry_data_quality();

// Accuracy validation functions
bool odometry_validate_accuracy(float expected_x, float expected_y, float expected_yaw, 
                               float position_tolerance, float yaw_tolerance);
void odometry_get_current_pose(float* x, float* y, float* yaw);