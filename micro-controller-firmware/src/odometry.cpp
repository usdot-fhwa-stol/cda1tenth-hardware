#include "odometry.h"
#include "debug.h"
#include "sensor_cache.h"
#include "debug_manager.h"

#include <math.h>
#include <string.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_platformio.h>
#include "car.h"
#include <rclc/executor.h>

extern Car   car;
extern bool  car_initialized;
extern float g_wheelbase;   // meters
extern SensorCache g_sensor_cache;  // Use global sensor cache instance

namespace {

  rcl_publisher_t s_odom_pub;
  rcl_timer_t s_odom_timer;
  rclc_support_t* s_support = nullptr;
  rclc_executor_t* s_executor = nullptr;
  bool s_inited = false;

  float carX = 0.0f;
  float carY = 0.0f;
  float carYaw = 0.0f;

  float s_wheel_radius = 0.03f;
  unsigned int s_period_ms = 20;

  uint32_t s_last_ms = 0;
  
  // Performance monitoring variables
  struct OdometryMetrics {
    uint32_t publish_count = 0;
    uint32_t stale_data_count = 0;
    uint32_t invalid_data_count = 0;
    uint32_t calculation_errors = 0;
    float last_calculation_time_ms = 0.0f;
    float max_calculation_time_ms = 0.0f;
    uint32_t last_metrics_reset = 0;
  } s_odom_metrics;

  geometry_msgs__msg__Quaternion yaw_to_quaternion(float yaw_rad) {
    geometry_msgs__msg__Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sinf(yaw_rad / 2.0f);
    q.w = cosf(yaw_rad / 2.0f);
    return q;
  }

  void odom_timer_cb(rcl_timer_t *timer, int64_t) {
    if (!timer || !car_initialized) return;
    
    uint32_t calc_start_time = micros();
    uint32_t now_ms = millis();
    
    // Calculate delta time
    float dt = (s_last_ms == 0) ? 0.0f : (now_ms - s_last_ms) / 1000.0f;
    s_last_ms = now_ms;
    
    // Skip calculation if dt is too large (indicates system pause/reset)
    if (dt > 1.0f) {
      s_odom_metrics.calculation_errors++;
      g_debug_manager.recordOdometryError();
      return;
    }

    // Get cached sensor data (non-blocking)
    SensorData sensor_data = g_sensor_cache.getCachedData();
    
    // Check data validity and freshness
    bool data_valid = sensor_data.valid && g_sensor_cache.isDataFresh(100); // 100ms max age
    
    if (!data_valid) {
      s_odom_metrics.invalid_data_count++;
      g_debug_manager.recordOdometryInvalidData();
      
      // Use fallback: continue with last known good values but don't update position
      if (!g_sensor_cache.isDataFresh(500)) { // If data is very stale (>500ms)
        s_odom_metrics.stale_data_count++;
        g_debug_manager.recordOdometryStaleData();
        DEBUG_WARN("Odometry: Sensor data too stale, skipping update");
        return;
      }
    }
    
    // Extract sensor values with error handling
    float gyro_z_rad = 0.0f;
    float right_rpm = 0.0f;
    float left_rpm = 0.0f;
    
    if (data_valid) {
      gyro_z_rad = sensor_data.gyro_z * (M_PI / 180.0f); // Convert to rad/s
      right_rpm = sensor_data.right_rpm;
      left_rpm = sensor_data.left_rpm;
      
      // Sanity checks on sensor data
      if (isnan(gyro_z_rad) || isnan(right_rpm) || isnan(left_rpm)) {
        s_odom_metrics.calculation_errors++;
        g_debug_manager.recordOdometryError();
        DEBUG_WARN("Odometry: NaN values in sensor data");
        return;
      }
      
      // Limit gyro rate to reasonable values
      if (fabs(gyro_z_rad) > 10.0f) { // 10 rad/s max
        gyro_z_rad = copysignf(10.0f, gyro_z_rad);
      }
      
      // Limit RPM to reasonable values
      if (fabs(right_rpm) > 5000.0f) right_rpm = copysignf(5000.0f, right_rpm);
      if (fabs(left_rpm) > 5000.0f) left_rpm = copysignf(5000.0f, left_rpm);
    }
    
    // Update orientation using gyroscope
    carYaw += gyro_z_rad * dt;
    
    // Normalize yaw to [-π, π]
    while (carYaw > M_PI) carYaw -= 2.0f * M_PI;
    while (carYaw < -M_PI) carYaw += 2.0f * M_PI;
    
    // Calculate linear velocity from wheel encoders
    float avg_rpm = 0.5f * (right_rpm + left_rpm);
    float linear_velocity = (avg_rpm / 60.0f) * 2.0f * M_PI * s_wheel_radius;
    
    // Limit velocity to reasonable values
    if (fabs(linear_velocity) > 5.0f) { // 5 m/s max
      linear_velocity = copysignf(5.0f, linear_velocity);
    }
    
    // Update position using velocity and orientation
    carX += linear_velocity * cosf(carYaw) * dt;
    carY += linear_velocity * sinf(carYaw) * dt;

    // Use static message to avoid repeated allocation/deallocation
    static nav_msgs__msg__Odometry odom;
    static bool odom_initialized = false;
    
    if (!odom_initialized) {
      nav_msgs__msg__Odometry__init(&odom);
      rosidl_runtime_c__String__assign(&odom.header.frame_id, "odom");
      rosidl_runtime_c__String__assign(&odom.child_frame_id, "base_link");
      odom_initialized = true;
    }

    // Fill in odometry message
    odom.header.stamp.sec = now_ms / 1000;
    odom.header.stamp.nanosec = (now_ms % 1000) * 1000000;
    
    odom.pose.pose.position.x = carX;
    odom.pose.pose.position.y = carY;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yaw_to_quaternion(carYaw);

    // Set covariance based on data quality
    float pos_covariance = data_valid ? 0.2f : 0.8f;  // Higher uncertainty for invalid data
    float yaw_covariance = data_valid ? 0.4f : 1.6f;
    
    // Reset covariance matrix
    for (int i = 0; i < 36; i++) {
      odom.pose.covariance[i] = 0.0;
    }
    odom.pose.covariance[0]  = pos_covariance;  // x
    odom.pose.covariance[7]  = pos_covariance;  // y
    odom.pose.covariance[35] = yaw_covariance;  // yaw

    odom.twist.twist.linear.x  = linear_velocity;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = gyro_z_rad;

    // Reset twist covariance matrix
    for (int i = 0; i < 36; i++) {
      odom.twist.covariance[i] = 0.0;
    }
    odom.twist.covariance[0]  = data_valid ? 0.1f : 0.4f;  // linear x
    odom.twist.covariance[35] = data_valid ? 0.2f : 0.8f;  // angular z

    // Publish odometry message
    rcl_ret_t ret = rcl_publish(&s_odom_pub, &odom, nullptr);
    
    // Update performance metrics
    uint32_t calc_end_time = micros();
    float calc_time_ms = (calc_end_time - calc_start_time) / 1000.0f;
    
    s_odom_metrics.last_calculation_time_ms = calc_time_ms;
    if (calc_time_ms > s_odom_metrics.max_calculation_time_ms) {
      s_odom_metrics.max_calculation_time_ms = calc_time_ms;
    }
    
    if (ret == RCL_RET_OK) {
      s_odom_metrics.publish_count++;
      RECORD_ODOM_PUBLISH();  // Record successful publish for debug manager
      g_debug_manager.recordOdometryCalculationTime(calc_time_ms);
    } else {
      s_odom_metrics.calculation_errors++;
      g_debug_manager.recordOdometryError();
      DEBUG_WARN("Odometry: Failed to publish message");
    }
  }
} // namespace

bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor)
{
  s_support  = support;
  s_executor = executor;

  if (rclc_publisher_init_default(
        &s_odom_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(
        &s_odom_timer, support, RCL_MS_TO_NS(s_period_ms), odom_timer_cb) != RCL_RET_OK) return false;

  if (rclc_executor_add_timer(executor, &s_odom_timer) != RCL_RET_OK) return false;

  s_inited = true;
  s_last_ms = 0;
  return true;
}

void odometry_fini(rcl_node_t* node)
{
  (void) rcl_timer_fini(&s_odom_timer);
  (void) rcl_publisher_fini(&s_odom_pub, node);
  s_inited = false;
}

void odometry_reset(float x, float y, float yaw_rad)
{
  carX = x;
  carY = y;
  carYaw = yaw_rad;
  s_last_ms = 0;
}

void odometry_set_period_ms(unsigned int period_ms)
{
  s_period_ms = period_ms;
  if (s_inited && s_support && s_executor) {
    (void) rcl_timer_fini(&s_odom_timer);
    (void) rclc_timer_init_default(&s_odom_timer, s_support, RCL_MS_TO_NS(s_period_ms), odom_timer_cb);
    (void) rclc_executor_add_timer(s_executor, &s_odom_timer);
    s_last_ms = 0;
  }
}

void odometry_set_wheel_radius(float r)
{
  s_wheel_radius = r;
}

// Performance monitoring functions
void odometry_get_performance_metrics(OdometryPerformanceMetrics* metrics) {
  if (metrics == nullptr) return;
  
  uint32_t now = millis();
  uint32_t time_since_reset = now - s_odom_metrics.last_metrics_reset;
  
  metrics->total_publishes = s_odom_metrics.publish_count;
  metrics->stale_data_events = s_odom_metrics.stale_data_count;
  metrics->invalid_data_events = s_odom_metrics.invalid_data_count;
  metrics->calculation_errors = s_odom_metrics.calculation_errors;
  metrics->last_calculation_time_ms = s_odom_metrics.last_calculation_time_ms;
  metrics->max_calculation_time_ms = s_odom_metrics.max_calculation_time_ms;
  
  // Calculate publish rate
  if (time_since_reset > 0) {
    metrics->publish_rate_hz = (float)s_odom_metrics.publish_count / (time_since_reset / 1000.0f);
  } else {
    metrics->publish_rate_hz = 0.0f;
  }
  
  // Calculate error rates
  if (s_odom_metrics.publish_count > 0) {
    metrics->stale_data_rate = (float)s_odom_metrics.stale_data_count / s_odom_metrics.publish_count;
    metrics->invalid_data_rate = (float)s_odom_metrics.invalid_data_count / s_odom_metrics.publish_count;
    metrics->error_rate = (float)s_odom_metrics.calculation_errors / s_odom_metrics.publish_count;
  } else {
    metrics->stale_data_rate = 0.0f;
    metrics->invalid_data_rate = 0.0f;
    metrics->error_rate = 0.0f;
  }
  
  metrics->data_quality_score = calculate_odometry_data_quality();
}

void odometry_reset_performance_metrics() {
  s_odom_metrics.publish_count = 0;
  s_odom_metrics.stale_data_count = 0;
  s_odom_metrics.invalid_data_count = 0;
  s_odom_metrics.calculation_errors = 0;
  s_odom_metrics.max_calculation_time_ms = 0.0f;
  s_odom_metrics.last_metrics_reset = millis();
}

float calculate_odometry_data_quality() {
  if (s_odom_metrics.publish_count == 0) return 1.0f;
  
  float quality_score = 1.0f;
  
  // Penalize for stale data
  float stale_rate = (float)s_odom_metrics.stale_data_count / s_odom_metrics.publish_count;
  quality_score -= stale_rate * 0.3f;
  
  // Penalize for invalid data
  float invalid_rate = (float)s_odom_metrics.invalid_data_count / s_odom_metrics.publish_count;
  quality_score -= invalid_rate * 0.4f;
  
  // Penalize for calculation errors
  float error_rate = (float)s_odom_metrics.calculation_errors / s_odom_metrics.publish_count;
  quality_score -= error_rate * 0.5f;
  
  // Penalize for slow calculations
  if (s_odom_metrics.last_calculation_time_ms > 10.0f) {
    quality_score -= 0.2f;
  }
  
  // Ensure score is between 0 and 1
  if (quality_score < 0.0f) quality_score = 0.0f;
  if (quality_score > 1.0f) quality_score = 1.0f;
  
  return quality_score;
}

bool odometry_validate_accuracy(float expected_x, float expected_y, float expected_yaw, 
                               float position_tolerance, float yaw_tolerance) {
  float pos_error = sqrtf((carX - expected_x) * (carX - expected_x) + 
                         (carY - expected_y) * (carY - expected_y));
  
  float yaw_error = fabs(carYaw - expected_yaw);
  // Handle yaw wraparound
  if (yaw_error > M_PI) {
    yaw_error = 2.0f * M_PI - yaw_error;
  }
  
  return (pos_error <= position_tolerance) && (yaw_error <= yaw_tolerance);
}

void odometry_get_current_pose(float* x, float* y, float* yaw) {
  if (x) *x = carX;
  if (y) *y = carY;
  if (yaw) *yaw = carYaw;
}
