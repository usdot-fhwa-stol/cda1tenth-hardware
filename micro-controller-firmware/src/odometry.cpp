#include "odometry.h"
#include "debug.h"

#include <math.h>
#include <string.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_platformio.h>
#include "car.h"
#include <rclc/executor.h>
#include <SparkFunLSM6DSO.h>

extern Car   car;
extern bool  car_initialized;
extern float g_wheelbase;   // meters
extern LSM6DSO IMU;
extern USBCDC USBSerial;

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
    
    // Debug counter
    odom_publish_count++;
    
    // Debug logging removed from odometry callback to prevent spam

    uint32_t now_ms = millis();
    float dt = (s_last_ms == 0) ? 0.0f : (now_ms - s_last_ms) / 1000.0f;
    s_last_ms = now_ms;

    // Use static cached values to avoid ANY blocking operations
    static float cached_gyro_z = 0.0f;
    static float cached_right_rpm = 0.0f;
    static float cached_left_rpm = 0.0f;
    static uint32_t last_sensor_read = 0;
    
    // Only read sensors every 200ms to minimize blocking
    if (now_ms - last_sensor_read > 200) {
      // These operations are still blocking but much less frequent
      cached_gyro_z = IMU.readFloatGyroZ() * (M_PI / 180.0f);
      // Use atomic reads to avoid mutex contention
      cached_right_rpm = car.getRightMotorRPMAtomic();
      cached_left_rpm = car.getLeftMotorRPMAtomic();
      last_sensor_read = now_ms;
    }
    
    carYaw += cached_gyro_z * dt;
    
    float avg_rpm = 0.5f * (cached_right_rpm + cached_left_rpm);
    float v = (avg_rpm / 60.0f) * 2.0f * M_PI * s_wheel_radius;

    carX += v * cosf(carYaw) * dt;
    carY += v * sinf(carYaw) * dt;

    // Use static message to avoid repeated allocation/deallocation
    static nav_msgs__msg__Odometry odom;
    static bool odom_initialized = false;
    
    if (!odom_initialized) {
      nav_msgs__msg__Odometry__init(&odom);
      rosidl_runtime_c__String__assign(&odom.header.frame_id, "odom");
      rosidl_runtime_c__String__assign(&odom.child_frame_id, "base_link");
      odom_initialized = true;
    }

    odom.pose.pose.position.x = carX;
    odom.pose.pose.position.y = carY;
    odom.pose.pose.orientation = yaw_to_quaternion(carYaw);

    odom.pose.covariance[0]  = 0.2;
    odom.pose.covariance[7]  = 0.2;
    odom.pose.covariance[35] = 0.4;

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = cached_gyro_z;

    (void) rcl_publish(&s_odom_pub, &odom, nullptr);
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
