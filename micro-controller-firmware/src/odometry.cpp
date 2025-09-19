#include "odometry.h"

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

nav_msgs__msg__Odometry s_odom_msg;

namespace {

  rcl_publisher_t s_odom_pub;
  rcl_timer_t s_odom_timer;

  float carX = 0.0f;
  float carY = 0.0f;
  float carYaw = 0.0f;
  float wheel_radius = 0.03f;

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
    if (timer == nullptr) return;

    uint32_t now_ms = millis();
    float dt = (s_last_ms == 0) ? 0.0f : (now_ms - s_last_ms) / 1000.0f;
    s_last_ms = now_ms;

    float wz = IMU.readFloatGyroZ() * (M_PI / 180.0f);

    carYaw += wz * dt;

    float right_rpm = car.getRightMotorRPM();
    float left_rpm  = car.getLeftMotorRPM();
    float avg_rpm   = 0.5f * (right_rpm + left_rpm);

    float v = (avg_rpm / 60.0f) * 2.0f * M_PI * wheel_radius;

    carX += v * cosf(carYaw) * dt;
    carY += v * sinf(carYaw) * dt;

    nav_msgs__msg__Odometry odom;
    nav_msgs__msg__Odometry__init(&odom);

    rosidl_runtime_c__String__assign(&odom.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom.child_frame_id, "base_link");
    
    odom.pose.pose.position.x = carX;
    odom.pose.pose.position.y = carY;
    odom.pose.pose.orientation = yaw_to_quaternion(carYaw);

    odom.pose.covariance[0]  = 0.2;
    odom.pose.covariance[7]  = 0.2;
    odom.pose.covariance[35] = 0.4;

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = wz;

    (void) rcl_publish(&s_odom_pub, &odom, nullptr);

    nav_msgs__msg__Odometry__fini(&odom);
  }

}

bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor)
{
  rcl_ret_t rc = rclc_publisher_init_default(
      &s_odom_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom");
  if (rc != RCL_RET_OK) return false;

  nav_msgs__msg__Odometry__init(&s_odom_msg);

  const unsigned int period_ms = 20;
  rc = rclc_timer_init_default(&s_odom_timer, support, RCL_MS_TO_NS(period_ms), odom_timer_cb);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_executor_add_timer(executor, &s_odom_timer);
  return rc == RCL_RET_OK;
}

void odometry_fini(rcl_node_t* node)
{
  (void) rcl_timer_fini(&s_odom_timer);
  (void) rcl_publisher_fini(&s_odom_pub, node);
  nav_msgs__msg__Odometry__fini(&s_odom_msg);
}

void odometry_reset(float x, float y, float yaw_rad)
{
  carX = (double)x;
  carY = (double)y;
  carYaw = (double)yaw_rad;
}