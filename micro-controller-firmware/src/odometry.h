#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

extern nav_msgs__msg__Odometry s_odom_msg;

bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);

void odometry_fini(rcl_node_t* node);

void odometry_reset(float x, float y, float yaw_rad);

void odometry_set_period_ms(unsigned int period_ms);
void odometry_set_wheel_radius(float r);