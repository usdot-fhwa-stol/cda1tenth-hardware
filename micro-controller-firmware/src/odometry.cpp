#include "odometry.h"
#include <math.h>

Odometry::Odometry()
{
    // Initialize with default values
    wheelbase_ = 0.3f;
    track_width_ = 0.2f;
    wheel_radius_ = 0.05f;

    // Reset state
    reset();

    // Initialize odometry message
    nav_msgs__msg__Odometry__init(&odom_msg_);

    // Set frame IDs
    odom_msg_.header.frame_id.data = (char *)"odom";
    odom_msg_.header.frame_id.size = 4;
    odom_msg_.header.frame_id.capacity = 4;

    odom_msg_.child_frame_id.data = (char *)"base_link";
    odom_msg_.child_frame_id.size = 9;
    odom_msg_.child_frame_id.capacity = 9;
}

Odometry::~Odometry()
{
    nav_msgs__msg__Odometry__fini(&odom_msg_);
}

void Odometry::initialize(float wheelbase, float track_width, float wheel_radius)
{
    wheelbase_ = wheelbase;
    track_width_ = track_width;
    wheel_radius_ = wheel_radius;
    reset();
}

void Odometry::update(float left_rpm, float right_rpm, float steering_angle, float dt)
{
    // Convert RPM to linear velocities (m/s)
    float left_velocity = (left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float right_velocity = (right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;

    // Update velocity
    updateVelocity(left_velocity, right_velocity);

    // Update pose using differential drive kinematics
    updatePose(left_velocity, right_velocity, dt);

    // Store current values for next iteration
    prev_left_rpm_ = left_rpm;
    prev_right_rpm_ = right_rpm;
    prev_theta_ = theta_;
    prev_time_ = millis();
}

void Odometry::getPose(float &x, float &y, float &theta)
{
    x = x_;
    y = y_;
    theta = theta_;
}

void Odometry::getVelocity(float &linear_x, float &angular_z)
{
    linear_x = linear_x_;
    angular_z = angular_z_;
}

void Odometry::reset()
{
    x_ = 0.0f;
    y_ = 0.0f;
    theta_ = 0.0f;
    linear_x_ = 0.0f;
    angular_z_ = 0.0f;

    prev_left_rpm_ = 0.0f;
    prev_right_rpm_ = 0.0f;
    prev_theta_ = 0.0f;
    prev_time_ = millis();
}

nav_msgs__msg__Odometry *Odometry::getOdometryMessage()
{
    // Update message with current state
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.position.z = 0.0f;

    // Convert theta to quaternion
    odom_msg_.pose.pose.orientation.x = 0.0f;
    odom_msg_.pose.pose.orientation.y = 0.0f;
    odom_msg_.pose.pose.orientation.z = sinf(theta_ / 2.0f);
    odom_msg_.pose.pose.orientation.w = cosf(theta_ / 2.0f);

    // Set velocity
    odom_msg_.twist.twist.linear.x = linear_x_;
    odom_msg_.twist.twist.linear.y = 0.0f;
    odom_msg_.twist.twist.linear.z = 0.0f;
    odom_msg_.twist.twist.angular.x = 0.0f;
    odom_msg_.twist.twist.angular.y = 0.0f;
    odom_msg_.twist.twist.angular.z = angular_z_;

    // Set covariance matrices (simplified - could be more sophisticated)
    // Position covariance (6x6 matrix)
    for (int i = 0; i < 36; i++)
    {
        odom_msg_.pose.covariance[i] = 0.0f;
    }
    odom_msg_.pose.covariance[0] = 0.01f;  // x variance
    odom_msg_.pose.covariance[7] = 0.01f;  // y variance
    odom_msg_.pose.covariance[35] = 0.01f; // theta variance

    // Velocity covariance (6x6 matrix)
    for (int i = 0; i < 36; i++)
    {
        odom_msg_.twist.covariance[i] = 0.0f;
    }
    odom_msg_.twist.covariance[0] = 0.01f;  // linear x variance
    odom_msg_.twist.covariance[35] = 0.01f; // angular z variance

    return &odom_msg_;
}

void Odometry::updatePose(float left_velocity, float right_velocity, float dt)
{
    // Differential drive kinematics
    float linear_velocity = (left_velocity + right_velocity) / 2.0f;
    float angular_velocity = (right_velocity - left_velocity) / track_width_;

    // Update orientation
    theta_ += angular_velocity * dt;
    normalizeAngle(theta_);

    // Update position
    x_ += linear_velocity * cosf(theta_) * dt;
    y_ += linear_velocity * sinf(theta_) * dt;
}

void Odometry::updateVelocity(float left_velocity, float right_velocity)
{
    // Calculate current velocities
    linear_x_ = (left_velocity + right_velocity) / 2.0f;
    angular_z_ = (right_velocity - left_velocity) / track_width_;
}

void Odometry::normalizeAngle(float &angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0f * M_PI;
    }
}
