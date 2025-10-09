#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/pose_with_covariance.h>
#include <geometry_msgs/msg/twist_with_covariance.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/quaternion.h>

class Odometry
{
public:
    Odometry();
    ~Odometry();

    // Initialize odometry with robot parameters
    void initialize(float wheelbase, float track_width, float wheel_radius);
    
    // Update odometry with current sensor data
    void update(float left_rpm, float right_rpm, float steering_angle, float dt);
    
    // Get current pose
    void getPose(float& x, float& y, float& theta);
    
    // Get current velocity
    void getVelocity(float& linear_x, float& angular_z);
    
    // Reset odometry to origin
    void reset();
    
    // Get odometry message for publishing
    nav_msgs__msg__Odometry* getOdometryMessage();

private:
    // Robot parameters
    float wheelbase_;
    float track_width_;
    float wheel_radius_;
    
    // Current state
    float x_;           // Position X (meters)
    float y_;           // Position Y (meters)
    float theta_;       // Orientation (radians)
    float linear_x_;    // Linear velocity (m/s)
    float angular_z_;   // Angular velocity (rad/s)
    
    // Previous values for integration
    float prev_left_rpm_;
    float prev_right_rpm_;
    float prev_theta_;
    unsigned long prev_time_;
    
    // Odometry message
    nav_msgs__msg__Odometry odom_msg_;
    
    // Helper functions
    void updatePose(float left_velocity, float right_velocity, float dt);
    void updateVelocity(float left_velocity, float right_velocity);
    void normalizeAngle(float& angle);
};

#endif // ODOMETRY_H
