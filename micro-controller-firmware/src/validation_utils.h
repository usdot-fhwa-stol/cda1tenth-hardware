#ifndef VALIDATION_UTILS_H
#define VALIDATION_UTILS_H

#include <math.h>
#include <stdbool.h>
#include "system_config.h"

// Forward declarations for ROS message types
struct geometry_msgs__msg__Twist;
struct nav_msgs__msg__Odometry;
struct geometry_msg__msg__VehicleGeometry;

// =============================================================================
// COMMON VALIDATION UTILITIES
// =============================================================================

namespace ValidationUtils {
    // Check if a float value is valid (not NaN or infinite)
    inline bool isValidFloat(float value) {
        return !isnan(value) && !isinf(value);
    }
    
    // Check if multiple float values are valid
    inline bool areValidFloats(float value1, float value2) {
        return isValidFloat(value1) && isValidFloat(value2);
    }
    
    inline bool areValidFloats(float value1, float value2, float value3) {
        return isValidFloat(value1) && isValidFloat(value2) && isValidFloat(value3);
    }
    
    inline bool areValidFloats(float value1, float value2, float value3, float value4) {
        return isValidFloat(value1) && isValidFloat(value2) && isValidFloat(value3) && isValidFloat(value4);
    }
    
    // Check if a value is within a range
    inline bool isInRange(float value, float min, float max) {
        return value >= min && value <= max;
    }
    
    // Check if a value is within absolute range
    inline bool isInAbsoluteRange(float value, float maxAbs) {
        return fabsf(value) <= maxAbs;
    }
    
    // Check if a value is positive
    inline bool isPositive(float value) {
        return value > 0.0f;
    }
    
    // Check if a value is non-negative
    inline bool isNonNegative(float value) {
        return value >= 0.0f;
    }
}

// =============================================================================
// MOTOR COMMAND VALIDATION
// =============================================================================

namespace MotorValidation {
    // Unified motor command validation
    inline bool validateMotorCommand(const MotorCommand_t& cmd) {
        // Check for NaN values
        if (!ValidationUtils::areValidFloats(cmd.steering_angle, cmd.drive_speed, 
                                           cmd.wheelbase, cmd.track_width)) {
            return false;
        }
        
        // Check steering angle limits
        if (!ValidationUtils::isInAbsoluteRange(cmd.steering_angle, MAX_STEERING_ANGLE_COMMAND)) {
            return false;
        }
        
        // Check drive speed limits
        if (!ValidationUtils::isInAbsoluteRange(cmd.drive_speed, MAX_DRIVE_SPEED)) {
            return false;
        }
        
        // Check wheelbase and track width are positive
        if (!ValidationUtils::isPositive(cmd.wheelbase) || !ValidationUtils::isPositive(cmd.track_width)) {
            return false;
        }
        
        // Check maximum limits
        if (cmd.wheelbase > MAX_WHEELBASE || cmd.track_width > MAX_TRACK_WIDTH) {
            return false;
        }
        
        return true;
    }
    
    // Motor status validation
    inline bool validateMotorStatus(const MotorStatus_t& status) {
        // Check for NaN values
        if (!ValidationUtils::areValidFloats(status.right_motor_rpm, status.left_motor_rpm, 
                                           status.steering_angle, status.steering_position)) {
            return false;
        }
        
        // Check RPM limits
        if (!ValidationUtils::isInAbsoluteRange(status.right_motor_rpm, MAX_MOTOR_RPM) ||
            !ValidationUtils::isInAbsoluteRange(status.left_motor_rpm, MAX_MOTOR_RPM)) {
            return false;
        }
        
        // Check steering angle limits
        if (!ValidationUtils::isInAbsoluteRange(status.steering_angle, MAX_STEERING_ANGLE_STATUS)) {
            return false;
        }
        
        return true;
    }
}

// =============================================================================
// ROS MESSAGE VALIDATION
// =============================================================================

namespace ROSValidation {
    // Twist message validation (implemented in source files that include ROS headers)
    bool validateTwistMessage(const geometry_msgs__msg__Twist* twist);
    
    // Odometry message validation (implemented in source files that include ROS headers)
    bool validateOdometryMessage(const nav_msgs__msg__Odometry* odom);
    
    // Vehicle geometry message validation (implemented in source files that include ROS headers)
    bool validateGeometryMessage(const geometry_msg__msg__VehicleGeometry* geom);
}

#endif // VALIDATION_UTILS_H
