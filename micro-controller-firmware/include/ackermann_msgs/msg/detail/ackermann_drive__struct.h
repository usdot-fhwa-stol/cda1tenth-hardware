// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__STRUCT_H_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/AckermannDrive in the package ackermann_msgs.
/**
  * Driving command for a car-like vehicle using Ackermann steering.
  *  $Id$
 */
typedef struct ackermann_msgs__msg__AckermannDrive
{
  /// Assumes Ackermann front-wheel steering. The left and right front
  /// wheels are generally at different angles. To simplify, the commanded
  /// angle corresponds to the yaw of a virtual wheel located at the
  /// center of the front axle, like on a tricycle.  Positive yaw is to
  /// the left. (This is *not* the angle of the steering wheel inside the
  /// passenger compartment.)
  ///
  /// Zero steering angle velocity means change the steering angle as
  /// quickly as possible. Positive velocity indicates a desired absolute
  /// rate of change either left or right. The controller tries not to
  /// exceed this limit in either direction, but sometimes it might.
  ///
  /// desired virtual angle (radians)
  float steering_angle;
  /// desired rate of change (radians/s)
  float steering_angle_velocity;
  /// Drive at requested speed, acceleration and jerk (the 1st, 2nd and
  /// 3rd derivatives of position). All are measured at the vehicle's
  /// center of rotation, typically the center of the rear axle. The
  /// controller tries not to exceed these limits in either direction, but
  /// sometimes it might.
  ///
  /// Speed is the desired scalar magnitude of the velocity vector.
  /// Direction is forward unless the sign is negative, indicating reverse.
  ///
  /// Zero acceleration means change speed as quickly as
  /// possible. Positive acceleration indicates a desired absolute
  /// magnitude; that includes deceleration.
  ///
  /// Zero jerk means change acceleration as quickly as possible. Positive
  /// jerk indicates a desired absolute rate of acceleration change in
  /// either direction (increasing or decreasing).
  ///
  /// desired forward speed (m/s)
  float speed;
  /// desired acceleration (m/s^2)
  float acceleration;
  /// desired jerk (m/s^3)
  float jerk;
} ackermann_msgs__msg__AckermannDrive;

// Struct for a sequence of ackermann_msgs__msg__AckermannDrive.
typedef struct ackermann_msgs__msg__AckermannDrive__Sequence
{
  ackermann_msgs__msg__AckermannDrive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ackermann_msgs__msg__AckermannDrive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__STRUCT_H_
