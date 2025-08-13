// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__STRUCT_H_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'drive'
#include "ackermann_msgs/msg/detail/ackermann_drive__struct.h"

/// Struct defined in msg/AckermannDriveStamped in the package ackermann_msgs.
/**
  * Time stamped drive command for robots with Ackermann steering.
  *  $Id$
 */
typedef struct ackermann_msgs__msg__AckermannDriveStamped
{
  std_msgs__msg__Header header;
  ackermann_msgs__msg__AckermannDrive drive;
} ackermann_msgs__msg__AckermannDriveStamped;

// Struct for a sequence of ackermann_msgs__msg__AckermannDriveStamped.
typedef struct ackermann_msgs__msg__AckermannDriveStamped__Sequence
{
  ackermann_msgs__msg__AckermannDriveStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ackermann_msgs__msg__AckermannDriveStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__STRUCT_H_
