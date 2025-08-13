// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__FUNCTIONS_H_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ackermann_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ackermann_msgs/msg/detail/ackermann_drive__struct.h"

/// Initialize msg/AckermannDrive message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ackermann_msgs__msg__AckermannDrive
 * )) before or use
 * ackermann_msgs__msg__AckermannDrive__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__init(ackermann_msgs__msg__AckermannDrive * msg);

/// Finalize msg/AckermannDrive message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
void
ackermann_msgs__msg__AckermannDrive__fini(ackermann_msgs__msg__AckermannDrive * msg);

/// Create msg/AckermannDrive message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ackermann_msgs__msg__AckermannDrive__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
ackermann_msgs__msg__AckermannDrive *
ackermann_msgs__msg__AckermannDrive__create();

/// Destroy msg/AckermannDrive message.
/**
 * It calls
 * ackermann_msgs__msg__AckermannDrive__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
void
ackermann_msgs__msg__AckermannDrive__destroy(ackermann_msgs__msg__AckermannDrive * msg);

/// Check for msg/AckermannDrive message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__are_equal(const ackermann_msgs__msg__AckermannDrive * lhs, const ackermann_msgs__msg__AckermannDrive * rhs);

/// Copy a msg/AckermannDrive message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__copy(
  const ackermann_msgs__msg__AckermannDrive * input,
  ackermann_msgs__msg__AckermannDrive * output);

/// Initialize array of msg/AckermannDrive messages.
/**
 * It allocates the memory for the number of elements and calls
 * ackermann_msgs__msg__AckermannDrive__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__Sequence__init(ackermann_msgs__msg__AckermannDrive__Sequence * array, size_t size);

/// Finalize array of msg/AckermannDrive messages.
/**
 * It calls
 * ackermann_msgs__msg__AckermannDrive__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
void
ackermann_msgs__msg__AckermannDrive__Sequence__fini(ackermann_msgs__msg__AckermannDrive__Sequence * array);

/// Create array of msg/AckermannDrive messages.
/**
 * It allocates the memory for the array and calls
 * ackermann_msgs__msg__AckermannDrive__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
ackermann_msgs__msg__AckermannDrive__Sequence *
ackermann_msgs__msg__AckermannDrive__Sequence__create(size_t size);

/// Destroy array of msg/AckermannDrive messages.
/**
 * It calls
 * ackermann_msgs__msg__AckermannDrive__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
void
ackermann_msgs__msg__AckermannDrive__Sequence__destroy(ackermann_msgs__msg__AckermannDrive__Sequence * array);

/// Check for msg/AckermannDrive message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__Sequence__are_equal(const ackermann_msgs__msg__AckermannDrive__Sequence * lhs, const ackermann_msgs__msg__AckermannDrive__Sequence * rhs);

/// Copy an array of msg/AckermannDrive messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
bool
ackermann_msgs__msg__AckermannDrive__Sequence__copy(
  const ackermann_msgs__msg__AckermannDrive__Sequence * input,
  ackermann_msgs__msg__AckermannDrive__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__FUNCTIONS_H_
