// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice
#include "ackermann_msgs/msg/detail/ackermann_drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ackermann_msgs__msg__AckermannDrive__init(ackermann_msgs__msg__AckermannDrive * msg)
{
  if (!msg) {
    return false;
  }
  // steering_angle
  // steering_angle_velocity
  // speed
  // acceleration
  // jerk
  return true;
}

void
ackermann_msgs__msg__AckermannDrive__fini(ackermann_msgs__msg__AckermannDrive * msg)
{
  if (!msg) {
    return;
  }
  // steering_angle
  // steering_angle_velocity
  // speed
  // acceleration
  // jerk
}

bool
ackermann_msgs__msg__AckermannDrive__are_equal(const ackermann_msgs__msg__AckermannDrive * lhs, const ackermann_msgs__msg__AckermannDrive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // steering_angle
  if (lhs->steering_angle != rhs->steering_angle) {
    return false;
  }
  // steering_angle_velocity
  if (lhs->steering_angle_velocity != rhs->steering_angle_velocity) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // jerk
  if (lhs->jerk != rhs->jerk) {
    return false;
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDrive__copy(
  const ackermann_msgs__msg__AckermannDrive * input,
  ackermann_msgs__msg__AckermannDrive * output)
{
  if (!input || !output) {
    return false;
  }
  // steering_angle
  output->steering_angle = input->steering_angle;
  // steering_angle_velocity
  output->steering_angle_velocity = input->steering_angle_velocity;
  // speed
  output->speed = input->speed;
  // acceleration
  output->acceleration = input->acceleration;
  // jerk
  output->jerk = input->jerk;
  return true;
}

ackermann_msgs__msg__AckermannDrive *
ackermann_msgs__msg__AckermannDrive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDrive * msg = (ackermann_msgs__msg__AckermannDrive *)allocator.allocate(sizeof(ackermann_msgs__msg__AckermannDrive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ackermann_msgs__msg__AckermannDrive));
  bool success = ackermann_msgs__msg__AckermannDrive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ackermann_msgs__msg__AckermannDrive__destroy(ackermann_msgs__msg__AckermannDrive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ackermann_msgs__msg__AckermannDrive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ackermann_msgs__msg__AckermannDrive__Sequence__init(ackermann_msgs__msg__AckermannDrive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDrive * data = NULL;

  if (size) {
    data = (ackermann_msgs__msg__AckermannDrive *)allocator.zero_allocate(size, sizeof(ackermann_msgs__msg__AckermannDrive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ackermann_msgs__msg__AckermannDrive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ackermann_msgs__msg__AckermannDrive__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ackermann_msgs__msg__AckermannDrive__Sequence__fini(ackermann_msgs__msg__AckermannDrive__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ackermann_msgs__msg__AckermannDrive__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ackermann_msgs__msg__AckermannDrive__Sequence *
ackermann_msgs__msg__AckermannDrive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDrive__Sequence * array = (ackermann_msgs__msg__AckermannDrive__Sequence *)allocator.allocate(sizeof(ackermann_msgs__msg__AckermannDrive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ackermann_msgs__msg__AckermannDrive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ackermann_msgs__msg__AckermannDrive__Sequence__destroy(ackermann_msgs__msg__AckermannDrive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ackermann_msgs__msg__AckermannDrive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ackermann_msgs__msg__AckermannDrive__Sequence__are_equal(const ackermann_msgs__msg__AckermannDrive__Sequence * lhs, const ackermann_msgs__msg__AckermannDrive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ackermann_msgs__msg__AckermannDrive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDrive__Sequence__copy(
  const ackermann_msgs__msg__AckermannDrive__Sequence * input,
  ackermann_msgs__msg__AckermannDrive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ackermann_msgs__msg__AckermannDrive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ackermann_msgs__msg__AckermannDrive * data =
      (ackermann_msgs__msg__AckermannDrive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ackermann_msgs__msg__AckermannDrive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ackermann_msgs__msg__AckermannDrive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ackermann_msgs__msg__AckermannDrive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
