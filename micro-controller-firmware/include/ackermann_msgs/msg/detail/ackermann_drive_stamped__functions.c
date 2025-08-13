// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice
#include "ackermann_msgs/msg/detail/ackermann_drive_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `drive`
#include "ackermann_msgs/msg/detail/ackermann_drive__functions.h"

bool
ackermann_msgs__msg__AckermannDriveStamped__init(ackermann_msgs__msg__AckermannDriveStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ackermann_msgs__msg__AckermannDriveStamped__fini(msg);
    return false;
  }
  // drive
  if (!ackermann_msgs__msg__AckermannDrive__init(&msg->drive)) {
    ackermann_msgs__msg__AckermannDriveStamped__fini(msg);
    return false;
  }
  return true;
}

void
ackermann_msgs__msg__AckermannDriveStamped__fini(ackermann_msgs__msg__AckermannDriveStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // drive
  ackermann_msgs__msg__AckermannDrive__fini(&msg->drive);
}

bool
ackermann_msgs__msg__AckermannDriveStamped__are_equal(const ackermann_msgs__msg__AckermannDriveStamped * lhs, const ackermann_msgs__msg__AckermannDriveStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // drive
  if (!ackermann_msgs__msg__AckermannDrive__are_equal(
      &(lhs->drive), &(rhs->drive)))
  {
    return false;
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDriveStamped__copy(
  const ackermann_msgs__msg__AckermannDriveStamped * input,
  ackermann_msgs__msg__AckermannDriveStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // drive
  if (!ackermann_msgs__msg__AckermannDrive__copy(
      &(input->drive), &(output->drive)))
  {
    return false;
  }
  return true;
}

ackermann_msgs__msg__AckermannDriveStamped *
ackermann_msgs__msg__AckermannDriveStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDriveStamped * msg = (ackermann_msgs__msg__AckermannDriveStamped *)allocator.allocate(sizeof(ackermann_msgs__msg__AckermannDriveStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ackermann_msgs__msg__AckermannDriveStamped));
  bool success = ackermann_msgs__msg__AckermannDriveStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ackermann_msgs__msg__AckermannDriveStamped__destroy(ackermann_msgs__msg__AckermannDriveStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ackermann_msgs__msg__AckermannDriveStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ackermann_msgs__msg__AckermannDriveStamped__Sequence__init(ackermann_msgs__msg__AckermannDriveStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDriveStamped * data = NULL;

  if (size) {
    data = (ackermann_msgs__msg__AckermannDriveStamped *)allocator.zero_allocate(size, sizeof(ackermann_msgs__msg__AckermannDriveStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ackermann_msgs__msg__AckermannDriveStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ackermann_msgs__msg__AckermannDriveStamped__fini(&data[i - 1]);
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
ackermann_msgs__msg__AckermannDriveStamped__Sequence__fini(ackermann_msgs__msg__AckermannDriveStamped__Sequence * array)
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
      ackermann_msgs__msg__AckermannDriveStamped__fini(&array->data[i]);
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

ackermann_msgs__msg__AckermannDriveStamped__Sequence *
ackermann_msgs__msg__AckermannDriveStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ackermann_msgs__msg__AckermannDriveStamped__Sequence * array = (ackermann_msgs__msg__AckermannDriveStamped__Sequence *)allocator.allocate(sizeof(ackermann_msgs__msg__AckermannDriveStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ackermann_msgs__msg__AckermannDriveStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ackermann_msgs__msg__AckermannDriveStamped__Sequence__destroy(ackermann_msgs__msg__AckermannDriveStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ackermann_msgs__msg__AckermannDriveStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ackermann_msgs__msg__AckermannDriveStamped__Sequence__are_equal(const ackermann_msgs__msg__AckermannDriveStamped__Sequence * lhs, const ackermann_msgs__msg__AckermannDriveStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ackermann_msgs__msg__AckermannDriveStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDriveStamped__Sequence__copy(
  const ackermann_msgs__msg__AckermannDriveStamped__Sequence * input,
  ackermann_msgs__msg__AckermannDriveStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ackermann_msgs__msg__AckermannDriveStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ackermann_msgs__msg__AckermannDriveStamped * data =
      (ackermann_msgs__msg__AckermannDriveStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ackermann_msgs__msg__AckermannDriveStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ackermann_msgs__msg__AckermannDriveStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ackermann_msgs__msg__AckermannDriveStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
