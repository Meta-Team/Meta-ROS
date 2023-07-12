// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from can_interfaces:msg/PosVelData.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/pos_vel_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
can_interfaces__msg__PosVelData__init(can_interfaces__msg__PosVelData * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // p_des
  // v_des
  return true;
}

void
can_interfaces__msg__PosVelData__fini(can_interfaces__msg__PosVelData * msg)
{
  if (!msg) {
    return;
  }
  // id
  // p_des
  // v_des
}

bool
can_interfaces__msg__PosVelData__are_equal(const can_interfaces__msg__PosVelData * lhs, const can_interfaces__msg__PosVelData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // p_des
  if (lhs->p_des != rhs->p_des) {
    return false;
  }
  // v_des
  if (lhs->v_des != rhs->v_des) {
    return false;
  }
  return true;
}

bool
can_interfaces__msg__PosVelData__copy(
  const can_interfaces__msg__PosVelData * input,
  can_interfaces__msg__PosVelData * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // p_des
  output->p_des = input->p_des;
  // v_des
  output->v_des = input->v_des;
  return true;
}

can_interfaces__msg__PosVelData *
can_interfaces__msg__PosVelData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__PosVelData * msg = (can_interfaces__msg__PosVelData *)allocator.allocate(sizeof(can_interfaces__msg__PosVelData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(can_interfaces__msg__PosVelData));
  bool success = can_interfaces__msg__PosVelData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
can_interfaces__msg__PosVelData__destroy(can_interfaces__msg__PosVelData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    can_interfaces__msg__PosVelData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
can_interfaces__msg__PosVelData__Sequence__init(can_interfaces__msg__PosVelData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__PosVelData * data = NULL;

  if (size) {
    data = (can_interfaces__msg__PosVelData *)allocator.zero_allocate(size, sizeof(can_interfaces__msg__PosVelData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = can_interfaces__msg__PosVelData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        can_interfaces__msg__PosVelData__fini(&data[i - 1]);
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
can_interfaces__msg__PosVelData__Sequence__fini(can_interfaces__msg__PosVelData__Sequence * array)
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
      can_interfaces__msg__PosVelData__fini(&array->data[i]);
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

can_interfaces__msg__PosVelData__Sequence *
can_interfaces__msg__PosVelData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__PosVelData__Sequence * array = (can_interfaces__msg__PosVelData__Sequence *)allocator.allocate(sizeof(can_interfaces__msg__PosVelData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = can_interfaces__msg__PosVelData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
can_interfaces__msg__PosVelData__Sequence__destroy(can_interfaces__msg__PosVelData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    can_interfaces__msg__PosVelData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
can_interfaces__msg__PosVelData__Sequence__are_equal(const can_interfaces__msg__PosVelData__Sequence * lhs, const can_interfaces__msg__PosVelData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!can_interfaces__msg__PosVelData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
can_interfaces__msg__PosVelData__Sequence__copy(
  const can_interfaces__msg__PosVelData__Sequence * input,
  can_interfaces__msg__PosVelData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(can_interfaces__msg__PosVelData);
    can_interfaces__msg__PosVelData * data =
      (can_interfaces__msg__PosVelData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!can_interfaces__msg__PosVelData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          can_interfaces__msg__PosVelData__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!can_interfaces__msg__PosVelData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
