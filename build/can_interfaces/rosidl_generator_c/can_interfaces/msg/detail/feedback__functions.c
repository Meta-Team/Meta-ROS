// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
can_interfaces__msg__Feedback__init(can_interfaces__msg__Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // mst_id
  // id
  // pos
  // vel
  // tor
  // t_mos
  // t_rotor
  return true;
}

void
can_interfaces__msg__Feedback__fini(can_interfaces__msg__Feedback * msg)
{
  if (!msg) {
    return;
  }
  // mst_id
  // id
  // pos
  // vel
  // tor
  // t_mos
  // t_rotor
}

bool
can_interfaces__msg__Feedback__are_equal(const can_interfaces__msg__Feedback * lhs, const can_interfaces__msg__Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mst_id
  if (lhs->mst_id != rhs->mst_id) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // pos
  if (lhs->pos != rhs->pos) {
    return false;
  }
  // vel
  if (lhs->vel != rhs->vel) {
    return false;
  }
  // tor
  if (lhs->tor != rhs->tor) {
    return false;
  }
  // t_mos
  if (lhs->t_mos != rhs->t_mos) {
    return false;
  }
  // t_rotor
  if (lhs->t_rotor != rhs->t_rotor) {
    return false;
  }
  return true;
}

bool
can_interfaces__msg__Feedback__copy(
  const can_interfaces__msg__Feedback * input,
  can_interfaces__msg__Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // mst_id
  output->mst_id = input->mst_id;
  // id
  output->id = input->id;
  // pos
  output->pos = input->pos;
  // vel
  output->vel = input->vel;
  // tor
  output->tor = input->tor;
  // t_mos
  output->t_mos = input->t_mos;
  // t_rotor
  output->t_rotor = input->t_rotor;
  return true;
}

can_interfaces__msg__Feedback *
can_interfaces__msg__Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__Feedback * msg = (can_interfaces__msg__Feedback *)allocator.allocate(sizeof(can_interfaces__msg__Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(can_interfaces__msg__Feedback));
  bool success = can_interfaces__msg__Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
can_interfaces__msg__Feedback__destroy(can_interfaces__msg__Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    can_interfaces__msg__Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
can_interfaces__msg__Feedback__Sequence__init(can_interfaces__msg__Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__Feedback * data = NULL;

  if (size) {
    data = (can_interfaces__msg__Feedback *)allocator.zero_allocate(size, sizeof(can_interfaces__msg__Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = can_interfaces__msg__Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        can_interfaces__msg__Feedback__fini(&data[i - 1]);
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
can_interfaces__msg__Feedback__Sequence__fini(can_interfaces__msg__Feedback__Sequence * array)
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
      can_interfaces__msg__Feedback__fini(&array->data[i]);
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

can_interfaces__msg__Feedback__Sequence *
can_interfaces__msg__Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  can_interfaces__msg__Feedback__Sequence * array = (can_interfaces__msg__Feedback__Sequence *)allocator.allocate(sizeof(can_interfaces__msg__Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = can_interfaces__msg__Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
can_interfaces__msg__Feedback__Sequence__destroy(can_interfaces__msg__Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    can_interfaces__msg__Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
can_interfaces__msg__Feedback__Sequence__are_equal(const can_interfaces__msg__Feedback__Sequence * lhs, const can_interfaces__msg__Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!can_interfaces__msg__Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
can_interfaces__msg__Feedback__Sequence__copy(
  const can_interfaces__msg__Feedback__Sequence * input,
  can_interfaces__msg__Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(can_interfaces__msg__Feedback);
    can_interfaces__msg__Feedback * data =
      (can_interfaces__msg__Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!can_interfaces__msg__Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          can_interfaces__msg__Feedback__fini(&data[i]);
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
    if (!can_interfaces__msg__Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
