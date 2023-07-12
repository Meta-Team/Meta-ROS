// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Feedback in the package can_interfaces.
typedef struct can_interfaces__msg__Feedback
{
  int8_t mst_id;
  int8_t id;
  float pos;
  float vel;
  float tor;
  float t_mos;
  float t_rotor;
} can_interfaces__msg__Feedback;

// Struct for a sequence of can_interfaces__msg__Feedback.
typedef struct can_interfaces__msg__Feedback__Sequence
{
  can_interfaces__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} can_interfaces__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_H_
