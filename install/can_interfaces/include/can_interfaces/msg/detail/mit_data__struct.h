// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from can_interfaces:msg/MITData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_H_
#define CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MITData in the package can_interfaces.
typedef struct can_interfaces__msg__MITData
{
  uint8_t id;
  float p_des;
  float v_des;
  float kp;
  float kd;
  float t_ff;
} can_interfaces__msg__MITData;

// Struct for a sequence of can_interfaces__msg__MITData.
typedef struct can_interfaces__msg__MITData__Sequence
{
  can_interfaces__msg__MITData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} can_interfaces__msg__MITData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_H_
