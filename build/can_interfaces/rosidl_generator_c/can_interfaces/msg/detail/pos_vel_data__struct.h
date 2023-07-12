// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from can_interfaces:msg/PosVelData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__STRUCT_H_
#define CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/PosVelData in the package can_interfaces.
typedef struct can_interfaces__msg__PosVelData
{
  uint8_t id;
  float p_des;
  float v_des;
} can_interfaces__msg__PosVelData;

// Struct for a sequence of can_interfaces__msg__PosVelData.
typedef struct can_interfaces__msg__PosVelData__Sequence
{
  can_interfaces__msg__PosVelData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} can_interfaces__msg__PosVelData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__STRUCT_H_
