// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from can_interfaces:msg/PosVelData.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/pos_vel_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "can_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "can_interfaces/msg/detail/pos_vel_data__struct.h"
#include "can_interfaces/msg/detail/pos_vel_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _PosVelData__ros_msg_type = can_interfaces__msg__PosVelData;

static bool _PosVelData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PosVelData__ros_msg_type * ros_message = static_cast<const _PosVelData__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: p_des
  {
    cdr << ros_message->p_des;
  }

  // Field name: v_des
  {
    cdr << ros_message->v_des;
  }

  return true;
}

static bool _PosVelData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PosVelData__ros_msg_type * ros_message = static_cast<_PosVelData__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: p_des
  {
    cdr >> ros_message->p_des;
  }

  // Field name: v_des
  {
    cdr >> ros_message->v_des;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_can_interfaces
size_t get_serialized_size_can_interfaces__msg__PosVelData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PosVelData__ros_msg_type * ros_message = static_cast<const _PosVelData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name p_des
  {
    size_t item_size = sizeof(ros_message->p_des);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name v_des
  {
    size_t item_size = sizeof(ros_message->v_des);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PosVelData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_can_interfaces__msg__PosVelData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_can_interfaces
size_t max_serialized_size_can_interfaces__msg__PosVelData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: p_des
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: v_des
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PosVelData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_can_interfaces__msg__PosVelData(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PosVelData = {
  "can_interfaces::msg",
  "PosVelData",
  _PosVelData__cdr_serialize,
  _PosVelData__cdr_deserialize,
  _PosVelData__get_serialized_size,
  _PosVelData__max_serialized_size
};

static rosidl_message_type_support_t _PosVelData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PosVelData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, can_interfaces, msg, PosVelData)() {
  return &_PosVelData__type_support;
}

#if defined(__cplusplus)
}
#endif
