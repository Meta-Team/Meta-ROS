// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/feedback__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "can_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "can_interfaces/msg/detail/feedback__struct.h"
#include "can_interfaces/msg/detail/feedback__functions.h"
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


using _Feedback__ros_msg_type = can_interfaces__msg__Feedback;

static bool _Feedback__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Feedback__ros_msg_type * ros_message = static_cast<const _Feedback__ros_msg_type *>(untyped_ros_message);
  // Field name: mst_id
  {
    cdr << ros_message->mst_id;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: pos
  {
    cdr << ros_message->pos;
  }

  // Field name: vel
  {
    cdr << ros_message->vel;
  }

  // Field name: tor
  {
    cdr << ros_message->tor;
  }

  // Field name: t_mos
  {
    cdr << ros_message->t_mos;
  }

  // Field name: t_rotor
  {
    cdr << ros_message->t_rotor;
  }

  return true;
}

static bool _Feedback__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Feedback__ros_msg_type * ros_message = static_cast<_Feedback__ros_msg_type *>(untyped_ros_message);
  // Field name: mst_id
  {
    cdr >> ros_message->mst_id;
  }

  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: pos
  {
    cdr >> ros_message->pos;
  }

  // Field name: vel
  {
    cdr >> ros_message->vel;
  }

  // Field name: tor
  {
    cdr >> ros_message->tor;
  }

  // Field name: t_mos
  {
    cdr >> ros_message->t_mos;
  }

  // Field name: t_rotor
  {
    cdr >> ros_message->t_rotor;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_can_interfaces
size_t get_serialized_size_can_interfaces__msg__Feedback(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Feedback__ros_msg_type * ros_message = static_cast<const _Feedback__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name mst_id
  {
    size_t item_size = sizeof(ros_message->mst_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos
  {
    size_t item_size = sizeof(ros_message->pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel
  {
    size_t item_size = sizeof(ros_message->vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tor
  {
    size_t item_size = sizeof(ros_message->tor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name t_mos
  {
    size_t item_size = sizeof(ros_message->t_mos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name t_rotor
  {
    size_t item_size = sizeof(ros_message->t_rotor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Feedback__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_can_interfaces__msg__Feedback(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_can_interfaces
size_t max_serialized_size_can_interfaces__msg__Feedback(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: mst_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: t_mos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: t_rotor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Feedback__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_can_interfaces__msg__Feedback(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Feedback = {
  "can_interfaces::msg",
  "Feedback",
  _Feedback__cdr_serialize,
  _Feedback__cdr_deserialize,
  _Feedback__get_serialized_size,
  _Feedback__max_serialized_size
};

static rosidl_message_type_support_t _Feedback__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Feedback,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, can_interfaces, msg, Feedback)() {
  return &_Feedback__type_support;
}

#if defined(__cplusplus)
}
#endif
