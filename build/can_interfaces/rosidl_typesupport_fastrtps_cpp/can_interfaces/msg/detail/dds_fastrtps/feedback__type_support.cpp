// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/feedback__rosidl_typesupport_fastrtps_cpp.hpp"
#include "can_interfaces/msg/detail/feedback__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace can_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
cdr_serialize(
  const can_interfaces::msg::Feedback & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: mst_id
  cdr << ros_message.mst_id;
  // Member: id
  cdr << ros_message.id;
  // Member: pos
  cdr << ros_message.pos;
  // Member: vel
  cdr << ros_message.vel;
  // Member: tor
  cdr << ros_message.tor;
  // Member: t_mos
  cdr << ros_message.t_mos;
  // Member: t_rotor
  cdr << ros_message.t_rotor;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  can_interfaces::msg::Feedback & ros_message)
{
  // Member: mst_id
  cdr >> ros_message.mst_id;

  // Member: id
  cdr >> ros_message.id;

  // Member: pos
  cdr >> ros_message.pos;

  // Member: vel
  cdr >> ros_message.vel;

  // Member: tor
  cdr >> ros_message.tor;

  // Member: t_mos
  cdr >> ros_message.t_mos;

  // Member: t_rotor
  cdr >> ros_message.t_rotor;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
get_serialized_size(
  const can_interfaces::msg::Feedback & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: mst_id
  {
    size_t item_size = sizeof(ros_message.mst_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pos
  {
    size_t item_size = sizeof(ros_message.pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel
  {
    size_t item_size = sizeof(ros_message.vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tor
  {
    size_t item_size = sizeof(ros_message.tor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: t_mos
  {
    size_t item_size = sizeof(ros_message.t_mos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: t_rotor
  {
    size_t item_size = sizeof(ros_message.t_rotor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
max_serialized_size_Feedback(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: mst_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: t_mos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: t_rotor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Feedback__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const can_interfaces::msg::Feedback *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Feedback__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<can_interfaces::msg::Feedback *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Feedback__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const can_interfaces::msg::Feedback *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Feedback__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Feedback(full_bounded, 0);
}

static message_type_support_callbacks_t _Feedback__callbacks = {
  "can_interfaces::msg",
  "Feedback",
  _Feedback__cdr_serialize,
  _Feedback__cdr_deserialize,
  _Feedback__get_serialized_size,
  _Feedback__max_serialized_size
};

static rosidl_message_type_support_t _Feedback__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Feedback__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace can_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_can_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<can_interfaces::msg::Feedback>()
{
  return &can_interfaces::msg::typesupport_fastrtps_cpp::_Feedback__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, can_interfaces, msg, Feedback)() {
  return &can_interfaces::msg::typesupport_fastrtps_cpp::_Feedback__handle;
}

#ifdef __cplusplus
}
#endif
