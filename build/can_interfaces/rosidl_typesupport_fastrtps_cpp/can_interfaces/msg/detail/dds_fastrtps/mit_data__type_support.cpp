// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from can_interfaces:msg/MITData.idl
// generated code does not contain a copyright notice
#include "can_interfaces/msg/detail/mit_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "can_interfaces/msg/detail/mit_data__struct.hpp"

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
  const can_interfaces::msg::MITData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: id
  cdr << ros_message.id;
  // Member: p_des
  cdr << ros_message.p_des;
  // Member: v_des
  cdr << ros_message.v_des;
  // Member: kp
  cdr << ros_message.kp;
  // Member: kd
  cdr << ros_message.kd;
  // Member: t_ff
  cdr << ros_message.t_ff;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  can_interfaces::msg::MITData & ros_message)
{
  // Member: id
  cdr >> ros_message.id;

  // Member: p_des
  cdr >> ros_message.p_des;

  // Member: v_des
  cdr >> ros_message.v_des;

  // Member: kp
  cdr >> ros_message.kp;

  // Member: kd
  cdr >> ros_message.kd;

  // Member: t_ff
  cdr >> ros_message.t_ff;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
get_serialized_size(
  const can_interfaces::msg::MITData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: p_des
  {
    size_t item_size = sizeof(ros_message.p_des);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_des
  {
    size_t item_size = sizeof(ros_message.v_des);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp
  {
    size_t item_size = sizeof(ros_message.kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kd
  {
    size_t item_size = sizeof(ros_message.kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: t_ff
  {
    size_t item_size = sizeof(ros_message.t_ff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_interfaces
max_serialized_size_MITData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: p_des
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: v_des
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kd
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: t_ff
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _MITData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const can_interfaces::msg::MITData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MITData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<can_interfaces::msg::MITData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MITData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const can_interfaces::msg::MITData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MITData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MITData(full_bounded, 0);
}

static message_type_support_callbacks_t _MITData__callbacks = {
  "can_interfaces::msg",
  "MITData",
  _MITData__cdr_serialize,
  _MITData__cdr_deserialize,
  _MITData__get_serialized_size,
  _MITData__max_serialized_size
};

static rosidl_message_type_support_t _MITData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MITData__callbacks,
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
get_message_type_support_handle<can_interfaces::msg::MITData>()
{
  return &can_interfaces::msg::typesupport_fastrtps_cpp::_MITData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, can_interfaces, msg, MITData)() {
  return &can_interfaces::msg::typesupport_fastrtps_cpp::_MITData__handle;
}

#ifdef __cplusplus
}
#endif
