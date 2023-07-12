// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from can_interfaces:msg/VelData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__VEL_DATA__BUILDER_HPP_
#define CAN_INTERFACES__MSG__DETAIL__VEL_DATA__BUILDER_HPP_

#include "can_interfaces/msg/detail/vel_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace can_interfaces
{

namespace msg
{

namespace builder
{

class Init_VelData_v_des
{
public:
  explicit Init_VelData_v_des(::can_interfaces::msg::VelData & msg)
  : msg_(msg)
  {}
  ::can_interfaces::msg::VelData v_des(::can_interfaces::msg::VelData::_v_des_type arg)
  {
    msg_.v_des = std::move(arg);
    return std::move(msg_);
  }

private:
  ::can_interfaces::msg::VelData msg_;
};

class Init_VelData_id
{
public:
  Init_VelData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelData_v_des id(::can_interfaces::msg::VelData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_VelData_v_des(msg_);
  }

private:
  ::can_interfaces::msg::VelData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::can_interfaces::msg::VelData>()
{
  return can_interfaces::msg::builder::Init_VelData_id();
}

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__VEL_DATA__BUILDER_HPP_
