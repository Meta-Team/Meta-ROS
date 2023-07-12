// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from can_interfaces:msg/PosVelData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__BUILDER_HPP_
#define CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__BUILDER_HPP_

#include "can_interfaces/msg/detail/pos_vel_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace can_interfaces
{

namespace msg
{

namespace builder
{

class Init_PosVelData_v_des
{
public:
  explicit Init_PosVelData_v_des(::can_interfaces::msg::PosVelData & msg)
  : msg_(msg)
  {}
  ::can_interfaces::msg::PosVelData v_des(::can_interfaces::msg::PosVelData::_v_des_type arg)
  {
    msg_.v_des = std::move(arg);
    return std::move(msg_);
  }

private:
  ::can_interfaces::msg::PosVelData msg_;
};

class Init_PosVelData_p_des
{
public:
  explicit Init_PosVelData_p_des(::can_interfaces::msg::PosVelData & msg)
  : msg_(msg)
  {}
  Init_PosVelData_v_des p_des(::can_interfaces::msg::PosVelData::_p_des_type arg)
  {
    msg_.p_des = std::move(arg);
    return Init_PosVelData_v_des(msg_);
  }

private:
  ::can_interfaces::msg::PosVelData msg_;
};

class Init_PosVelData_id
{
public:
  Init_PosVelData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PosVelData_p_des id(::can_interfaces::msg::PosVelData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_PosVelData_p_des(msg_);
  }

private:
  ::can_interfaces::msg::PosVelData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::can_interfaces::msg::PosVelData>()
{
  return can_interfaces::msg::builder::Init_PosVelData_id();
}

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__POS_VEL_DATA__BUILDER_HPP_
