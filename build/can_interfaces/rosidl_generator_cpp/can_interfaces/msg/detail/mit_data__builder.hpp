// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from can_interfaces:msg/MITData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__MIT_DATA__BUILDER_HPP_
#define CAN_INTERFACES__MSG__DETAIL__MIT_DATA__BUILDER_HPP_

#include "can_interfaces/msg/detail/mit_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace can_interfaces
{

namespace msg
{

namespace builder
{

class Init_MITData_t_ff
{
public:
  explicit Init_MITData_t_ff(::can_interfaces::msg::MITData & msg)
  : msg_(msg)
  {}
  ::can_interfaces::msg::MITData t_ff(::can_interfaces::msg::MITData::_t_ff_type arg)
  {
    msg_.t_ff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

class Init_MITData_kd
{
public:
  explicit Init_MITData_kd(::can_interfaces::msg::MITData & msg)
  : msg_(msg)
  {}
  Init_MITData_t_ff kd(::can_interfaces::msg::MITData::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MITData_t_ff(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

class Init_MITData_kp
{
public:
  explicit Init_MITData_kp(::can_interfaces::msg::MITData & msg)
  : msg_(msg)
  {}
  Init_MITData_kd kp(::can_interfaces::msg::MITData::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MITData_kd(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

class Init_MITData_v_des
{
public:
  explicit Init_MITData_v_des(::can_interfaces::msg::MITData & msg)
  : msg_(msg)
  {}
  Init_MITData_kp v_des(::can_interfaces::msg::MITData::_v_des_type arg)
  {
    msg_.v_des = std::move(arg);
    return Init_MITData_kp(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

class Init_MITData_p_des
{
public:
  explicit Init_MITData_p_des(::can_interfaces::msg::MITData & msg)
  : msg_(msg)
  {}
  Init_MITData_v_des p_des(::can_interfaces::msg::MITData::_p_des_type arg)
  {
    msg_.p_des = std::move(arg);
    return Init_MITData_v_des(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

class Init_MITData_id
{
public:
  Init_MITData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MITData_p_des id(::can_interfaces::msg::MITData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MITData_p_des(msg_);
  }

private:
  ::can_interfaces::msg::MITData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::can_interfaces::msg::MITData>()
{
  return can_interfaces::msg::builder::Init_MITData_id();
}

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__MIT_DATA__BUILDER_HPP_
