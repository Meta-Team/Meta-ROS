// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
#define CAN_INTERFACES__MSG__DETAIL__FEEDBACK__BUILDER_HPP_

#include "can_interfaces/msg/detail/feedback__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace can_interfaces
{

namespace msg
{

namespace builder
{

class Init_Feedback_t_rotor
{
public:
  explicit Init_Feedback_t_rotor(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  ::can_interfaces::msg::Feedback t_rotor(::can_interfaces::msg::Feedback::_t_rotor_type arg)
  {
    msg_.t_rotor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_t_mos
{
public:
  explicit Init_Feedback_t_mos(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_t_rotor t_mos(::can_interfaces::msg::Feedback::_t_mos_type arg)
  {
    msg_.t_mos = std::move(arg);
    return Init_Feedback_t_rotor(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_tor
{
public:
  explicit Init_Feedback_tor(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_t_mos tor(::can_interfaces::msg::Feedback::_tor_type arg)
  {
    msg_.tor = std::move(arg);
    return Init_Feedback_t_mos(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_vel
{
public:
  explicit Init_Feedback_vel(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_tor vel(::can_interfaces::msg::Feedback::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_Feedback_tor(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_pos
{
public:
  explicit Init_Feedback_pos(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_vel pos(::can_interfaces::msg::Feedback::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_Feedback_vel(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_id
{
public:
  explicit Init_Feedback_id(::can_interfaces::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_pos id(::can_interfaces::msg::Feedback::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Feedback_pos(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

class Init_Feedback_mst_id
{
public:
  Init_Feedback_mst_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Feedback_id mst_id(::can_interfaces::msg::Feedback::_mst_id_type arg)
  {
    msg_.mst_id = std::move(arg);
    return Init_Feedback_id(msg_);
  }

private:
  ::can_interfaces::msg::Feedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::can_interfaces::msg::Feedback>()
{
  return can_interfaces::msg::builder::Init_Feedback_mst_id();
}

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
