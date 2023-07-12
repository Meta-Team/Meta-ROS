// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from can_interfaces:msg/MITData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__MIT_DATA__TRAITS_HPP_
#define CAN_INTERFACES__MSG__DETAIL__MIT_DATA__TRAITS_HPP_

#include "can_interfaces/msg/detail/mit_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<can_interfaces::msg::MITData>()
{
  return "can_interfaces::msg::MITData";
}

template<>
inline const char * name<can_interfaces::msg::MITData>()
{
  return "can_interfaces/msg/MITData";
}

template<>
struct has_fixed_size<can_interfaces::msg::MITData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<can_interfaces::msg::MITData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<can_interfaces::msg::MITData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CAN_INTERFACES__MSG__DETAIL__MIT_DATA__TRAITS_HPP_
