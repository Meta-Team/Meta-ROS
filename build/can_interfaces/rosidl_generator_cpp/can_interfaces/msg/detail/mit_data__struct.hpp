// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from can_interfaces:msg/MITData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_HPP_
#define CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__can_interfaces__msg__MITData __attribute__((deprecated))
#else
# define DEPRECATED__can_interfaces__msg__MITData __declspec(deprecated)
#endif

namespace can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MITData_
{
  using Type = MITData_<ContainerAllocator>;

  explicit MITData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->p_des = 0.0f;
      this->v_des = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->t_ff = 0.0f;
    }
  }

  explicit MITData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->p_des = 0.0f;
      this->v_des = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->t_ff = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _p_des_type =
    float;
  _p_des_type p_des;
  using _v_des_type =
    float;
  _v_des_type v_des;
  using _kp_type =
    float;
  _kp_type kp;
  using _kd_type =
    float;
  _kd_type kd;
  using _t_ff_type =
    float;
  _t_ff_type t_ff;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__p_des(
    const float & _arg)
  {
    this->p_des = _arg;
    return *this;
  }
  Type & set__v_des(
    const float & _arg)
  {
    this->v_des = _arg;
    return *this;
  }
  Type & set__kp(
    const float & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const float & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__t_ff(
    const float & _arg)
  {
    this->t_ff = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    can_interfaces::msg::MITData_<ContainerAllocator> *;
  using ConstRawPtr =
    const can_interfaces::msg::MITData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<can_interfaces::msg::MITData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<can_interfaces::msg::MITData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::MITData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::MITData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::MITData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::MITData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<can_interfaces::msg::MITData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<can_interfaces::msg::MITData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__can_interfaces__msg__MITData
    std::shared_ptr<can_interfaces::msg::MITData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__can_interfaces__msg__MITData
    std::shared_ptr<can_interfaces::msg::MITData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MITData_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->p_des != other.p_des) {
      return false;
    }
    if (this->v_des != other.v_des) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->t_ff != other.t_ff) {
      return false;
    }
    return true;
  }
  bool operator!=(const MITData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MITData_

// alias to use template instance with default allocator
using MITData =
  can_interfaces::msg::MITData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__MIT_DATA__STRUCT_HPP_
