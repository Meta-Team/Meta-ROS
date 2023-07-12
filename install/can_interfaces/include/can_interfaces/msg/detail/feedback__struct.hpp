// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from can_interfaces:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
#define CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__can_interfaces__msg__Feedback __attribute__((deprecated))
#else
# define DEPRECATED__can_interfaces__msg__Feedback __declspec(deprecated)
#endif

namespace can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Feedback_
{
  using Type = Feedback_<ContainerAllocator>;

  explicit Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mst_id = 0;
      this->id = 0;
      this->pos = 0.0f;
      this->vel = 0.0f;
      this->tor = 0.0f;
      this->t_mos = 0.0f;
      this->t_rotor = 0.0f;
    }
  }

  explicit Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mst_id = 0;
      this->id = 0;
      this->pos = 0.0f;
      this->vel = 0.0f;
      this->tor = 0.0f;
      this->t_mos = 0.0f;
      this->t_rotor = 0.0f;
    }
  }

  // field types and members
  using _mst_id_type =
    int8_t;
  _mst_id_type mst_id;
  using _id_type =
    int8_t;
  _id_type id;
  using _pos_type =
    float;
  _pos_type pos;
  using _vel_type =
    float;
  _vel_type vel;
  using _tor_type =
    float;
  _tor_type tor;
  using _t_mos_type =
    float;
  _t_mos_type t_mos;
  using _t_rotor_type =
    float;
  _t_rotor_type t_rotor;

  // setters for named parameter idiom
  Type & set__mst_id(
    const int8_t & _arg)
  {
    this->mst_id = _arg;
    return *this;
  }
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pos(
    const float & _arg)
  {
    this->pos = _arg;
    return *this;
  }
  Type & set__vel(
    const float & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__tor(
    const float & _arg)
  {
    this->tor = _arg;
    return *this;
  }
  Type & set__t_mos(
    const float & _arg)
  {
    this->t_mos = _arg;
    return *this;
  }
  Type & set__t_rotor(
    const float & _arg)
  {
    this->t_rotor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    can_interfaces::msg::Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const can_interfaces::msg::Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<can_interfaces::msg::Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<can_interfaces::msg::Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<can_interfaces::msg::Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<can_interfaces::msg::Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__can_interfaces__msg__Feedback
    std::shared_ptr<can_interfaces::msg::Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__can_interfaces__msg__Feedback
    std::shared_ptr<can_interfaces::msg::Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Feedback_ & other) const
  {
    if (this->mst_id != other.mst_id) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->pos != other.pos) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->tor != other.tor) {
      return false;
    }
    if (this->t_mos != other.t_mos) {
      return false;
    }
    if (this->t_rotor != other.t_rotor) {
      return false;
    }
    return true;
  }
  bool operator!=(const Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Feedback_

// alias to use template instance with default allocator
using Feedback =
  can_interfaces::msg::Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
