// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from can_interfaces:msg/VelData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__VEL_DATA__STRUCT_HPP_
#define CAN_INTERFACES__MSG__DETAIL__VEL_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__can_interfaces__msg__VelData __attribute__((deprecated))
#else
# define DEPRECATED__can_interfaces__msg__VelData __declspec(deprecated)
#endif

namespace can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelData_
{
  using Type = VelData_<ContainerAllocator>;

  explicit VelData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->v_des = 0.0f;
    }
  }

  explicit VelData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->v_des = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _v_des_type =
    float;
  _v_des_type v_des;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__v_des(
    const float & _arg)
  {
    this->v_des = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    can_interfaces::msg::VelData_<ContainerAllocator> *;
  using ConstRawPtr =
    const can_interfaces::msg::VelData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<can_interfaces::msg::VelData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<can_interfaces::msg::VelData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::VelData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::VelData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      can_interfaces::msg::VelData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<can_interfaces::msg::VelData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<can_interfaces::msg::VelData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<can_interfaces::msg::VelData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__can_interfaces__msg__VelData
    std::shared_ptr<can_interfaces::msg::VelData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__can_interfaces__msg__VelData
    std::shared_ptr<can_interfaces::msg::VelData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelData_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->v_des != other.v_des) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelData_

// alias to use template instance with default allocator
using VelData =
  can_interfaces::msg::VelData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace can_interfaces

#endif  // CAN_INTERFACES__MSG__DETAIL__VEL_DATA__STRUCT_HPP_
