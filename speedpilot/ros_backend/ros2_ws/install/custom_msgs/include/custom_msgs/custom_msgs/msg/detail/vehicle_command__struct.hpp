// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/vehicle_command.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__VehicleCommand __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__VehicleCommand __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleCommand_
{
  using Type = VehicleCommand_<ContainerAllocator>;

  explicit VehicleCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->speed = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit VehicleCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->speed = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _speed_type =
    float;
  _speed_type speed;
  using _angle_type =
    float;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::VehicleCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::VehicleCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::VehicleCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::VehicleCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__VehicleCommand
    std::shared_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__VehicleCommand
    std::shared_ptr<custom_msgs::msg::VehicleCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleCommand_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleCommand_

// alias to use template instance with default allocator
using VehicleCommand =
  custom_msgs::msg::VehicleCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_HPP_
