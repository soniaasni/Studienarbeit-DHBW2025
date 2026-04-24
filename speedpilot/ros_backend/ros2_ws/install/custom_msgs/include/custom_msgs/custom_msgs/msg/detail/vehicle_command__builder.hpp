// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/vehicle_command.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/vehicle_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleCommand_angle
{
public:
  explicit Init_VehicleCommand_angle(::custom_msgs::msg::VehicleCommand & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::VehicleCommand angle(::custom_msgs::msg::VehicleCommand::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::VehicleCommand msg_;
};

class Init_VehicleCommand_speed
{
public:
  explicit Init_VehicleCommand_speed(::custom_msgs::msg::VehicleCommand & msg)
  : msg_(msg)
  {}
  Init_VehicleCommand_angle speed(::custom_msgs::msg::VehicleCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_VehicleCommand_angle(msg_);
  }

private:
  ::custom_msgs::msg::VehicleCommand msg_;
};

class Init_VehicleCommand_command
{
public:
  Init_VehicleCommand_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleCommand_speed command(::custom_msgs::msg::VehicleCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_VehicleCommand_speed(msg_);
  }

private:
  ::custom_msgs::msg::VehicleCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::VehicleCommand>()
{
  return custom_msgs::msg::builder::Init_VehicleCommand_command();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_
