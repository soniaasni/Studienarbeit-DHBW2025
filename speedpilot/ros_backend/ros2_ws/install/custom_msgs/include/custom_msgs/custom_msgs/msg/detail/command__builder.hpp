// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Command.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/command.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Command_command
{
public:
  Init_Command_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_msgs::msg::Command command(::custom_msgs::msg::Command::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Command>()
{
  return custom_msgs::msg::builder::Init_Command_command();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
