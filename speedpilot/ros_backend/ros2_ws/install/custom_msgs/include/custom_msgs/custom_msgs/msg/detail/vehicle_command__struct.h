// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/vehicle_command.h"


#ifndef CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/VehicleCommand in the package custom_msgs.
typedef struct custom_msgs__msg__VehicleCommand
{
  /// The command to be executed
  rosidl_runtime_c__String command;
  /// magnitude of speed (positive: forward, negative: backward)
  float speed;
  /// steering angle (positive: right, negative: left)
  float angle;
} custom_msgs__msg__VehicleCommand;

// Struct for a sequence of custom_msgs__msg__VehicleCommand.
typedef struct custom_msgs__msg__VehicleCommand__Sequence
{
  custom_msgs__msg__VehicleCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__VehicleCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__VEHICLE_COMMAND__STRUCT_H_
