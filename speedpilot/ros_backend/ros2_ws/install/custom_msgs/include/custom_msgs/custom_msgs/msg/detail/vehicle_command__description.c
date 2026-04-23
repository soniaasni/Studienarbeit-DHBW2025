// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_msgs:msg/VehicleCommand.idl
// generated code does not contain a copyright notice

#include "custom_msgs/msg/detail/vehicle_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
const rosidl_type_hash_t *
custom_msgs__msg__VehicleCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x88, 0x83, 0x7b, 0xb7, 0x3d, 0x53, 0xa7, 0xd5,
      0x0c, 0x8b, 0x0b, 0x60, 0xaa, 0x56, 0xd1, 0xdb,
      0xe8, 0x33, 0xc6, 0xbc, 0x17, 0xae, 0x49, 0x73,
      0x12, 0x8b, 0xb4, 0xe9, 0xcc, 0xd7, 0x7f, 0xf8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_msgs__msg__VehicleCommand__TYPE_NAME[] = "custom_msgs/msg/VehicleCommand";

// Define type names, field names, and default values
static char custom_msgs__msg__VehicleCommand__FIELD_NAME__command[] = "command";
static char custom_msgs__msg__VehicleCommand__FIELD_NAME__speed[] = "speed";
static char custom_msgs__msg__VehicleCommand__FIELD_NAME__angle[] = "angle";

static rosidl_runtime_c__type_description__Field custom_msgs__msg__VehicleCommand__FIELDS[] = {
  {
    {custom_msgs__msg__VehicleCommand__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__VehicleCommand__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__VehicleCommand__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_msgs__msg__VehicleCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_msgs__msg__VehicleCommand__TYPE_NAME, 30, 30},
      {custom_msgs__msg__VehicleCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string command     # The command to be executed\n"
  "float32 speed      # magnitude of speed (positive: forward, negative: backward)\n"
  "float32 angle      # steering angle (positive: right, negative: left)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_msgs__msg__VehicleCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_msgs__msg__VehicleCommand__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 197, 197},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_msgs__msg__VehicleCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_msgs__msg__VehicleCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
