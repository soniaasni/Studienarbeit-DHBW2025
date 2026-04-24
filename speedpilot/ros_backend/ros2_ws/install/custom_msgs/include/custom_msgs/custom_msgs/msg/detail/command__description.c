// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#include "custom_msgs/msg/detail/command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
const rosidl_type_hash_t *
custom_msgs__msg__Command__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0x15, 0x71, 0x9b, 0xd8, 0xc8, 0x5c, 0x4a,
      0x29, 0x59, 0x94, 0x7f, 0x98, 0x97, 0x32, 0x92,
      0x69, 0xe3, 0xbc, 0xfd, 0x6f, 0xd2, 0xe0, 0xaf,
      0x1b, 0x22, 0x18, 0x03, 0xcb, 0x6c, 0x2b, 0xbb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_msgs__msg__Command__TYPE_NAME[] = "custom_msgs/msg/Command";

// Define type names, field names, and default values
static char custom_msgs__msg__Command__FIELD_NAME__command[] = "command";

static rosidl_runtime_c__type_description__Field custom_msgs__msg__Command__FIELDS[] = {
  {
    {custom_msgs__msg__Command__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_msgs__msg__Command__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_msgs__msg__Command__TYPE_NAME, 23, 23},
      {custom_msgs__msg__Command__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "string command # The command to be executed";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_msgs__msg__Command__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_msgs__msg__Command__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_msgs__msg__Command__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_msgs__msg__Command__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
