#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__VehicleCommand() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__VehicleCommand__init(msg: *mut VehicleCommand) -> bool;
    fn custom_msgs__msg__VehicleCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleCommand>, size: usize) -> bool;
    fn custom_msgs__msg__VehicleCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleCommand>);
    fn custom_msgs__msg__VehicleCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleCommand>) -> bool;
}

// Corresponds to custom_msgs__msg__VehicleCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleCommand {
    /// The command to be executed
    pub command: rosidl_runtime_rs::String,

    /// magnitude of speed (positive: forward, negative: backward)
    pub speed: f32,

    /// steering angle (positive: right, negative: left)
    pub angle: f32,

}



impl Default for VehicleCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__VehicleCommand__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__VehicleCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleCommand where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/VehicleCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__VehicleCommand() }
  }
}


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Command() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__Command__init(msg: *mut Command) -> bool;
    fn custom_msgs__msg__Command__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Command>, size: usize) -> bool;
    fn custom_msgs__msg__Command__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Command>);
    fn custom_msgs__msg__Command__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Command>, out_seq: *mut rosidl_runtime_rs::Sequence<Command>) -> bool;
}

// Corresponds to custom_msgs__msg__Command
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Command {
    /// The command to be executed
    pub command: rosidl_runtime_rs::String,

}



impl Default for Command {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__Command__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__Command__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Command {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Command__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Command__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Command__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Command {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Command where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/Command";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Command() }
  }
}


