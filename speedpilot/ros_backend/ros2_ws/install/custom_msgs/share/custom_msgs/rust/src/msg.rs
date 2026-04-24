#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to custom_msgs__msg__VehicleCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleCommand {
    /// The command to be executed
    pub command: std::string::String,

    /// magnitude of speed (positive: forward, negative: backward)
    pub speed: f32,

    /// steering angle (positive: right, negative: left)
    pub angle: f32,

}



impl Default for VehicleCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleCommand::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleCommand {
  type RmwMsg = super::msg::rmw::VehicleCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command.as_str().into(),
        speed: msg.speed,
        angle: msg.angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command.as_str().into(),
      speed: msg.speed,
      angle: msg.angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: msg.command.to_string(),
      speed: msg.speed,
      angle: msg.angle,
    }
  }
}


// Corresponds to custom_msgs__msg__Command

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Command {
    /// The command to be executed
    pub command: std::string::String,

}



impl Default for Command {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Command::default())
  }
}

impl rosidl_runtime_rs::Message for Command {
  type RmwMsg = super::msg::rmw::Command;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: msg.command.to_string(),
    }
  }
}


