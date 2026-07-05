//! Stable identifiers for objects and tools.
//!
//! Ids are opaque, monotonically allocated integers. They are stable across renames (the name is the mutable human
//! label; the id is the immutable handle other objects reference) and survive a serialize/deserialize round-trip so
//! cross-object links — a CNC job pointing at the Gerber it was cut from — stay valid after a project is reopened.

use serde::{Deserialize, Serialize};
use std::fmt;

/// An opaque, stable handle to an [`crate::Object`] within one [`crate::ObjectCollection`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ObjectId(pub u64);

impl fmt::Display for ObjectId {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "#{}", self.0)
  }
}

/// An opaque, stable handle to a [`crate::ToolEntry`] within one [`crate::ToolDatabase`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ToolId(pub u64);

impl fmt::Display for ToolId {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "tool#{}", self.0)
  }
}
