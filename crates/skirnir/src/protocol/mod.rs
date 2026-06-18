//! The framework-agnostic streaming protocol layer.
//!
//! Everything here is pure and synchronous — no async, no time, no transport, no UI. It is the directly
//! unit-testable heart of the engine: the character-count window, the line-terminator deframer, the
//! response parser, the real-time command set, the lifecycle enum, and the [`core::ProtocolCore`] state
//! machine that wires them together. The async [`crate::engine`] is a thin driver over this core.

pub mod core;
pub mod flow;
pub mod lifecycle;
pub mod realtime;
pub mod response;
pub mod settings;
pub mod status;
pub mod terminator;

pub use core::{Effect, ProtocolCore};
pub use flow::{DEFAULT_RX_BUFFER, FlowWindow};
pub use lifecycle::ConnectionState;
pub use realtime::RealtimeCommand;
pub use response::{Response, is_grbl_evidence, parse_line};
pub use settings::{SettingMeta, SettingValue, parse_setting_meta, parse_setting_value, setting_write_line};
pub use status::{MachineState, PinState, PositionKind, RunState, StatusReport, parse_status};
pub use terminator::LineReassembler;
