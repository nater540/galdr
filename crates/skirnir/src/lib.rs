//! `skirnir` — the native (Linux-first) GCode sender that streams to the Galdr ESP32-S3 firmware over USB
//! CDC serial, implementing the host side of the grblHAL streaming contract.
//!
//! The crate is split so the GUI is a swappable detail and the protocol is host-testable:
//!
//! - [`protocol`] — the pure, synchronous streaming state machine: the character-count window, the
//!   CRLF/LFCR deframer, the response parser, the real-time command set, the lifecycle enum, and the
//!   [`ProtocolCore`] that ties them together. No async, no I/O, no UI — directly unit-testable.
//! - [`transport`] — the [`Transport`] byte-movement trait, its in-memory loopback fake for tests, and the
//!   real `tokio-serial` adapter (behind the `serial` feature).
//! - [`engine`] — the async driver that owns a [`Transport`], pumps it through the protocol core, applies the
//!   emitted effects, and bridges the UI over command/event channels. The future egui UI drives this.
//! - [`error`] — the typed error surface. The engine never panics on a runtime condition; it surfaces these.
//!
//! The intended boundary: a UI calls [`engine::Engine::connect`] with a transport, then sends
//! [`engine::Command`]s and drains [`engine::Event`]s over the returned [`engine::EngineHandle`] — never
//! touching the protocol core or the transport directly.

#![deny(unsafe_code)]

pub mod app;
pub mod engine;
pub mod error;
pub mod protocol;
pub mod transport;

// The engine API the UI drives.
pub use engine::{Command, Engine, EngineHandle, Event};
// The transport boundary and its fakes/adapters.
pub use transport::Transport;
// The protocol vocabulary the UI renders and the contract types callers match on.
pub use protocol::{ConnectionState, RealtimeCommand, Response};
// The typed error surface.
pub use error::{EngineError, TransportError};
