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
//! - [`reconnect`] — the pure host-side backoff schedule the shell consults to auto-reconnect after a drop
//!   (notably the ESP32-S3's USB re-enumeration on soft reset). No timer/transport — just the delay decision.
//! - [`error`] — the typed error surface. The engine never panics on a runtime condition; it surfaces these.
//! - [`profile`] — the cross-session project/profile store: the rotary-A center and connection/UI defaults
//!   persisted as a versioned RON file under the OS config dir (DOC-11 §1.3). Framework-agnostic, so the GUI
//!   and the headless `--cli` path share one store; read failures fall back to defaults rather than panicking.
//! - [`config`] — the startup-loaded JSON app config: appearance/themes, UI defaults, connection/streaming and
//!   toolpath-render tuning, resolved into a runtime [`config::Palette`]/[`config::ToolpathStyle`] for the views.
//!   The appearance sibling of [`profile`], with the same never-panic, versioned, atomic-write contract.
//! - [`store`] — shared host-side persistence primitives (the OS config dir + an atomic file write) the config
//!   store builds on.
//!
//! The intended boundary: a UI calls [`engine::Engine::connect`] with a transport, then sends
//! [`engine::Command`]s and drains [`engine::Event`]s over the returned [`engine::EngineHandle`] — never
//! touching the protocol core or the transport directly.

#![deny(unsafe_code)]

pub mod app;
// The app config resolves an egui `Palette`/`ToolpathStyle`, so it depends on eframe and is gui-gated like the
// theme/views/shell layers it feeds. The headless `--cli` path renders nothing and has no use for it.
#[cfg(feature = "gui")]
pub mod config;
pub mod engine;
pub mod error;
pub mod eta;
pub mod profile;
pub mod protocol;
pub mod reconnect;
pub mod store;
pub mod transport;

// The engine API the UI drives.
pub use engine::{Command, Engine, EngineHandle, Event};
// The transport boundary and its fakes/adapters.
pub use transport::Transport;
// The protocol vocabulary the UI renders and the contract types callers match on.
pub use protocol::{ConnectionState, RealtimeCommand, Response};
// The reconnect schedule the shell drives.
pub use reconnect::{ReconnectConfig, ReconnectPolicy};
// The typed error surface.
pub use error::{EngineError, TransportError};
// The cross-session profile/project store.
pub use profile::{Prefs, Profile, ProfileError, RotarySetup};
// The startup-loaded app config and the runtime appearance it resolves into (gui-gated, like the views it feeds).
#[cfg(feature = "gui")]
pub use app::theme::Palette;
#[cfg(feature = "gui")]
pub use config::{Config, ToolpathStyle};
