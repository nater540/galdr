//! The connection / streaming lifecycle the UI observes.
//!
//! Modelled as one explicit enum rather than scattered booleans so the engine and UI can never disagree
//! about what is happening. The async engine owns the single authoritative value and emits
//! [`crate::engine::Event::StateChanged`] on every transition.

/// Where the engine is in its connection and streaming lifecycle. Transitions are driven by transport
/// events (connect/disconnect), parsed firmware responses (`ok`/`error`/`ALARM`/banner), and host intents
/// (load a program, feed-hold, resume).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
  /// No transport is attached. The engine is inert.
  Disconnected,

  /// A transport is attached but readiness has not yet been confirmed (awaiting banner / first report).
  Connecting,

  /// Connected and ready, with no program streaming — manual lines and real-time commands are allowed.
  Idle,

  /// A program is actively streaming, with the character-count window driving line release.
  Streaming,

  /// A feed hold is in effect (`!`); motion is paused but the stream and connection are intact.
  Hold,

  /// The firmware is in an alarm state and will refuse G-code until it is cleared (`$X` / `$H` / reset).
  Alarm,

  /// A line-level `error:N` halted the stream. grblHAL holds subsequent lines until reset / empty line /
  /// `$` command, so we stop releasing program lines and surface the error rather than streaming on.
  Error,
}

impl ConnectionState {
  /// Whether new program lines may be released to the firmware in this state. Only [`Self::Streaming`]
  /// releases; every other state (including [`Self::Hold`]) holds the send-ahead window.
  pub fn can_release_program_lines(self) -> bool {
    matches!(self, ConnectionState::Streaming)
  }

  /// Whether the engine currently has a live transport, regardless of streaming progress.
  pub fn is_connected(self) -> bool {
    !matches!(self, ConnectionState::Disconnected | ConnectionState::Connecting)
  }
}
