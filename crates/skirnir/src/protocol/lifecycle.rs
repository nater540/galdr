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

  /// Whether the link is up and ready (banner/first report confirmed): everything except [`Self::Disconnected`]
  /// and the not-yet-ready [`Self::Connecting`]. Drives UI that should only act on a *ready* board (e.g. enabling
  /// jog/stream controls). It is deliberately NOT the test for "is a port open" — see [`Self::has_transport`].
  pub fn is_connected(self) -> bool {
    !matches!(self, ConnectionState::Disconnected | ConnectionState::Connecting)
  }

  /// Whether a transport is attached — the engine owns an open serial port — regardless of whether readiness has
  /// been confirmed yet. True for [`Self::Connecting`] AND every connected state; false only for
  /// [`Self::Disconnected`]. The UI gates its Disconnect affordance on THIS, not [`Self::is_connected`]: a connect
  /// that stalls in `Connecting` (the ESP32-S3 can fail to volunteer readiness) still holds the OS port open, so
  /// the operator must always be able to tear it down and release the file descriptor for another tool (espflash).
  pub fn has_transport(self) -> bool {
    !matches!(self, ConnectionState::Disconnected)
  }
}

#[cfg(test)]
mod tests {
  use super::ConnectionState::*;

  #[test]
  fn is_connected_is_true_only_once_a_board_is_ready() {
    // Only the post-handshake states count as connected; `Connecting` is attached-but-not-yet-ready.
    assert!(!Disconnected.is_connected());
    assert!(!Connecting.is_connected());
    for state in [Idle, Streaming, Hold, Alarm, Error] {
      assert!(state.is_connected(), "{state:?} should read as connected");
    }
  }

  #[test]
  fn has_transport_covers_connecting_so_a_stalled_connect_can_be_torn_down() {
    // `Disconnected` is the only state with no open port; everything else — crucially `Connecting`, where a stalled
    // handshake still holds the FD — must report an attached transport so the Disconnect affordance stays available.
    assert!(!Disconnected.has_transport());
    for state in [Connecting, Idle, Streaming, Hold, Alarm, Error] {
      assert!(state.has_transport(), "{state:?} holds an open port and must offer Disconnect");
    }
  }
}
