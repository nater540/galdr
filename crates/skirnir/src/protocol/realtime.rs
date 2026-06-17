//! Real-time single-byte commands, injected out-of-band.
//!
//! Per `docs/gcode-streaming.md` these bytes are picked out *before* the line buffer: they are sent
//! immediately, never line-buffered, never acknowledged with `ok`, and — critically — never counted
//! against the character-count window. The engine emits the mapped byte ahead of any queued program
//! bytes the instant the host requests it, so a `?` or `!` mid-stream never corrupts the line stream.

/// A real-time command the host can inject at any instant. Each maps to exactly one byte on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RealtimeCommand {
  /// `0x18` (Ctrl-X) — soft reset / abort. Halts motion, resets the parser/planner, re-emits the banner.
  SoftReset,
  /// `?` — request a status report (`<...>`).
  StatusReport,
  /// `~` — cycle start / resume.
  CycleStart,
  /// `!` — feed hold.
  FeedHold,
  /// `0x84` — safety door: suspends into DOOR state, kills spindle/coolant.
  SafetyDoor,
  /// `0x85` — jog cancel: feed-hold + flush planner; ignored by the firmware if not jogging.
  JogCancel,
  /// `0x87` — request a full real-time report (used during connect to detect a grblHAL controller).
  FullReport,

  /// Feed override: set to 100%.
  FeedOverrideReset,
  /// Feed override: coarse +10%.
  FeedOverridePlus10,
  /// Feed override: coarse -10%.
  FeedOverrideMinus10,
  /// Feed override: fine +1%.
  FeedOverridePlus1,
  /// Feed override: fine -1%.
  FeedOverrideMinus1,

  /// Rapid override: set to 100%.
  RapidOverrideReset,
  /// Rapid override: 50%.
  RapidOverride50,
  /// Rapid override: 25%.
  RapidOverride25,

  /// Spindle override: set to 100%.
  SpindleOverrideReset,
  /// Spindle override: coarse +10%.
  SpindleOverridePlus10,
  /// Spindle override: coarse -10%.
  SpindleOverrideMinus10,
  /// Spindle override: fine +1%.
  SpindleOverridePlus1,
  /// Spindle override: fine -1%.
  SpindleOverrideMinus1,
  /// Spindle stop toggle.
  SpindleStopToggle,

  /// Coolant flood toggle.
  CoolantFloodToggle,
  /// Coolant mist toggle.
  CoolantMistToggle,
}

impl RealtimeCommand {
  /// The single wire byte this command maps to. The override bytes follow grbl 1.1 exactly (`0x90`–`0xA1`).
  pub fn byte(self) -> u8 {
    match self {
      RealtimeCommand::SoftReset => 0x18,
      RealtimeCommand::StatusReport => b'?',
      RealtimeCommand::CycleStart => b'~',
      RealtimeCommand::FeedHold => b'!',
      RealtimeCommand::SafetyDoor => 0x84,
      RealtimeCommand::JogCancel => 0x85,
      RealtimeCommand::FullReport => 0x87,
      RealtimeCommand::FeedOverrideReset => 0x90,
      RealtimeCommand::FeedOverridePlus10 => 0x91,
      RealtimeCommand::FeedOverrideMinus10 => 0x92,
      RealtimeCommand::FeedOverridePlus1 => 0x93,
      RealtimeCommand::FeedOverrideMinus1 => 0x94,
      RealtimeCommand::RapidOverrideReset => 0x95,
      RealtimeCommand::RapidOverride50 => 0x96,
      RealtimeCommand::RapidOverride25 => 0x97,
      RealtimeCommand::SpindleOverrideReset => 0x99,
      RealtimeCommand::SpindleOverridePlus10 => 0x9A,
      RealtimeCommand::SpindleOverrideMinus10 => 0x9B,
      RealtimeCommand::SpindleOverridePlus1 => 0x9C,
      RealtimeCommand::SpindleOverrideMinus1 => 0x9D,
      RealtimeCommand::SpindleStopToggle => 0x9E,
      RealtimeCommand::CoolantFloodToggle => 0xA0,
      RealtimeCommand::CoolantMistToggle => 0xA1,
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn core_realtime_bytes_match_the_grbl_spec() {
    assert_eq!(RealtimeCommand::SoftReset.byte(), 0x18);
    assert_eq!(RealtimeCommand::StatusReport.byte(), b'?');
    assert_eq!(RealtimeCommand::CycleStart.byte(), b'~');
    assert_eq!(RealtimeCommand::FeedHold.byte(), b'!');
    assert_eq!(RealtimeCommand::JogCancel.byte(), 0x85);
  }

  #[test]
  fn override_bytes_cover_the_0x90_to_0xa1_range() {
    assert_eq!(RealtimeCommand::FeedOverrideReset.byte(), 0x90);
    assert_eq!(RealtimeCommand::RapidOverrideReset.byte(), 0x95);
    assert_eq!(RealtimeCommand::SpindleOverrideReset.byte(), 0x99);
    assert_eq!(RealtimeCommand::CoolantMistToggle.byte(), 0xA1);
  }
}
