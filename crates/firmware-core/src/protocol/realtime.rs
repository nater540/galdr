//! Real-time command classification (grblHAL single-byte real-time commands).

/// A real-time command: a single byte intercepted out of the RX stream the instant it arrives, ahead of
/// the line buffer. It never enters a line and never receives an `ok`. Both the printable grbl-1.1 forms
/// and the grblHAL top-bit-set forms (advertised by `RT+` in `NEWOPT`) classify to the same variant so a
/// sender may use either. Stage-1 acts on the first four; the rest are carried so the driver can dispatch
/// or ignore them without re-scanning, and so Stage 2 can light them up without changing this surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RealtimeCommand {
  /// `?` / `0x80` — status report request. The driver answers with a `<...>` report immediately.
  StatusReport,
  /// `~` / `0x81` — cycle start / resume.
  CycleStart,
  /// `!` / `0x82` — feed hold.
  FeedHold,
  /// `0x18` (Ctrl-X) — soft reset / abort: halt motion, reset parser/planner, re-emit the banner.
  SoftReset,
  /// `0x19` (Ctrl-Y) — stop: like soft reset but leaves more internal state intact (no full warm reset).
  Stop,
  /// `0x83` — request the parser-state (`$G`) report on demand.
  ParserStateReport,
  /// `0x84` — safety door: suspend into the DOOR state, kill spindle/coolant.
  SafetyDoor,
  /// `0x85` — jog cancel: feed-hold plus a planner flush; ignored if not jogging.
  JogCancel,
  /// `0x86` — Galdr graceful program stop: decelerate the running/held program to a controlled stop at a block
  /// boundary, flush the planner queue + any in-progress arc, clear the program/modal-run state, and return to
  /// Idle (NOT alarm) with machine position RETAINED. Distinct from the `0x18` abort (which raises ALARM:3 on a
  /// mid-cycle reset and loses positional certainty); ignored (benign) when not running or held. This is a Galdr
  /// extension — grbl has no dedicated graceful-stop real-time byte, modelling it as feed-hold then reset.
  ProgramStop,
  /// `0x87` — full real-time report (all change-only elements plus the alarm substate). Answered even in
  /// otherwise-locked states so a sender can detect an extended (grblHAL) controller on connect.
  FullStatusReport,
  /// `0x8C` — toggle the auto real-time report mode (`$481`). Carried for Stage 3; no Stage-1 action.
  ToggleAutoReport,
  /// `0x88` — toggle the optional-stop switch that gates `M1`. grblHAL leaves this OFF by default, so an `M1` is a
  /// no-op until the operator enables it; the firmware holds the toggle and consults it when an `M1` pause runs.
  ToggleOptionalStop,
  /// A feed / rapid / spindle / coolant override byte (`0x90`–`0x9E`, `0xA0`–`0xA4`). The raw byte is
  /// preserved so the override handler can decode the specific adjustment without a second classify pass.
  Override(u8),
}

/// Classify a single byte as a real-time command, or `None` if it is ordinary line content. Both the
/// printable grbl forms and the grblHAL top-bit-set forms map to the same [`RealtimeCommand`]. The RX
/// scanner calls this on every byte and diverts any `Some` result before line assembly.
///
/// Note on `?`/`~`/`!`: grblHAL ignores the *printable* forms while reading `$`-command or message input
/// so those characters can appear in passwords/strings. Stage 1 does not implement that input-mode
/// suppression — these are always intercepted — which is the more conservative, sender-compatible
/// behavior; Stage 2 can gate the printable forms on an input-mode flag held by the orchestrator.
pub fn classify_realtime(byte: u8) -> Option<RealtimeCommand> {
  match byte {
    b'?' | 0x80 => Some(RealtimeCommand::StatusReport),
    b'~' | 0x81 => Some(RealtimeCommand::CycleStart),
    b'!' | 0x82 => Some(RealtimeCommand::FeedHold),
    0x18 => Some(RealtimeCommand::SoftReset),
    0x19 => Some(RealtimeCommand::Stop),
    0x83 => Some(RealtimeCommand::ParserStateReport),
    0x84 => Some(RealtimeCommand::SafetyDoor),
    0x85 => Some(RealtimeCommand::JogCancel),
    0x86 => Some(RealtimeCommand::ProgramStop),
    0x87 => Some(RealtimeCommand::FullStatusReport),
    0x8C => Some(RealtimeCommand::ToggleAutoReport),
    0x88 => Some(RealtimeCommand::ToggleOptionalStop),
    // Feed (0x90-0x94), rapid (0x95-0x97), spindle (0x99-0x9E), coolant (0xA0-0xA1), tool/probe
    // (0xA3-0xA4) overrides. Preserve the raw byte for the override decoder.
    0x90..=0x9E | 0xA0..=0xA4 => Some(RealtimeCommand::Override(byte)),
    _ => None,
  }
}
