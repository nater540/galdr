//! Pure, egui-free presentation logic for the machine-state badge, the transport (Run/Hold/Stop) group, and
//! the alarm/error banner copy.
//!
//! The design (`Skirnir.dc.html`, §05) demands that each lifecycle phase have an unmistakable signature in the
//! toolbar badge and status bar, and that the badge reflect the *firmware-reported* run state (Jog/Home/Door/
//! Check), not just the host's connection lifecycle. The host's [`ConnectionState`] only knows Idle/Streaming/
//! Hold/Alarm/Error; the firmware's [`RunState`] is richer. So the badge is a small derivation over both: when
//! a status report is in hand the firmware's run state wins, otherwise we fall back to the connection state.
//!
//! All of this is pure mapping with no egui types, so it lives outside the `gui` feature gate and is unit-
//! tested without a window. The colour tokens themselves live in [`super::theme`]; this module decides *which*
//! semantic state the badge/segment is in, and the theme turns that into a concrete colour.

use crate::protocol::{ConnectionState, RunState};

/// The semantic state shown by the toolbar/status badge. This is the union the design's badge palette is keyed
/// on (§01 "Machine state"): it is richer than [`ConnectionState`] because the firmware reports Jog/Home/Door/
/// Check/Sleep that the host lifecycle does not model, and it is the single source of truth the theme colours
/// and the views label.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BadgeState {
  /// No transport attached.
  Disconnected,
  /// Transport attached, readiness not yet confirmed (awaiting banner / first report).
  Connecting,
  /// Connected, ready, idle.
  Idle,
  /// Actively streaming / the firmware reports `Run`.
  Run,
  /// A `$J=` jog is executing (`Jog`).
  Jog,
  /// Feed hold in effect (`Hold`).
  Hold,
  /// Running the homing cycle (`Home`).
  Home,
  /// Safety door open / interlock (`Door`).
  Door,
  /// `$C` check mode (`Check`).
  Check,
  /// Sleep state (`Sleep`).
  Sleep,
  /// Controller is in an alarm state.
  Alarm,
  /// A line-level `error:N` halted the stream.
  Error,
}

impl BadgeState {
  /// Derive the badge state from the host lifecycle plus the firmware's last-reported run state.
  ///
  /// Rule: an `Alarm`/`Error` host lifecycle is authoritative and always wins (the host latches these on the
  /// `ALARM:`/`error:` response and must not let a stale `<Run>` report paper over them). Otherwise, when a
  /// status report is in hand and the connection is live, the firmware's `RunState` drives the badge so
  /// Jog/Home/Door/Check render distinctly. With no report yet (or while disconnected/connecting) we fall back
  /// to the host lifecycle.
  pub fn derive(connection: ConnectionState, run_state: Option<RunState>) -> Self {
    match connection {
      ConnectionState::Disconnected => return BadgeState::Disconnected,
      ConnectionState::Connecting => return BadgeState::Connecting,
      // Host-latched fault states are authoritative regardless of any in-flight telemetry.
      ConnectionState::Alarm => return BadgeState::Alarm,
      ConnectionState::Error => return BadgeState::Error,
      _ => {}
    }
    // Live and un-faulted: prefer the firmware's own run state when we have one.
    if let Some(state) = run_state.and_then(Self::from_run_state) {
      return state;
    }
    // No usable report; fall back to the host lifecycle for the remaining live states.
    match connection {
      ConnectionState::Streaming => BadgeState::Run,
      ConnectionState::Hold => BadgeState::Hold,
      _ => BadgeState::Idle,
    }
  }

  /// Map a firmware [`RunState`] to a badge state, or `None` for states better taken from the host lifecycle
  /// (`Unknown`, and `Tool` which the badge does not distinguish from running).
  fn from_run_state(run: RunState) -> Option<Self> {
    match run {
      RunState::Idle => Some(BadgeState::Idle),
      RunState::Run => Some(BadgeState::Run),
      RunState::Jog => Some(BadgeState::Jog),
      RunState::Hold => Some(BadgeState::Hold),
      RunState::Home => Some(BadgeState::Home),
      RunState::Door => Some(BadgeState::Door),
      RunState::Check => Some(BadgeState::Check),
      RunState::Sleep => Some(BadgeState::Sleep),
      RunState::Alarm => Some(BadgeState::Alarm),
      // Tool-change and unrecognised tokens defer to the host lifecycle.
      RunState::Tool | RunState::Unknown => None,
    }
  }

  /// The short, uppercase label the badge shows. Per the design, the label is the *primary* signal (colour is
  /// secondary), so every state has a distinct, glanceable word.
  pub fn label(self) -> &'static str {
    match self {
      BadgeState::Disconnected => "DISCONNECTED",
      BadgeState::Connecting => "CONNECTING",
      BadgeState::Idle => "IDLE",
      BadgeState::Run => "RUN",
      BadgeState::Jog => "JOG",
      BadgeState::Hold => "HOLD",
      BadgeState::Home => "HOMING",
      BadgeState::Door => "DOOR",
      BadgeState::Check => "CHECK",
      BadgeState::Sleep => "SLEEP",
      BadgeState::Alarm => "ALARM",
      BadgeState::Error => "ERROR",
    }
  }
}

/// Which segment of the toolbar Run/Hold/Stop group is the active (emphasised) one, and whether the group is
/// interactive, for a given badge state. The design shows the leading segment switching label/colour with the
/// machine state ("Running" while streaming, "Resume" amber while held). Keeping this a pure decision lets the
/// view stay a dumb renderer and lets us test the enable/emphasis matrix directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportGroup {
  /// Whether the Run/cycle-start segment is enabled (a program can be started or a hold resumed).
  pub run_enabled: bool,
  /// Whether the leading segment should read "Resume" (held) rather than "Run"/"Running".
  pub run_is_resume: bool,
  /// Whether the leading segment is emphasised because the machine is actively running.
  pub run_active: bool,
  /// Whether the Hold segment is enabled (only meaningful while running/jogging).
  pub hold_enabled: bool,
  /// Whether the Stop (soft-reset) segment is enabled (any live connection can be stopped/reset).
  pub stop_enabled: bool,
}

impl TransportGroup {
  /// Decide the Run/Hold/Stop segment matrix for a badge state and whether a program is loaded. `has_program`
  /// gates whether the Run segment can *start* a stream from Idle.
  pub fn for_state(state: BadgeState, has_program: bool) -> Self {
    match state {
      // No live connection: the whole group is inert.
      BadgeState::Disconnected | BadgeState::Connecting => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: false,
      },
      // Idle: Run can start a stream if a program is loaded; nothing to hold; stop/reset is always available.
      BadgeState::Idle | BadgeState::Check | BadgeState::Sleep => TransportGroup {
        run_enabled: has_program,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
      },
      // Actively moving: the leading segment is emphasised; hold and stop are live.
      BadgeState::Run | BadgeState::Jog | BadgeState::Home => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: true,
        hold_enabled: true,
        stop_enabled: true,
      },
      // Held or door-suspended: the leading segment becomes "Resume"; stop stays live.
      BadgeState::Hold | BadgeState::Door => TransportGroup {
        run_enabled: true,
        run_is_resume: true,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
      },
      // Faulted: only stop/reset (and the banner's own actions) can recover; no run/hold.
      BadgeState::Alarm | BadgeState::Error => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
      },
    }
  }
}

/// A short human gloss for a grbl `ALARM:N` code, for the alarm banner's detail line. The set follows grbl 1.1 /
/// grblHAL; an unknown code falls back to a generic message so the banner never shows a bare number alone.
pub fn alarm_detail(code: u32) -> &'static str {
  match code {
    1 => "Hard limit triggered. Position likely lost — re-home before continuing.",
    2 => "Soft limit: a commanded motion exceeded the machine travel.",
    3 => "Abort during cycle: reset while in motion. Position lost — re-home.",
    4 => "Probe fail: the probe did not contact within the programmed travel.",
    5 => "Probe fail: the probe was already triggered before the move started.",
    6 => "Homing fail: reset during homing.",
    7 => "Homing fail: safety door opened during homing.",
    8 => "Homing fail: pull-off did not clear the limit switch.",
    9 => "Homing fail: a limit switch was not found within the search distance.",
    10 => "EStop asserted. Clear the emergency stop, then reset.",
    11 => "Homing required: home the machine before running this command.",
    _ => "Controller is locked. $X to unlock or $H to home before continuing.",
  }
}

/// A short human gloss for a grbl `error:N` code, for the console and the stream-error banner. Covers the codes
/// a sender hits most; unknown codes fall back to a generic message.
pub fn error_detail(code: u32) -> &'static str {
  match code {
    1 => "Expected G-code word letter but found none.",
    2 => "Numeric value format is invalid or missing.",
    3 => "Unsupported or invalid `$` system command.",
    9 => "G-code locked out during alarm or jog.",
    15 => "Travel exceeded: a jog target is outside the machine envelope.",
    20 => "Unsupported or invalid G-code command in the block.",
    22 => "Feed rate has not been set or is undefined.",
    33 => "Motion command has an invalid target.",
    _ => "G-code error: the stream is halted until reset or a `$` command clears it.",
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn disconnected_and_connecting_come_straight_from_the_lifecycle() {
    assert_eq!(BadgeState::derive(ConnectionState::Disconnected, None), BadgeState::Disconnected);
    assert_eq!(BadgeState::derive(ConnectionState::Connecting, Some(RunState::Run)), BadgeState::Connecting);
  }

  #[test]
  fn firmware_run_state_drives_the_badge_when_live_and_unfaulted() {
    // A live connection with a fresh report should surface the firmware's richer states.
    assert_eq!(BadgeState::derive(ConnectionState::Idle, Some(RunState::Jog)), BadgeState::Jog);
    assert_eq!(BadgeState::derive(ConnectionState::Streaming, Some(RunState::Home)), BadgeState::Home);
    assert_eq!(BadgeState::derive(ConnectionState::Idle, Some(RunState::Door)), BadgeState::Door);
    assert_eq!(BadgeState::derive(ConnectionState::Idle, Some(RunState::Check)), BadgeState::Check);
  }

  #[test]
  fn host_latched_faults_win_over_any_telemetry() {
    // Even if a stale <Run> report is in hand, a latched Alarm/Error must dominate.
    assert_eq!(BadgeState::derive(ConnectionState::Alarm, Some(RunState::Run)), BadgeState::Alarm);
    assert_eq!(BadgeState::derive(ConnectionState::Error, Some(RunState::Idle)), BadgeState::Error);
  }

  #[test]
  fn falls_back_to_lifecycle_without_a_report() {
    assert_eq!(BadgeState::derive(ConnectionState::Streaming, None), BadgeState::Run);
    assert_eq!(BadgeState::derive(ConnectionState::Hold, None), BadgeState::Hold);
    assert_eq!(BadgeState::derive(ConnectionState::Idle, None), BadgeState::Idle);
  }

  #[test]
  fn unknown_and_tool_run_states_defer_to_the_lifecycle() {
    assert_eq!(BadgeState::derive(ConnectionState::Streaming, Some(RunState::Unknown)), BadgeState::Run);
    assert_eq!(BadgeState::derive(ConnectionState::Idle, Some(RunState::Tool)), BadgeState::Idle);
  }

  #[test]
  fn every_badge_state_has_a_distinct_uppercase_label() {
    let states = [
      BadgeState::Disconnected,
      BadgeState::Connecting,
      BadgeState::Idle,
      BadgeState::Run,
      BadgeState::Jog,
      BadgeState::Hold,
      BadgeState::Home,
      BadgeState::Door,
      BadgeState::Check,
      BadgeState::Sleep,
      BadgeState::Alarm,
      BadgeState::Error,
    ];
    let labels: Vec<&str> = states.iter().map(|s| s.label()).collect();
    let unique: std::collections::HashSet<&str> = labels.iter().copied().collect();
    assert_eq!(labels.len(), unique.len(), "labels must be unique per state");
    assert!(labels.iter().all(|l| l == &l.to_uppercase()), "labels are uppercase");
  }

  #[test]
  fn transport_group_is_inert_while_disconnected() {
    let group = TransportGroup::for_state(BadgeState::Disconnected, true);
    assert!(!group.run_enabled && !group.hold_enabled && !group.stop_enabled);
  }

  #[test]
  fn idle_can_run_only_with_a_program_loaded() {
    assert!(!TransportGroup::for_state(BadgeState::Idle, false).run_enabled);
    assert!(TransportGroup::for_state(BadgeState::Idle, true).run_enabled);
    // Idle can always be reset, and there is nothing to hold.
    let idle = TransportGroup::for_state(BadgeState::Idle, true);
    assert!(idle.stop_enabled && !idle.hold_enabled);
  }

  #[test]
  fn running_emphasises_run_and_enables_hold_and_stop() {
    let run = TransportGroup::for_state(BadgeState::Run, false);
    assert!(run.run_active && run.hold_enabled && run.stop_enabled);
    assert!(!run.run_enabled, "cannot start a stream while already running");
  }

  #[test]
  fn hold_offers_resume_and_stop() {
    let hold = TransportGroup::for_state(BadgeState::Hold, false);
    assert!(hold.run_enabled && hold.run_is_resume);
    assert!(!hold.hold_enabled && hold.stop_enabled);
  }

  #[test]
  fn faulted_states_only_allow_stop() {
    for state in [BadgeState::Alarm, BadgeState::Error] {
      let group = TransportGroup::for_state(state, true);
      assert!(group.stop_enabled);
      assert!(!group.run_enabled && !group.hold_enabled);
    }
  }

  #[test]
  fn alarm_and_error_details_are_specific_with_a_generic_fallback() {
    assert!(alarm_detail(1).contains("Hard limit"));
    assert!(alarm_detail(9).contains("Homing"));
    assert!(alarm_detail(9999).contains("locked"), "unknown codes get a generic gloss");
    assert!(error_detail(9).contains("locked out"));
    assert!(error_detail(9999).contains("halted"), "unknown error codes get a generic gloss");
  }
}
