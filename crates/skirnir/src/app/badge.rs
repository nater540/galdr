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
  /// An M6 manual tool change is held awaiting resume (`Tool`): the operator inserts the tool, then cycle-starts
  /// (`~`) to continue. Distinct from `Hold` so the UI can show the tool-change affordance, not a generic hold.
  Tool,
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
      // An M6 manual tool change is its own badge state so the UI can surface the tool-change affordance.
      RunState::Tool => Some(BadgeState::Tool),
      // An unrecognised token defers to the host lifecycle rather than guessing.
      RunState::Unknown => None,
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
      BadgeState::Tool => "TOOL CHANGE",
      BadgeState::Alarm => "ALARM",
      BadgeState::Error => "ERROR",
    }
  }

  /// The i18n message key for this state's badge label (see `assets/i18n/*.ftl`). The views resolve this through
  /// `tr!` so the badge word is localized; [`Self::label`] remains the untranslated reference used off-screen (and
  /// by tests). Kept in lock-step with `label` by the exhaustive test below.
  pub fn label_key(self) -> &'static str {
    match self {
      BadgeState::Disconnected => "badge-disconnected",
      BadgeState::Connecting => "badge-connecting",
      BadgeState::Idle => "badge-idle",
      BadgeState::Run => "badge-run",
      BadgeState::Jog => "badge-jog",
      BadgeState::Hold => "badge-hold",
      BadgeState::Home => "badge-home",
      BadgeState::Door => "badge-door",
      BadgeState::Check => "badge-check",
      BadgeState::Sleep => "badge-sleep",
      BadgeState::Tool => "badge-tool",
      BadgeState::Alarm => "badge-alarm",
      BadgeState::Error => "badge-error",
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
  /// Whether the Stop (graceful program-stop, `0x86`) segment is enabled. The everyday "stop the job cleanly"
  /// control: it needs a live, ready board, so it is gated like the rest of the group (off while Disconnected /
  /// Connecting). Contrast [`Self::abort_enabled`].
  pub stop_enabled: bool,
  /// Whether the separate Abort / E-stop control (the hard soft-reset, `0x18` → `ALARM:3`) is enabled. Broader
  /// than [`Self::stop_enabled`]: it is live the instant a transport is attached — including mid-handshake
  /// (`Connecting`), where a stalled board still holds the FD — so the operator can always force an emergency
  /// reset without waiting for readiness. Mirrors [`ConnectionState::has_transport`](crate::protocol::ConnectionState::has_transport).
  pub abort_enabled: bool,
}

impl TransportGroup {
  /// Decide the Run/Hold/Stop segment matrix for a badge state and whether a program is loaded. `has_program`
  /// gates whether the Run segment can *start* a stream from Idle.
  pub fn for_state(state: BadgeState, has_program: bool) -> Self {
    match state {
      // No transport at all: the whole group is inert, Abort included (nothing to reset).
      BadgeState::Disconnected => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: false,
        abort_enabled: false,
      },
      // Connecting: a transport is open but the board is not yet ready. No clean Run/Hold/Stop, but the hard Abort
      // must stay live so a stalled handshake can be force-reset and the FD released.
      BadgeState::Connecting => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: false,
        abort_enabled: true,
      },
      // Idle: Run can start a stream if a program is loaded; nothing to hold; clean stop and hard abort available.
      BadgeState::Idle | BadgeState::Check | BadgeState::Sleep => TransportGroup {
        run_enabled: has_program,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
        abort_enabled: true,
      },
      // Actively moving: the leading segment is emphasised; hold, clean stop and hard abort are all live.
      BadgeState::Run | BadgeState::Jog | BadgeState::Home => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: true,
        hold_enabled: true,
        stop_enabled: true,
        abort_enabled: true,
      },
      // Held, door-suspended, or holding for a manual tool change: the leading segment becomes "Resume" and a
      // cycle-start (`~`) continues. A tool change resumes through the very same path as a feed hold, so it shares
      // this row rather than introducing a second resume control. Clean stop and hard abort stay live throughout.
      BadgeState::Hold | BadgeState::Door | BadgeState::Tool => TransportGroup {
        run_enabled: true,
        run_is_resume: true,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
        abort_enabled: true,
      },
      // Faulted: a graceful stop is a no-op here, but the operator can still issue it; the hard Abort is the real
      // recovery. Keep both live alongside the banner's own actions; no run/hold.
      BadgeState::Alarm | BadgeState::Error => TransportGroup {
        run_enabled: false,
        run_is_resume: false,
        run_active: false,
        hold_enabled: false,
        stop_enabled: true,
        abort_enabled: true,
      },
    }
  }
}

// The human glosses for `ALARM:N` / `error:N` codes used to live here as hardcoded `&'static str` tables. They
// were superseded by [`crate::protocol::codes`], whose canonical static fallback (corrected for the old 4/5
// probe-alarm inversion) is enriched at runtime from the firmware's `$EE`/`$EA` enumeration via the
// [`crate::protocol::CodeBook`]. The banner and console now both decode through that single source.

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
  fn an_unknown_run_state_defers_to_the_lifecycle() {
    assert_eq!(BadgeState::derive(ConnectionState::Streaming, Some(RunState::Unknown)), BadgeState::Run);
  }

  #[test]
  fn the_tool_run_state_drives_its_own_badge_distinct_from_hold() {
    // An M6 manual tool change is now its own badge state — NOT folded into the host lifecycle (which would have
    // shown Idle/Run) — so the UI can surface the tool-change affordance. It arrives mid-stream, so the connection
    // is typically `Streaming`, but the firmware run state wins regardless.
    assert_eq!(BadgeState::derive(ConnectionState::Streaming, Some(RunState::Tool)), BadgeState::Tool);
    assert_eq!(BadgeState::derive(ConnectionState::Idle, Some(RunState::Tool)), BadgeState::Tool);
    assert_ne!(BadgeState::Tool, BadgeState::Hold, "Tool change is distinct from a feed hold");
  }

  #[test]
  fn the_tool_change_resumes_through_the_same_cycle_start_path_as_a_hold() {
    // The Tool-change transport row must read "Resume" and route through the existing cycle-start path, exactly
    // like a feed hold — no second resume control. Stop and Abort stay live so the operator can still bail out.
    let tool = TransportGroup::for_state(BadgeState::Tool, false);
    assert!(tool.run_enabled && tool.run_is_resume, "Tool change offers Resume");
    assert!(!tool.hold_enabled, "nothing to hold while already paused for a tool change");
    assert!(tool.stop_enabled && tool.abort_enabled);
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
      BadgeState::Tool,
      BadgeState::Alarm,
      BadgeState::Error,
    ];
    let labels: Vec<&str> = states.iter().map(|s| s.label()).collect();
    let unique: std::collections::HashSet<&str> = labels.iter().copied().collect();
    assert_eq!(labels.len(), unique.len(), "labels must be unique per state");
    assert!(labels.iter().all(|l| l == &l.to_uppercase()), "labels are uppercase");
  }

  #[test]
  fn every_badge_label_key_resolves_to_its_reference_label_in_en_us() {
    // The views render `label_key()` through `tr!`, so the en-US value MUST equal the untranslated reference
    // `label()` — otherwise a rename drifts the shipped English badge from the tested one. Checked against an
    // isolated bundle (not the global registry) so this stays deterministic and never races the global tests.
    let mut t = crate::i18n::Translator::new();
    t.load_text(crate::i18n::EN_US, crate::i18n::EN_US_FTL).expect("bundled en-US must parse");
    t.set_language(crate::i18n::EN_US);
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
      BadgeState::Tool,
      BadgeState::Alarm,
      BadgeState::Error,
    ];
    for state in states {
      let args = crate::i18n::fluent::FluentArgs::new();
      assert_eq!(t.translate(state.label_key(), &args), state.label(), "en-US badge value for {state:?}");
    }
  }

  #[test]
  fn transport_group_is_inert_while_disconnected() {
    let group = TransportGroup::for_state(BadgeState::Disconnected, true);
    assert!(!group.run_enabled && !group.hold_enabled && !group.stop_enabled);
    // With no transport there is nothing to abort either.
    assert!(!group.abort_enabled, "Abort is inert with no open transport");
  }

  #[test]
  fn abort_is_available_whenever_a_transport_is_attached_including_while_connecting() {
    // Abort (the emergency hard reset, `0x18`) must reach the board the instant a transport is open — even mid-
    // handshake (`Connecting`), where a stalled board still holds the FD — so the operator can always force a reset.
    // It is the one transport control that does not wait for readiness, mirroring the lifecycle's `has_transport`.
    assert!(!TransportGroup::for_state(BadgeState::Disconnected, false).abort_enabled);
    for state in [
      BadgeState::Connecting,
      BadgeState::Idle,
      BadgeState::Run,
      BadgeState::Jog,
      BadgeState::Home,
      BadgeState::Hold,
      BadgeState::Door,
      BadgeState::Check,
      BadgeState::Sleep,
      BadgeState::Tool,
      BadgeState::Alarm,
      BadgeState::Error,
    ] {
      assert!(
        TransportGroup::for_state(state, false).abort_enabled,
        "{state:?} holds a transport and must allow Abort",
      );
    }
  }

  #[test]
  fn the_graceful_stop_and_the_hard_abort_are_independently_gated() {
    // Stop (graceful, `0x86`) is gated like the rest of the transport group — it needs a live, ready board. Abort
    // (hard, `0x18`) is broader: it is live the moment a transport attaches. While `Connecting` the two diverge,
    // which is the whole point of splitting them: no clean stop yet, but an emergency reset always.
    let connecting = TransportGroup::for_state(BadgeState::Connecting, true);
    assert!(!connecting.stop_enabled, "no graceful Stop before the board is ready");
    assert!(connecting.abort_enabled, "Abort is available while connecting");
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

}
