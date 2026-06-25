//! UI intents: the one-way vocabulary the views emit and the shell translates into engine [`Command`]s.
//!
//! Keeping intents separate from [`crate::engine::Command`] lets the views speak in UI terms (connect to a
//! named port, jog by a step, load a file) while the shell owns the policy of turning those into the engine's
//! lower-level commands and side effects (opening a transport, reading a file, echoing to the console). The
//! views never touch the engine handle directly, which keeps them pure renderers of [`super::ViewState`].

use crate::app::badge::BadgeState;
use crate::app::overrides::OverrideAxis;
use crate::protocol::RealtimeCommand;

/// An axis the jog controls target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
  /// X axis.
  X,
  /// Y axis.
  Y,
  /// Z axis.
  Z,
}

impl Axis {
  /// The G-code axis letter.
  pub fn letter(self) -> char {
    match self {
      Axis::X => 'X',
      Axis::Y => 'Y',
      Axis::Z => 'Z',
    }
  }

  /// The index of this axis in a machine-coordinate position vector (X=0, Y=1, Z=2), matching the `[PRB:]` /
  /// `MPos:` report order. Used to pull the radial reading for the probed axis out of a `ProbeResult` position.
  pub fn index(self) -> usize {
    match self {
      Axis::X => 0,
      Axis::Y => 1,
      Axis::Z => 2,
    }
  }
}

/// A direction along an axis: positive or negative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
  /// Toward increasing coordinate.
  Pos,
  /// Toward decreasing coordinate.
  Neg,
}

impl Dir {
  /// The signed multiplier (`+1.0` / `-1.0`) this direction applies to a jog distance.
  pub fn sign(self) -> f64 {
    match self {
      Dir::Pos => 1.0,
      Dir::Neg => -1.0,
    }
  }
}

/// An intent emitted by a view. The shell drains these each frame and acts on them; nothing here performs I/O.
#[derive(Debug, Clone, PartialEq)]
pub enum Intent {
  /// Open the serial port at the given path/baud and attach the engine.
  Connect { path: String, baud: u32 },
  /// Tear down the connection.
  Disconnect,
  /// Re-enumerate available serial ports (refresh the port dropdown).
  RefreshPorts,
  /// Actively probe the port at `path` to confirm it speaks grblHAL: open it, send `?`/`$I`, and wait briefly
  /// for grbl evidence, then surface the verdict. Opt-in only — opening the ESP32-S3 toggles its auto-reset
  /// line, so this never runs as part of a refresh. Refused while connected (the engine already holds a port).
  IdentifyPort { path: String },

  /// Stream the lines of the file at this path as a program.
  OpenProgram(std::path::PathBuf),
  /// Begin streaming the currently loaded program.
  StartStream,
  /// Simulate the loaded program: build a physics-based job-time estimate ([`crate::eta::EtaTimeline`]) from the
  /// firmware `$$` settings (falling back to machine defaults when none are loaded) over the loaded file. A PURE
  /// host computation — it sends no engine command and needs no live link — so it works while disconnected. The
  /// shell stores the timeline so the upfront ETA shows before a stream and the live remaining drains physically.
  Simulate,

  /// Send a single manual G-code/`$` line entered in the console.
  SendLine(String),
  /// Wipe the console log buffer — the right-click "Clear" affordance on the console body. A pure view-state
  /// mutation (no I/O); the shell handles it by calling [`super::ViewState::clear_console`].
  ClearConsole,
  /// Inject a real-time single-byte command out-of-band.
  Realtime(RealtimeCommand),
  /// Drive a feed/spindle override slider to an absolute `target` percent. grbl has no "set override to N%"
  /// command, only relative ±10/±1/reset steps, so the shell reads the live `Ov:` value and emits the minimal
  /// step sequence via [`crate::app::overrides::override_commands`]. Carries only the intent (axis + target);
  /// the current value and the byte arithmetic stay out of the view.
  SetOverride { axis: OverrideAxis, target: u32 },

  /// Jog `axis` in `dir` by `distance` (mm) at `feed` (mm/min). The shell forms the `$J=` line.
  Jog { axis: Axis, dir: Dir, distance: f64, feed: f64 },
  /// Begin a continuous (press-and-hold) jog. The shell streams short `$J=` increments at `feed` for as long as
  /// the control is held, rather than one long move — so a jog-cancel on release stops within one short block
  /// instead of running a long move to its far boundary. Emitted on the press edge; ended by [`Intent::JogStop`].
  JogStart { axis: Axis, dir: Dir, feed: f64 },
  /// End a continuous jog: stop streaming increments and inject jog-cancel (`0x85`), which flushes the queued jog
  /// blocks and halts the active one. Safe to send when not jogging — the firmware ignores it. Emitted on release.
  JogStop,

  /// Dismiss the latched alarm/error banner.
  DismissBanner,
  /// Probe Z with `G38.2` toward `depth` (mm, negative) at `feed` (mm/min), then set work-Z to
  /// `plate_thickness`. The shell sequences the probe + zeroing lines.
  ProbeZ { depth: f64, feed: f64, plate_thickness: f64 },

  /// Fetch the firmware's settings: send `$$` (live values) and `$ES` (the enumeration metadata that labels
  /// each row), so the settings panel populates from the controller rather than a hardcoded table.
  RequestSettings,
  /// Write one setting edit back to the firmware as a `$<n>=<value>` line. The firmware validates the value and
  /// answers `ok`/`error:N`; the shell re-reads the single setting afterwards so the panel reflects the truth.
  WriteSetting { number: u32, value: String },
  /// Commit every staged settings edit: the shell flushes the dirty store as ordered `$<n>=<value>` writes, then
  /// `$$` to re-confirm, then clears the staging. The single explicit Save boundary for the settings dialog —
  /// the per-field edits only stage locally, so this is what actually reaches the firmware.
  SaveSettings,

  /// Run the homing cycle (`$H`).
  Home,
  /// Begin streaming if idle, or resume from a feed hold (`~`) — the toolbar Run/Resume segment. The shell
  /// chooses between starting a stream and a cycle-start based on the live state.
  RunOrResume,
  /// Set the work-coordinate zero on the given axes to the current position (`G10 L20 P0 …`). An empty set is
  /// treated as "all of X, Y, Z" (the "Zero XYZ" button).
  SetWorkZero { axes: Vec<Axis> },

  /// Start a fresh rotary center-finder run for a dowel of `dowel_diameter` (mm) indexed at `index_angle_deg`
  /// (DOC-11 §1.2), using the operator-tuned `params` for every rotary-safe touch (retract clearance, side-probe
  /// descent height, settle, feed, depth). The shell builds the [`crate::app::rotary_center::WizardState`] and
  /// waits for the operator to position and trigger each touch. Replaces any wizard already in progress.
  RotaryCenterStart {
    dowel_diameter: f64,
    index_angle_deg: f64,
    params: crate::app::rotary_probe::RotaryProbeParams,
  },
  /// Trigger the wizard's next touch (the operator has jogged to the approach): the shell asks the wizard which
  /// touch is due, emits the rotary-safe probe lines, and arms the Phase 0 latch. Inert if no wizard is running
  /// or one is already probing.
  RotaryCenterProbe,
  /// Send the wizard's "move to the computed Y center" positioning move, after both Y touches — the mandatory
  /// step before the top probe so it reads the diameter, not a chord. Inert unless the wizard is at that step.
  RotaryCenterMoveToYc,
  /// Write the found center to the active WCS via the wizard's offered `G10 L2` line (Y/Z only, never A). Inert
  /// until the wizard has a computed center.
  RotaryCenterWriteWcs,
  /// Select which feature work-Z0 lands on when the center is written: the rotary axis centerline (default) or
  /// the probed top surface. Only affects the Z word of the offered `G10 L2`. Inert if no wizard is running.
  RotaryCenterSetZDatum(crate::app::rotary_center::ZDatum),
  /// Cancel the rotary center-finder run, discarding its state.
  RotaryCenterCancel,
  /// Re-apply the rotary center saved in the profile (DOC-11 §1.3) to the active WCS — the `G10 L2` line the
  /// last center-finder run persisted (Y/Z only, never A) — so a restart restores the found center without
  /// re-probing. Inert if no center has ever been saved.
  ApplySavedRotaryCenter,

  /// Start a Phase 2 180°-flip center-verify (DOC-11 §2.1): probe a feature along `axis`/`dir` at `angle_deg`,
  /// then at `angle_deg + 180`, and compute the residual offset from the rotation centerline. Cancels any other
  /// in-flight probe op.
  FlipVerifyStart { angle_deg: f64, axis: Axis, dir: Dir },
  /// Start a Phase 2 runout report (DOC-11 §2.2): probe a feature along `axis`/`dir` at `n` evenly-spaced angles
  /// from `start_deg`, then report TIR / eccentricity. Read-only. Cancels any other in-flight probe op.
  RunoutStart { n: usize, start_deg: f64, axis: Axis, dir: Dir },
  /// Trigger the running sweep's next touch (the operator has jogged the approach). Inert if no sweep is running
  /// or one is already probing. Shared by both Phase 2 wizards.
  SweepProbe,
  /// Write the completed flip-verify's `G10 L2` correction (the verified axis only, never A). Inert unless a
  /// finished flip-verify sweep is present. (Runout never writes — it has no such intent.)
  FlipVerifyWriteCorrection,
  /// Cancel the running Phase 2 sweep, discarding its state.
  SweepCancel,
}

/// Build a `G10 L20 P0` line that sets the active work-coordinate system's offset so each listed axis reads
/// the given value at the current machine position (grbl's "set this position to N"). `L20` (not `L2`) sets
/// the offset *relative to the current position*, the correct semantics for "make here read N" — the building
/// block both the "Zero here" buttons and the Z-probe zeroing rely on, kept pure so it is unit-tested without
/// a window.
pub fn work_offset_line(values: &[(Axis, f64)]) -> String {
  let mut line = String::from("G10 L20 P0");
  for (axis, value) in values {
    line.push(' ');
    line.push(axis.letter());
    // `{:.3}` matches the precision the shell uses elsewhere; an integer 0 still prints as `0.000`, which grbl
    // parses identically to `0`.
    line.push_str(&format!("{value:.3}"));
  }
  line
}

/// Build a `G10 L20 P0` line that sets the active work-coordinate system's zero on the given axes to the
/// current machine position (grbl's "set work zero here"). An empty/zeroed `axes` list zeros all of X, Y, Z.
/// This is the pure formatting the "Zero X / Y / Z / XYZ" buttons rely on; it delegates to
/// [`work_offset_line`] with a value of `0.0` per axis so there is a single place that builds the offset line.
pub fn work_zero_line(axes: &[Axis]) -> String {
  let targets: &[Axis] = if axes.is_empty() { &[Axis::X, Axis::Y, Axis::Z] } else { axes };
  let values: Vec<(Axis, f64)> = targets.iter().map(|&axis| (axis, 0.0)).collect();
  work_offset_line(&values)
}

/// Build the `$J=` line for a jog: a relative (`G91`), millimetre (`G21`) move of `distance` mm along `axis` in
/// `dir` at `feed` mm/min. Jog is modal-independent in grblHAL, so the explicit `G91 G21` prefix keeps it
/// predictable regardless of the running program's modal context. Pure so the wire form is unit-tested without a
/// window. Used for both step jogs and the short increments the shell streams for a continuous (held) jog.
pub fn jog_line(axis: Axis, dir: Dir, distance: f64, feed: f64) -> String {
  let signed = distance * dir.sign();
  format!("$J=G91 G21 {}{:.3} F{:.0}", axis.letter(), signed, feed)
}

/// A keyboard chord the shell lifts out of egui's per-frame input and feeds to [`key_to_intent`]. Kept as a
/// small egui-free enum (rather than `egui::Key`) so the keyboard policy is a pure mapping unit-tested without
/// a window; the shell does the thin translation from `egui::Key` to this on the keys it cares about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hotkey {
  /// An arrow key: jog X/Y in the plane (←/→ = X∓, ↑/↓ = Y±).
  ArrowLeft,
  /// See [`Hotkey::ArrowLeft`].
  ArrowRight,
  /// See [`Hotkey::ArrowLeft`].
  ArrowUp,
  /// See [`Hotkey::ArrowLeft`].
  ArrowDown,
  /// Page Up: jog Z up (away from the work).
  PageUp,
  /// Page Down: jog Z down (toward the work).
  PageDown,
  /// Escape: cancel a jog, or — when not jogging — soft-reset/abort.
  Escape,
  /// Feed hold (the `!` realtime command), bound to `H` for one-handed pause.
  FeedHold,
  /// Cycle start / resume (the `~` realtime command), bound to `R`.
  CycleResume,
}

impl Hotkey {
  /// The `(axis, dir)` a directional hotkey jogs, or `None` for the non-jog chords (Escape/hold/resume). The
  /// machine-bed convention: ↑ = Y+, ↓ = Y−, → = X+, ← = X−, PageUp = Z+, PageDown = Z−.
  fn jog_motion(self) -> Option<(Axis, Dir)> {
    match self {
      Hotkey::ArrowRight => Some((Axis::X, Dir::Pos)),
      Hotkey::ArrowLeft => Some((Axis::X, Dir::Neg)),
      Hotkey::ArrowUp => Some((Axis::Y, Dir::Pos)),
      Hotkey::ArrowDown => Some((Axis::Y, Dir::Neg)),
      Hotkey::PageUp => Some((Axis::Z, Dir::Pos)),
      Hotkey::PageDown => Some((Axis::Z, Dir::Neg)),
      _ => None,
    }
  }
}

/// Map a pressed [`Hotkey`] to the [`Intent`] it should fire, given the live machine state and the jog
/// settings. Pure so the keyboard policy is unit-tested without a window.
///
/// Rules, all chosen to never command a move the firmware would reject or that would surprise the operator:
/// - Directional keys issue a **step** jog of `jog_step` mm at `jog_feed`, but only while the machine is in a
///   state that accepts `$J=` (`Idle`/`Jog`); in any other state they are inert, matching the jog pad's own
///   enable gate. A step jog (not a continuous one) is used for the keyboard because key-repeat already gives
///   a natural press-and-hold cadence and a held key that ends without a clean release must never leave the
///   axis coasting.
/// - `Escape` cancels a jog while jogging (`0x85`), else issues a soft reset/abort (`0x18`) — the universal
///   "stop now" reflex. It is always available while connected so it can rescue a runaway.
/// - `FeedHold`/`CycleResume` map to `!`/`~` whenever connected, mirroring the toolbar Hold/Resume.
///
/// Returns `None` when the key has no effect in the current state (e.g. an arrow key while streaming), so the
/// shell can leave egui's normal handling of that key untouched.
pub fn key_to_intent(key: Hotkey, badge: BadgeState, jog_step: f64, jog_feed: f64) -> Option<Intent> {
  let connected = !matches!(badge, BadgeState::Disconnected | BadgeState::Connecting);
  match key {
    Hotkey::Escape if connected => {
      // While a jog is running, Escape cancels just the jog; otherwise it is the panic soft-reset.
      let cmd = if matches!(badge, BadgeState::Jog) {
        RealtimeCommand::JogCancel
      } else {
        RealtimeCommand::SoftReset
      };
      Some(Intent::Realtime(cmd))
    }
    Hotkey::FeedHold if connected => Some(Intent::Realtime(RealtimeCommand::FeedHold)),
    Hotkey::CycleResume if connected => Some(Intent::Realtime(RealtimeCommand::CycleStart)),
    // Directional jogs only fire where grblHAL accepts `$J=` (Idle/Jog), exactly like the jog pad's gate.
    _ => {
      let (axis, dir) = key.jog_motion()?;
      if matches!(badge, BadgeState::Idle | BadgeState::Jog) {
        Some(Intent::Jog { axis, dir, distance: jog_step, feed: jog_feed })
      } else {
        None
      }
    }
  }
}

/// A frame-scoped sink the views push [`Intent`]s into. The shell creates one per frame, passes `&mut` to the
/// views, then drains it. A plain `Vec` is right: there are only a handful of intents per frame and ordering
/// matters (e.g. open-then-start).
#[derive(Debug, Default)]
pub struct IntentSink {
  intents: Vec<Intent>,
}

impl IntentSink {
  /// A fresh, empty sink.
  pub fn new() -> Self {
    Self::default()
  }

  /// Record an intent for the shell to act on after the frame is built.
  pub fn push(&mut self, intent: Intent) {
    self.intents.push(intent);
  }

  /// Take all recorded intents, leaving the sink empty.
  pub fn drain(&mut self) -> Vec<Intent> {
    std::mem::take(&mut self.intents)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn axis_letters_and_dir_signs() {
    assert_eq!(Axis::X.letter(), 'X');
    assert_eq!(Axis::Z.letter(), 'Z');
    assert_eq!(Dir::Pos.sign(), 1.0);
    assert_eq!(Dir::Neg.sign(), -1.0);
  }

  #[test]
  fn work_zero_line_zeros_named_axes_or_all_three() {
    assert_eq!(work_zero_line(&[Axis::Z]), "G10 L20 P0 Z0.000");
    assert_eq!(work_zero_line(&[Axis::X, Axis::Y]), "G10 L20 P0 X0.000 Y0.000");
    // An empty set means "Zero XYZ".
    assert_eq!(work_zero_line(&[]), "G10 L20 P0 X0.000 Y0.000 Z0.000");
  }

  #[test]
  fn work_offset_line_sets_per_axis_values() {
    // The shared builder formats each axis at 3-decimal precision; this is what the Z-probe zeroing uses.
    assert_eq!(work_offset_line(&[(Axis::Z, 1.5)]), "G10 L20 P0 Z1.500");
    assert_eq!(work_offset_line(&[(Axis::X, 2.0), (Axis::Y, -3.25)]), "G10 L20 P0 X2.000 Y-3.250");
  }

  #[test]
  fn jog_line_forms_a_relative_mm_move() {
    // A step jog is an explicit `G91 G21` relative-mm move so it is independent of the program's modal state.
    assert_eq!(jog_line(Axis::X, Dir::Pos, 1.0, 500.0), "$J=G91 G21 X1.000 F500");
    assert_eq!(jog_line(Axis::Y, Dir::Neg, 0.5, 250.0), "$J=G91 G21 Y-0.500 F250");
  }

  #[test]
  fn arrow_keys_step_jog_only_when_jogging_is_allowed() {
    // In Idle an arrow fires a step jog of the configured step/feed in the matching axis/dir.
    let intent = key_to_intent(Hotkey::ArrowRight, BadgeState::Idle, 2.0, 400.0);
    assert_eq!(intent, Some(Intent::Jog { axis: Axis::X, dir: Dir::Pos, distance: 2.0, feed: 400.0 }));
    // PageDown jogs Z down; PageUp jogs Z up.
    assert_eq!(
      key_to_intent(Hotkey::PageDown, BadgeState::Idle, 1.0, 300.0),
      Some(Intent::Jog { axis: Axis::Z, dir: Dir::Neg, distance: 1.0, feed: 300.0 })
    );
    // While running, an arrow key is inert — the firmware would reject a `$J=` there anyway.
    assert_eq!(key_to_intent(Hotkey::ArrowUp, BadgeState::Run, 1.0, 300.0), None);
    // Disconnected: nothing to command.
    assert_eq!(key_to_intent(Hotkey::ArrowLeft, BadgeState::Disconnected, 1.0, 300.0), None);
  }

  #[test]
  fn escape_cancels_a_jog_but_soft_resets_otherwise() {
    // Mid-jog, Escape cancels just the jog (a gentle stop, no alarm).
    assert_eq!(
      key_to_intent(Hotkey::Escape, BadgeState::Jog, 1.0, 300.0),
      Some(Intent::Realtime(RealtimeCommand::JogCancel))
    );
    // Not jogging, Escape is the panic soft-reset/abort.
    assert_eq!(
      key_to_intent(Hotkey::Escape, BadgeState::Run, 1.0, 300.0),
      Some(Intent::Realtime(RealtimeCommand::SoftReset))
    );
    // Disconnected, there is nothing to reset.
    assert_eq!(key_to_intent(Hotkey::Escape, BadgeState::Disconnected, 1.0, 300.0), None);
  }

  #[test]
  fn hold_and_resume_keys_map_to_their_realtime_bytes_when_connected() {
    assert_eq!(
      key_to_intent(Hotkey::FeedHold, BadgeState::Run, 1.0, 300.0),
      Some(Intent::Realtime(RealtimeCommand::FeedHold))
    );
    assert_eq!(
      key_to_intent(Hotkey::CycleResume, BadgeState::Hold, 1.0, 300.0),
      Some(Intent::Realtime(RealtimeCommand::CycleStart))
    );
    // Both are inert while disconnected.
    assert_eq!(key_to_intent(Hotkey::FeedHold, BadgeState::Disconnected, 1.0, 300.0), None);
  }

  #[test]
  fn sink_records_and_drains_in_order() {
    let mut sink = IntentSink::new();
    sink.push(Intent::RefreshPorts);
    sink.push(Intent::Disconnect);
    let drained = sink.drain();
    assert_eq!(drained, vec![Intent::RefreshPorts, Intent::Disconnect]);
    assert!(sink.drain().is_empty(), "drain leaves the sink empty");
  }
}
