//! The rotary-safe probe primitive: one index-then-probe touch, as a pure line builder.
//!
//! Every rotary probe is **index-then-probe**, never probe-while-rotating — probing while the A axis turns
//! rotates the surface normal under a fixed probe vector (cosine error, invalid tip-radius compensation), so the
//! universal practice is to index A to a fixed angle, HOLD, then probe linearly. This module emits the exact line
//! sequence for one such touch; the center-finder wizard ([`super::rotary_center`]) composes it, and the shell
//! sends the lines and awaits the result via the Phase 0 latch ([`super::view_state::ProbeOp`]).
//!
//! The sequence (DOC-11 §1.1) is:
//! 1. `G53 G0 Z<clearance>` — retract Z to a MACHINE-coordinate safe clearance (`G53` = move in machine coords,
//!    so the clearance is absolute regardless of the active WCS), clearing the dowel before the rotary indexes.
//! 2. `G0 A<angle>` — index the rotary to the touch angle.
//! 3. `G4 P<settle>` — a settle dwell so rotary backlash/oscillation damps out before the probe.
//! 4. `G38.2 <axis><signed depth> F<feed>` — a single LINEAR probe along X/Y/Z. **Never an `A` word** — the
//!    firmware rejects a rotary word in a probe (verified), and probing through a rotary axis is unsound.
//!
//! `clearance` and `settle` are **bench-tuned** parameters surfaced in the UI (see [`RotaryProbeParams`]); this
//! module never invents magic numbers — the caller passes them in. Pure and synchronous, so the emitted sequence
//! is unit-tested without a window or hardware.

use super::intent::{Axis, Dir};

/// Conservative default Z retract clearance (machine coordinates, mm) before a rotary index. grbl machine-Z is
/// typically negative below the home/top, so a small negative default keeps the tool clear without assuming the
/// machine's exact travel. Surfaced in the UI and overridable — bench-tuned, not load-bearing.
pub const DEFAULT_CLEARANCE_MM: f64 = -2.0;

/// Conservative default settle dwell (seconds) after a rotary index, before the probe, so backlash/oscillation
/// damps out. Surfaced in the UI and overridable — bench-tuned.
pub const DEFAULT_SETTLE_SECS: f64 = 0.5;

/// Conservative default probe feed (mm/min) for a rotary touch. Matches the cautious touch-off feed elsewhere in
/// the app; surfaced in the UI and overridable.
pub const DEFAULT_PROBE_FEED: f64 = 50.0;

/// Conservative default probe travel (mm) — how far the linear `G38.2` advances looking for contact before it
/// gives up (a no-contact probe runs this far, then alarms). Surfaced in the UI and overridable.
pub const DEFAULT_PROBE_DEPTH_MM: f64 = 10.0;

/// The bench-tuned parameters shared across every rotary-safe touch in a wizard run. Held once (in the wizard /
/// UI state) and passed to [`rotary_safe_probe_lines`] per touch, so a single set of conservative defaults
/// applies to the whole run rather than being re-entered per probe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotaryProbeParams {
  /// Z retract clearance in MACHINE coordinates (mm), emitted as `G53 G0 Z<clearance>` before the index.
  pub clearance_mm: f64,
  /// Settle dwell (seconds) after the index, emitted as `G4 P<settle>` before the probe.
  pub settle_secs: f64,
  /// Probe feed (mm/min) for the `G38.2`.
  pub feed: f64,
  /// Probe travel magnitude (mm); the signed direction comes from the per-touch [`Dir`].
  pub depth_mm: f64,
}

impl Default for RotaryProbeParams {
  fn default() -> Self {
    RotaryProbeParams {
      clearance_mm: DEFAULT_CLEARANCE_MM,
      settle_secs: DEFAULT_SETTLE_SECS,
      feed: DEFAULT_PROBE_FEED,
      depth_mm: DEFAULT_PROBE_DEPTH_MM,
    }
  }
}

/// One rotary-safe touch: index the rotary to `angle`, then probe along `axis` in `dir`, using the shared bench
/// `params`. Pure description of a single touch the wizard composes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotaryTouch {
  /// The A angle (degrees) to index to before probing.
  pub angle_deg: f64,
  /// The linear axis (X/Y/Z) to probe along — never A.
  pub axis: Axis,
  /// The direction along that axis the probe advances.
  pub dir: Dir,
}

/// Emit the exact line sequence for one rotary-safe touch: retract, index (absolute), settle, a single
/// RELATIVE linear probe, then restore absolute mode. Returns the five lines in send order. The probe line
/// carries only the chosen linear axis word — **never** an `A` word (the firmware rejects a rotary word in a
/// probe, and probing through rotation is unsound).
///
/// **The probe must be RELATIVE.** Under the power-on default `G90` (absolute), `G38.2 Z-<depth>` resolves the
/// target as an absolute WORK coordinate — so after the operator jogs to an approach, the probe would travel to
/// the wrong place (a wrong-distance/wrong-direction uncontrolled move). Per `docs/tlo-offsets.md`, a probe is
/// wrapped in `G91` (incremental) … `G90`, so `G38.2 <axis>-<depth>` advances `<depth>` mm FROM the current
/// position. The index move stays ABSOLUTE (`G90 G0 A<angle>` indexes A to the angle, not a relative turn).
///
/// Formatting matches the rest of the app: positions at 3 decimals, feed as an integer. The probe distance is
/// `params.depth_mm` signed by `touch.dir`, so a `Dir::Neg` Z probe reads `G38.2 Z-<depth>`.
pub fn rotary_safe_probe_lines(touch: RotaryTouch, params: RotaryProbeParams) -> Vec<String> {
  let signed_depth = params.depth_mm * touch.dir.sign();
  vec![
    // 1. Retract Z to the machine-coordinate clearance (G53 = machine coords) so the index never drags the tool
    //    through the dowel.
    format!("G53 G0 Z{:.3}", params.clearance_mm),
    // 2. Index the rotary to the touch angle — ABSOLUTE (`G90`), so A lands at the angle regardless of mode.
    format!("G90 G0 A{:.3}", touch.angle_deg),
    // 3. Settle dwell so backlash/oscillation damps out before the probe.
    format!("G4 P{:.3}", params.settle_secs),
    // 4. A single LINEAR, RELATIVE probe (`G91`) — advances `signed_depth` mm from the current position, not to an
    //    absolute coordinate. Only the linear axis word, never A.
    format!("G91 G38.2 {}{:.3} F{:.0}", touch.axis.letter(), signed_depth, params.feed),
    // 5. Restore absolute mode so subsequent moves (index, positioning) are not silently incremental.
    "G90".to_string(),
  ]
}

#[cfg(test)]
mod tests {
  use super::*;

  fn params() -> RotaryProbeParams {
    RotaryProbeParams { clearance_mm: -2.0, settle_secs: 0.5, feed: 50.0, depth_mm: 10.0 }
  }

  /// The probe line is the one starting with `G91 G38.2` (now line index 3), not the trailing `G90`.
  fn probe_line(lines: &[String]) -> &String {
    lines.iter().find(|l| l.contains("G38.2")).expect("a probe line")
  }

  #[test]
  fn emits_the_retract_index_settle_relative_probe_restore_sequence_in_order() {
    let touch = RotaryTouch { angle_deg: 90.0, axis: Axis::Y, dir: Dir::Neg };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(lines, vec![
      "G53 G0 Z-2.000".to_string(),
      "G90 G0 A90.000".to_string(),
      "G4 P0.500".to_string(),
      "G91 G38.2 Y-10.000 F50".to_string(),
      "G90".to_string(),
    ]);
  }

  #[test]
  fn the_probe_is_relative_and_restored_to_absolute_afterward() {
    // Under G90 a `G38.2 Z-d` would resolve as an ABSOLUTE target (wrong place / crash). The probe must be
    // wrapped G91…G90: incremental for the probe, then back to absolute so later moves are not silently relative.
    let touch = RotaryTouch { angle_deg: 0.0, axis: Axis::Z, dir: Dir::Neg };
    let lines = rotary_safe_probe_lines(touch, params());
    assert!(probe_line(&lines).starts_with("G91 G38.2 "), "the probe must be incremental (G91); got {lines:?}");
    assert_eq!(lines.last().unwrap(), "G90", "absolute mode must be restored after the probe; got {lines:?}");
  }

  #[test]
  fn the_probe_line_never_carries_an_a_word() {
    // The defining safety property: no probe line may contain a rotary `A` word (the firmware rejects it). Check
    // the probe line across several angles/axes.
    for (axis, dir) in [(Axis::X, Dir::Pos), (Axis::Y, Dir::Neg), (Axis::Z, Dir::Neg)] {
      let touch = RotaryTouch { angle_deg: 123.456, axis, dir };
      let lines = rotary_safe_probe_lines(touch, params());
      let probe = probe_line(&lines);
      assert!(!probe.contains('A'), "a probe line must never contain an A word; got {probe:?}");
    }
  }

  #[test]
  fn the_index_carries_the_a_angle_absolutely_but_not_the_probe() {
    // The `A` word belongs ONLY to the index move, never the probe — and the index is ABSOLUTE (G90).
    let touch = RotaryTouch { angle_deg: 180.0, axis: Axis::Y, dir: Dir::Pos };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(lines[1], "G90 G0 A180.000", "the index move carries the A angle, absolutely");
  }

  #[test]
  fn a_positive_direction_probes_toward_increasing_coordinate() {
    let touch = RotaryTouch { angle_deg: 0.0, axis: Axis::Y, dir: Dir::Pos };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(probe_line(&lines), "G91 G38.2 Y10.000 F50", "a Dir::Pos probe advances toward +Y");
  }

  #[test]
  fn the_retract_is_a_machine_coordinate_g53_move() {
    // The clearance must be a `G53` (machine-coordinate) move so it is absolute regardless of the active WCS —
    // otherwise a set work offset could make the "safe" Z land in the part.
    let touch = RotaryTouch { angle_deg: 0.0, axis: Axis::Z, dir: Dir::Neg };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(lines[0], "G53 G0 Z-2.000", "the retract must be a G53 machine-coordinate move");
  }
}
