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

/// Default machine-Z (mm) a SIDE (X/Y) probe descends to before it goes in laterally — it must sit WITHIN the
/// dowel's Z-extent (below the top, above the bottom) or the lateral probe never meets the flank. This is
/// SETUP-SPECIFIC (it depends on where the dowel is mounted), so the default is only a placeholder the operator
/// MUST tune for the bench; it is intentionally well below the clearance default so the side touch reaches a real
/// flank rather than sweeping over the top. Used ONLY for X/Y touches — a Z (top) touch descends as the probe
/// itself and ignores it.
pub const DEFAULT_SIDE_PROBE_Z: f64 = -10.0;

/// The bench-tuned parameters shared across every rotary-safe touch in a wizard run. Held once (in the wizard /
/// UI state) and passed to [`rotary_safe_probe_lines`] per touch, so a single set of conservative defaults
/// applies to the whole run rather than being re-entered per probe.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RotaryProbeParams {
  /// Z retract clearance in MACHINE coordinates (mm), emitted as `G53 G0 Z<clearance>` before the index.
  pub clearance_mm: f64,
  /// Settle dwell (seconds) after the index, emitted as `G4 P<settle>` before the probe.
  pub settle_secs: f64,
  /// Probe feed (mm/min) for the `G38.2`.
  pub feed: f64,
  /// Probe travel magnitude (mm); the signed direction comes from the per-touch [`Dir`].
  pub depth_mm: f64,
  /// Machine-Z (mm) a SIDE (X/Y) touch descends to after the index, before probing laterally — emitted as
  /// `G53 G0 Z<side_probe_z>`. It must lie within the dowel's Z-extent for the lateral probe to meet the flank,
  /// and because it is a single shared value BOTH opposing side touches probe at the identical height — which is
  /// exactly what makes their midpoint cancel the tool radius (a height mismatch would bias `Y_c`). A Z (top)
  /// touch ignores it and probes straight down from `clearance_mm`.
  pub side_probe_z: f64,
}

impl Default for RotaryProbeParams {
  fn default() -> Self {
    RotaryProbeParams {
      clearance_mm: DEFAULT_CLEARANCE_MM,
      settle_secs: DEFAULT_SETTLE_SECS,
      feed: DEFAULT_PROBE_FEED,
      depth_mm: DEFAULT_PROBE_DEPTH_MM,
      side_probe_z: DEFAULT_SIDE_PROBE_Z,
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

/// Emit the exact line sequence for one rotary-safe touch: retract, index (absolute), settle, (for a SIDE touch)
/// descend to the side-probe height, a single RELATIVE linear probe, then restore absolute mode. Returns the
/// lines in send order. The probe line carries only the chosen linear axis word — **never** an `A` word (the
/// firmware rejects a rotary word in a probe, and probing through rotation is unsound).
///
/// **Side (X/Y) vs top (Z) touches need different Z handling — this is the crux.** The retract lifts the tool
/// CLEAR of the dowel so the A index never drags through it. But a SIDE probe then has to come back DOWN into the
/// dowel's Z-extent before it can meet the flank — at the operator's approach Y the tool is off to the side, so
/// the descent to `side_probe_z` is clear of the dowel. A single `clearance_mm` cannot serve both: above the top
/// (safe for the index and for a top probe to descend onto) is too high for a lateral probe to ever touch the
/// flank. So a side touch inserts a `G53 G0 Z<side_probe_z>` step after the settle; a TOP (Z) touch omits it and
/// drops straight down from the clearance height (the descent IS the probe). Sharing one `side_probe_z` keeps the
/// two opposing side touches at an identical height, which is what lets their midpoint cancel the tool radius.
///
/// **The probe must be RELATIVE.** Under the power-on default `G90` (absolute), `G38.2 Z-<depth>` resolves the
/// target as an absolute WORK coordinate — so after the operator jogs to an approach, the probe would travel to
/// the wrong place (a wrong-distance/wrong-direction uncontrolled move). Per `docs/tlo-offsets.md`, a probe is
/// wrapped in `G91` (incremental) … `G90`, so `G38.2 <axis>-<depth>` advances `<depth>` mm FROM the current
/// position. The index and the side descent stay ABSOLUTE machine moves (`G90 G0 A<angle>`, `G53 G0 Z…`).
///
/// Formatting matches the rest of the app: positions at 3 decimals, feed as an integer. The probe distance is
/// `params.depth_mm` signed by `touch.dir`, so a `Dir::Neg` Z probe reads `G38.2 Z-<depth>`.
pub fn rotary_safe_probe_lines(touch: RotaryTouch, params: RotaryProbeParams) -> Vec<String> {
  let signed_depth = params.depth_mm * touch.dir.sign();
  let mut lines = vec![
    // 1. Retract Z to the machine-coordinate clearance (G53 = machine coords) so the index never drags the tool
    //    through the dowel.
    format!("G53 G0 Z{:.3}", params.clearance_mm),
    // 2. Index the rotary to the touch angle — ABSOLUTE (`G90`), so A lands at the angle regardless of mode.
    format!("G90 G0 A{:.3}", touch.angle_deg),
    // 3. Settle dwell so backlash/oscillation damps out before the probe.
    format!("G4 P{:.3}", params.settle_secs),
  ];
  // 4. A SIDE (X/Y) touch must descend to the in-dowel side-probe height before probing laterally — the descent
  //    happens at the approach Y, clear of the dowel. A TOP (Z) touch skips this and descends as the probe itself.
  if matches!(touch.axis, Axis::X | Axis::Y) {
    lines.push(format!("G53 G0 Z{:.3}", params.side_probe_z));
  }
  // 5. A single LINEAR, RELATIVE probe (`G91`) — advances `signed_depth` mm from the current position, not to an
  //    absolute coordinate. Only the linear axis word, never A.
  lines.push(format!("G91 G38.2 {}{:.3} F{:.0}", touch.axis.letter(), signed_depth, params.feed));
  // 6. Restore absolute mode so subsequent moves (index, positioning) are not silently incremental.
  lines.push("G90".to_string());
  lines
}

#[cfg(test)]
mod tests {
  use super::*;

  fn params() -> RotaryProbeParams {
    RotaryProbeParams { clearance_mm: -2.0, settle_secs: 0.5, feed: 50.0, depth_mm: 10.0, side_probe_z: -8.0 }
  }

  /// The probe line is the one starting with `G91 G38.2` (now line index 3), not the trailing `G90`.
  fn probe_line(lines: &[String]) -> &String {
    lines.iter().find(|l| l.contains("G38.2")).expect("a probe line")
  }

  #[test]
  fn emits_the_retract_index_settle_descend_relative_probe_restore_sequence_for_a_side_touch() {
    // A SIDE (Y) touch retracts clear, indexes, settles, DESCENDS to the side-probe height, then probes laterally.
    let touch = RotaryTouch { angle_deg: 90.0, axis: Axis::Y, dir: Dir::Neg };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(lines, vec![
      "G53 G0 Z-2.000".to_string(),
      "G90 G0 A90.000".to_string(),
      "G4 P0.500".to_string(),
      "G53 G0 Z-8.000".to_string(),
      "G91 G38.2 Y-10.000 F50".to_string(),
      "G90".to_string(),
    ]);
  }

  #[test]
  fn a_top_z_touch_omits_the_side_descend_and_drops_from_the_clearance_height() {
    // A TOP (Z) touch must NOT descend to side_probe_z first — the probe IS the descent, straight down from the
    // clearance height. Inserting a side descend here would drive the tool into the dowel top before probing.
    let touch = RotaryTouch { angle_deg: 0.0, axis: Axis::Z, dir: Dir::Neg };
    let lines = rotary_safe_probe_lines(touch, params());
    assert_eq!(lines, vec![
      "G53 G0 Z-2.000".to_string(),
      "G90 G0 A0.000".to_string(),
      "G4 P0.500".to_string(),
      "G91 G38.2 Z-10.000 F50".to_string(),
      "G90".to_string(),
    ]);
    // Belt-and-suspenders: only the single clearance retract appears, never a second (side) Z move.
    assert_eq!(lines.iter().filter(|l| l.starts_with("G53 G0 Z")).count(), 1);
  }

  #[test]
  fn both_opposing_side_touches_descend_to_the_identical_height_so_their_midpoint_cancels_the_radius() {
    // The correctness property the fix exists for: the two opposing Y touches must probe at the SAME Z, else the
    // chord half-widths differ and the midpoint is biased off the true center. One shared side_probe_z guarantees
    // it regardless of probe direction.
    let p = params();
    let left = rotary_safe_probe_lines(RotaryTouch { angle_deg: 0.0, axis: Axis::Y, dir: Dir::Pos }, p);
    let right = rotary_safe_probe_lines(RotaryTouch { angle_deg: 0.0, axis: Axis::Y, dir: Dir::Neg }, p);
    let descend = |ls: &[String]| ls.iter().rev().find(|l| l.starts_with("G53 G0 Z")).cloned().unwrap();
    assert_eq!(descend(&left), descend(&right), "both side touches must descend to the same height");
    assert_eq!(descend(&left), "G53 G0 Z-8.000");
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
