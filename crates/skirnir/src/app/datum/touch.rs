//! The two-stage `G38.3` latch touch: a fast search, a retract, then a slow re-probe — as a pure line builder.
//!
//! This is the non-rotary sibling of [`crate::app::rotary_probe::rotary_safe_probe_lines`], and it mirrors
//! ioSender / OpenCNCPilot's probing action (MIT): a probe is done in TWO passes so the KEPT reading is taken
//! slowly for repeatability without spending the whole cycle crawling. First a fast `G38.3` searches for the
//! surface; on contact the tool retracts by `latch_distance`; then a slow `G38.3` re-approaches and THAT second
//! (slow) contact is the trustworthy reading. The operator has already jogged to the approach standoff, so every
//! move here is RELATIVE (`G91`) from the current position — the caller does any absolute positioning.
//!
//! **Why `G38.3`, not `G38.2`.** `G38.3` is the *no-alarm* probe: a miss does not raise `ALARM:5` and lock the
//! g-code state, it just reports `[PRB:…:0]`. A datum wizard chains many touches, so an alarm between them would
//! wedge the whole flow; with `G38.3` a miss is detected in SOFTWARE (the `:0` flag) and the wizard aborts
//! cleanly. Note the KEPT reading is the SLOW probe — the shell arms its probe latch only before the slow pass,
//! so the fast pass's `ok` is consumed by ordinary flow control and only the slow `[PRB:]` resolves the latch.
//!
//! Feeds and distances are bench-tuned parameters ([`ProbeParams`], persisted with the profile); this module
//! invents no magic numbers beyond the conservative defaults. Pure and synchronous, so the emitted sequence is
//! unit-tested without a window or hardware.

use super::super::intent::{Axis, Dir};

/// Default lateral approach standoff (mm) — how far off a face the operator parks before a side touch. Inflated
/// by the tip radius at use so the fast search always starts clear of the surface. Bench-tuned, overridable.
pub const DEFAULT_XY_CLEARANCE_MM: f64 = 5.0;

/// Default Z plunge (mm, downward magnitude) for a SIDE (X/Y) touch: how far below the top surface the tip drops
/// so a lateral probe meets the flank rather than sweeping over the top. Setup-specific — the operator tunes it.
pub const DEFAULT_DEPTH_MM: f64 = 5.0;

/// Default maximum FAST-probe travel (mm): how far the first `G38.3` searches for contact before giving up. A
/// miss at this distance reports `[PRB:…:0]` (no alarm, thanks to `G38.3`). Bench-tuned, overridable.
pub const DEFAULT_PROBE_DISTANCE_MM: f64 = 25.0;

/// Default retract (mm) between the fast contact and the slow re-probe — the "latch". Also the floor the slow
/// re-probe travel is derived from (`max(latch·1.5, 2)`). Bench-tuned, overridable.
pub const DEFAULT_LATCH_DISTANCE_MM: f64 = 2.0;

/// Default FAST search feed (mm/min): quick, to find the surface without crawling. Bench-tuned, overridable.
pub const DEFAULT_PROBE_FEED: f64 = 200.0;

/// Default SLOW latch feed (mm/min): the precise, repeatable second approach whose contact is KEPT. Overridable.
pub const DEFAULT_LATCH_FEED: f64 = 50.0;

/// Default rapids feed (mm/min) for the wizard's absolute positioning moves (a `G1`-paced "rapid" when the
/// machine has no true `G0`, or simply the speed the shell uses to reposition between touches). Overridable.
pub const DEFAULT_RAPIDS_FEED: f64 = 1000.0;

/// Default probe tip/ball diameter (mm) for lateral tip-radius compensation ([`super::comp::edge_coord`]).
/// Setup-specific (it is the actual tool/ball the operator mounted), so this is only a placeholder to tune.
pub const DEFAULT_PROBE_DIAMETER_MM: f64 = 2.0;

/// Default corner slide (mm): how far each corner touch is offset ALONG the other face, so it contacts a clean
/// flank clear of the corner's rounding/fillet. Reused as the inside-corner clamp on `xy_clearance`. Overridable.
pub const DEFAULT_OFFSET_MM: f64 = 5.0;

/// The bench-tuned parameters shared across every datum touch in a wizard run — the non-rotary analogue of
/// [`crate::app::rotary_probe::RotaryProbeParams`]. Held once (in the UI / persisted profile) and passed to
/// [`touch_lines`] per touch, so one conservative set applies to the whole run. `serde` so it survives a session
/// in the profile store; `Copy` because it is a handful of scalars threaded through the shell.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProbeParams {
  /// Lateral approach standoff (mm), inflated by the tip radius at use. See [`DEFAULT_XY_CLEARANCE_MM`].
  /// RESERVED: only consumed by the corner-slide AUTO-positioning, which is deferred (the corner approach is
  /// operator-jogged for now, like the rotary wizard), so this is persisted but not yet applied to emitted g-code.
  pub xy_clearance: f64,
  /// Z plunge (mm, downward magnitude) for a SIDE touch. See [`DEFAULT_DEPTH_MM`].
  pub depth: f64,
  /// Maximum FAST-probe travel (mm). See [`DEFAULT_PROBE_DISTANCE_MM`].
  pub probe_distance: f64,
  /// Retract (mm) between the fast contact and the slow re-probe. See [`DEFAULT_LATCH_DISTANCE_MM`].
  pub latch_distance: f64,
  /// FAST search feed (mm/min). See [`DEFAULT_PROBE_FEED`].
  pub probe_feed: f64,
  /// SLOW latch feed (mm/min) — the KEPT reading's feed. See [`DEFAULT_LATCH_FEED`].
  pub latch_feed: f64,
  /// Rapids feed (mm/min) for repositioning moves. See [`DEFAULT_RAPIDS_FEED`].
  pub rapids_feed: f64,
  /// Probe tip/ball diameter (mm) for lateral tip-radius compensation. See [`DEFAULT_PROBE_DIAMETER_MM`].
  pub probe_diameter: f64,
  /// Corner slide (mm) along the other face, and the inside-corner clamp on `xy_clearance`. See
  /// [`DEFAULT_OFFSET_MM`]. RESERVED for the deferred corner-slide auto-positioning (see [`xy_clearance`]);
  /// persisted so the value survives, but not yet applied — the inside-corner standoff clamp lives with that
  /// deferred auto-slide.
  pub offset: f64,
}

impl Default for ProbeParams {
  fn default() -> Self {
    ProbeParams {
      xy_clearance: DEFAULT_XY_CLEARANCE_MM,
      depth: DEFAULT_DEPTH_MM,
      probe_distance: DEFAULT_PROBE_DISTANCE_MM,
      latch_distance: DEFAULT_LATCH_DISTANCE_MM,
      probe_feed: DEFAULT_PROBE_FEED,
      latch_feed: DEFAULT_LATCH_FEED,
      rapids_feed: DEFAULT_RAPIDS_FEED,
      probe_diameter: DEFAULT_PROBE_DIAMETER_MM,
      offset: DEFAULT_OFFSET_MM,
    }
  }
}

/// One datum touch: probe along a linear axis in a direction. `dir` is the APPROACH direction — the sign the
/// probe travels — which is also the tip-comp `af` the caller feeds [`super::comp::edge_coord`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Touch {
  /// The linear axis (X/Y/Z) to probe along.
  pub axis: Axis,
  /// The direction the probe advances (and the tip-comp approach sign).
  pub dir: Dir,
}

/// The slow re-probe travels `max(latch_distance·1.5, MIN_SLOW_TRAVEL_MM)` — always enough to re-reach the
/// surface it just retracted from, with a floor so a tiny latch distance still gives the slow pass room to
/// contact. Mirrors ioSender's `max(LatchDistance·1.5, 2 mm)`.
const MIN_SLOW_TRAVEL_MM: f64 = 2.0;

/// Emit the two-stage latch touch for `t` using the shared bench `params`, in send order:
///
/// 0. `G21` — establish MILLIMETRES first. The probe distances (`probe_distance`, `latch_distance`) are bench
///    values in mm, but the `G91 G38.3 <dist>` words are interpreted in the machine's CURRENT units — so a
///    machine left in `G20` (inch) would scale the probe travel 25.4× (an over-travel crash). Asserting `G21`
///    up front makes the whole touch unit-safe regardless of the prior modal units. `G21` is idempotent.
/// 1. `G91 G38.3 <axis><±probe_distance> F<probe_feed>` — the fast search (relative). On contact it stops.
/// 2. `G91 G0 <axis><∓latch_distance>` — retract by the latch distance (opposite sign), clearing the surface.
/// 3. `G91 G38.3 <axis><±max(latch_distance·1.5, 2)> F<latch_feed>` — the slow re-probe. THIS contact is KEPT
///    (the shell arms its latch only before this line), so the reading is the slow, repeatable one.
/// 4. `G90` — restore absolute mode so later positioning is not silently incremental.
///
/// All strokes are RELATIVE (`G91`) — the operator has already jogged to the approach standoff, so each probe
/// advances FROM the current position, never to an absolute coordinate (which under `G90` would send the tool to
/// the wrong place). Positions render at 3 decimals, feeds as integers, matching the rest of the app.
pub fn touch_lines(t: Touch, params: &ProbeParams) -> Vec<String> {
  let sign = t.dir.sign();
  let axis = t.axis.letter();
  let fast = sign * params.probe_distance;
  let retract = -sign * params.latch_distance;
  let slow = sign * (params.latch_distance * 1.5).max(MIN_SLOW_TRAVEL_MM);
  vec![
    // Establish mm before any unit-sensitive probe distance (safe under a G20 machine).
    "G21".to_string(),
    format!("G91 G38.3 {axis}{fast:.3} F{:.0}", params.probe_feed),
    format!("G91 G0 {axis}{retract:.3}"),
    format!("G91 G38.3 {axis}{slow:.3} F{:.0}", params.latch_feed),
    "G90".to_string(),
  ]
}

#[cfg(test)]
mod tests {
  use super::*;

  fn params() -> ProbeParams {
    ProbeParams {
      xy_clearance: 5.0,
      depth: 5.0,
      probe_distance: 10.0,
      latch_distance: 1.0,
      probe_feed: 200.0,
      latch_feed: 50.0,
      rapids_feed: 1000.0,
      probe_diameter: 2.0,
      offset: 5.0,
    }
  }

  #[test]
  fn emits_the_fast_retract_slow_restore_sequence_toward_negative() {
    // A −Y touch: fast search toward −Y, retract +Y by the latch distance, slow re-probe toward −Y, restore G90.
    let lines = touch_lines(Touch { axis: Axis::Y, dir: Dir::Neg }, &params());
    assert_eq!(
      lines,
      vec![
        "G21".to_string(),
        "G91 G38.3 Y-10.000 F200".to_string(),
        "G91 G0 Y1.000".to_string(),
        "G91 G38.3 Y-2.000 F50".to_string(),
        "G90".to_string(),
      ]
    );
  }

  #[test]
  fn the_touch_establishes_millimetres_up_front_so_a_g20_machine_cannot_over_travel() {
    // The bench probe distances are mm; the leading `G21` makes the whole touch unit-safe regardless of the
    // machine's prior units (a `G20` machine would otherwise scale the `G91 G38.3` probe travel 25.4×).
    let lines = touch_lines(Touch { axis: Axis::Z, dir: Dir::Neg }, &params());
    assert_eq!(lines.first().map(String::as_str), Some("G21"), "the touch must establish mm first; got {lines:?}");
  }

  #[test]
  fn emits_the_sequence_toward_positive() {
    // A +X touch: fast +X, retract −X, slow +X.
    let lines = touch_lines(Touch { axis: Axis::X, dir: Dir::Pos }, &params());
    assert_eq!(
      lines,
      vec![
        "G21".to_string(),
        "G91 G38.3 X10.000 F200".to_string(),
        "G91 G0 X-1.000".to_string(),
        "G91 G38.3 X2.000 F50".to_string(),
        "G90".to_string(),
      ]
    );
  }

  #[test]
  fn both_probes_are_the_no_alarm_g38_3_not_g38_2() {
    // The defining safety property for chaining touches: both stages use G38.3 (no alarm on a miss), never G38.2.
    let lines = touch_lines(Touch { axis: Axis::Z, dir: Dir::Neg }, &params());
    let probes: Vec<&String> = lines.iter().filter(|l| l.contains("G38")).collect();
    assert_eq!(probes.len(), 2, "there must be exactly two probe passes; got {lines:?}");
    for probe in probes {
      assert!(probe.contains("G38.3"), "a datum probe must be the no-alarm G38.3; got {probe:?}");
      assert!(!probe.contains("G38.2"), "a datum probe must not be the alarming G38.2; got {probe:?}");
    }
  }

  #[test]
  fn the_slow_travel_floors_at_two_millimetres() {
    // With a tiny latch distance, latch·1.5 is below the 2 mm floor, so the slow re-probe travels the 2 mm floor
    // to be sure it re-reaches the surface it retracted from.
    let mut p = params();
    p.latch_distance = 0.5; // 0.5·1.5 = 0.75 < 2.
    let lines = touch_lines(Touch { axis: Axis::X, dir: Dir::Pos }, &p);
    assert_eq!(lines[3], "G91 G38.3 X2.000 F50", "the slow travel must floor at 2 mm; got {lines:?}");
  }

  #[test]
  fn the_slow_travel_scales_with_a_larger_latch() {
    // With a larger latch distance, latch·1.5 exceeds the floor and is used (so the slow pass always re-reaches).
    let mut p = params();
    p.latch_distance = 4.0; // 4·1.5 = 6 > 2.
    let lines = touch_lines(Touch { axis: Axis::X, dir: Dir::Pos }, &p);
    assert_eq!(lines[1], "G91 G38.3 X10.000 F200");
    assert_eq!(lines[2], "G91 G0 X-4.000", "the retract is the full latch distance");
    assert_eq!(lines[3], "G91 G38.3 X6.000 F50", "the slow travel is latch·1.5 when that exceeds the floor");
  }

  #[test]
  fn every_stroke_is_relative_and_absolute_mode_is_restored() {
    // Under G90 a `G38.3 Z-d` would resolve as an absolute target (wrong place). Every stroke must be G91, and
    // the trailing G90 restores absolute mode so later positioning is not silently incremental.
    let lines = touch_lines(Touch { axis: Axis::Z, dir: Dir::Neg }, &params());
    // The three motion strokes (after the leading `G21`) must all be relative; the trailing `G90` restores absolute.
    for motion in &lines[1..4] {
      assert!(motion.starts_with("G91 "), "every stroke must be relative (G91); got {motion:?}");
    }
    assert_eq!(lines.last().unwrap(), "G90", "absolute mode must be restored last; got {lines:?}");
  }
}
