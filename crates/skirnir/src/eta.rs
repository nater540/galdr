//! Host-side job-time estimation (ETA) — the thin glue over [`cnc_kinematics::sim`].
//!
//! Parses a loaded program with the SAME [`cnc_kinematics::gcode`] parser the firmware uses, drives the
//! shared [`simulate`](cnc_kinematics::sim::simulate) over it (the real planner + 32-block look-ahead +
//! junction-deviation cornering), and folds the resulting per-command timeline into a PER-LINE timeline:
//! cumulative seconds, with feed-move and rapid-move time tracked separately so a live feed/rapid override
//! can rescale the remaining estimate. This module owns no motion math — that all lives in `cnc-kinematics`
//! next to the planner — so the ETA cannot drift from the motion the board runs.
//!
//! Scope of this first cut: kinematic move time + G4 dwell, mapped to lines, with override-aware remaining.
//! The documented follow-ups (spindle `$392`/`$393` dwell, and the dense-small-segment throughput floor with
//! its bench-measured params) plug in at the marked seams — see `docs/skirnir-eta-design.md` §4–§5.

use cnc_kinematics::gcode::Parser;
use cnc_kinematics::motion::MotionConfig;
use cnc_kinematics::planner::PlannerConfig;
use cnc_kinematics::sim::{simulate, SimStep};

/// The estimated time attributed to one program line, split so a live override can rescale the remaining
/// feed and rapid portions independently.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LineEta {
  /// Feed-move (G1/G2/G3) seconds on this line, at 100 % feed override.
  pub feed_seconds: f64,
  /// Rapid-move (G0) seconds on this line, at 100 % rapid override.
  pub rapid_seconds: f64,
  /// Fixed seconds no override rescales: G4 dwell (and, later, spindle spin-up/reverse dwell).
  pub fixed_seconds: f64,
  /// Cumulative seconds to the END of this line at 100 % overrides (running total of all line times).
  pub cumulative_seconds: f64,
}

impl LineEta {
  /// Total seconds on this line at 100 % overrides.
  pub fn seconds(&self) -> f64 {
    self.feed_seconds + self.rapid_seconds + self.fixed_seconds
  }
}

/// The simulated timeline for a whole program: one [`LineEta`] per source line, the program total, and the
/// operator-pause lines.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EtaTimeline {
  /// One entry per source line (blank / comment / modal-only lines carry zero time), so the live projection
  /// can index by the firmware's reported line (`Ln:`) or the host's acked-line count.
  pub lines: Vec<LineEta>,
  /// Total modeled motion + dwell time at 100 % overrides, in seconds.
  pub total_seconds: f64,
  /// Source-line indices that pause for the operator (M0 / M1 / M6) — surfaced, not timed, since the wait is
  /// unbounded. The displayed total is therefore a MINIMAL cut time plus these annotated pauses.
  pub pauses: Vec<usize>,
}

impl EtaTimeline {
  /// Build the timeline for `program` (one string per source line) using the firmware motion settings.
  pub fn build(program: &[String], planner_config: &PlannerConfig, motion_config: &MotionConfig) -> Self {
    // Parse each line, remembering which source line produced each planner command. A line may produce none
    // (a comment, blank, `$`-setting, or pure modal change like G90), so the command→line map is sparse.
    let mut parser = Parser::new();
    let mut commands = Vec::new();
    let mut command_line = Vec::new();
    for (idx, line) in program.iter().enumerate() {
      if let Ok(Some(cmd)) = parser.parse_line(line.as_bytes()) {
        commands.push(cmd);
        command_line.push(idx);
      }
    }

    let steps = simulate(&commands, planner_config, motion_config);

    let mut lines = vec![LineEta::default(); program.len()];
    let mut pauses = Vec::new();
    for (cmd_idx, step) in steps.iter().enumerate() {
      let line = command_line[cmd_idx];
      match *step {
        SimStep::Move { seconds, rapid } => {
          if rapid {
            lines[line].rapid_seconds += seconds;
          } else {
            lines[line].feed_seconds += seconds;
          }
        }
        SimStep::Dwell { seconds } => lines[line].fixed_seconds += seconds,
        SimStep::Pause { .. } => pauses.push(line),
        // Spindle/ProgramEnd/Untimed carry no modeled time yet — the spindle `$392`/`$393` dwell is a
        // documented follow-up that needs the spindle settings + the M3↔M4 reversal state.
        SimStep::Spindle(_) | SimStep::ProgramEnd | SimStep::Untimed => {}
      }
    }

    let mut running = 0.0f64;
    for line in &mut lines {
      running += line.seconds();
      line.cumulative_seconds = running;
    }

    EtaTimeline { lines, total_seconds: running, pauses }
  }

  /// The estimated seconds REMAINING after `completed_lines` source lines have finished, with the live feed
  /// and rapid override fractions applied (`1.0` = 100 %). Feed time scales by `1/feed_override` and rapid
  /// time by `1/rapid_override` (a 50 % override doubles that portion's time); fixed dwell time is unscaled.
  /// `completed_lines` past the program end returns 0.
  pub fn remaining_seconds(&self, completed_lines: usize, feed_override: f64, rapid_override: f64) -> f64 {
    let feed_scale = if feed_override > 0.0 { 1.0 / feed_override } else { 1.0 };
    let rapid_scale = if rapid_override > 0.0 { 1.0 / rapid_override } else { 1.0 };
    let from = completed_lines.min(self.lines.len());
    self.lines[from..]
      .iter()
      .map(|l| l.feed_seconds * feed_scale + l.rapid_seconds * rapid_scale + l.fixed_seconds)
      .sum()
  }
}

/// Build the [`PlannerConfig`] and [`MotionConfig`] from the firmware's `$$` settings, via a getter the caller
/// wires to its settings store (e.g. `|n| settings.value_of(n).and_then(|s| s.parse().ok())`). Every field
/// falls back to its firmware default when the setting is absent or unparseable, so a partial (or empty)
/// settings snapshot still yields a usable estimate — the caller should flag "using default machine settings"
/// when settings have not been fetched.
pub fn configs_from_settings(get: impl Fn(u32) -> Option<f64>) -> (PlannerConfig, MotionConfig) {
  let mut planner = PlannerConfig::default();
  // $100-103 steps/mm, $110-113 max rate mm/min, $120-123 accel mm/s² — X/Y/Z/A at base + 0..3.
  for axis in 0..4usize {
    if let Some(v) = get(100 + axis as u32) {
      planner.steps_per_mm[axis] = v as f32;
    }
    if let Some(v) = get(110 + axis as u32) {
      planner.max_rate_mm_min[axis] = v as f32;
    }
    if let Some(v) = get(120 + axis as u32) {
      planner.accel_mm_s2[axis] = v as f32;
    }
  }
  if let Some(v) = get(11) {
    planner.junction_deviation_mm = v as f32;
  }
  if let Some(v) = get(12) {
    planner.arc_tolerance_mm = v as f32;
  }
  if let Some(v) = get(376) {
    planner.rotary_mask = v as u8;
  }

  let mut motion = MotionConfig::default();
  // $0 step-pulse µs → ticks (tick_hz is 1 MHz, so 1 µs == 1 tick). tick_hz / min_low_ticks are firmware
  // constants, not `$`-settings, so they keep their defaults.
  if let Some(v) = get(0) {
    motion.step_pulse_ticks = v.max(0.0) as u32;
  }
  (planner, motion)
}

#[cfg(test)]
mod tests {
  use super::*;

  fn prog(text: &str) -> Vec<String> {
    text.lines().map(|l| l.to_string()).collect()
  }

  fn build(text: &str) -> EtaTimeline {
    EtaTimeline::build(&prog(text), &PlannerConfig::default(), &MotionConfig::default())
  }

  #[test]
  fn one_line_eta_per_source_line() {
    let program = prog("G0 X10\n(a comment)\nG1 X20 F500\nG90\n");
    let eta = EtaTimeline::build(&program, &PlannerConfig::default(), &MotionConfig::default());
    assert_eq!(eta.lines.len(), program.len(), "one LineEta per source line, comments/modal included");
  }

  #[test]
  fn feed_and_rapid_time_land_on_the_right_lines_and_split() {
    let eta = build("G0 X50\nG1 X100 F500\n");
    assert!(eta.lines[0].rapid_seconds > 0.0 && eta.lines[0].feed_seconds == 0.0, "line 0 is a rapid");
    assert!(eta.lines[1].feed_seconds > 0.0 && eta.lines[1].rapid_seconds == 0.0, "line 1 is a feed");
    assert!(eta.total_seconds > 0.0);
    // Cumulative is monotone and ends at the total.
    assert!(eta.lines[0].cumulative_seconds <= eta.lines[1].cumulative_seconds);
    assert!((eta.lines.last().unwrap().cumulative_seconds - eta.total_seconds).abs() < 1e-9);
  }

  #[test]
  fn dwell_is_fixed_time_and_pauses_are_surfaced() {
    let eta = build("G1 X10 F500\nG4 P2.5\nM0\n");
    assert_eq!(eta.lines[1].fixed_seconds, 2.5, "G4 P2.5 is 2.5 fixed seconds");
    assert_eq!(eta.pauses, vec![2], "M0 on line 2 is an operator pause");
  }

  #[test]
  fn remaining_drains_to_zero_and_matches_total_at_the_start() {
    let eta = build("G1 X20 F500\nG1 X40 F500\nG1 X60 F500\n");
    assert!((eta.remaining_seconds(0, 1.0, 1.0) - eta.total_seconds).abs() < 1e-9, "all remaining at start");
    assert_eq!(eta.remaining_seconds(eta.lines.len(), 1.0, 1.0), 0.0, "nothing remaining at the end");
    // After one line, remaining is less than the whole.
    assert!(eta.remaining_seconds(1, 1.0, 1.0) < eta.total_seconds);
  }

  #[test]
  fn a_feed_override_below_100_percent_lengthens_the_remaining_estimate() {
    let eta = build("G1 X100 F500\n");
    let nominal = eta.remaining_seconds(0, 1.0, 1.0);
    let slow = eta.remaining_seconds(0, 0.5, 1.0);
    assert!((slow - nominal * 2.0).abs() < 1e-9, "50 % feed doubles feed time: {slow} vs {nominal}");
  }

  #[test]
  fn configs_from_settings_overrides_present_and_falls_back_for_absent() {
    // Only $120 (X accel) and $0 (step pulse) supplied; everything else must keep its default.
    let (planner, motion) = configs_from_settings(|n| match n {
      120 => Some(25.0),
      0 => Some(4.0),
      _ => None,
    });
    let default_pc = PlannerConfig::default();
    assert_eq!(planner.accel_mm_s2[0], 25.0, "$120 overrides X accel");
    assert_eq!(planner.accel_mm_s2[1], default_pc.accel_mm_s2[1], "Y accel falls back to default");
    assert_eq!(planner.steps_per_mm, default_pc.steps_per_mm, "absent steps/mm keep defaults");
    assert_eq!(motion.step_pulse_ticks, 4, "$0 overrides the step-pulse ticks");
    assert_eq!(motion.tick_hz, MotionConfig::default().tick_hz, "tick_hz is a firmware constant");
  }

  #[test]
  fn empty_settings_yield_a_usable_default_estimate() {
    let (planner, motion) = configs_from_settings(|_| None);
    assert_eq!(planner, PlannerConfig::default());
    assert_eq!(motion, MotionConfig::default());
    // And a program still estimates against those defaults.
    let eta = EtaTimeline::build(&prog("G1 X100 F500\n"), &planner, &motion);
    assert!(eta.total_seconds > 0.0);
  }
}
