//! Offline job-time simulation: drive the real planner over a whole program and integrate motion time.
//!
//! This is the host-side ETA engine. It feeds a [`PlannerCommand`](crate::gcode::PlannerCommand) stream
//! through the SAME [`Planner`](crate::planner::Planner) the firmware runs — including the bounded
//! 32-block look-ahead and junction-deviation cornering — popping blocks to slide the window exactly as the
//! motion executor does, then sums each block's
//! [`estimate_block_time`](crate::motion::estimate_block_time). Because it reuses the firmware's own planner
//! and timing, the resulting ETA cannot drift from the motion the board produces.
//!
//! Requires the `sim` feature (it allocates a per-command timeline, so it is gated off the no-alloc firmware
//! build). The output is one [`SimStep`] per input command, in order, so a caller can map each entry back to
//! the program line that produced it and layer on the dwell / spindle / throughput factors.

use alloc::vec::Vec;

use crate::gcode::{PlannerCommand, SpindleState};
use crate::motion::{estimate_block_time, MotionConfig};
use crate::planner::{Block, Planner, PlannerConfig, PlannerError, PlannerOutcome};

/// One entry in the simulated timeline, 1:1 with an input [`PlannerCommand`](crate::gcode::PlannerCommand)
/// (so the caller can map it back to the program line that produced it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimStep {
  /// A motion command (move or arc) and its total execution time in seconds, summed over its (possibly
  /// subdivided) blocks. `rapid` is true for a G0 traverse and false for a G1/G2/G3 feed move, so the caller
  /// can split feed-time from rapid-time (e.g. to rescale remaining time by live feed/rapid overrides). A
  /// zero-travel line reports `seconds = 0.0`.
  Move {
    /// Total execution time of this command's motion, in seconds.
    seconds: f64,
    /// True for a G0 rapid traverse; false for a G1/G2/G3 feed move.
    rapid: bool,
  },
  /// A G4 dwell of `seconds`.
  Dwell {
    /// The dwell duration in seconds (the G4 `P` word).
    seconds: f64,
  },
  /// An M3/M4/M5 spindle state change. Carries no time itself; the caller adds the `$392`/`$393` spin-up /
  /// reversal dwell from settings (the planner is deliberately settings-free).
  Spindle(SpindleState),
  /// A program pause (M0 / M1 / M6) — an unbounded operator wait, so it carries no modeled time. The caller
  /// surfaces it (e.g. "pauses at line N") rather than counting it.
  Pause {
    /// True for M1 optional stop.
    optional: bool,
    /// True for M6 manual tool change.
    tool_change: bool,
  },
  /// M30 program end.
  ProgramEnd,
  /// A command that moves nothing and is not time-modeled: a coordinate-system op (G10/G92/G54-59/G43.1/
  /// G49), a coolant change (M7/M8/M9), a G28/G30 predefined recall, or a G38 probe (its stop point is
  /// unknown ahead of the run). Kept in the timeline for 1:1 line mapping, with zero modeled time.
  Untimed,
}

/// A timeline slot before block times are resolved. Motion commands record the global block range they
/// occupy (resolved to a duration in the second pass, once every block's settled exit speed is known); every
/// other command resolves to its final [`SimStep`] immediately.
enum Slot {
  Move { start: usize, end: usize },
  Done(SimStep),
}

/// Simulate a whole program: drive `commands` through a fresh planner configured by `planner_config`,
/// integrate each motion block with `motion_config`, and return one [`SimStep`] per input command, in order
/// (`output.len() == commands.len()`).
///
/// The planner queue holds at most `BLOCK_QUEUE_LEN` blocks, so blocks are popped to make room as the stream
/// is fed — sliding the look-ahead window exactly like the motion executor — and accumulated with their
/// settled entry speeds. Each block's exit speed is the next block's entry speed (or 0 at a stop); a dwell /
/// pause / probe flushes look-ahead, which the planner models by resetting the junction state so the next
/// move starts from rest — so the "exit = next entry" rule yields the correct full stop at those boundaries
/// for free.
pub fn simulate(
  commands: &[PlannerCommand],
  planner_config: &PlannerConfig,
  motion_config: &MotionConfig,
) -> Vec<SimStep> {
  let mut planner = Planner::new(*planner_config);
  // Every block ever enqueued, in execution (FIFO) order, with its settled entry speed.
  let mut blocks: Vec<Block> = Vec::new();
  let mut slots: Vec<Slot> = Vec::with_capacity(commands.len());
  // The running count of blocks enqueued across all commands so far; the global index of each block in
  // `blocks` once everything is drained. Independent of when a block is popped.
  let mut enqueued_total = 0usize;

  for command in commands {
    // Resolve the command to an outcome, popping queued blocks to free space if a normal move does not fit.
    // The move path is all-or-nothing (a QueueFull never advanced position), so retrying the same command
    // after a pop is exact. Arcs never report QueueFull — they return `ArcPending` and are drained below.
    let outcome = loop {
      match planner.plan_command(command) {
        Err(PlannerError::QueueFull) => match planner.pop_block() {
          Some(block) => blocks.push(block),
          // No block to pop yet a full queue is contradictory; bail rather than spin.
          None => break Err(PlannerError::QueueFull),
        },
        other => break other,
      }
    };

    let slot = match outcome {
      Ok(PlannerOutcome::Queued { blocks: n }) => {
        let start = enqueued_total;
        enqueued_total += n;
        Slot::Move { start, end: enqueued_total }
      }
      Ok(PlannerOutcome::ArcPending { enqueued }) => {
        // The arc did not fully fit: drain a block to free space, resume, repeat until the arc completes.
        let start = enqueued_total;
        enqueued_total += enqueued;
        while planner.arc_pending() {
          if let Some(block) = planner.pop_block() {
            blocks.push(block);
          }
          match planner.resume_arc() {
            Ok(PlannerOutcome::Queued { blocks: m }) => {
              enqueued_total += m;
              break;
            }
            Ok(PlannerOutcome::ArcPending { enqueued: m }) => enqueued_total += m,
            // A resume should only ever report a (possibly zero-block) arc outcome; anything else ends it.
            _ => break,
          }
        }
        Slot::Move { start, end: enqueued_total }
      }
      Ok(PlannerOutcome::Dwell { seconds }) => Slot::Done(SimStep::Dwell { seconds: seconds as f64 }),
      Ok(PlannerOutcome::Spindle(state, _speed)) => Slot::Done(SimStep::Spindle(state)),
      Ok(PlannerOutcome::ProgramPause { optional, tool_change }) => {
        Slot::Done(SimStep::Pause { optional, tool_change })
      }
      Ok(PlannerOutcome::ProgramEnd) => Slot::Done(SimStep::ProgramEnd),
      // Predefined recalls, probes, coordinate ops, and coolant changes are not time-modeled (the probe
      // stop point is unknown ahead of the run; the others move nothing). Keep the 1:1 mapping as untimed.
      Ok(PlannerOutcome::GoToPredefined { .. })
      | Ok(PlannerOutcome::Probe { .. })
      | Ok(PlannerOutcome::Coordinate(_))
      | Ok(PlannerOutcome::Coolant(_)) => Slot::Done(SimStep::Untimed),
      // A rejected command (e.g. an invalid arc) produces no motion; keep the slot as untimed.
      Err(_) => Slot::Done(SimStep::Untimed),
    };
    slots.push(slot);
  }

  // Drain the remaining queue. The final recalculate has the tail decelerate to a stop, so these carry their
  // settled entry speeds.
  while let Some(block) = planner.pop_block() {
    blocks.push(block);
  }

  resolve(slots, &blocks, motion_config)
}

/// Second pass: each block's exit speed is the NEXT block's settled entry speed (or 0 for the final block),
/// so resolve every motion slot's summed time and flatten to one [`SimStep`] per command.
fn resolve(slots: Vec<Slot>, blocks: &[Block], motion_config: &MotionConfig) -> Vec<SimStep> {
  slots
    .into_iter()
    .map(|slot| match slot {
      Slot::Done(step) => step,
      Slot::Move { start, end } => {
        let mut seconds = 0.0f64;
        for i in start..end {
          // Exit speed = next block's entry; 0 at the very end or across a flushed boundary (the next move
          // there starts from rest, so its entry is already 0).
          let exit_sq = blocks.get(i + 1).map_or(0.0, |next| next.entry_speed_sq);
          seconds += estimate_block_time(&blocks[i], exit_sq, motion_config);
        }
        let rapid = if start < end { blocks[start].rapid } else { false };
        SimStep::Move { seconds, rapid }
      }
    })
    .collect()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::gcode::Parser;

  /// Parse a multi-line GCode program into the planner-command stream, exactly as `skirnir` will.
  fn commands(program: &str) -> Vec<PlannerCommand> {
    let mut parser = Parser::new();
    let mut out = Vec::new();
    for line in program.lines() {
      if let Ok(Some(cmd)) = parser.parse_line(line.as_bytes()) {
        out.push(cmd);
      }
    }
    out
  }

  /// Total modeled MOTION time of a program (sum of `Move` and `Dwell` seconds).
  fn total_seconds(steps: &[SimStep]) -> f64 {
    steps
      .iter()
      .map(|s| match s {
        SimStep::Move { seconds, .. } | SimStep::Dwell { seconds } => *seconds,
        _ => 0.0,
      })
      .sum()
  }

  fn default_sim(program: &str) -> Vec<SimStep> {
    simulate(&commands(program), &PlannerConfig::default(), &MotionConfig::default())
  }

  #[test]
  fn output_is_one_step_per_command() {
    let cmds = commands("G0 X10\nG1 X20 F500\nG4 P1.5\nM30\n");
    let steps = simulate(&cmds, &PlannerConfig::default(), &MotionConfig::default());
    assert_eq!(steps.len(), cmds.len(), "exactly one SimStep per input command");
  }

  #[test]
  fn classifies_each_command_kind() {
    let steps = default_sim("G0 X10\nG1 X20 F500\nG4 P2.5\nM3 S1000\nM0\nM30\n");
    assert!(matches!(steps[0], SimStep::Move { rapid: true, .. }), "G0 is a rapid move");
    assert!(matches!(steps[1], SimStep::Move { rapid: false, .. }), "G1 is a feed move");
    assert_eq!(steps[2], SimStep::Dwell { seconds: 2.5 }, "G4 P2.5 passes through exactly");
    assert!(matches!(steps[3], SimStep::Spindle(_)), "M3 is a spindle change");
    assert!(matches!(steps[4], SimStep::Pause { .. }), "M0 is a program pause");
    assert_eq!(steps[5], SimStep::ProgramEnd, "M30 is program end");
  }

  #[test]
  fn a_feed_move_takes_a_plausible_time() {
    // G1 X100 at F500 (= 500 mm/min = 8.333 mm/s, the default max rate). With accel 10 mm/s² the ideal
    // cruise-only time is 100 / 8.333 ≈ 12.0 s; the accel + decel ramps add a little. Bound it loosely.
    let steps = default_sim("G1 X100 F500\n");
    let SimStep::Move { seconds, .. } = steps[0] else { panic!("expected a move") };
    let cruise_only = 100.0 / (500.0 / 60.0);
    assert!(seconds > cruise_only, "ramps add time over the ideal cruise, got {seconds}");
    assert!(seconds < cruise_only * 1.3, "but a 100 mm move is cruise-dominated, got {seconds}");
  }

  #[test]
  fn collinear_junction_keeps_speed_but_a_corner_does_not() {
    // Two collinear +X moves never slow at the shared point (the planner allows full junction speed), so their
    // total time ≈ a single straight move. An L-shaped path forces a near-stop at the 90° corner, so the same
    // travel takes materially longer — proving the look-ahead/junction cornering is live through the sim.
    let straight = total_seconds(&default_sim("G1 X50 F500\nG1 X100 F500\n"));
    let single = total_seconds(&default_sim("G1 X100 F500\n"));
    let corner = total_seconds(&default_sim("G1 X50 F500\nG1 X50 Y50 F500\n"));
    assert!((straight - single).abs() < single * 0.02, "collinear split ≈ single move ({straight} vs {single})");
    assert!(corner > straight * 1.05, "the 90° corner costs real time ({corner} vs {straight})");
  }

  #[test]
  fn a_dwell_between_moves_forces_a_full_stop() {
    // The same two collinear moves run faster end-to-end than when a G4 dwell splits them: the dwell flushes
    // look-ahead, so the first move must decelerate to a stop and the second re-accelerate from rest. Compare
    // only the MOVE time (exclude the zero-length dwell itself).
    let blended = total_seconds(&default_sim("G1 X50 F500\nG1 X100 F500\n"));
    let split = total_seconds(&default_sim("G1 X50 F500\nG4 P0\nG1 X100 F500\n"));
    assert!(split > blended, "a dwell-forced stop costs accel/decel time ({split} vs {blended})");
  }

  #[test]
  fn an_arc_is_summed_into_one_move_step() {
    // A G2/G3 arc subdivides into many chord blocks inside the planner, but the timeline reports ONE Move step
    // for the single arc command, with the chord times summed and a non-zero duration.
    let steps = default_sim("G17\nG1 X10 F500\nG2 X20 Y0 I5 J0 F500\n");
    let arc = steps.last().expect("a final step");
    assert!(matches!(arc, SimStep::Move { rapid: false, seconds } if *seconds > 0.0), "arc → one timed move");
  }
}
