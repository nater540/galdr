//! UI intents: the one-way vocabulary the views emit and the shell translates into engine [`Command`]s.
//!
//! Keeping intents separate from [`crate::engine::Command`] lets the views speak in UI terms (connect to a
//! named port, jog by a step, load a file) while the shell owns the policy of turning those into the engine's
//! lower-level commands and side effects (opening a transport, reading a file, echoing to the console). The
//! views never touch the engine handle directly, which keeps them pure renderers of [`super::ViewState`].

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

  /// Send a single manual G-code/`$` line entered in the console.
  SendLine(String),
  /// Inject a real-time single-byte command out-of-band.
  Realtime(RealtimeCommand),

  /// Jog `axis` in `dir` by `distance` (mm) at `feed` (mm/min). The shell forms the `$J=` line.
  Jog { axis: Axis, dir: Dir, distance: f64, feed: f64 },

  /// Dismiss the latched alarm/error banner.
  DismissBanner,
  /// Probe Z with `G38.2` toward `depth` (mm, negative) at `feed` (mm/min), then set work-Z to
  /// `plate_thickness`. The shell sequences the probe + zeroing lines.
  ProbeZ { depth: f64, feed: f64, plate_thickness: f64 },

  /// Run the homing cycle (`$H`).
  Home,
  /// Begin streaming if idle, or resume from a feed hold (`~`) — the toolbar Run/Resume segment. The shell
  /// chooses between starting a stream and a cycle-start based on the live state.
  RunOrResume,
  /// Set the work-coordinate zero on the given axes to the current position (`G10 L20 P0 …`). An empty set is
  /// treated as "all of X, Y, Z" (the "Zero XYZ" button).
  SetWorkZero { axes: Vec<Axis> },
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
  fn sink_records_and_drains_in_order() {
    let mut sink = IntentSink::new();
    sink.push(Intent::RefreshPorts);
    sink.push(Intent::Disconnect);
    let drained = sink.drain();
    assert_eq!(drained, vec![Intent::RefreshPorts, Intent::Disconnect]);
    assert!(sink.drain().is_empty(), "drain leaves the sink empty");
  }
}
