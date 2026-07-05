//! The [`Postprocessor`] trait — Eitri's port of FlatCAM's per-controller "preprocessor" modules — plus the hook
//! data types and a [`Registry`] of built-in dialects.
//!
//! FlatCAM's postprocessors are Python modules exposing hook methods (`start_code`, `spindle_code`, `linear_code`,
//! `end_code`, …) that each controller overrides. Eitri models this as one trait with a method per hook. The design
//! is split deliberately (see [`crate::emit`]): the **emitter** decides motions (rapid to start, plunge, cut, lift,
//! peck), and the **postprocessor** renders each motion as text in its dialect. So the load-bearing set of hooks is
//! the union of what a controller must emit: program frame, spindle, tool change, rapid / linear / arc moves, dwell,
//! and comments.
//!
//! Only the two *frame* hooks ([`Postprocessor::start_code`] / [`Postprocessor::end_code`]) are required — they carry
//! the dialect's preamble and postamble. Every motion/spindle/tool/dwell/comment hook has a default implementation
//! that renders standard RS-274 text through the program's [`OutputFormat`], because that layer is identical across
//! grbl and LinuxCNC-style controllers. A minimal second dialect therefore only overrides the frame hooks.

use std::collections::BTreeMap;

use crate::arc::{ArcDir, ArcOffset};
use crate::format::OutputFormat;
use crate::program::Program;

/// Context for the program frame hooks: an optional program name/title a dialect may print as a header comment.
#[derive(Debug, Clone, Default)]
pub struct JobContext {
  /// A human-readable job name for the header comment (e.g. `"isolation"`), or `None` for no title.
  pub name: Option<String>,
}

/// Spindle-on parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spindle {
  /// Commanded spindle speed (RPM).
  pub rpm: f64,
}

/// A tool change: the tool number and its diameter (millimetres), the latter only for the annotation comment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolChange {
  /// Tool number (the `T` word).
  pub number: u32,
  /// Tool diameter (millimetres), for the accompanying comment.
  pub diameter: f64,
}

/// A set of axis words for a move — any subset of X/Y/Z. Absent axes are omitted from the line so a Z-only plunge or
/// an XY-only reposition renders cleanly.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Axes {
  /// X target (millimetres), if this move sets X.
  pub x: Option<f64>,
  /// Y target (millimetres), if this move sets Y.
  pub y: Option<f64>,
  /// Z target (millimetres), if this move sets Z.
  pub z: Option<f64>,
}

impl Axes {
  /// An X/Y move (no Z change).
  pub fn xy(x: f64, y: f64) -> Axes {
    Axes { x: Some(x), y: Some(y), z: None }
  }

  /// A Z-only move (a plunge or lift).
  pub fn z(z: f64) -> Axes {
    Axes { x: None, y: None, z: Some(z) }
  }
}

/// An arc feed move in the XY plane (G17): the endpoint plus the centre offset and turning direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcMove {
  /// Endpoint X (millimetres).
  pub x: f64,
  /// Endpoint Y (millimetres).
  pub y: f64,
  /// Centre offset relative to the arc start.
  pub offset: ArcOffset,
  /// Turning direction, which selects `G2`/`G3`.
  pub dir: ArcDir,
}

/// A controller dialect: renders each motion the emitter produces into G-code text for a specific machine.
///
/// Implementors must provide [`Postprocessor::name`], [`Postprocessor::format`], and the two frame hooks. The
/// motion/spindle/tool/dwell/comment hooks default to standard RS-274 rendering through [`Postprocessor::format`].
pub trait Postprocessor {
  /// The dialect's registry name (e.g. `"grbl"`).
  fn name(&self) -> &str;

  /// The output formatting (coordinate/feed precision, units, comment style, line ending) for this dialect.
  fn format(&self) -> OutputFormat;

  /// Emit the program preamble (units, distance mode, plane, work offset, feed mode, and any header comment).
  fn start_code(&self, prog: &mut Program, job: &JobContext);

  /// Emit the program postamble (spindle off if needed, program end).
  fn end_code(&self, prog: &mut Program, job: &JobContext);

  /// A rapid positioning move (`G0`). No feed word — grbl uses the `$110/$111/$112` rapid rates.
  fn rapid(&self, prog: &mut Program, axes: Axes) {
    let mut line = String::from("G0");
    append_axes(&mut line, axes, *prog.format());
    prog.push(line);
  }

  /// A linear feed move (`G1`) at `feed` (units/min). F is emitted on every line so the output is unambiguous
  /// regardless of modal feed state — the contract requires F present on `G1`.
  fn linear(&self, prog: &mut Program, axes: Axes, feed: f64) {
    let fmt = *prog.format();
    let mut line = String::from("G1");
    append_axes(&mut line, axes, fmt);
    line.push_str(" F");
    line.push_str(&fmt.feed(feed));
    prog.push(line);
  }

  /// An arc feed move (`G2`/`G3`) with I/J centre offsets, at `feed`. F is always present, per the contract.
  fn arc(&self, prog: &mut Program, arc: ArcMove, feed: f64) {
    let fmt = prog.format();
    let line = format!(
      "{} X{} Y{} I{} J{} F{}",
      arc.dir.word(),
      fmt.coord(arc.x),
      fmt.coord(arc.y),
      fmt.coord(arc.offset.i),
      fmt.coord(arc.offset.j),
      fmt.feed(feed),
    );
    prog.push(line);
  }

  /// A dwell (`G4 P<seconds>`).
  fn dwell(&self, prog: &mut Program, seconds: f64) {
    prog.push(format!("G4 P{seconds:.3}"));
  }

  /// Spindle on, clockwise, at the given speed (`M3 S<rpm>`).
  fn spindle_on(&self, prog: &mut Program, spindle: Spindle) {
    prog.push(format!("M3 S{}", spindle.rpm.round() as i64));
  }

  /// Spindle off (`M5`).
  fn spindle_off(&self, prog: &mut Program) {
    prog.push("M5");
  }

  /// A tool change (`M6 T<n>`), preceded by an annotating comment.
  fn tool_change(&self, prog: &mut Program, tool: ToolChange) {
    prog.comment(&format!("tool {} — {:.3} mm", tool.number, tool.diameter));
    prog.push(format!("M6 T{}", tool.number));
  }

  /// A free-standing comment line, in the dialect's comment style.
  fn comment(&self, prog: &mut Program, text: &str) {
    prog.comment(text);
  }
}

/// Append the present axis words of `axes` (in X, Y, Z order) to `line`, formatted to `fmt`'s coordinate precision.
fn append_axes(line: &mut String, axes: Axes, fmt: OutputFormat) {
  if let Some(x) = axes.x {
    line.push_str(" X");
    line.push_str(&fmt.coord(x));
  }
  if let Some(y) = axes.y {
    line.push_str(" Y");
    line.push_str(&fmt.coord(y));
  }
  if let Some(z) = axes.z {
    line.push_str(" Z");
    line.push_str(&fmt.coord(z));
  }
}

/// A registry of named postprocessor dialects. Ships with the built-ins ([`crate::GrblHal`], [`crate::Generic`])
/// and accepts further registrations, mirroring FlatCAM's discoverable preprocessor set.
pub struct Registry {
  posts: BTreeMap<String, Box<dyn Postprocessor>>,
}

impl Registry {
  /// An empty registry.
  pub fn new() -> Registry {
    Registry { posts: BTreeMap::new() }
  }

  /// A registry pre-populated with the built-in dialects.
  pub fn with_builtins() -> Registry {
    let mut reg = Registry::new();
    reg.register(Box::new(crate::GrblHal::new()));
    reg.register(Box::new(crate::Generic::new()));
    reg
  }

  /// Register a dialect under its [`Postprocessor::name`], replacing any existing dialect with that name.
  pub fn register(&mut self, post: Box<dyn Postprocessor>) {
    self.posts.insert(post.name().to_string(), post);
  }

  /// Look up a dialect by name.
  pub fn get(&self, name: &str) -> Option<&dyn Postprocessor> {
    self.posts.get(name).map(|b| b.as_ref())
  }

  /// The registered dialect names, sorted.
  pub fn names(&self) -> Vec<&str> {
    self.posts.keys().map(|s| s.as_str()).collect()
  }
}

impl Default for Registry {
  fn default() -> Registry {
    Registry::with_builtins()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn default_rapid_has_no_feed_word() {
    let mut prog = Program::new(OutputFormat::default());
    let post = crate::GrblHal::new();
    post.rapid(&mut prog, Axes::xy(1.0, 2.0));
    assert_eq!(prog.lines(), &["G0 X1.0000 Y2.0000".to_string()]);
  }

  #[test]
  fn default_linear_carries_a_feed_word() {
    let mut prog = Program::new(OutputFormat::default());
    let post = crate::GrblHal::new();
    post.linear(&mut prog, Axes::xy(1.0, 2.0), 120.0);
    assert_eq!(prog.lines(), &["G1 X1.0000 Y2.0000 F120.0".to_string()]);
  }

  #[test]
  fn z_only_move_omits_x_and_y() {
    let mut prog = Program::new(OutputFormat::default());
    let post = crate::GrblHal::new();
    post.linear(&mut prog, Axes::z(-0.5), 60.0);
    assert_eq!(prog.lines(), &["G1 Z-0.5000 F60.0".to_string()]);
  }

  #[test]
  fn registry_holds_builtins_and_new_registrations() {
    let reg = Registry::with_builtins();
    assert!(reg.get("grbl").is_some());
    assert!(reg.get("generic").is_some());
    assert!(reg.get("nonesuch").is_none());
    assert_eq!(reg.names(), vec!["generic", "grbl"]);
  }
}
