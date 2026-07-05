//! G-code import: read existing NC back into a motion/geometry preview.
//!
//! FlatCAM opens G-code to *visualize* it (and to re-post it); this reproduces that visualize path. The low-level
//! tokenizing is the shared [`eitri_gcode::lex`] lexer (plan §8) — this module only walks modal state over the
//! lexed words: distance mode (`G90`/`G91`), units (`G20`/`G21`), and the motion group (`G0`/`G1`/`G2`/`G3`),
//! turning each commanded move into a [`PreviewMove`]. Arcs (`G2`/`G3`) are flattened via
//! [`eitri_geo::flatten_arc`] at the shared chord tolerance, so imported arc geometry matches native precision.
//!
//! This is a preview, not a re-derivation of cut *intent*: it recovers where the tool went (rapids vs cuts), which
//! is exactly what a canvas needs and what an emit→import round-trip can be checked against.

use std::f64::consts::TAU;

use geo_types::{Coord, LineString};

use eitri_core::{GEOM_EPSILON_MM, MM_PER_INCH, Result};
use eitri_geo::flatten_arc;

use crate::diagnostic::Skipped;

/// A point in the machine's 3D coordinate space (millimetres). Z is carried so the preview can tell plunges from
/// planar cuts, even though the recovered *geometry* is the XY projection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
  /// X (millimetres).
  pub x: f64,
  /// Y (millimetres).
  pub y: f64,
  /// Z (millimetres); negative is below the work surface.
  pub z: f64,
}

impl Point3 {
  /// The XY projection as a `geo_types` coordinate.
  pub fn xy(self) -> Coord<f64> {
    Coord { x: self.x, y: self.y }
  }
}

/// Whether a move is a non-cutting rapid (`G0`) or a cutting feed move (`G1`/`G2`/`G3`, the latter flattened).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionKind {
  /// A rapid positioning move (`G0`) — no material removed.
  Rapid,
  /// A cutting move at feed rate (`G1`, or a flattened `G2`/`G3` chord).
  Cut,
}

/// One straight motion segment in the recovered program. Arcs are pre-flattened into several `Cut` segments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreviewMove {
  /// Rapid or cut.
  pub kind: MotionKind,
  /// Segment start.
  pub from: Point3,
  /// Segment end.
  pub to: Point3,
}

/// The recovered motion preview: the ordered segments plus any diagnostics for constructs that were approximated
/// or skipped (out-of-plane arcs, arcs missing a centre).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GcodePreview {
  /// Every motion segment in program order.
  pub moves: Vec<PreviewMove>,
  /// Loud diagnostics for anything approximated or dropped.
  pub skipped: Vec<Skipped>,
}

impl GcodePreview {
  /// The cutting geometry as XY polylines: each maximal run of consecutive [`MotionKind::Cut`] segments, split at
  /// every rapid, projected to XY with coincident points collapsed. Pure plunge moves (Z-only) leave no XY trace,
  /// so a plunge-then-cut sequence recovers exactly the cut contour — which is what makes an emit→import
  /// round-trip assert cleanly against the original toolpath.
  pub fn cut_polylines(&self) -> Vec<LineString<f64>> {
    let mut out = Vec::new();
    let mut cur: Vec<Coord<f64>> = Vec::new();
    let push_run = |cur: &mut Vec<Coord<f64>>, out: &mut Vec<LineString<f64>>| {
      if cur.len() >= 2 {
        out.push(LineString(std::mem::take(cur)));
      } else {
        cur.clear();
      }
    };
    for mv in &self.moves {
      match mv.kind {
        MotionKind::Rapid => push_run(&mut cur, &mut out),
        MotionKind::Cut => {
          if cur.is_empty() {
            cur.push(mv.from.xy());
          }
          let to = mv.to.xy();
          if !coincident(*cur.last().expect("non-empty"), to) {
            cur.push(to);
          }
        }
      }
    }
    push_run(&mut cur, &mut out);
    out
  }
}

/// Whether two XY coordinates are the same point within [`GEOM_EPSILON_MM`].
fn coincident(a: Coord<f64>, b: Coord<f64>) -> bool {
  (a.x - b.x).abs() < GEOM_EPSILON_MM && (a.y - b.y).abs() < GEOM_EPSILON_MM
}

/// The active modal state a G-code walk carries between blocks.
struct Modal {
  pos: Point3,
  absolute: bool,
  units_scale: f64,
  motion: Option<u8>,
}

impl Default for Modal {
  fn default() -> Modal {
    // grbl powers up in absolute (G90), millimetres are the eitri canonical (units_scale converts G20 inches),
    // and no motion group is modal until a G0/1/2/3 is seen.
    Modal { pos: Point3 { x: 0.0, y: 0.0, z: 0.0 }, absolute: true, units_scale: 1.0, motion: None }
  }
}

/// Import a G-code program into a [`GcodePreview`]. Lexes with the shared lexer, then walks modal state; a lex
/// failure surfaces as [`eitri_core::Error::Parse`].
pub fn import_gcode(src: &str) -> Result<GcodePreview> {
  let lines = eitri_gcode::lex(src)?;
  let mut modal = Modal::default();
  let mut preview = GcodePreview::default();
  let mut out_of_plane = false;

  for line in &lines {
    // Apply modal G-words first: distance mode, units, plane, and the motion group. Multiple G-words can share a
    // block (a preamble like `G90 G21 G54 G17 G94`); each is applied, and the motion group takes the last seen.
    for w in line.words.iter().filter(|w| w.letter == 'G') {
      match w.value.round() as i64 {
        0 => modal.motion = Some(0),
        1 => modal.motion = Some(1),
        2 => modal.motion = Some(2),
        3 => modal.motion = Some(3),
        90 => modal.absolute = true,
        91 => modal.absolute = false,
        20 => modal.units_scale = MM_PER_INCH,
        21 => modal.units_scale = 1.0,
        18 | 19 => out_of_plane = true,
        _ => {}
      }
    }

    walk_motion(line, &mut modal, out_of_plane, &mut preview);
    out_of_plane = false;
  }
  Ok(preview)
}

/// Emit the move(s) commanded by one block given the current modal state.
fn walk_motion(line: &eitri_gcode::Line, modal: &mut Modal, out_of_plane: bool, preview: &mut GcodePreview) {
  let has_axis = line.has('X') || line.has('Y') || line.has('Z');
  let is_arc = matches!(modal.motion, Some(2) | Some(3));
  let has_center = line.has('I') || line.has('J') || line.has('R');
  // A block commands motion when it carries an axis word, or is a full-circle arc given only by its centre.
  if modal.motion.is_none() || !(has_axis || (is_arc && has_center)) {
    return;
  }
  let s = modal.units_scale;
  let target = Point3 {
    x: resolve(line.value('X'), modal.pos.x, modal.absolute, s),
    y: resolve(line.value('Y'), modal.pos.y, modal.absolute, s),
    z: resolve(line.value('Z'), modal.pos.z, modal.absolute, s),
  };

  match modal.motion {
    Some(0) => preview.moves.push(PreviewMove { kind: MotionKind::Rapid, from: modal.pos, to: target }),
    Some(1) => preview.moves.push(PreviewMove { kind: MotionKind::Cut, from: modal.pos, to: target }),
    Some(2) | Some(3) => {
      let ccw = modal.motion == Some(3);
      emit_arc(line, modal.pos, target, ccw, out_of_plane, s, preview);
    }
    _ => {}
  }
  modal.pos = target;
}

/// Resolve one axis word to an absolute millimetre coordinate. Absent → unchanged. Absolute → the word (scaled);
/// relative (`G91`) → added to the current position.
fn resolve(word: Option<f64>, current: f64, absolute: bool, scale: f64) -> f64 {
  match word {
    None => current,
    Some(v) if absolute => v * scale,
    Some(v) => current + v * scale,
  }
}

/// Flatten a `G2`/`G3` arc into `Cut` segments. Prefers the `I`/`J` incremental centre (what the eitri emitter
/// produces); falls back to the `R` radius form; degrades to a straight cut with a diagnostic if the centre is
/// absent or the arc is out of the `G17` XY plane.
fn emit_arc(
  line: &eitri_gcode::Line,
  start: Point3,
  target: Point3,
  ccw: bool,
  out_of_plane: bool,
  scale: f64,
  preview: &mut GcodePreview,
) {
  if out_of_plane {
    preview.skipped.push(Skipped::new("G2/G3 arc outside the G17 XY plane", "approximated as a straight cut"));
    preview.moves.push(PreviewMove { kind: MotionKind::Cut, from: start, to: target });
    return;
  }

  let centre = arc_centre(line, start, target, ccw, scale);
  let Some((cx, cy)) = centre else {
    preview.skipped.push(Skipped::new("G2/G3 arc without I/J or R centre", "approximated as a straight cut"));
    preview.moves.push(PreviewMove { kind: MotionKind::Cut, from: start, to: target });
    return;
  };

  let r = (start.x - cx).hypot(start.y - cy);
  let start_angle = (start.y - cy).atan2(start.x - cx);
  let end_angle = (target.y - cy).atan2(target.x - cx);
  let full = coincident(start.xy(), target.xy());
  let sweep = arc_sweep(start_angle, end_angle, ccw, full);

  let pts = flatten_arc(cx, cy, r, start_angle, sweep);
  let n = pts.len();
  let mut prev = start;
  for (k, p) in pts.iter().enumerate() {
    // Linearly interpolate Z along the arc so a helical move (rare here, but correct) is not flattened to a plane.
    let t = (k + 1) as f64 / n as f64;
    let to = Point3 { x: p.x, y: p.y, z: start.z + (target.z - start.z) * t };
    preview.moves.push(PreviewMove { kind: MotionKind::Cut, from: prev, to });
    prev = to;
  }
}

/// The arc centre in absolute millimetres from either the `I`/`J` incremental offset (centre relative to the arc
/// start — grbl's convention) or, failing that, the `R` radius form. `None` when neither is present.
fn arc_centre(line: &eitri_gcode::Line, start: Point3, target: Point3, ccw: bool, scale: f64) -> Option<(f64, f64)> {
  if line.has('I') || line.has('J') {
    let i = line.value('I').unwrap_or(0.0) * scale;
    let j = line.value('J').unwrap_or(0.0) * scale;
    return Some((start.x + i, start.y + j));
  }
  let r = line.value('R')? * scale;
  centre_from_radius(start, target, r, ccw)
}

/// Reconstruct an arc centre from the signed `R` radius (grbl's radius form): the centre lies on the perpendicular
/// bisector of the chord, offset by the apothem. A negative `R` selects the major arc. `None` for a degenerate
/// (zero-length or too-short-for-radius) chord.
fn centre_from_radius(start: Point3, target: Point3, r: f64, ccw: bool) -> Option<(f64, f64)> {
  let dx = target.x - start.x;
  let dy = target.y - start.y;
  let chord = dx.hypot(dy);
  if chord < GEOM_EPSILON_MM {
    return None;
  }
  let half = chord / 2.0;
  let h_sq = r * r - half * half;
  if h_sq < 0.0 {
    return None;
  }
  let h = h_sq.sqrt();
  let mx = (start.x + target.x) / 2.0;
  let my = (start.y + target.y) / 2.0;
  // Unit chord normal (left of the chord direction).
  let nx = -dy / chord;
  let ny = dx / chord;
  // The apothem side that yields the minor arc for R>0 depends on the turn sense; a negative R flips to the major
  // arc. This mirrors grbl's `R`-form sign handling.
  let sign = if (r < 0.0) == ccw { 1.0 } else { -1.0 };
  Some((mx + sign * h * nx, my + sign * h * ny))
}

/// The signed sweep (radians) from `start_angle` to `end_angle` for the given turn sense. CCW sweeps land in
/// `(0, TAU]`, CW in `[-TAU, 0)`; a full circle (coincident endpoints flagged `full`) sweeps a whole turn.
fn arc_sweep(start_angle: f64, end_angle: f64, ccw: bool, full: bool) -> f64 {
  if full {
    return if ccw { TAU } else { -TAU };
  }
  let ccw_amount = (end_angle - start_angle).rem_euclid(TAU);
  if ccw {
    if ccw_amount <= f64::EPSILON { TAU } else { ccw_amount }
  } else if ccw_amount <= f64::EPSILON {
    -TAU
  } else {
    ccw_amount - TAU
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn walks_rapid_and_cut_into_moves() {
    let preview = import_gcode("G90 G21\nG0 X0 Y0\nG1 Z-1 F60\nG1 X10 Y0 F120\n").expect("import");
    assert_eq!(preview.moves.len(), 3, "one rapid + plunge + cut");
    assert_eq!(preview.moves[0].kind, MotionKind::Rapid);
    assert_eq!(preview.moves[1].kind, MotionKind::Cut);
    assert_eq!(preview.moves[2].to, Point3 { x: 10.0, y: 0.0, z: -1.0 });
  }

  #[test]
  fn relative_distance_mode_accumulates() {
    let preview = import_gcode("G91\nG1 X5\nG1 X5\n").expect("import");
    assert_eq!(preview.moves.last().unwrap().to.x, 10.0, "G91 moves accumulate");
  }

  #[test]
  fn inch_units_convert_to_millimetres() {
    let preview = import_gcode("G90 G20\nG1 X1 Y0\n").expect("import");
    assert!((preview.moves[0].to.x - MM_PER_INCH).abs() < 1e-9, "G20 inch coord -> mm");
  }

  #[test]
  fn cut_polylines_recover_the_contour_without_the_plunge() {
    // A plunge (Z only) then three planar cuts: the recovered XY polyline is exactly the contour, the plunge leaves
    // no XY trace, and the rapid up closes the run.
    let src = "G90 G21\nG0 X0 Y0\nG1 Z-1 F60\nG1 X10 Y0 F120\nG1 X10 Y10\nG1 X0 Y0\nG0 Z2\n";
    let polys = import_gcode(src).expect("import").cut_polylines();
    assert_eq!(polys.len(), 1);
    let pts: Vec<(f64, f64)> = polys[0].0.iter().map(|c| (c.x, c.y)).collect();
    assert_eq!(pts, vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 0.0)]);
  }

  #[test]
  fn g3_quarter_arc_recovers_points_on_the_circle() {
    // CCW quarter arc from (1,0) to (0,1) about the origin (I/J = centre - start = (-1, 0)).
    let src = "G90 G21\nG1 X1 Y0 F120\nG3 X0 Y1 I-1 J0 F120\n";
    let preview = import_gcode(src).expect("import");
    assert!(preview.skipped.is_empty(), "a valid in-plane arc is not skipped");
    // Every arc segment endpoint must sit on the unit circle, and the last must land at (0,1).
    let arc_moves: Vec<&PreviewMove> = preview.moves.iter().filter(|m| m.kind == MotionKind::Cut).collect();
    for m in &arc_moves {
      let r = m.to.x.hypot(m.to.y);
      assert!((r - 1.0).abs() < 1e-6 || (m.to.x - 1.0).abs() < 1e-9, "point off circle: {:?}", m.to);
    }
    let end = arc_moves.last().unwrap().to;
    assert!((end.x - 0.0).abs() < 1e-6 && (end.y - 1.0).abs() < 1e-6, "arc end {end:?}");
    assert!(arc_moves.len() >= 8, "quarter arc should flatten into several chords");
  }

  #[test]
  fn arc_without_centre_degrades_loudly_to_a_straight_cut() {
    let preview = import_gcode("G90 G21\nG1 X0 Y0\nG2 X10 Y0\n").expect("import");
    assert_eq!(preview.skipped.len(), 1, "missing-centre arc is reported");
    assert!(preview.moves.iter().any(|m| m.kind == MotionKind::Cut && m.to.x == 10.0));
  }
}
