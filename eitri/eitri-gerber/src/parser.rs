//! The stateful RS-274X interpreter: walk the lexed statements, build flashes/draws/regions honouring polarity,
//! and assemble the final copper `MultiPolygon`.
//!
//! Provenance: reproduces the interpretation model of FlatCAM's `camlib` Gerber parser (aperture flashes, stroked
//! draws, `G36/G37` regions, and `LP` dark/clear accumulation), rebuilt on the `eitri-geo` backend.
//!
//! Deferred constructs (rare; each errors loudly rather than silently dropping geometry): step-repeat (`SR`),
//! aperture blocks (`AB`), and single-quadrant arc mode (`G74`). Aperture transforms (`LM`/`LR`/`LS`) and X2
//! attribute commands (`TF`/`TA`/`TO`/`TD`) are accepted and ignored. See docs/eitri-porting-plan.md §4.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use eitri_core::{CancelToken, ProgressEvent, ProgressReporter, Unit};
use eitri_geo::{CapStyle, DefaultBackend, GeoBackend, buffer_path};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use crate::aperture::{Aperture, parse_ad};
use crate::error::{GerberError, Result};
use crate::format::CoordinateFormat;
use crate::geometry::arc_segment_count;
use crate::lexer::{Statement, tokenize};
use crate::macros::MacroDef;

/// Poll the cancellation token every this many statements so a long parse stays responsive.
const CANCEL_CHECK_INTERVAL: usize = 256;

/// The resolved result of parsing a Gerber file: the assembled copper plus the aperture table and modes.
#[derive(Debug, Clone)]
pub struct GerberImage {
  /// The final copper geometry (all dark added, all clear subtracted, in order).
  pub copper: MultiPolygon<f64>,
  /// The aperture table, keyed by D-code.
  pub apertures: BTreeMap<u32, Aperture>,
  /// The file's coordinate unit.
  pub unit: Unit,
  /// The coordinate format from the `FS` block.
  pub format: CoordinateFormat,
}

impl GerberImage {
  /// The axis-aligned bounding box of the copper as `(min_x, min_y, max_x, max_y)`, or `None` if empty.
  pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
    let mut it = self.copper.0.iter().flat_map(|p| p.exterior().0.iter());
    let first = it.next()?;
    let mut b = (first.x, first.y, first.x, first.y);
    for c in self.copper.0.iter().flat_map(|p| p.exterior().0.iter()) {
      b.0 = b.0.min(c.x);
      b.1 = b.1.min(c.y);
      b.2 = b.2.max(c.x);
      b.3 = b.3.max(c.y);
    }
    Some(b)
  }
}

/// Dark adds copper; clear subtracts it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Polarity {
  Dark,
  Clear,
}

/// Interpolation mode for `D01` draws.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InterpMode {
  Linear,
  ClockwiseArc,
  CounterClockwiseArc,
}

/// Parse a Gerber source into a [`GerberImage`], reporting progress and honouring cancellation. This is the public
/// entry point; the parse can be long (thousands of primitives), so callers pass a reporter and token per §12.
pub fn parse_gerber(source: &str, progress: &ProgressReporter, cancel: &CancelToken) -> Result<GerberImage> {
  progress.emit(ProgressEvent::Started { label: "parse gerber".to_string() });
  let statements = tokenize(source)?;
  let total = statements.len() as u64;

  let backend = DefaultBackend::new();
  let mut interp = Interpreter::new(&backend);

  for (index, located) in statements.iter().enumerate() {
    if index % CANCEL_CHECK_INTERVAL == 0 {
      if cancel.is_cancelled() {
        return Err(GerberError::Cancelled);
      }
      progress.advance(index as u64, total);
    }
    interp.step(located.line, &located.statement)?;
  }

  let image = interp.finish()?;
  progress.emit(ProgressEvent::Finished);
  Ok(image)
}

/// Holds all interpreter state as the statement stream is walked.
struct Interpreter<'a> {
  backend: &'a dyn GeoBackend,
  format: Option<CoordinateFormat>,
  unit: Option<Unit>,
  apertures: BTreeMap<u32, Aperture>,
  macros: BTreeMap<String, MacroDef>,
  current_aperture: Option<u32>,
  interp: InterpMode,
  polarity: Polarity,
  x: f64,
  y: f64,
  in_region: bool,
  region_contours: Vec<Vec<Coord<f64>>>,
  region_current: Vec<Coord<f64>>,
  // Every emitted primitive, tagged with the polarity in force, kept in order for correct dark/clear assembly.
  primitives: Vec<(Polarity, Polygon<f64>)>,
}

impl<'a> Interpreter<'a> {
  fn new(backend: &'a dyn GeoBackend) -> Interpreter<'a> {
    Interpreter {
      backend,
      format: None,
      unit: None,
      apertures: BTreeMap::new(),
      macros: BTreeMap::new(),
      current_aperture: None,
      interp: InterpMode::Linear,
      polarity: Polarity::Dark,
      x: 0.0,
      y: 0.0,
      in_region: false,
      region_contours: Vec::new(),
      region_current: Vec::new(),
      primitives: Vec::new(),
    }
  }

  /// Millimetres-per-file-unit; errors if `MO` has not been seen yet.
  fn unit_scale(&self, line: usize) -> Result<f64> {
    self.unit.map(|u| u.mm_per_unit()).ok_or(GerberError::Syntax {
      line,
      message: "unit (MO) must be set before apertures or coordinates".to_string(),
    })
  }

  fn step(&mut self, line: usize, statement: &Statement) -> Result<()> {
    match statement {
      Statement::Extended(blocks) => self.extended(line, blocks),
      Statement::Word(word) => self.word(line, word),
    }
  }

  /// Handle an extended `%...%` command from its sub-blocks.
  fn extended(&mut self, line: usize, blocks: &[String]) -> Result<()> {
    let Some(first) = blocks.first() else { return Ok(()) };
    let code = &first[..first.len().min(2)];
    match code {
      "FS" => {
        self.format = Some(CoordinateFormat::parse_fs(&first[2..], line)?);
      }
      "MO" => {
        self.unit = Some(match &first[2..] {
          "MM" => Unit::Millimeters,
          "IN" => Unit::Inches,
          other => return Err(GerberError::Syntax { line, message: format!("unknown unit '{other}'") }),
        });
      }
      "AD" => {
        let scale = self.unit_scale(line)?;
        let (code, aperture) = parse_ad(&first[2..], |v| v * scale, line)?;
        self.apertures.insert(code, aperture);
      }
      "AM" => {
        // Macros are stored in file units and their resolved geometry is scaled to mm at flash time (see
        // Aperture::flash), so no unit is needed at definition.
        let name = first[2..].to_string();
        let rest: Vec<&str> = blocks[1..].iter().map(String::as_str).collect();
        let def = MacroDef::parse(&name, &rest, line)?;
        self.macros.insert(name, def);
      }
      "LP" => {
        self.polarity = match &first[2..] {
          "D" => Polarity::Dark,
          "C" => Polarity::Clear,
          other => return Err(GerberError::Syntax { line, message: format!("unknown polarity '{other}'") }),
        };
      }
      // X2 attributes and aperture transforms: accepted and ignored (documented deferral).
      "TF" | "TA" | "TO" | "TD" | "LM" | "LR" | "LS" | "IN" | "LN" | "IP" | "AS" | "MI" | "OF" | "SF" => {}
      // Constructs that would silently change geometry if ignored — refuse loudly instead.
      "SR" => return Err(GerberError::Unsupported { line, message: "step-repeat (SR) is not supported".to_string() }),
      "AB" => return Err(GerberError::Unsupported { line, message: "aperture blocks (AB) are not supported".to_string() }),
      other => return Err(GerberError::Unsupported { line, message: format!("extended command '{other}'") }),
    }
    Ok(())
  }

  /// Handle a function-code word block.
  fn word(&mut self, line: usize, word: &str) -> Result<()> {
    if word.starts_with("G04") {
      return Ok(()); // comment
    }
    let words = split_words(word);

    let mut new_x = self.x;
    let mut new_y = self.y;
    let mut i_off: Option<f64> = None;
    let mut j_off: Option<f64> = None;
    let mut operation: Option<u32> = None;

    for (letter, value) in &words {
      match letter {
        'G' => self.apply_g_code(line, value)?,
        'D' => {
          let d: u32 = value.parse().map_err(|_| GerberError::Syntax { line, message: format!("bad D code '{value}'") })?;
          if d >= 10 {
            if !self.apertures.contains_key(&d) {
              return Err(GerberError::UndefinedAperture(d));
            }
            self.current_aperture = Some(d);
          } else {
            operation = Some(d);
          }
        }
        'X' => new_x = self.decode(line, value)?,
        'Y' => new_y = self.decode(line, value)?,
        'I' => i_off = Some(self.decode(line, value)?),
        'J' => j_off = Some(self.decode(line, value)?),
        'M' => { /* M00/M01/M02 — end of file / stop; nothing to accumulate. */ }
        other => return Err(GerberError::Syntax { line, message: format!("unexpected word letter '{other}'") }),
      }
    }

    if let Some(op) = operation {
      self.operate(line, op, new_x, new_y, i_off, j_off)?;
    } else {
      // A bare coordinate move with no D-op keeps the modal point in sync (some files emit this).
      self.x = new_x;
      self.y = new_y;
    }
    Ok(())
  }

  fn apply_g_code(&mut self, line: usize, value: &str) -> Result<()> {
    match value {
      "01" | "1" => self.interp = InterpMode::Linear,
      "02" | "2" => self.interp = InterpMode::ClockwiseArc,
      "03" | "3" => self.interp = InterpMode::CounterClockwiseArc,
      "36" => {
        self.in_region = true;
        self.region_contours.clear();
        self.region_current.clear();
      }
      "37" => {
        self.finish_region_contour();
        self.emit_region()?;
        self.in_region = false;
      }
      "74" => return Err(GerberError::Unsupported { line, message: "single-quadrant arc mode (G74)".to_string() }),
      // G75 multi-quadrant (default), G54 aperture-select prefix, G70/G71 unit, G90/G91 notation — accepted no-ops
      // here (the effective state is carried by FS/MO or the following D-code).
      "75" | "54" | "70" | "71" | "90" | "91" | "0" | "55" => {}
      other => return Err(GerberError::Unsupported { line, message: format!("G-code G{other}") }),
    }
    Ok(())
  }

  fn decode(&self, line: usize, value: &str) -> Result<f64> {
    let format = self.format.as_ref().ok_or(GerberError::MissingFormat)?;
    let raw = format.decode(value, line)?;
    Ok(raw * self.unit_scale(line)?)
  }

  /// Execute a D01/D02/D03 operation at the (already mm-scaled) target point.
  fn operate(&mut self, line: usize, op: u32, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Result<()> {
    match op {
      1 => {
        // Draw / interpolate.
        if self.in_region {
          self.region_interpolate(tx, ty, i, j);
        } else {
          self.stroke(line, tx, ty, i, j)?;
        }
        self.x = tx;
        self.y = ty;
      }
      2 => {
        // Move.
        if self.in_region {
          // Starting a new contour: close the running one first.
          self.finish_region_contour();
        }
        self.x = tx;
        self.y = ty;
      }
      3 => {
        // Flash.
        if self.in_region {
          return Err(GerberError::Syntax { line, message: "flash (D03) not allowed inside a region".to_string() });
        }
        self.flash(line, tx, ty)?;
        self.x = tx;
        self.y = ty;
      }
      other => return Err(GerberError::Syntax { line, message: format!("invalid operation code D0{other}") }),
    }
    Ok(())
  }

  /// Stroke a draw from the current point to `(tx, ty)` with the current aperture, adding the band as a primitive.
  fn stroke(&mut self, line: usize, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Result<()> {
    let code = self.current_aperture.ok_or(GerberError::Syntax { line, message: "draw with no aperture selected".to_string() })?;
    let radius = self.apertures[&code].stroke_radius();
    let path = self.interpolated_path(tx, ty, i, j);
    let band = buffer_path(&LineString(path), radius, CapStyle::Round)?;
    self.push_multipolygon(band);
    Ok(())
  }

  /// Flash the current aperture at `(tx, ty)`.
  fn flash(&mut self, line: usize, tx: f64, ty: f64) -> Result<()> {
    let code = self.current_aperture.ok_or(GerberError::Syntax { line, message: "flash with no aperture selected".to_string() })?;
    let aperture = self.apertures[&code].clone();
    let scale = self.unit_scale(line)?;
    let geom = aperture.flash(tx, ty, &self.macros, self.backend, scale, line)?;
    self.push_multipolygon(geom);
    Ok(())
  }

  /// Add every polygon of a multipolygon as a primitive under the current polarity.
  fn push_multipolygon(&mut self, mp: MultiPolygon<f64>) {
    for poly in mp.0 {
      self.primitives.push((self.polarity, poly));
    }
  }

  /// Append the interpolated points (current point excluded, target included) to the region contour.
  fn region_interpolate(&mut self, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) {
    if self.region_current.is_empty() {
      self.region_current.push(Coord { x: self.x, y: self.y });
    }
    let path = self.interpolated_path(tx, ty, i, j);
    // path[0] is the current point (already present); push the rest.
    self.region_current.extend(path.into_iter().skip(1));
  }

  fn finish_region_contour(&mut self) {
    if self.region_current.len() >= 3 {
      self.region_contours.push(std::mem::take(&mut self.region_current));
    } else {
      self.region_current.clear();
    }
  }

  /// Build the region's filled polygons (even-odd nesting) and add them as primitives.
  fn emit_region(&mut self) -> Result<()> {
    let contours = std::mem::take(&mut self.region_contours);
    for poly in assemble_region(contours) {
      self.primitives.push((self.polarity, poly));
    }
    Ok(())
  }

  /// The polyline from the current point to `(tx, ty)`: a straight segment when linear, a flattened arc otherwise.
  fn interpolated_path(&self, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Vec<Coord<f64>> {
    match self.interp {
      InterpMode::Linear => vec![Coord { x: self.x, y: self.y }, Coord { x: tx, y: ty }],
      InterpMode::ClockwiseArc | InterpMode::CounterClockwiseArc => {
        let ccw = matches!(self.interp, InterpMode::CounterClockwiseArc);
        flatten_arc(self.x, self.y, tx, ty, i.unwrap_or(0.0), j.unwrap_or(0.0), ccw)
      }
    }
  }

  /// Assemble the final copper: fold the ordered primitives into copper, batching each same-polarity run through a
  /// single `union_all`, then unioning (dark) or differencing (clear) into the running result.
  fn finish(mut self) -> Result<GerberImage> {
    let format = self.format.ok_or(GerberError::MissingFormat)?;
    let unit = self.unit.unwrap_or(Unit::Millimeters);

    let mut copper = MultiPolygon::new(Vec::new());
    let primitives = std::mem::take(&mut self.primitives);
    let mut idx = 0;
    while idx < primitives.len() {
      let polarity = primitives[idx].0;
      let mut group: Vec<Polygon<f64>> = Vec::new();
      while idx < primitives.len() && primitives[idx].0 == polarity {
        group.push(primitives[idx].1.clone());
        idx += 1;
      }
      let merged = self.backend.union_all(&group)?;
      copper = match polarity {
        Polarity::Dark => {
          let combined: Vec<Polygon<f64>> = copper.0.into_iter().chain(merged.0).collect();
          self.backend.union_all(&combined)?
        }
        Polarity::Clear => self.backend.difference(&copper, &merged)?,
      };
    }

    Ok(GerberImage { copper, apertures: self.apertures, unit, format })
  }
}

/// Split a word block into `(letter, value)` pairs. A value is the run of sign/digit/dot chars after a letter.
fn split_words(block: &str) -> Vec<(char, String)> {
  let mut out = Vec::new();
  let mut chars = block.chars().peekable();
  while let Some(&c) = chars.peek() {
    if c.is_ascii_alphabetic() {
      chars.next();
      let mut value = String::new();
      while let Some(&d) = chars.peek() {
        if d.is_ascii_digit() || d == '+' || d == '-' || d == '.' {
          value.push(d);
          chars.next();
        } else {
          break;
        }
      }
      out.push((c, value));
    } else {
      chars.next();
    }
  }
  out
}

/// Flatten a circular arc from `(sx, sy)` to `(ex, ey)` with centre offset `(i, j)` (multi-quadrant) into points,
/// including both endpoints. `ccw` selects the sweep direction.
fn flatten_arc(sx: f64, sy: f64, ex: f64, ey: f64, i: f64, j: f64, ccw: bool) -> Vec<Coord<f64>> {
  let (cx, cy) = (sx + i, sy + j);
  let radius = ((sx - cx).powi(2) + (sy - cy).powi(2)).sqrt();
  if radius <= f64::EPSILON {
    return vec![Coord { x: sx, y: sy }, Coord { x: ex, y: ey }];
  }
  let a_start = (sy - cy).atan2(sx - cx);
  let a_end = (ey - cy).atan2(ex - cx);

  // Determine the swept angle in [0, TAU) in the chosen direction; a full circle (start == end) sweeps TAU.
  let mut sweep = if ccw { a_end - a_start } else { a_start - a_end };
  while sweep <= 0.0 {
    sweep += TAU;
  }
  let coincident = (sx - ex).abs() < 1e-9 && (sy - ey).abs() < 1e-9;
  if coincident {
    sweep = TAU;
  }

  let steps = arc_segment_count(radius, sweep).max(1);
  let mut points = Vec::with_capacity(steps + 1);
  for k in 0..=steps {
    let t = sweep * (k as f64) / (steps as f64);
    let angle = if ccw { a_start + t } else { a_start - t };
    points.push(Coord { x: cx + radius * angle.cos(), y: cy + radius * angle.sin() });
  }
  points
}

/// Assemble region contours into filled polygons using even-odd nesting: a contour enclosed by an even number of
/// others is solid, an odd number makes it a hole of the nearest enclosing solid.
fn assemble_region(contours: Vec<Vec<Coord<f64>>>) -> Vec<Polygon<f64>> {
  let rings: Vec<LineString<f64>> = contours.into_iter().map(LineString).collect();
  let reps: Vec<Coord<f64>> = rings.iter().map(representative_point).collect();

  // depth[k] = how many other rings contain ring k's representative point.
  let depths: Vec<usize> = (0..rings.len())
    .map(|k| (0..rings.len()).filter(|&m| m != k && point_in_ring(reps[k], &rings[m])).count())
    .collect();

  let mut polygons = Vec::new();
  for (k, ring) in rings.iter().enumerate() {
    if depths[k] % 2 != 0 {
      continue; // odd depth => a hole, attached below to its enclosing solid
    }
    // Holes of this solid: rings one level deeper that this ring contains.
    let holes: Vec<LineString<f64>> = rings
      .iter()
      .enumerate()
      .filter(|&(m, _)| m != k && depths[m] == depths[k] + 1 && point_in_ring(reps[m], ring))
      .map(|(_, r)| r.clone())
      .collect();
    polygons.push(Polygon::new(ring.clone(), holes));
  }
  polygons
}

/// The mean of a ring's vertices — a representative interior point for simple (convex-ish) contours.
fn representative_point(ring: &LineString<f64>) -> Coord<f64> {
  let n = ring.0.len().max(1) as f64;
  let (sx, sy) = ring.0.iter().fold((0.0, 0.0), |(sx, sy), c| (sx + c.x, sy + c.y));
  Coord { x: sx / n, y: sy / n }
}

/// Ray-casting point-in-polygon test against a ring's vertices.
fn point_in_ring(p: Coord<f64>, ring: &LineString<f64>) -> bool {
  let pts = &ring.0;
  let n = pts.len();
  if n < 3 {
    return false;
  }
  let mut inside = false;
  let mut jdx = n - 1;
  for idx in 0..n {
    let (xi, yi) = (pts[idx].x, pts[idx].y);
    let (xj, yj) = (pts[jdx].x, pts[jdx].y);
    let intersects = (yi > p.y) != (yj > p.y) && p.x < (xj - xi) * (p.y - yi) / (yj - yi) + xi;
    if intersects {
      inside = !inside;
    }
    jdx = idx;
  }
  inside
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_core::{CancelToken, ProgressReporter};
  use geo::algorithm::area::Area;

  fn parse(src: &str) -> GerberImage {
    parse_gerber(src, &ProgressReporter::silent(), &CancelToken::new()).expect("parse")
  }

  #[test]
  fn flashes_a_circle_pad() {
    // 2mm circle flashed at (5,5): area ~ pi.
    let src = "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,2*%\nD10*\nX5000000Y5000000D03*\nM02*\n";
    let img = parse(src);
    assert!((img.copper.unsigned_area() - std::f64::consts::PI).abs() < 0.05, "area {}", img.copper.unsigned_area());
    let (minx, miny, maxx, maxy) = img.bounds().unwrap();
    assert!((minx - 4.0).abs() < 0.05 && (maxx - 6.0).abs() < 0.05 && (miny - 4.0).abs() < 0.05 && (maxy - 6.0).abs() < 0.05);
  }

  #[test]
  fn strokes_a_trace() {
    // A 10mm horizontal trace with a 1mm round aperture: capsule area 10*1 + pi*0.25 ≈ 10.785.
    let src = "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,1*%\nD10*\nX0Y0D02*\nX10000000Y0D01*\nM02*\n";
    let img = parse(src);
    assert!((img.copper.unsigned_area() - 10.785).abs() < 0.05, "area {}", img.copper.unsigned_area());
  }

  #[test]
  fn fills_a_g36_region() {
    // A 10x10 square region: area 100.
    let src = "%FSLAX36Y36*%\n%MOMM*%\nG36*\nX0Y0D02*\nX10000000Y0D01*\nX10000000Y10000000D01*\nX0Y10000000D01*\nX0Y0D01*\nG37*\nM02*\n";
    let img = parse(src);
    assert!((img.copper.unsigned_area() - 100.0).abs() < 1e-3, "area {}", img.copper.unsigned_area());
  }

  #[test]
  fn lp_clear_subtracts_from_dark() {
    // Dark 10x10 region, then a clear 4x4 region inside: 100 - 16 = 84.
    let src = concat!(
      "%FSLAX36Y36*%\n%MOMM*%\n",
      "G36*\nX0Y0D02*\nX10000000Y0D01*\nX10000000Y10000000D01*\nX0Y10000000D01*\nX0Y0D01*\nG37*\n",
      "%LPC*%\n",
      "G36*\nX3000000Y3000000D02*\nX7000000Y3000000D01*\nX7000000Y7000000D01*\nX3000000Y7000000D01*\nX3000000Y3000000D01*\nG37*\n",
      "M02*\n"
    );
    let img = parse(src);
    assert!((img.copper.unsigned_area() - 84.0).abs() < 1e-3, "area {}", img.copper.unsigned_area());
  }

  #[test]
  fn macro_aperture_flash() {
    // Define a macro that is a circle of diameter $1, then flash it.
    let src = concat!(
      "%FSLAX36Y36*%\n%MOMM*%\n",
      "%AMROUND*\n1,1,$1,0,0*%\n",
      "%ADD10ROUND,3*%\n",
      "D10*\nX0Y0D03*\nM02*\n"
    );
    let img = parse(src);
    // Circle diameter 3 -> radius 1.5 -> area ~ pi*2.25 = 7.07.
    assert!((img.copper.unsigned_area() - std::f64::consts::PI * 2.25).abs() < 0.05, "area {}", img.copper.unsigned_area());
  }

  #[test]
  fn cancellation_stops_the_parse() {
    let src = "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,1*%\nD10*\nX0Y0D03*\nM02*\n";
    let cancel = CancelToken::new();
    cancel.cancel();
    let err = parse_gerber(src, &ProgressReporter::silent(), &cancel);
    assert!(matches!(err, Err(GerberError::Cancelled)));
  }

  #[test]
  fn coordinate_before_fs_errors() {
    assert!(parse_gerber("X100Y100D02*\n", &ProgressReporter::silent(), &CancelToken::new()).is_err());
  }
}
