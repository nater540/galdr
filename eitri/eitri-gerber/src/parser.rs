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
use eitri_geo::{CapStyle, DefaultBackend, GeoBackend, buffer_path, ring_interior_point};
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
  // The last D01/D02/D03 operation, retained so a coordinate line that omits its operation code repeats it (RS-274X
  // operation codes are modal). `None` until the first explicit operation.
  modal_op: Option<u32>,
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
      modal_op: None,
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
      // X2 attributes, aperture transforms, and pure annotations: accepted and ignored (documented deferral).
      "TF" | "TA" | "TO" | "TD" | "LM" | "LR" | "LS" | "IN" | "LN" => {}
      "IP" => {
        // Image polarity. Positive is the default and a genuine no-op. Negative inverts the whole image
        // (copper<->clearance) against an unbounded background, which we do not implement — refuse loudly rather
        // than pass inverted-meaning geometry through unchanged.
        match &first[2..] {
          "POS" => {}
          "NEG" => return Err(GerberError::Unsupported { line, message: "negative image polarity (IPNEG)".to_string() }),
          other => return Err(GerberError::Syntax { line, message: format!("unknown image polarity '{other}'") }),
        }
      }
      // Deprecated whole-image transforms that mirror/offset/scale/swap the coordinate data. Ignoring them would
      // silently misplace every primitive, so refuse loudly (they are rare and disproportionate to implement).
      "MI" => return Err(GerberError::Unsupported { line, message: "image mirror (MI) is not supported".to_string() }),
      "OF" => return Err(GerberError::Unsupported { line, message: "image offset (OF) is not supported".to_string() }),
      "SF" => return Err(GerberError::Unsupported { line, message: "image scale (SF) is not supported".to_string() }),
      "AS" => return Err(GerberError::Unsupported { line, message: "axis select (AS) is not supported".to_string() }),
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
    let mut has_coord = false;

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
        'X' => { new_x = self.decode(line, value)?; has_coord = true; }
        'Y' => { new_y = self.decode(line, value)?; has_coord = true; }
        'I' => { i_off = Some(self.decode(line, value)?); has_coord = true; }
        'J' => { j_off = Some(self.decode(line, value)?); has_coord = true; }
        'M' => { /* M00/M01/M02 — end of file / stop; nothing to accumulate. */ }
        other => return Err(GerberError::Syntax { line, message: format!("unexpected word letter '{other}'") }),
      }
    }

    // Operation codes are modal: an explicit D01/D02/D03 sets the mode; a coordinate line that omits the D-word
    // repeats the last one. A line with no coordinate data (aperture-select or G/M only) triggers no operation.
    match operation {
      Some(op) => {
        self.modal_op = Some(op);
        self.operate(line, op, new_x, new_y, i_off, j_off)?;
      }
      None if has_coord => match self.modal_op {
        Some(op) => self.operate(line, op, new_x, new_y, i_off, j_off)?,
        // Coordinate data before any operation code was ever set: just track the modal point.
        None => {
          self.x = new_x;
          self.y = new_y;
        }
      },
      None => {}
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
          self.region_interpolate(line, tx, ty, i, j)?;
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
  /// Only a circular aperture can meaningfully stroke; any other aperture is refused loudly (see
  /// [`Aperture::stroke_radius`]) rather than silently emitting a wrong-width or empty band.
  fn stroke(&mut self, line: usize, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Result<()> {
    let code = self.current_aperture.ok_or(GerberError::Syntax { line, message: "draw with no aperture selected".to_string() })?;
    let radius = self.apertures[&code].stroke_radius().ok_or_else(|| GerberError::Unsupported {
      line,
      message: "stroke (D01) with a non-circular aperture (only circular apertures may draw)".to_string(),
    })?;
    let path = self.interpolated_path(line, tx, ty, i, j)?;
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
  fn region_interpolate(&mut self, line: usize, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Result<()> {
    if self.region_current.is_empty() {
      self.region_current.push(Coord { x: self.x, y: self.y });
    }
    let path = self.interpolated_path(line, tx, ty, i, j)?;
    // path[0] is the current point (already present); push the rest.
    self.region_current.extend(path.into_iter().skip(1));
    Ok(())
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
  /// A `G02`/`G03` arc with neither `I` nor `J` is malformed (radius zero, no centre) — reject it as invalid
  /// geometry rather than silently degenerating it into a straight chord (mirrors the firmware's `error:33`).
  fn interpolated_path(&self, line: usize, tx: f64, ty: f64, i: Option<f64>, j: Option<f64>) -> Result<Vec<Coord<f64>>> {
    match self.interp {
      InterpMode::Linear => Ok(vec![Coord { x: self.x, y: self.y }, Coord { x: tx, y: ty }]),
      InterpMode::ClockwiseArc | InterpMode::CounterClockwiseArc => {
        if i.is_none() && j.is_none() {
          return Err(GerberError::InvalidGeometry {
            line,
            message: "circular interpolation (G02/G03) with neither I nor J offset".to_string(),
          });
        }
        let ccw = matches!(self.interp, InterpMode::CounterClockwiseArc);
        Ok(flatten_arc(self.x, self.y, tx, ty, i.unwrap_or(0.0), j.unwrap_or(0.0), ccw))
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
/// others is solid, an odd number makes it a hole of the nearest enclosing solid. Containment is probed with a
/// robust interior point (see [`eitri_geo::ring_interior_point`]) — a concave contour's vertex mean can fall
/// outside the contour and silently flip the depth parity, corrupting the nesting.
fn assemble_region(contours: Vec<Vec<Coord<f64>>>) -> Vec<Polygon<f64>> {
  let rings: Vec<LineString<f64>> = contours.into_iter().map(LineString).collect();
  let reps: Vec<Option<Coord<f64>>> = rings.iter().map(ring_interior_point).collect();

  // depth[k] = how many other rings contain ring k's representative point. A ring with no valid interior point
  // (degenerate) is skipped entirely below, so its depth is irrelevant.
  let depths: Vec<usize> = (0..rings.len())
    .map(|k| match reps[k] {
      Some(p) => (0..rings.len()).filter(|&m| m != k && point_in_ring(p, &rings[m])).count(),
      None => 0,
    })
    .collect();

  let mut polygons = Vec::new();
  for (k, ring) in rings.iter().enumerate() {
    let Some(_) = reps[k] else { continue }; // degenerate ring contributes no fill
    if depths[k] % 2 != 0 {
      continue; // odd depth => a hole, attached below to its enclosing solid
    }
    // Holes of this solid: rings one level deeper that this ring contains.
    let holes: Vec<LineString<f64>> = rings
      .iter()
      .enumerate()
      .filter(|&(m, _)| {
        m != k && depths[m] == depths[k] + 1 && reps[m].is_some_and(|pm| point_in_ring(pm, ring))
      })
      .map(|(_, r)| r.clone())
      .collect();
    polygons.push(Polygon::new(ring.clone(), holes));
  }
  polygons
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
  fn concave_region_outer_is_not_demoted_by_a_bad_representative_point() {
    // Finding #2: even-odd region nesting must probe each contour with a point that is genuinely inside it. Here a
    // concave outer (a U: a 10x10 square minus a top-middle notch) and a small disjoint square sitting in that notch.
    // The U contour's VERTEX MEAN is (5, 6.25) — in the notch, and inside the square — so the old mean-of-vertices
    // probe counted the U as enclosed (odd depth) and demoted it to a (bogus) hole of the square, collapsing two
    // solids into one malformed polygon. A robust interior point lands in the U's body, keeping both as solids.
    let coord = |x: f64, y: f64| Coord { x, y };
    let u = vec![
      coord(0.0, 0.0), coord(10.0, 0.0), coord(10.0, 10.0), coord(7.0, 10.0),
      coord(7.0, 5.0), coord(3.0, 5.0), coord(3.0, 10.0), coord(0.0, 10.0),
    ];
    let square = vec![coord(4.0, 6.0), coord(6.0, 6.0), coord(6.0, 7.0), coord(4.0, 7.0)];

    // Precondition: the U contour's vertex mean lands inside the square (the exact trap the fix removes).
    let n = u.len() as f64;
    let mean = Coord { x: u.iter().map(|c| c.x).sum::<f64>() / n, y: u.iter().map(|c| c.y).sum::<f64>() / n };
    assert!(point_in_ring(mean, &LineString(square.clone())), "precondition: U's vertex mean falls in the square");

    let polys = assemble_region(vec![u, square]);
    assert_eq!(polys.len(), 2, "the U and the square must remain two disjoint solids");
    assert!(polys.iter().all(|p| p.interiors().is_empty()), "neither disjoint solid should carry a spurious hole");
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

  fn try_parse(src: &str) -> Result<GerberImage> {
    parse_gerber(src, &ProgressReporter::silent(), &CancelToken::new())
  }

  #[test]
  fn negative_image_polarity_is_refused_loudly() {
    // Finding #1: %IPNEG*% inverts the whole image; we do not implement it, so it must error rather than pass the
    // image through with its copper/clearance meaning silently inverted. Positive polarity stays a no-op.
    let neg = try_parse("%FSLAX36Y36*%\n%MOMM*%\n%IPNEG*%\nM02*\n");
    assert!(matches!(neg, Err(GerberError::Unsupported { .. })), "IPNEG must be refused, got {neg:?}");
    let pos = try_parse("%FSLAX36Y36*%\n%MOMM*%\n%IPPOS*%\n%ADD10C,1*%\nD10*\nX0Y0D03*\nM02*\n");
    assert!(pos.is_ok(), "IPPOS is the default and must parse, got {pos:?}");
  }

  #[test]
  fn image_transforms_are_refused_loudly() {
    // Finding #1: MI/OF/SF/AS move, mirror, scale or swap the whole image; ignoring them misplaces every primitive.
    for cmd in ["%MIA1B0*%", "%OFA5B0*%", "%SFA2B2*%", "%ASAXBY*%"] {
      let src = format!("%FSLAX36Y36*%\n%MOMM*%\n{cmd}\nM02*\n");
      let err = try_parse(&src);
      assert!(matches!(err, Err(GerberError::Unsupported { .. })), "{cmd} must be refused, got {err:?}");
    }
  }

  #[test]
  fn stroke_with_a_macro_aperture_errors_not_empty() {
    // Finding #4: a D01 draw with a macro aperture selected used to yield no copper (stroke_radius returned 0). A
    // non-circular aperture cannot meaningfully stroke, so it must be refused loudly.
    let src = concat!(
      "%FSLAX36Y36*%\n%MOMM*%\n",
      "%AMROUND*\n1,1,$1,0,0*%\n",
      "%ADD10ROUND,3*%\n",
      "D10*\nX0Y0D02*\nX1000000Y0D01*\nM02*\n"
    );
    let err = try_parse(src);
    assert!(matches!(err, Err(GerberError::Unsupported { .. })), "macro stroke must be refused, got {err:?}");
  }

  #[test]
  fn arc_with_no_offset_is_invalid_geometry() {
    // Finding #5: a G02/G03 with neither I nor J is malformed (no centre). It must error, not degenerate silently
    // into a straight chord.
    let src = concat!(
      "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,1*%\n",
      "D10*\nX5000000Y0D02*\nG02*\nX0Y0D01*\nM02*\n"
    );
    let err = try_parse(src);
    assert!(matches!(err, Err(GerberError::InvalidGeometry { .. })), "arc with no I/J must error, got {err:?}");
  }

  #[test]
  fn operation_code_is_modal_across_coordinate_lines() {
    // Finding #6: a bare coordinate line after `…D01*` repeats the previous operation (a stroke), not a move. Here a
    // stroke to (1,0) then a bare line to (2,0) must produce ONE continuous capsule from x=0 to x=2, not stop at 1.
    let src = concat!(
      "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,1*%\n",
      "D10*\nX0Y0D02*\nX1000000Y0D01*\nX2000000Y0*\nM02*\n"
    );
    let img = parse(src);
    // Capsule 0->2 with a 1mm round aperture: 2*1 + pi*0.25 ~= 2.785. A single stroke to x=1 would be ~1.785.
    assert!((img.copper.unsigned_area() - 2.785).abs() < 0.05, "area {}", img.copper.unsigned_area());
    let (_, _, maxx, _) = img.bounds().unwrap();
    assert!((maxx - 2.5).abs() < 0.05, "the modal stroke must reach x=2 (maxx {maxx})");
  }
}
