//! The Excellon parser: infer or accept the number format, read tool definitions and drill/slot hits, and expose
//! them as a resolved [`ExcellonImage`]. Provenance: FlatCAM's `Excellon` class in `camlib`.
//!
//! Deferred constructs (rare; flagged rather than silently mishandled): repeat/pattern commands (`R`) and routed
//! slots built from `G00`/`M15`/`G01`/`M16` motion — only `G85` canned slots are assembled here. See §5.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use eitri_core::{CancelToken, ProgressEvent, ProgressReporter, Unit};
use eitri_geo::{CapStyle, buffer_path};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use crate::error::{ExcellonError, Result};
use crate::format::{NumberFormat, ZeroSuppression};

/// Poll cancellation every this many source lines.
const CANCEL_CHECK_INTERVAL: usize = 512;

/// A drill tool: a number mapped to a diameter (millimetres).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tool {
  /// Tool diameter in millimetres.
  pub diameter: f64,
}

/// A single drill or slot hit. Coordinates are in millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrillHit {
  /// A point drill with the given tool.
  Drill { tool: u32, x: f64, y: f64 },
  /// A routed/canned slot: a segment from `start` to `end` cut with the given tool.
  Slot { tool: u32, start: (f64, f64), end: (f64, f64) },
}

impl DrillHit {
  /// The tool number this hit uses.
  pub fn tool(&self) -> u32 {
    match self {
      DrillHit::Drill { tool, .. } | DrillHit::Slot { tool, .. } => *tool,
    }
  }
}

/// The resolved contents of an Excellon file: the tool table, the ordered hits, and the number format used.
#[derive(Debug, Clone)]
pub struct ExcellonImage {
  /// Tool table, keyed by tool number.
  pub tools: BTreeMap<u32, Tool>,
  /// Drill and slot hits, in file order.
  pub hits: Vec<DrillHit>,
  /// The number format that decoded the coordinates.
  pub format: NumberFormat,
}

impl ExcellonImage {
  /// Build the geometry for a hit: a filled circle for a drill, a stroked capsule for a slot (a slot is a segment,
  /// not a point — buffered by the tool radius, per §5). Returns `None` if the hit's tool is unknown.
  pub fn hit_geometry(&self, hit: &DrillHit) -> Result<Option<MultiPolygon<f64>>> {
    let Some(tool) = self.tools.get(&hit.tool()) else { return Ok(None) };
    let radius = tool.diameter / 2.0;
    let geom = match hit {
      DrillHit::Drill { x, y, .. } => MultiPolygon::new(vec![circle_polygon(*x, *y, radius)]),
      DrillHit::Slot { start, end, .. } => {
        let path = LineString(vec![Coord { x: start.0, y: start.1 }, Coord { x: end.0, y: end.1 }]);
        buffer_path(&path, radius, CapStyle::Round)?
      }
    };
    Ok(Some(geom))
  }
}

/// Parse an Excellon drill file. If `override_format` is `Some`, it is used verbatim; otherwise the format is
/// inferred from the header (units, zero suppression, digit counts). Progress and cancellation are threaded per §12.
pub fn parse_excellon(
  source: &str,
  override_format: Option<NumberFormat>,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<ExcellonImage> {
  progress.emit(ProgressEvent::Started { label: "parse excellon".to_string() });

  let format = match override_format {
    Some(f) => f,
    None => infer_format(source),
  };

  let lines: Vec<&str> = source.lines().collect();
  let total = lines.len() as u64;

  let mut tools: BTreeMap<u32, Tool> = BTreeMap::new();
  let mut hits: Vec<DrillHit> = Vec::new();
  let mut current_tool: Option<u32> = None;
  let (mut x, mut y) = (0.0f64, 0.0f64);

  for (index, raw) in lines.iter().enumerate() {
    if index % CANCEL_CHECK_INTERVAL == 0 {
      if cancel.is_cancelled() {
        return Err(ExcellonError::Cancelled);
      }
      progress.advance(index as u64, total);
    }
    let line_no = index + 1;
    let line = raw.trim();
    if line.is_empty() || line.starts_with(';') || line == "%" {
      continue;
    }

    // Tool definition: T<n>C<dia> (may appear in header or body). Selection: bare T<n>.
    if let Some(rest) = line.strip_prefix('T') {
      let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
      if let Ok(number) = digits.parse::<u32>() {
        let tail = &rest[digits.len()..];
        if let Some(cpos) = tail.find('C') {
          // Definition: read the diameter after 'C', up to the next letter.
          let dia_str: String = tail[cpos + 1..].chars().take_while(|c| c.is_ascii_digit() || *c == '.' || *c == '-').collect();
          let diameter: f64 = dia_str.parse().map_err(|_| ExcellonError::Syntax {
            line: line_no,
            message: format!("invalid tool diameter in '{line}'"),
          })?;
          tools.insert(number, Tool { diameter: diameter * format.unit.mm_per_unit() });
        } else {
          current_tool = Some(number);
        }
        continue;
      }
    }

    // Motion/mode words we do not need to model are skipped, but a coordinate hit (with X or Y) is processed.
    if line.contains('X') || line.contains('Y') {
      process_hit(line, line_no, &format, &tools, current_tool, &mut x, &mut y, &mut hits)?;
    }
    // Anything else (G-codes, M-codes, mode words) is ignored for Phase 2.
  }

  progress.emit(ProgressEvent::Finished);
  Ok(ExcellonImage { tools, hits, format })
}

/// Process a coordinate line into a drill or a `G85` slot hit.
#[allow(clippy::too_many_arguments)]
fn process_hit(
  line: &str,
  line_no: usize,
  format: &NumberFormat,
  tools: &BTreeMap<u32, Tool>,
  current_tool: Option<u32>,
  x: &mut f64,
  y: &mut f64,
  hits: &mut Vec<DrillHit>,
) -> Result<()> {
  let tool = current_tool.ok_or(ExcellonError::Syntax { line: line_no, message: "coordinate before any tool selection".to_string() })?;
  if !tools.contains_key(&tool) {
    return Err(ExcellonError::UndefinedTool(tool));
  }

  // A G85 slot splits the line into a start point (before G85) and an end point (after it).
  if let Some((head, tail)) = line.split_once("G85") {
    let (sx, sy) = read_xy(head, format, line_no, *x, *y)?;
    let (ex, ey) = read_xy(tail, format, line_no, sx, sy)?;
    *x = ex;
    *y = ey;
    hits.push(DrillHit::Slot { tool, start: (sx, sy), end: (ex, ey) });
    return Ok(());
  }

  let (nx, ny) = read_xy(line, format, line_no, *x, *y)?;
  *x = nx;
  *y = ny;
  hits.push(DrillHit::Drill { tool, x: nx, y: ny });
  Ok(())
}

/// Read `X`/`Y` coordinate words from a fragment, decoding to millimetres and falling back to the modal values.
fn read_xy(fragment: &str, format: &NumberFormat, line_no: usize, default_x: f64, default_y: f64) -> Result<(f64, f64)> {
  let mut x = default_x;
  let mut y = default_y;
  let mut chars = fragment.chars().peekable();
  while let Some(&c) = chars.peek() {
    if c == 'X' || c == 'Y' {
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
      let decoded = format.to_mm(format.decode(&value, line_no)?);
      if c == 'X' {
        x = decoded;
      } else {
        y = decoded;
      }
    } else {
      chars.next();
    }
  }
  Ok((x, y))
}

/// Infer the number format from the header directives. Falls back to millimetres with the conventional default when
/// the file declares nothing.
fn infer_format(source: &str) -> NumberFormat {
  let mut unit: Option<Unit> = None;
  let mut suppression: Option<ZeroSuppression> = None;
  let mut digits: Option<(u8, u8)> = None;

  for raw in source.lines() {
    let line = raw.trim();
    let upper = line.to_ascii_uppercase();
    if upper.starts_with("METRIC") {
      unit = Some(Unit::Millimeters);
      scan_directives(line, &mut suppression, &mut digits);
    } else if upper.starts_with("INCH") {
      unit = Some(Unit::Inches);
      scan_directives(line, &mut suppression, &mut digits);
    } else if let Some(rest) = upper.strip_prefix(";FILE_FORMAT=") {
      digits = parse_digit_spec(rest).or(digits);
    } else if upper == "LZ" || upper == "TZ" {
      suppression = ZeroSuppression::from_keyword(&upper).or(suppression);
    }
  }

  let unit = unit.unwrap_or(Unit::Millimeters);
  let base = NumberFormat::default_for(unit);
  let (integer_digits, decimal_digits) = digits.unwrap_or((base.integer_digits, base.decimal_digits));
  NumberFormat {
    unit,
    integer_digits,
    decimal_digits,
    zero_suppression: suppression.unwrap_or(base.zero_suppression),
  }
}

/// Scan the comma-separated tail of a `METRIC`/`INCH` line for a zero-suppression keyword and a digit spec.
fn scan_directives(line: &str, suppression: &mut Option<ZeroSuppression>, digits: &mut Option<(u8, u8)>) {
  for part in line.split(',').skip(1) {
    let part = part.trim();
    if let Some(zs) = ZeroSuppression::from_keyword(&part.to_ascii_uppercase()) {
      *suppression = Some(zs);
    } else if let Some(d) = parse_digit_spec(part) {
      *digits = Some(d);
    }
  }
}

/// Parse a digit specification: `3:3`, `3.3`, or a `000.000` zero-run pattern → `(integer, decimal)`.
fn parse_digit_spec(spec: &str) -> Option<(u8, u8)> {
  let spec = spec.trim();
  let (left, right) = spec.split_once([':', '.'])?;
  // A zero-run pattern like `000.000` counts characters; a numeric pair like `3.3` parses the numbers.
  if left.chars().all(|c| c == '0') && right.chars().all(|c| c == '0') && !left.is_empty() && !right.is_empty() {
    return Some((left.len() as u8, right.len() as u8));
  }
  let int: u8 = left.trim().parse().ok()?;
  let dec: u8 = right.trim().parse().ok()?;
  Some((int, dec))
}

/// A filled circle approximated by a polygon — used to render drill points as geometry.
fn circle_polygon(cx: f64, cy: f64, r: f64) -> Polygon<f64> {
  let n = 48usize;
  let ring: Vec<Coord<f64>> = (0..n)
    .map(|i| {
      let a = TAU * (i as f64) / (n as f64);
      Coord { x: cx + r * a.cos(), y: cy + r * a.sin() }
    })
    .collect();
  Polygon::new(LineString(ring), Vec::new())
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_core::{CancelToken, ProgressReporter};
  use geo::algorithm::area::Area;

  fn parse(src: &str, over: Option<NumberFormat>) -> ExcellonImage {
    parse_excellon(src, over, &ProgressReporter::silent(), &CancelToken::new()).expect("parse")
  }

  #[test]
  fn metric_declared_leading_suppression() {
    // METRIC,TZ (TZ => leading suppressed), FILE_FORMAT 3:3. Tool 1 = 0.8mm; one hit at (10, 5).
    let src = "M48\nMETRIC,TZ,3.3\nT1C0.8\n%\nT1\nX10000Y5000\nM30\n";
    let img = parse(src, None);
    assert_eq!(img.format.unit, Unit::Millimeters);
    assert_eq!(img.format.zero_suppression, ZeroSuppression::Leading);
    assert!((img.tools[&1].diameter - 0.8).abs() < 1e-9);
    assert_eq!(img.hits.len(), 1);
    match img.hits[0] {
      DrillHit::Drill { tool, x, y } => {
        assert_eq!(tool, 1);
        assert!((x - 10.0).abs() < 1e-6 && (y - 5.0).abs() < 1e-6);
      }
      other => panic!("expected drill, got {other:?}"),
    }
  }

  #[test]
  fn inch_declared_trailing_suppression() {
    // INCH,LZ (LZ => trailing suppressed), 2.4. Tool 1 = 0.04in. Hit "X01Y005" -> (1.0in, 0.5in) -> mm.
    let src = "M48\nINCH,LZ,2.4\nT1C0.04\n%\nT1\nX01Y005\nM30\n";
    let img = parse(src, None);
    assert_eq!(img.format.unit, Unit::Inches);
    assert_eq!(img.format.zero_suppression, ZeroSuppression::Trailing);
    match img.hits[0] {
      DrillHit::Drill { x, y, .. } => {
        assert!((x - 25.4).abs() < 1e-4, "x {x}");
        assert!((y - 12.7).abs() < 1e-4, "y {y}");
      }
      other => panic!("expected drill, got {other:?}"),
    }
  }

  #[test]
  fn undeclared_format_defaults_to_metric_with_explicit_decimals() {
    // No METRIC/INCH; explicit-decimal coordinates parse regardless of the inferred digit counts.
    let src = "M48\nT1C1.0\n%\nT1\nX2.5Y3.0\nM30\n";
    let img = parse(src, None);
    assert_eq!(img.format.unit, Unit::Millimeters);
    match img.hits[0] {
      DrillHit::Drill { x, y, .. } => assert!((x - 2.5).abs() < 1e-9 && (y - 3.0).abs() < 1e-9),
      other => panic!("expected drill, got {other:?}"),
    }
  }

  #[test]
  fn override_format_wins_over_header() {
    // Header says INCH, but the override forces metric leading 3.3 — coordinates decode as metric.
    let over = NumberFormat {
      unit: Unit::Millimeters,
      integer_digits: 3,
      decimal_digits: 3,
      zero_suppression: ZeroSuppression::Leading,
    };
    let src = "M48\nINCH,LZ,2.4\nT1C0.8\n%\nT1\nX1500Y2000\nM30\n";
    let img = parse(src, Some(over));
    assert_eq!(img.format.unit, Unit::Millimeters);
    match img.hits[0] {
      DrillHit::Drill { x, y, .. } => assert!((x - 1.5).abs() < 1e-6 && (y - 2.0).abs() < 1e-6),
      other => panic!("expected drill, got {other:?}"),
    }
    // Tool diameter converted using the override's unit (mm), so 0.8 stays 0.8.
    assert!((img.tools[&1].diameter - 0.8).abs() < 1e-9);
  }

  #[test]
  fn g85_slot_becomes_a_segment_with_geometry() {
    // A slot from (2,2) to (8,2) with a 1mm tool: two distinct endpoints, and a capsule of area 6*1 + pi*0.25.
    let src = "M48\nMETRIC,LZ,3.3\nT1C1.0\n%\nT1\nX002000Y002000G85X008000Y002000\nM30\n";
    let img = parse(src, None);
    assert_eq!(img.hits.len(), 1);
    match img.hits[0] {
      DrillHit::Slot { start, end, .. } => {
        assert!((start.0 - 2.0).abs() < 1e-6 && (end.0 - 8.0).abs() < 1e-6);
      }
      other => panic!("expected slot, got {other:?}"),
    }
    let geom = img.hit_geometry(&img.hits[0]).unwrap().unwrap();
    assert!((geom.unsigned_area() - (6.0 + std::f64::consts::PI * 0.25)).abs() < 0.05, "area {}", geom.unsigned_area());
  }

  #[test]
  fn drill_geometry_area_matches_tool() {
    let src = "M48\nMETRIC,LZ,3.3\nT1C2.0\n%\nT1\nX000000Y000000\nM30\n";
    let img = parse(src, None);
    let geom = img.hit_geometry(&img.hits[0]).unwrap().unwrap();
    // 2mm tool -> r1 -> area ~ pi.
    assert!((geom.unsigned_area() - std::f64::consts::PI).abs() < 0.02, "area {}", geom.unsigned_area());
  }

  #[test]
  fn cancellation_stops_the_parse() {
    let cancel = CancelToken::new();
    cancel.cancel();
    let err = parse_excellon("M48\nMETRIC\n%\n", None, &ProgressReporter::silent(), &cancel);
    assert!(matches!(err, Err(ExcellonError::Cancelled)));
  }

  #[test]
  fn hit_before_tool_is_an_error() {
    let src = "M48\nMETRIC,LZ,3.3\n%\nX1000Y1000\nM30\n";
    let err = parse_excellon(src, None, &ProgressReporter::silent(), &CancelToken::new());
    assert!(err.is_err());
  }
}
