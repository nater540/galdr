//! Board cutout / profile — a routing path around the board outline with holding tabs (bridges) left uncut.
//!
//! Provenance: FlatCAM's cutout tool (see `docs/eitri-porting-plan.md` §7.4). The outline is offset **outward** by
//! the tool radius (plus an optional margin) so the bit cuts *outside* the board, then the closed offset ring is
//! split at tab locations into open arcs — the cut segments — leaving a gap of `tab_width` at each tab so the board
//! stays anchored in the stock. The §7.4 porting risk is exactly the split: cleanly cutting the closed ring at tab
//! locations while preserving vertex order and traversal direction. That split is [`split_ring_at_tabs`], unit
//! tested in isolation. The resulting open arcs reach G-code through the shared Phase-4 emitter.

use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use eitri_core::{CancelToken, Error, ProgressEvent, ProgressReporter, Result};
use eitri_geo::{GeoBackend, JoinType};

use crate::isolation::{MillingDirection, RingPath};
use crate::optimize::Point;

/// The board outline to cut around.
#[derive(Debug, Clone, PartialEq)]
pub enum CutoutOutline {
  /// A simple axis-aligned rectangle from `min` to `max`.
  Rectangle {
    /// Lower-left corner (millimetres).
    min: Point,
    /// Upper-right corner (millimetres).
    max: Point,
  },
  /// A geometry-derived outline; its merged silhouette exteriors are each cut.
  Geometry(MultiPolygon<f64>),
}

/// Where holding tabs are placed around each cut ring.
#[derive(Debug, Clone, PartialEq)]
pub enum TabPlacement {
  /// `n` evenly-spaced tabs, the first centred at the ring start.
  Count(usize),
  /// Tabs centred at these fractions of the ring perimeter (each in `[0, 1)`), measured from the ring start.
  AtFractions(Vec<f64>),
}

impl TabPlacement {
  /// The tab centre arc-lengths (millimetres from the ring start) for a ring of the given `perimeter`.
  fn centers(&self, perimeter: f64) -> Result<Vec<f64>> {
    match self {
      TabPlacement::Count(n) => Ok((0..*n).map(|k| (k as f64) * perimeter / (*n as f64)).collect()),
      TabPlacement::AtFractions(fracs) => {
        if fracs.iter().any(|f| f.is_nan() || !(0.0..1.0).contains(f)) {
          return Err(Error::InvalidGeometry("cutout tab fractions must lie in [0, 1)".to_string()));
        }
        Ok(fracs.iter().map(|f| f * perimeter).collect())
      }
    }
  }
}

/// Parameters for a cutout operation. Distances are millimetres.
#[derive(Debug, Clone, PartialEq)]
pub struct CutoutParams {
  /// Routing tool diameter (millimetres); must be positive.
  pub tool_diameter: f64,
  /// Width of the uncut gap left at each tab (millimetres, `>= 0`).
  pub tab_width: f64,
  /// Tab placement around each cut ring.
  pub tabs: TabPlacement,
  /// Extra outward offset beyond the tool radius (millimetres, `>= 0`), e.g. to clear the board edge.
  pub margin: f64,
  /// Milling direction, which sets the cut-ring winding.
  pub direction: MillingDirection,
  /// Corner join style for the outward offset.
  pub join: JoinType,
  /// Miter limit ratio (only meaningful for [`JoinType::Miter`]).
  pub miter_limit: f64,
}

impl Default for CutoutParams {
  fn default() -> CutoutParams {
    CutoutParams {
      tool_diameter: 1.0,
      tab_width: 2.0,
      tabs: TabPlacement::Count(4),
      margin: 0.0,
      direction: MillingDirection::Conventional,
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }
}

impl CutoutParams {
  /// The tool radius (millimetres).
  fn radius(&self) -> f64 {
    self.tool_diameter / 2.0
  }

  /// Validate the parameter domain.
  fn validate(&self) -> Result<()> {
    if self.tool_diameter.is_nan() || self.tool_diameter <= 0.0 {
      return Err(Error::InvalidGeometry("cutout tool diameter must be positive".to_string()));
    }
    if self.tab_width.is_nan() || self.tab_width < 0.0 {
      return Err(Error::InvalidGeometry("cutout tab width must be non-negative".to_string()));
    }
    if self.margin.is_nan() || self.margin < 0.0 {
      return Err(Error::InvalidGeometry("cutout margin must be non-negative".to_string()));
    }
    Ok(())
  }
}

/// The result of a cutout operation: the open profile arcs between tabs, and how many tabs were placed.
#[derive(Debug, Clone, PartialEq)]
pub struct CutoutResult {
  /// The open cut arcs, in ring-then-arc order.
  pub paths: Vec<RingPath>,
  /// Total number of holding tabs placed across all cut rings.
  pub tab_count: usize,
  /// The milling direction the arcs were generated for.
  pub direction: MillingDirection,
}

impl CutoutResult {
  /// Number of cut arcs produced.
  pub fn len(&self) -> usize {
    self.paths.len()
  }

  /// Whether no cut arcs were produced.
  pub fn is_empty(&self) -> bool {
    self.paths.is_empty()
  }

  /// Wrap the cut arcs as toolpaths for the shared Phase-4 emitter.
  pub fn toolpaths(&self) -> crate::IsolationToolpaths<RingPath> {
    crate::IsolationToolpaths::from_paths(self.paths.iter().cloned(), self.direction.winding())
  }
}

/// Generate a board-profile cutout from `outline`: offset the outline outward by `radius + margin`, then split each
/// closed cut ring at its tabs into open arcs. Progress advances per source outline; cancellation is polled per
/// outline.
pub fn cutout<B>(
  outline: &CutoutOutline,
  params: &CutoutParams,
  backend: &B,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<CutoutResult>
where
  B: GeoBackend,
{
  params.validate()?;
  progress.emit(ProgressEvent::Started { label: "cutout".to_string() });
  cancel.check()?;

  // The source silhouettes to cut around: an explicit rectangle, or the merged exteriors of a geometry outline.
  let sources: Vec<Polygon<f64>> = match outline {
    CutoutOutline::Rectangle { min, max } => vec![rectangle(min.x, min.y, max.x, max.y)],
    CutoutOutline::Geometry(mp) => backend.union_all(&mp.0)?.0,
  };

  let dist = params.radius() + params.margin;
  let winding = params.direction.winding();
  let total = sources.len() as u64;
  let mut paths = Vec::new();
  let mut tab_count = 0;

  for (i, src) in sources.iter().enumerate() {
    cancel.check()?;
    // Cut around the outer silhouette only — interior holes are not part of the board profile.
    let exterior_only = Polygon::new(src.exterior().clone(), Vec::new());
    let offset = backend.offset(&exterior_only, dist, params.join, params.miter_limit)?;
    for poly in &offset.0 {
      let ring = ring_points(&backend.normalize_winding(poly, winding).exterior().clone());
      let perimeter = ring_perimeter(&ring);
      let centers = params.tabs.centers(perimeter)?;
      tab_count += centers.len();
      paths.extend(split_ring_at_tabs(&ring, &centers, params.tab_width));
    }
    progress.advance((i + 1) as u64, total);
  }

  progress.emit(ProgressEvent::Finished);
  Ok(CutoutResult { paths, tab_count, direction: params.direction })
}

/// Split a closed ring (`points`, first == last) into open arcs by removing an interval of `tab_width` centred at
/// each arc-length in `centers`. Vertex order and traversal direction are preserved: each arc is a contiguous
/// forward sub-path of the ring. With no centres (or a zero tab width) the whole ring is returned as one closed
/// path; if the tabs cover the whole perimeter nothing is cut.
pub(crate) fn split_ring_at_tabs(points: &[Point], centers: &[f64], tab_width: f64) -> Vec<RingPath> {
  let perimeter = ring_perimeter(points);
  if points.len() < 4 || perimeter <= 0.0 {
    return Vec::new();
  }
  let half = tab_width / 2.0;
  // No tabs (or zero-width tabs) => cut the whole closed ring in one pass.
  if centers.is_empty() || half <= 0.0 {
    return vec![RingPath::closed(points.to_vec())];
  }
  // Tabs consume the whole perimeter => there is nothing left to cut.
  if tab_width * (centers.len() as f64) >= perimeter {
    return Vec::new();
  }

  let cum = cumulative_arclengths(points);
  // Build augmented vertices: every ring vertex (excluding the closing duplicate) plus a break point at each tab
  // boundary, all keyed by arc-length in `[0, perimeter)`.
  let mut aug: Vec<(f64, Point)> = Vec::with_capacity(points.len() + centers.len() * 2);
  for i in 0..points.len() - 1 {
    aug.push((cum[i], points[i]));
  }
  for &c in centers {
    for edge in [wrap(c - half, perimeter), wrap(c + half, perimeter)] {
      aug.push((edge, point_at_arclength(points, &cum, edge)));
    }
  }
  aug.sort_by(|a, b| a.0.total_cmp(&b.0));
  aug.dedup_by(|a, b| (a.0 - b.0).abs() <= 1.0e-9);
  let n = aug.len();

  // An edge (between consecutive augmented vertices) is a tab gap when its arc-length midpoint is within a tab.
  let in_tab = |s: f64| centers.iter().any(|&c| circular_distance(s, c, perimeter) < half - 1.0e-9);
  let edge_is_gap = |i: usize| {
    let a = aug[i].0;
    let b = aug[(i + 1) % n].0;
    let len = wrap(b - a, perimeter);
    in_tab(wrap(a + len / 2.0, perimeter))
  };

  // Start the circular walk just after a gap so a kept run never straddles the seam. If no gap exists the tabs
  // failed to cut anything (all within rounding) — return the closed ring.
  let Some(start) = (0..n).find(|&i| edge_is_gap(i)) else {
    return vec![RingPath::closed(points.to_vec())];
  };

  let mut paths = Vec::new();
  let mut current: Vec<Point> = Vec::new();
  for step in 0..n {
    let i = (start + step) % n;
    if edge_is_gap(i) {
      if current.len() >= 2 {
        paths.push(RingPath::open(std::mem::take(&mut current)));
      }
      current.clear();
    } else {
      if current.is_empty() {
        current.push(aug[i].1);
      }
      current.push(aug[(i + 1) % n].1);
    }
  }
  if current.len() >= 2 {
    paths.push(RingPath::open(current));
  }
  paths
}

/// The cumulative arc-length at each ring vertex; `cum[i]` is the distance from the start to vertex `i`, and the
/// last entry is the full perimeter.
fn cumulative_arclengths(points: &[Point]) -> Vec<f64> {
  let mut cum = Vec::with_capacity(points.len());
  let mut acc = 0.0;
  cum.push(0.0);
  for w in points.windows(2) {
    acc += w[0].distance_to(w[1]);
    cum.push(acc);
  }
  cum
}

/// The perimeter (total edge length) of a closed ring.
fn ring_perimeter(points: &[Point]) -> f64 {
  points.windows(2).map(|w| w[0].distance_to(w[1])).sum()
}

/// The point at arc-length `target` (clamped to `[0, perimeter]`) along the ring.
fn point_at_arclength(points: &[Point], cum: &[f64], target: f64) -> Point {
  let perimeter = *cum.last().unwrap_or(&0.0);
  let t = target.clamp(0.0, perimeter);
  // Find the segment [i, i+1] whose cumulative span contains `t`.
  let mut i = 0;
  while i + 1 < cum.len() && cum[i + 1] < t {
    i += 1;
  }
  if i + 1 >= points.len() {
    return *points.last().unwrap_or(&Point::default());
  }
  let seg = cum[i + 1] - cum[i];
  if seg <= 0.0 {
    return points[i];
  }
  let f = (t - cum[i]) / seg;
  Point::new(points[i].x + (points[i + 1].x - points[i].x) * f, points[i].y + (points[i + 1].y - points[i].y) * f)
}

/// The circular (shorter-way-around) distance between two arc-lengths on a ring of the given `perimeter`.
fn circular_distance(a: f64, b: f64, perimeter: f64) -> f64 {
  let d = (a - b).rem_euclid(perimeter);
  d.min(perimeter - d)
}

/// Wrap an arc-length into `[0, perimeter)`.
fn wrap(s: f64, perimeter: f64) -> f64 {
  s.rem_euclid(perimeter)
}

/// The vertices of a closed `geo` ring as [`Point`]s.
fn ring_points(ring: &LineString<f64>) -> Vec<Point> {
  ring.0.iter().map(|c| Point::new(c.x, c.y)).collect()
}

/// An axis-aligned rectangle polygon with a CCW exterior ring.
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
  Polygon::new(
    LineString(vec![
      Coord { x: x0, y: y0 },
      Coord { x: x1, y: y0 },
      Coord { x: x1, y: y1 },
      Coord { x: x0, y: y1 },
      Coord { x: x0, y: y0 },
    ]),
    vec![],
  )
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::{DefaultBackend, bounds};

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  fn backend() -> DefaultBackend {
    DefaultBackend::new()
  }

  /// A CCW unit-less square ring [(0,0),(10,0),(10,10),(0,10),(0,0)] with perimeter 40.
  fn square_ring() -> Vec<Point> {
    vec![
      Point::new(0.0, 0.0),
      Point::new(10.0, 0.0),
      Point::new(10.0, 10.0),
      Point::new(0.0, 10.0),
      Point::new(0.0, 0.0),
    ]
  }

  fn total_length(paths: &[RingPath]) -> f64 {
    paths.iter().map(|p| ring_perimeter(&p.points)).sum()
  }

  #[test]
  fn split_leaves_one_arc_per_tab_with_the_right_total_length() {
    // Four mid-edge tabs (fractions 1/8, 3/8, 5/8, 7/8 -> arc-lengths 5, 15, 25, 35), each 2mm wide.
    let ring = square_ring();
    let centers = TabPlacement::AtFractions(vec![0.125, 0.375, 0.625, 0.875]).centers(40.0).expect("centers");
    let arcs = split_ring_at_tabs(&ring, &centers, 2.0);
    assert_eq!(arcs.len(), 4, "four tabs split a ring into four arcs");
    assert!(arcs.iter().all(|a| !a.is_closed()), "cut arcs are open");
    // Cut length is the perimeter minus the four 2mm gaps: 40 - 8 = 32.
    assert!((total_length(&arcs) - 32.0).abs() < 1e-6, "cut length {}", total_length(&arcs));
  }

  #[test]
  fn split_preserves_forward_vertex_order_and_direction() {
    // The mid-edge tabs land on straight edges, so each gap is a straight 2mm segment: the Euclidean distance from
    // one arc's end to the next arc's start equals the tab width, and the arcs advance in ring (CCW) order.
    let ring = square_ring();
    let centers = TabPlacement::AtFractions(vec![0.125, 0.375, 0.625, 0.875]).centers(40.0).expect("centers");
    let arcs = split_ring_at_tabs(&ring, &centers, 2.0);
    for pair in arcs.windows(2) {
      let gap = pair[0].end().distance_to(pair[1].start());
      assert!((gap - 2.0).abs() < 1e-6, "straight mid-edge gap should equal the tab width, got {gap}");
    }
    // The first arc starts on the bottom edge after the first tab (x around 6, y == 0) and moves toward +x.
    let first = &arcs[0];
    assert!((first.start().y).abs() < 1e-9, "first arc rides the bottom edge");
    assert!(first.points[1].x > first.start().x, "arc advances in the ring's forward (CCW) direction");
  }

  #[test]
  fn reversing_the_ring_reverses_the_cut_direction() {
    let mut cw = square_ring();
    cw.reverse();
    let centers = TabPlacement::AtFractions(vec![0.125, 0.375, 0.625, 0.875]).centers(40.0).expect("centers");
    let arcs = split_ring_at_tabs(&cw, &centers, 2.0);
    assert_eq!(arcs.len(), 4);
    // On the reversed (CW) ring the bottom-edge arc advances toward -x, the opposite of the CCW case.
    let bottom = arcs.iter().find(|a| a.start().y.abs() < 1e-9).expect("a bottom-edge arc");
    assert!(bottom.points[1].x < bottom.start().x, "reversed ring cuts the bottom edge toward -x");
  }

  #[test]
  fn no_tabs_yields_a_single_closed_ring() {
    let arcs = split_ring_at_tabs(&square_ring(), &[], 2.0);
    assert_eq!(arcs.len(), 1);
    assert!(arcs[0].is_closed(), "with no tabs the whole ring is cut as one closed loop");
  }

  #[test]
  fn tabs_covering_the_perimeter_cut_nothing() {
    // Two 25mm tabs on a 40mm ring cover more than the whole perimeter.
    let centers = TabPlacement::Count(2).centers(40.0).expect("centers");
    let arcs = split_ring_at_tabs(&square_ring(), &centers, 25.0);
    assert!(arcs.is_empty(), "over-wide tabs leave nothing to cut");
  }

  #[test]
  fn a_tab_at_the_seam_splits_cleanly() {
    // A tab centred at arc-length 0 (the ring start / a corner) must still split without a wrapped or dropped arc.
    let centers = TabPlacement::AtFractions(vec![0.0, 0.5]).centers(40.0).expect("centers");
    let arcs = split_ring_at_tabs(&square_ring(), &centers, 2.0);
    assert_eq!(arcs.len(), 2, "a seam-straddling tab still yields two arcs");
    assert!((total_length(&arcs) - 36.0).abs() < 1e-6, "40 - 2*2 = 36");
  }

  #[test]
  fn rectangle_cutout_offsets_outward_and_places_the_tab_count() {
    let outline = CutoutOutline::Rectangle { min: Point::new(0.0, 0.0), max: Point::new(20.0, 20.0) };
    let params = CutoutParams { tool_diameter: 2.0, tab_width: 3.0, tabs: TabPlacement::Count(4), ..Default::default() };
    let (p, c) = silent();
    let result = cutout(&outline, &params, &backend(), &p, &c).expect("cutout");
    assert_eq!(result.tab_count, 4, "four tabs requested and placed");
    assert_eq!(result.len(), 4, "four tabs split the ring into four arcs");
    // The cut ring is offset outward by the tool radius, so it extends below/left of the board's (0,0) corner.
    let pts: Vec<_> = result.paths.iter().flat_map(|a| a.points.iter().copied()).collect();
    assert!(pts.iter().any(|pt| pt.x < 0.0 && pt.y < 5.0), "cut ring lies outside the board outline");
  }

  #[test]
  fn geometry_outline_cutout_produces_arcs() {
    let board = MultiPolygon::new(vec![rectangle(0.0, 0.0, 30.0, 15.0)]);
    let outline = CutoutOutline::Geometry(board);
    let params = CutoutParams { tool_diameter: 1.6, tab_width: 2.0, tabs: TabPlacement::Count(6), ..Default::default() };
    let (p, c) = silent();
    let result = cutout(&outline, &params, &backend(), &p, &c).expect("cutout");
    assert_eq!(result.tab_count, 6);
    assert!(!result.is_empty(), "a geometry outline produces cut arcs");
    let (x0, y0, x1, y1) = bounds(&MultiPolygon::new(vec![rectangle(0.0, 0.0, 30.0, 15.0)])).expect("bounds");
    // Every arc point sits within a tool-radius-and-a-bit outside the board bounds (a loose sanity envelope).
    assert!(result.paths.iter().flat_map(|a| &a.points).all(|pt| {
      pt.x >= x0 - 3.0 && pt.x <= x1 + 3.0 && pt.y >= y0 - 3.0 && pt.y <= y1 + 3.0
    }));
  }

  #[test]
  fn invalid_params_and_fractions_are_rejected() {
    let outline = CutoutOutline::Rectangle { min: Point::new(0.0, 0.0), max: Point::new(10.0, 10.0) };
    let (p, c) = silent();
    let bad_tool = CutoutParams { tool_diameter: 0.0, ..Default::default() };
    assert!(cutout(&outline, &bad_tool, &backend(), &p, &c).is_err());
    let bad_frac = CutoutParams { tabs: TabPlacement::AtFractions(vec![1.5]), ..Default::default() };
    assert!(cutout(&outline, &bad_frac, &backend(), &p, &c).is_err());
  }

  #[test]
  fn cancelled_cutout_returns_error() {
    let outline = CutoutOutline::Rectangle { min: Point::new(0.0, 0.0), max: Point::new(10.0, 10.0) };
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(cutout(&outline, &CutoutParams::default(), &backend(), &ProgressReporter::silent(), &cancel).is_err());
  }
}
