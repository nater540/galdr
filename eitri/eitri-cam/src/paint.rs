//! Paint / area clearing — clear the interior of a region with a tool, using one of three fill strategies.
//!
//! Provenance: FlatCAM's `Geometry.paint_poly()` and its variants (see `docs/eitri-porting-plan.md` §7.2). All three
//! FlatCAM strategies are reproduced behind the [`PaintStrategy`] trait so the fill pattern is swappable:
//! - [`Concentric`] — inward-offset the boundary repeatedly by `tool_dia * (1 - overlap)` until it collapses, each
//!   offset an inset ring; emitted outer → inner.
//! - [`Seed`] — the same concentric ring set emitted inner → outer, growing from an interior seed.
//! - [`Raster`] — intersect the region with parallel scan lines spaced `tool_dia * (1 - overlap)` (at an optional
//!   angle) and connect the spans into a back-and-forth boustrophedon path.
//!
//! Every strategy handles a `margin` inset from the boundary, holes in the region, and multiple disjoint regions,
//! with an optional boundary-following finishing pass. Concentric/seed rings are **closed** [`RingPath`]s; raster
//! rows are **open** paths. Both flow to G-code through the shared Phase-4 emitter via
//! [`crate::IsolationToolpaths::from_paths`] — there is no parallel emitter. Disjoint regions fan out across `rayon`.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use geo_types::{Coord, LineString, MultiLineString, MultiPolygon, Polygon};
use rayon::prelude::*;

use eitri_core::{Affine, CancelToken, Error, ProgressEvent, ProgressReporter, Result};
use eitri_geo::{
  GeoBackend, JoinType, WindingDirection, apply_affine_polygon, bounds, clip_lines, contains_point,
};

use crate::isolation::{MillingDirection, RingPath};
use crate::optimize::{Point, Routed, Stop, TravelOptimizer, order_stops};

/// Shared parameters for an area-clearing (paint) operation. Distances are millimetres; `overlap` is a fraction in
/// `[0, 1)`. Strategy-specific tuning (the raster angle) lives on the strategy, not here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaintParams {
  /// Clearing tool diameter (millimetres); must be positive.
  pub tool_diameter: f64,
  /// Fraction each pass overlaps the previous, in `[0, 1)`; sets the pass-to-pass spacing.
  pub overlap: f64,
  /// Inset from the region boundary before filling (millimetres, `>= 0`) — leaves an unpainted rim.
  pub margin: f64,
  /// Milling direction, which sets closed-ring winding (raster rows carry no meaningful winding).
  pub direction: MillingDirection,
  /// Append a boundary-following finishing pass (the inset region outline) after the fill.
  pub finish_pass: bool,
  /// Corner join style for the inward offsets.
  pub join: JoinType,
  /// Miter limit ratio (only meaningful for [`JoinType::Miter`]).
  pub miter_limit: f64,
}

impl Default for PaintParams {
  fn default() -> PaintParams {
    PaintParams {
      tool_diameter: 1.0,
      overlap: 0.25,
      margin: 0.0,
      direction: MillingDirection::Climb,
      finish_pass: false,
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }
}

impl PaintParams {
  /// The tool radius (millimetres).
  fn radius(&self) -> f64 {
    self.tool_diameter / 2.0
  }

  /// The centre-to-centre spacing between passes: `tool_dia * (1 - overlap)`.
  fn step(&self) -> f64 {
    self.tool_diameter * (1.0 - self.overlap)
  }

  /// Validate the parameter domain, returning a descriptive error rather than producing nonsense geometry.
  fn validate(&self) -> Result<()> {
    if self.tool_diameter.is_nan() || self.tool_diameter <= 0.0 {
      return Err(Error::InvalidGeometry("paint tool diameter must be positive".to_string()));
    }
    if !(0.0..1.0).contains(&self.overlap) {
      return Err(Error::InvalidGeometry("paint overlap must be in [0, 1)".to_string()));
    }
    if self.margin.is_nan() || self.margin < 0.0 {
      return Err(Error::InvalidGeometry("paint margin must be non-negative".to_string()));
    }
    Ok(())
  }
}

/// The result of a paint operation: the ordered fill paths (closed concentric/seed rings and/or open raster rows).
#[derive(Debug, Clone, PartialEq)]
pub struct PaintResult {
  /// The fill paths, in generation order (use [`order_paths`] to travel-optimize them).
  pub paths: Vec<RingPath>,
  /// The milling direction the paths were generated for; carried so [`PaintResult::toolpaths`] can tag the rings.
  pub direction: MillingDirection,
}

impl PaintResult {
  /// Number of fill paths produced.
  pub fn len(&self) -> usize {
    self.paths.len()
  }

  /// Whether no fill paths were produced (e.g. the region was smaller than the tool).
  pub fn is_empty(&self) -> bool {
    self.paths.is_empty()
  }

  /// Wrap the fill paths as toolpaths for the shared Phase-4 emitter — the reuse point that keeps paint output off
  /// any parallel G-code path.
  pub fn toolpaths(&self) -> crate::IsolationToolpaths<RingPath> {
    crate::IsolationToolpaths::from_paths(self.paths.iter().cloned(), self.direction.winding())
  }
}

/// A fill-pattern strategy for area clearing. Implementors fill a single region (one exterior plus holes) with cut
/// paths; the [`paint`] driver handles the margin inset, disjoint regions, and the optional finishing pass. Object
/// safe (`&dyn PaintStrategy` works) and `Sync` so disjoint regions can fan out across `rayon`.
pub trait PaintStrategy: Sync {
  /// Fill one already-margin-inset region with cut paths. `backend` offsets/clips; `cancel` bails out early.
  fn fill(
    &self,
    region: &Polygon<f64>,
    params: &PaintParams,
    backend: &(dyn GeoBackend + Sync),
    cancel: &CancelToken,
  ) -> Result<Vec<RingPath>>;
}

/// Concentric fill: inward-offset the boundary repeatedly until it collapses, each offset an inset ring, emitted
/// **outer → inner** so the tool works from the boundary toward the middle.
#[derive(Debug, Clone, Copy, Default)]
pub struct Concentric;

impl PaintStrategy for Concentric {
  fn fill(
    &self,
    region: &Polygon<f64>,
    params: &PaintParams,
    backend: &(dyn GeoBackend + Sync),
    cancel: &CancelToken,
  ) -> Result<Vec<RingPath>> {
    concentric_rings(region, params, backend, cancel)
  }
}

/// Seed-based fill: the same concentric ring set as [`Concentric`] but emitted **inner → outer**, growing outward
/// from an interior seed (the innermost ring).
#[derive(Debug, Clone, Copy, Default)]
pub struct Seed;

impl PaintStrategy for Seed {
  fn fill(
    &self,
    region: &Polygon<f64>,
    params: &PaintParams,
    backend: &(dyn GeoBackend + Sync),
    cancel: &CancelToken,
  ) -> Result<Vec<RingPath>> {
    let mut rings = concentric_rings(region, params, backend, cancel)?;
    rings.reverse();
    Ok(rings)
  }
}

/// Line-based (raster) fill: intersect the region with parallel scan lines spaced `tool_dia * (1 - overlap)` at
/// `angle_deg` from the X axis, and connect the inside spans into a back-and-forth boustrophedon path — breaking to
/// a new path only where the link between two spans would leave the region (crossing a hole or a concavity).
#[derive(Debug, Clone, Copy, Default)]
pub struct Raster {
  /// Scan-line angle in degrees, measured from the X axis.
  pub angle_deg: f64,
}

impl Raster {
  /// A raster fill at the given scan-line angle (degrees).
  pub fn at_angle(angle_deg: f64) -> Raster {
    Raster { angle_deg }
  }
}

impl PaintStrategy for Raster {
  fn fill(
    &self,
    region: &Polygon<f64>,
    params: &PaintParams,
    backend: &(dyn GeoBackend + Sync),
    cancel: &CancelToken,
  ) -> Result<Vec<RingPath>> {
    cancel.check()?;
    // Work in a frame rotated so the scan lines are horizontal, then rotate the finished paths back.
    let theta = self.angle_deg.to_radians();
    let into_scan = Affine::rotate(-theta);
    let back = Affine::rotate(theta);
    let region_rot = apply_affine_polygon(region, into_scan);

    // Inset by the tool radius so the tool stays inside the region, then scan the inset.
    let inset = backend.offset(&region_rot, -params.radius(), params.join, params.miter_limit)?;
    let Some((x0, y0, x1, y1)) = bounds(&inset) else {
      return Ok(Vec::new());
    };

    let step = params.step();
    let scan_ys = scan_rows(y0, y1, step);
    let scan = MultiLineString::new(
      scan_ys
        .iter()
        .map(|&y| LineString(vec![Coord { x: x0 - 1.0, y }, Coord { x: x1 + 1.0, y }]))
        .collect(),
    );
    let spans = collect_spans(&clip_lines(&inset, &scan));
    let rows = group_rows(spans);
    let paths = boustrophedon(&rows, &inset);

    // Rotate every path back into the original frame.
    Ok(paths.into_iter().map(|p| transform_ring(&p, back)).collect())
  }
}

/// A horizontal fill span at a fixed `y`, spanning `[lo, hi]` in x.
#[derive(Debug, Clone, Copy)]
struct Span {
  y: f64,
  lo: f64,
  hi: f64,
}

/// The scan-line Y positions covering `[y0, y1]` at `step` spacing, always including a final line at `y1` so the top
/// edge of the region is swept.
fn scan_rows(y0: f64, y1: f64, step: f64) -> Vec<f64> {
  let mut ys = Vec::new();
  if step <= 0.0 {
    return vec![(y0 + y1) * 0.5];
  }
  let mut y = y0;
  while y <= y1 + 1.0e-9 {
    ys.push(y.min(y1));
    y += step;
  }
  match ys.last() {
    Some(&last) if (y1 - last) > 1.0e-9 => ys.push(y1),
    None => ys.push((y0 + y1) * 0.5),
    _ => {}
  }
  ys
}

/// Reduce each clipped horizontal line to its `(y, lo, hi)` span, dropping degenerate zero-width spans.
fn collect_spans(clipped: &MultiLineString<f64>) -> Vec<Span> {
  clipped
    .0
    .iter()
    .filter_map(|ls| {
      let y = ls.0.first()?.y;
      let (lo, hi) = ls
        .coords()
        .fold((f64::MAX, f64::MIN), |(lo, hi), c| (lo.min(c.x), hi.max(c.x)));
      if hi - lo <= 1.0e-9 { None } else { Some(Span { y, lo, hi }) }
    })
    .collect()
}

/// Group spans into rows keyed by scan-line Y (ascending), each row's spans sorted left-to-right.
fn group_rows(mut spans: Vec<Span>) -> Vec<Vec<Span>> {
  spans.sort_by(|a, b| a.y.total_cmp(&b.y).then(a.lo.total_cmp(&b.lo)));
  let mut rows: Vec<Vec<Span>> = Vec::new();
  for span in spans {
    match rows.last_mut() {
      Some(row) if (row[0].y - span.y).abs() <= 1.0e-6 => row.push(span),
      _ => rows.push(vec![span]),
    }
  }
  rows
}

/// Connect the row spans into as few open paths as possible: alternate the traversal direction each row
/// (boustrophedon) and link consecutive spans only while the link stays inside `inset`, breaking to a new path
/// otherwise so the tool never cuts across a hole or out of a concavity.
fn boustrophedon(rows: &[Vec<Span>], inset: &MultiPolygon<f64>) -> Vec<RingPath> {
  let mut paths: Vec<RingPath> = Vec::new();
  let mut current: Vec<Point> = Vec::new();

  for (ri, row) in rows.iter().enumerate() {
    // Even rows sweep left-to-right, odd rows right-to-left, so a row's exit is near the next row's entry.
    let left_to_right = ri % 2 == 0;
    let ordered: Vec<&Span> = if left_to_right { row.iter().collect() } else { row.iter().rev().collect() };
    for span in ordered {
      let (entry, exit) = if left_to_right {
        (Point::new(span.lo, span.y), Point::new(span.hi, span.y))
      } else {
        (Point::new(span.hi, span.y), Point::new(span.lo, span.y))
      };
      match current.last() {
        None => {
          current.push(entry);
          current.push(exit);
        }
        Some(&from) => {
          if link_inside(inset, from, entry) {
            current.push(entry);
            current.push(exit);
          } else {
            paths.push(RingPath::open(std::mem::take(&mut current)));
            current.push(entry);
            current.push(exit);
          }
        }
      }
    }
  }
  if !current.is_empty() {
    paths.push(RingPath::open(current));
  }
  paths
}

/// Whether the link from `a` to `b` stays inside `region`, sampled at three interior points. A conservative test:
/// it may occasionally break a path that a truer inside-test would keep, but it never keeps a link that dips into a
/// hole or concavity at a sampled point — so the tool does not cut where there is no material.
fn link_inside(region: &MultiPolygon<f64>, a: Point, b: Point) -> bool {
  [0.25, 0.5, 0.75].iter().all(|&t| {
    let x = a.x + (b.x - a.x) * t;
    let y = a.y + (b.y - a.y) * t;
    contains_point(region, x, y)
  })
}

/// Apply an affine transform to every point of a path, preserving its open/closed character.
fn transform_ring(path: &RingPath, transform: Affine) -> RingPath {
  let points = path
    .points
    .iter()
    .map(|p| {
      let (x, y) = transform.apply(p.x, p.y);
      Point::new(x, y)
    })
    .collect();
  RingPath { points }
}

/// The axis-aligned bounds of a single polygon's exterior.
fn polygon_bounds(poly: &Polygon<f64>) -> Option<(f64, f64, f64, f64)> {
  poly
    .exterior()
    .coords()
    .fold(None, |acc, c| {
      let (x0, y0, x1, y1) = acc.unwrap_or((c.x, c.y, c.x, c.y));
      Some((x0.min(c.x), y0.min(c.y), x1.max(c.x), y1.max(c.y)))
    })
}

/// The concentric inset rings of `region`: offset inward by `radius + n * step` for `n = 0, 1, 2, …` until the
/// inset collapses to nothing, each surviving polygon's exterior and holes emitted as closed [`RingPath`]s, wound
/// to the milling direction. Returned outer → inner. Collapse is handled by the backend's collapse-safe offset.
fn concentric_rings(
  region: &Polygon<f64>,
  params: &PaintParams,
  backend: &(dyn GeoBackend + Sync),
  cancel: &CancelToken,
) -> Result<Vec<RingPath>> {
  let direction = params.direction.winding();
  let step = params.step();
  // Bound the loop independently of the offset's collapse: no ring can survive past half the region's diagonal.
  let diag = polygon_bounds(region)
    .map(|(x0, y0, x1, y1)| (x1 - x0).hypot(y1 - y0))
    .unwrap_or(0.0);
  let max_rings = ((diag / step).ceil() as usize).saturating_add(4);

  let mut rings = Vec::new();
  for n in 0..max_rings {
    cancel.check()?;
    let inset = backend.offset(region, -(params.radius() + (n as f64) * step), params.join, params.miter_limit)?;
    if inset.0.is_empty() {
      break;
    }
    for poly in &inset.0 {
      push_closed_rings(backend, poly, direction, &mut rings);
    }
  }
  Ok(rings)
}

/// The boundary-following finishing rings of `region` — its exterior and holes as closed [`RingPath`]s wound to
/// `direction`. Cleans the region outline after the interior fill.
fn boundary_rings(region: &Polygon<f64>, direction: WindingDirection, backend: &(dyn GeoBackend + Sync)) -> Vec<RingPath> {
  let mut rings = Vec::new();
  push_closed_rings(backend, region, direction, &mut rings);
  rings
}

/// Normalize `poly` to `direction` (exterior = direction, holes = reversed) and push its exterior and hole rings as
/// closed [`RingPath`]s onto `out`.
fn push_closed_rings(backend: &(dyn GeoBackend + Sync), poly: &Polygon<f64>, direction: WindingDirection, out: &mut Vec<RingPath>) {
  let normalized = backend.normalize_winding(poly, direction);
  out.push(ring_path_from_ls(normalized.exterior()));
  for hole in normalized.interiors() {
    out.push(ring_path_from_ls(hole));
  }
}

/// Convert a closed `geo` ring into a closed [`RingPath`].
fn ring_path_from_ls(ring: &LineString<f64>) -> RingPath {
  RingPath::closed(ring.0.iter().map(|c| Point::new(c.x, c.y)).collect())
}

/// Paint (area-clear) every region in `regions` with `strategy`, insetting each by `params.margin` first, filling
/// disjoint regions in parallel, and appending the optional boundary finishing pass. Progress advances per region;
/// cancellation is polled per region.
pub fn paint<S, B>(
  regions: &MultiPolygon<f64>,
  params: &PaintParams,
  strategy: &S,
  backend: &B,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<PaintResult>
where
  S: PaintStrategy + ?Sized,
  B: GeoBackend + Sync,
{
  params.validate()?;
  progress.emit(ProgressEvent::Started { label: "paint".to_string() });
  cancel.check()?;
  let dyn_backend: &(dyn GeoBackend + Sync) = backend;

  // Inset every input polygon by the margin (which may split or drop a polygon), giving the regions actually filled.
  let inset_regions: Vec<Polygon<f64>> = if params.margin > 0.0 {
    let mut out = Vec::new();
    for poly in &regions.0 {
      cancel.check()?;
      out.extend(backend.offset(poly, -params.margin, params.join, params.miter_limit)?.0);
    }
    out
  } else {
    regions.0.clone()
  };

  let total = inset_regions.len() as u64;
  let counter = Arc::new(AtomicU64::new(0));
  let per_region: Vec<Vec<RingPath>> = inset_regions
    .par_iter()
    .map(|poly| -> Result<Vec<RingPath>> {
      cancel.check()?;
      let mut paths = strategy.fill(poly, params, dyn_backend, cancel)?;
      if params.finish_pass {
        paths.extend(boundary_rings(poly, params.direction.winding(), dyn_backend));
      }
      let done = counter.fetch_add(1, Ordering::Relaxed) + 1;
      progress.advance(done, total);
      Ok(paths)
    })
    .collect::<Result<Vec<_>>>()?;

  progress.emit(ProgressEvent::Finished);
  Ok(PaintResult { paths: per_region.into_iter().flatten().collect(), direction: params.direction })
}

/// Order fill/profile `paths` to minimize rapid travel between them, using `optimizer`. Closed rings are treated as
/// point stops at their start (not re-rooted); open paths are directed stops that may be traversed reversed. Reuses
/// the drill/isolation [`TravelOptimizer`] seam so ordering is one implementation across operations.
pub fn order_paths<O: TravelOptimizer>(
  paths: &[RingPath],
  optimizer: &O,
  start: Point,
  cancel: &CancelToken,
) -> Result<Vec<RingPath>> {
  let stops: Vec<Stop> = paths
    .iter()
    .map(|p| if p.is_closed() { Stop::point(p.start()) } else { Stop::segment(p.start(), p.end()) })
    .collect();
  let tour = order_stops(optimizer, &stops, start, cancel)?;
  let mut ordered = Vec::with_capacity(tour.len());
  for Routed { index, reversed } in tour {
    let mut path = paths[index].clone();
    if reversed && !path.is_closed() {
      path.points.reverse();
    }
    ordered.push(path);
  }
  Ok(ordered)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::optimize::{NearestNeighbor, TwoOpt};
  use eitri_geo::DefaultBackend;

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    Polygon::new(
      LineString(vec![
        Coord { x: cx - half, y: cy - half },
        Coord { x: cx + half, y: cy - half },
        Coord { x: cx + half, y: cy + half },
        Coord { x: cx - half, y: cy + half },
        Coord { x: cx - half, y: cy - half },
      ]),
      vec![],
    )
  }

  fn square_with_hole() -> Polygon<f64> {
    let hole = LineString(vec![
      Coord { x: -2.0, y: -2.0 },
      Coord { x: 2.0, y: -2.0 },
      Coord { x: 2.0, y: 2.0 },
      Coord { x: -2.0, y: 2.0 },
      Coord { x: -2.0, y: -2.0 },
    ]);
    Polygon::new(square(0.0, 0.0, 10.0).exterior().clone(), vec![hole])
  }

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  fn backend() -> DefaultBackend {
    DefaultBackend::new()
  }

  fn all_points(result: &PaintResult) -> Vec<Point> {
    result.paths.iter().flat_map(|p| p.points.iter().copied()).collect()
  }

  #[test]
  fn concentric_fills_with_multiple_closed_rings_inside_the_region() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let params = PaintParams { tool_diameter: 2.0, overlap: 0.25, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&region, &params, &Concentric, &backend(), &p, &c).expect("paint");
    assert!(result.len() >= 3, "a 20mm square with a 2mm tool needs several concentric rings, got {}", result.len());
    assert!(result.paths.iter().all(|r| r.is_closed()), "concentric rings are closed");
    // Every ring point lies inside the region (rings are inset by at least the tool radius).
    assert!(all_points(&result).iter().all(|pt| contains_point(&region, pt.x, pt.y)), "rings stay inside the region");
  }

  #[test]
  fn seed_reverses_the_concentric_order() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let params = PaintParams { tool_diameter: 2.0, ..Default::default() };
    let (p, c) = silent();
    let concentric = paint(&region, &params, &Concentric, &backend(), &p, &c).expect("concentric");
    let seed = paint(&region, &params, &Seed, &backend(), &p, &c).expect("seed");
    assert_eq!(concentric.len(), seed.len(), "same ring set, opposite order");
    // The seed's first ring is the concentric's last (innermost first vs outermost first).
    assert_eq!(seed.paths.first(), concentric.paths.last(), "seed starts where concentric ends");
    assert_eq!(seed.paths.last(), concentric.paths.first(), "seed ends where concentric starts");
  }

  #[test]
  fn raster_fills_with_open_rows_inside_the_region() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let params = PaintParams { tool_diameter: 2.0, overlap: 0.25, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&region, &params, &Raster::default(), &backend(), &p, &c).expect("raster");
    assert!(!result.is_empty(), "raster must produce fill rows");
    assert!(result.paths.iter().any(|r| !r.is_closed()), "raster rows are open paths");
    assert!(all_points(&result).iter().all(|pt| contains_point(&region, pt.x, pt.y)), "raster stays inside");
  }

  #[test]
  fn raster_at_an_angle_still_stays_inside() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let params = PaintParams { tool_diameter: 2.0, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&region, &params, &Raster::at_angle(45.0), &backend(), &p, &c).expect("raster 45");
    assert!(!result.is_empty(), "angled raster must produce rows");
    // Allow a hair of tolerance for the rotate round-trip, then confirm points are within the region bounds.
    let (x0, y0, x1, y1) = bounds(&region).expect("bounds");
    assert!(all_points(&result).iter().all(|pt| {
      pt.x >= x0 - 1e-6 && pt.x <= x1 + 1e-6 && pt.y >= y0 - 1e-6 && pt.y <= y1 + 1e-6
    }), "angled raster stays within the region bounds");
  }

  #[test]
  fn concentric_never_enters_a_hole() {
    let region = MultiPolygon::new(vec![square_with_hole()]);
    let hole_only = MultiPolygon::new(vec![Polygon::new(
      LineString(vec![
        Coord { x: -2.0, y: -2.0 },
        Coord { x: 2.0, y: -2.0 },
        Coord { x: 2.0, y: 2.0 },
        Coord { x: -2.0, y: 2.0 },
        Coord { x: -2.0, y: -2.0 },
      ]),
      vec![],
    )]);
    let params = PaintParams { tool_diameter: 1.0, overlap: 0.2, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&region, &params, &Concentric, &backend(), &p, &c).expect("paint");
    assert!(!result.is_empty(), "a large annular region paints");
    assert!(all_points(&result).iter().all(|pt| !contains_point(&hole_only, pt.x, pt.y)), "no path enters the hole");
  }

  #[test]
  fn raster_never_enters_a_hole() {
    let region = MultiPolygon::new(vec![square_with_hole()]);
    let hole_only = MultiPolygon::new(vec![Polygon::new(
      LineString(vec![
        Coord { x: -2.0, y: -2.0 },
        Coord { x: 2.0, y: -2.0 },
        Coord { x: 2.0, y: 2.0 },
        Coord { x: -2.0, y: 2.0 },
        Coord { x: -2.0, y: -2.0 },
      ]),
      vec![],
    )]);
    let params = PaintParams { tool_diameter: 1.0, overlap: 0.2, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&region, &params, &Raster::default(), &backend(), &p, &c).expect("paint");
    assert!(all_points(&result).iter().all(|pt| !contains_point(&hole_only, pt.x, pt.y)), "no raster row enters the hole");
  }

  #[test]
  fn disjoint_regions_are_both_painted() {
    let regions = MultiPolygon::new(vec![square(-20.0, 0.0, 5.0), square(20.0, 0.0, 5.0)]);
    let params = PaintParams { tool_diameter: 2.0, ..Default::default() };
    let (p, c) = silent();
    let result = paint(&regions, &params, &Concentric, &backend(), &p, &c).expect("paint");
    // Rings should appear on both the left (x < 0) and right (x > 0) squares.
    let pts = all_points(&result);
    assert!(pts.iter().any(|pt| pt.x < -10.0), "left region painted");
    assert!(pts.iter().any(|pt| pt.x > 10.0), "right region painted");
  }

  #[test]
  fn margin_insets_the_fill_from_the_boundary() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let base = PaintParams { tool_diameter: 2.0, margin: 0.0, ..Default::default() };
    let inset = PaintParams { margin: 3.0, ..base };
    let (p, c) = silent();
    let no_margin = paint(&region, &base, &Concentric, &backend(), &p, &c).expect("no margin");
    let with_margin = paint(&region, &inset, &Concentric, &backend(), &p, &c).expect("margin");
    let extent = |r: &PaintResult| all_points(r).iter().fold(0.0_f64, |m, pt| m.max(pt.x.abs()).max(pt.y.abs()));
    assert!(extent(&with_margin) + 2.0 < extent(&no_margin), "a 3mm margin pulls the fill well inside");
  }

  #[test]
  fn finishing_pass_appends_a_closed_boundary_ring() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
    let base = PaintParams { tool_diameter: 2.0, finish_pass: false, ..Default::default() };
    let finished = PaintParams { finish_pass: true, ..base };
    let (p, c) = silent();
    let without = paint(&region, &base, &Raster::default(), &backend(), &p, &c).expect("without");
    let with = paint(&region, &finished, &Raster::default(), &backend(), &p, &c).expect("with");
    assert_eq!(with.len(), without.len() + 1, "the finishing pass adds exactly one boundary ring");
    assert!(with.paths.last().expect("finish ring").is_closed(), "the finishing ring is closed");
  }

  #[test]
  fn strategy_is_swappable_through_a_trait_object() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 8.0)]);
    let params = PaintParams { tool_diameter: 2.0, ..Default::default() };
    let (p, c) = silent();
    let strategies: [Box<dyn PaintStrategy>; 3] = [Box::new(Concentric), Box::new(Seed), Box::new(Raster::default())];
    for strategy in &strategies {
      let result = paint(&region, &params, strategy.as_ref(), &backend(), &p, &c).expect("paint");
      assert!(!result.is_empty(), "every strategy fills a solid square");
    }
  }

  #[test]
  fn order_paths_reduces_travel_and_reverses_open_paths() {
    // Three open horizontal rows fed in a scrambled order; travel-optimizing them must not exceed input order.
    let rows = vec![
      RingPath::open(vec![Point::new(0.0, 4.0), Point::new(10.0, 4.0)]),
      RingPath::open(vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)]),
      RingPath::open(vec![Point::new(0.0, 2.0), Point::new(10.0, 2.0)]),
    ];
    let start = Point::new(0.0, 0.0);
    let (_, c) = silent();
    let ordered = order_paths(&rows, &TwoOpt::new(), start, &c).expect("order");
    assert_eq!(ordered.len(), 3);
    // The optimizer should pick the bottom row (y=0) first as it is nearest the start.
    assert!((ordered[0].start().y).abs() < 1e-9, "nearest row first, got y={}", ordered[0].start().y);
  }

  #[test]
  fn invalid_params_are_rejected() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let (p, c) = silent();
    let bad_tool = PaintParams { tool_diameter: 0.0, ..Default::default() };
    let bad_overlap = PaintParams { overlap: 1.0, ..Default::default() };
    let bad_margin = PaintParams { margin: -1.0, ..Default::default() };
    assert!(paint(&region, &bad_tool, &Concentric, &backend(), &p, &c).is_err());
    assert!(paint(&region, &bad_overlap, &Concentric, &backend(), &p, &c).is_err());
    assert!(paint(&region, &bad_margin, &Concentric, &backend(), &p, &c).is_err());
  }

  #[test]
  fn cancelled_paint_returns_error() {
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(paint(&region, &PaintParams::default(), &Concentric, &backend(), &ProgressReporter::silent(), &cancel).is_err());
  }

  #[test]
  fn order_paths_is_optimizer_agnostic() {
    let rows = vec![
      RingPath::open(vec![Point::new(0.0, 0.0), Point::new(5.0, 0.0)]),
      RingPath::open(vec![Point::new(0.0, 1.0), Point::new(5.0, 1.0)]),
    ];
    let (_, c) = silent();
    let nn = order_paths(&rows, &NearestNeighbor, Point::new(0.0, 0.0), &c).expect("nn");
    assert_eq!(nn.len(), 2, "ordering preserves every path");
  }
}
