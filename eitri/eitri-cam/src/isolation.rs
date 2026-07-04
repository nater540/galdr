//! Isolation routing — offset copper outward and cut its boundary so traces are electrically separated.
//!
//! To isolate copper, every copper polygon is offset *outward* and the resulting boundary rings become the cut
//! path. Pass `n` (0-based) offsets by `tool_radius + n * (tool_dia * (1 - overlap))`, so successive passes are
//! concentric rings widening the isolation gap. See `docs/eitri-porting-plan.md` §7.1. Provenance: FlatCAM's
//! `Gerber.isolation_geometry()`.
//!
//! Milling direction (climb vs conventional) is set by ring **winding**, which each offset backend reports
//! differently — the §7.1 porting risk. Eitri routes every ring through [`eitri_geo::GeoBackend::normalize_winding`]
//! so the orientation is deterministic and flips cleanly between the two directions.
//!
//! Two output flavours share the [`IsolationRing`]/[`IsolationToolpaths`] envelope:
//! - [`isolate`] — the must-have: Clipper2 polyline rings via the [`GeoBackend`]. Handles copper with holes
//!   (annular pads isolate on both edges), multiple disjoint polygons, pass combining, and collapse-safe thin gaps.
//! - [`isolate_arc`] — the arc-preserving option via [`eitri_geo::offset_arc`], keeping true arcs so Phase 4 can
//!   emit `G02`/`G03`. It is exterior-boundary only and does not combine passes (see the function docs); use
//!   [`isolate`] when holes or combining are needed.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use geo_types::{LineString, MultiPolygon, Polygon};
use rayon::prelude::*;

use eitri_core::{CancelToken, Error, ProgressEvent, ProgressReporter, Result};
use eitri_geo::{ArcPolyline, ArcVertex, GeoBackend, JoinType, WindingDirection, offset_arc};

use crate::optimize::Point;

/// Which way the cutter goes around a ring, expressed as a milling convention rather than a raw winding. The
/// mapping to a concrete [`WindingDirection`] is centralized in [`MillingDirection::winding`] so it can be swapped
/// in one place if a machine's spindle sense demands the opposite.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MillingDirection {
  /// Climb milling — the cutter's edge advances into the material with the feed; exterior rings run CCW.
  Climb,
  /// Conventional milling — the cutter's edge opposes the feed; exterior rings run CW.
  Conventional,
}

impl MillingDirection {
  /// The exterior-ring winding this milling direction produces. Holes take the opposite winding (handled by
  /// [`eitri_geo::GeoBackend::normalize_winding`]).
  pub fn winding(self) -> WindingDirection {
    match self {
      MillingDirection::Climb => WindingDirection::Ccw,
      MillingDirection::Conventional => WindingDirection::Cw,
    }
  }
}

/// Parameters for an isolation routing operation. Distances are millimetres; `overlap` is a fraction in `[0, 1)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IsolationParams {
  /// Isolation tool diameter (millimetres); must be positive.
  pub tool_diameter: f64,
  /// Number of concentric passes; must be at least one.
  pub passes: usize,
  /// Fraction each pass overlaps the previous, in `[0, 1)`; sets the pass-to-pass spacing.
  pub overlap: f64,
  /// Union overlapping same-pass rings so the cutter does not retrace shared boundary between close traces.
  pub combine: bool,
  /// Milling direction, which sets the ring winding.
  pub direction: MillingDirection,
  /// Corner join style for the outward offset.
  pub join: JoinType,
  /// Miter limit ratio (only meaningful for [`JoinType::Miter`]).
  pub miter_limit: f64,
}

impl Default for IsolationParams {
  fn default() -> IsolationParams {
    IsolationParams {
      tool_diameter: 0.2,
      passes: 1,
      overlap: 0.0,
      combine: false,
      direction: MillingDirection::Climb,
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }
}

impl IsolationParams {
  /// The outward offset distance (millimetres) for pass `n` (0-based): `radius + n * dia * (1 - overlap)`.
  pub fn offset_for(&self, pass: usize) -> f64 {
    let radius = self.tool_diameter / 2.0;
    radius + (pass as f64) * self.tool_diameter * (1.0 - self.overlap)
  }

  /// Validate the parameter domain, returning a descriptive error rather than producing nonsense geometry.
  fn validate(&self) -> Result<()> {
    if self.tool_diameter.is_nan() || self.tool_diameter <= 0.0 {
      return Err(Error::InvalidGeometry("isolation tool diameter must be positive".to_string()));
    }
    if self.passes == 0 {
      return Err(Error::InvalidGeometry("isolation needs at least one pass".to_string()));
    }
    if !(0.0..1.0).contains(&self.overlap) {
      return Err(Error::InvalidGeometry("isolation overlap must be in [0, 1)".to_string()));
    }
    Ok(())
  }
}

/// A closed ring toolpath as a polyline of points (millimetres). The first and last point coincide.
#[derive(Debug, Clone, PartialEq)]
pub struct RingPath {
  /// Ring vertices in order; closed (first == last).
  pub points: Vec<Point>,
}

/// One isolation cut ring, tagged with the pass and offset that produced it and the winding it was normalized to.
/// Generic over the geometry payload so the polyline ([`RingPath`]) and arc ([`ArcPolyline`]) flavours share it.
#[derive(Debug, Clone, PartialEq)]
pub struct IsolationRing<G> {
  /// Zero-based pass index.
  pub pass: usize,
  /// Outward offset distance from the copper edge (millimetres).
  pub offset: f64,
  /// The winding this ring was normalized to (exterior = requested direction, holes = the opposite).
  pub winding: WindingDirection,
  /// The ring geometry — a [`RingPath`] or an [`ArcPolyline`].
  pub geometry: G,
}

/// The full set of isolation rings from one operation, in pass-major then polygon order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct IsolationToolpaths<G> {
  /// The cut rings.
  pub rings: Vec<IsolationRing<G>>,
}

impl<G> IsolationToolpaths<G> {
  /// Number of rings produced.
  pub fn len(&self) -> usize {
    self.rings.len()
  }

  /// Whether no rings were produced (e.g. every feature was smaller than the tool).
  pub fn is_empty(&self) -> bool {
    self.rings.is_empty()
  }
}

/// Isolate `copper` into polyline cut rings via the geometry `backend`. Each copper polygon is offset outward once
/// per pass; with `params.combine` the same-pass offsets of all polygons are unioned so overlapping rings merge.
/// Copper polygons with holes isolate on both their outer and inner edges. Independent polygons are offset in
/// parallel. Progress advances once per copper polygon; cancellation is polled per polygon.
pub fn isolate<B>(
  backend: &B,
  copper: &MultiPolygon<f64>,
  params: &IsolationParams,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<IsolationToolpaths<RingPath>>
where
  B: GeoBackend + Sync,
{
  params.validate()?;
  progress.emit(ProgressEvent::Started { label: "isolation".to_string() });
  cancel.check()?;

  let direction = params.direction.winding();
  let total = copper.0.len() as u64;
  let counter = Arc::new(AtomicU64::new(0));

  // Offset each copper polygon by every pass distance, in parallel. `per_poly[i][pass]` is the offset region of
  // polygon `i` at pass `pass`; the backend does its own per-call allocation so the fan-out shares no state.
  let per_poly: Vec<Vec<MultiPolygon<f64>>> = copper
    .0
    .par_iter()
    .map(|poly| -> Result<Vec<MultiPolygon<f64>>> {
      cancel.check()?;
      let mut passes = Vec::with_capacity(params.passes);
      for pass in 0..params.passes {
        passes.push(backend.offset(poly, params.offset_for(pass), params.join, params.miter_limit)?);
      }
      let done = counter.fetch_add(1, Ordering::Relaxed) + 1;
      progress.advance(done, total);
      Ok(passes)
    })
    .collect::<Result<Vec<_>>>()?;

  let mut rings: Vec<IsolationRing<RingPath>> = Vec::new();
  for pass in 0..params.passes {
    let offset = params.offset_for(pass);
    if params.combine {
      // Merge every polygon's pass-`pass` offset so overlapping rings from neighbouring traces become one path.
      let solids: Vec<Polygon<f64>> = per_poly.iter().flat_map(|passes| passes[pass].0.iter().cloned()).collect();
      let merged = backend.union_all(&solids)?;
      push_rings(backend, &merged, direction, pass, offset, &mut rings);
    } else {
      for passes in &per_poly {
        push_rings(backend, &passes[pass], direction, pass, offset, &mut rings);
      }
    }
  }

  progress.emit(ProgressEvent::Finished);
  Ok(IsolationToolpaths { rings })
}

/// Extract every ring of `mp` (exterior + holes) as a [`RingPath`], normalizing winding so the exterior takes
/// `direction` and holes take its reverse, and append them to `out`.
fn push_rings<B: GeoBackend>(
  backend: &B,
  mp: &MultiPolygon<f64>,
  direction: WindingDirection,
  pass: usize,
  offset: f64,
  out: &mut Vec<IsolationRing<RingPath>>,
) {
  for poly in &mp.0 {
    let normalized = backend.normalize_winding(poly, direction);
    out.push(IsolationRing { pass, offset, winding: direction, geometry: ring_path(normalized.exterior()) });
    for hole in normalized.interiors() {
      out.push(IsolationRing { pass, offset, winding: direction.reversed(), geometry: ring_path(hole) });
    }
  }
}

/// Convert a closed `LineString` ring into a [`RingPath`].
fn ring_path(ring: &LineString<f64>) -> RingPath {
  RingPath { points: ring.0.iter().map(|c| Point::new(c.x, c.y)).collect() }
}

/// Isolate `copper` into **arc-preserving** cut rings via [`eitri_geo::offset_arc`], keeping true arcs so Phase 4
/// can emit `G02`/`G03` rather than a chain of tiny linear moves.
///
/// Scope, by construction of the underlying arc offset: this offsets each copper polygon's **exterior** boundary
/// only (inner/hole edges are not isolated) and does **not** honour `params.combine` (arc rings cannot be unioned
/// without flattening). Use [`isolate`] when hole isolation or pass combining is required. Milling direction *is*
/// honoured: each ring is reversed to match `params.direction`. Independent polygons offset in parallel.
pub fn isolate_arc(
  copper: &MultiPolygon<f64>,
  params: &IsolationParams,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<IsolationToolpaths<ArcPolyline>> {
  params.validate()?;
  progress.emit(ProgressEvent::Started { label: "isolation (arc)".to_string() });
  cancel.check()?;

  let direction = params.direction.winding();
  let total = copper.0.len() as u64;
  let counter = Arc::new(AtomicU64::new(0));

  // `per_poly[i][pass]` is the arc rings of polygon `i` at pass `pass`. `offset_arc` normalizes its input and
  // prunes collapsing insets cleanly, so an empty vector simply means the feature was smaller than the tool.
  let per_poly: Vec<Vec<Vec<ArcPolyline>>> = copper
    .0
    .par_iter()
    .map(|poly| -> Result<Vec<Vec<ArcPolyline>>> {
      cancel.check()?;
      let mut passes = Vec::with_capacity(params.passes);
      for pass in 0..params.passes {
        let rings = offset_arc(poly, params.offset_for(pass))?;
        passes.push(rings.into_iter().map(|ring| orient_arc(ring, direction)).collect());
      }
      let done = counter.fetch_add(1, Ordering::Relaxed) + 1;
      progress.advance(done, total);
      Ok(passes)
    })
    .collect::<Result<Vec<_>>>()?;

  let mut rings: Vec<IsolationRing<ArcPolyline>> = Vec::new();
  for pass in 0..params.passes {
    let offset = params.offset_for(pass);
    for passes in &per_poly {
      for ring in &passes[pass] {
        rings.push(IsolationRing { pass, offset, winding: direction, geometry: ring.clone() });
      }
    }
  }

  progress.emit(ProgressEvent::Finished);
  Ok(IsolationToolpaths { rings })
}

/// Return `ring` wound in `direction`, reversing it (bulge-correct) if its vertex polygon is wound the other way.
fn orient_arc(ring: ArcPolyline, direction: WindingDirection) -> ArcPolyline {
  if arc_vertex_winding(&ring) == direction { ring } else { reverse_arc(&ring) }
}

/// The winding of an arc polyline judged from its vertex polygon (the shoelace sign; bulges are ignored, which is
/// reliable for the near-convex rings an outward offset produces).
fn arc_vertex_winding(ring: &ArcPolyline) -> WindingDirection {
  let n = ring.vertices.len();
  let mut area2 = 0.0;
  for i in 0..n {
    let a = ring.vertices[i];
    let b = ring.vertices[(i + 1) % n];
    area2 += a.x * b.y - b.x * a.y;
  }
  if area2 >= 0.0 { WindingDirection::Ccw } else { WindingDirection::Cw }
}

/// Reverse the traversal of a closed arc polyline. Vertices reverse order and each segment's bulge negates and
/// shifts to the vertex it now leaves — so `reverse_arc(reverse_arc(p)) == p`.
fn reverse_arc(ring: &ArcPolyline) -> ArcPolyline {
  let n = ring.vertices.len();
  if n == 0 {
    return ring.clone();
  }
  let mut out = Vec::with_capacity(n);
  for k in 0..n {
    let src = ring.vertices[(n - k) % n];
    // The segment leaving reversed-position `k` is the original segment `(n-1-k)` traversed backward.
    let bulge = -ring.vertices[(n - 1 - k) % n].bulge;
    out.push(ArcVertex { x: src.x, y: src.y, bulge });
  }
  ArcPolyline { vertices: out }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;
  use geo::algorithm::winding_order::{Winding, WindingOrder};
  use geo_types::Coord;

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    let ring = LineString(vec![
      Coord { x: cx - half, y: cy - half },
      Coord { x: cx + half, y: cy - half },
      Coord { x: cx + half, y: cy + half },
      Coord { x: cx - half, y: cy + half },
      Coord { x: cx - half, y: cy - half },
    ]);
    Polygon::new(ring, vec![])
  }

  fn bounds(path: &RingPath) -> (f64, f64, f64, f64) {
    path.points.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |(x0, y0, x1, y1), p| {
      (x0.min(p.x), y0.min(p.y), x1.max(p.x), y1.max(p.y))
    })
  }

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  #[test]
  fn offset_for_matches_the_documented_spacing() {
    let params = IsolationParams { tool_diameter: 0.4, passes: 3, overlap: 0.25, ..Default::default() };
    // radius = 0.2; step = dia*(1-overlap) = 0.4*0.75 = 0.3.
    assert!((params.offset_for(0) - 0.2).abs() < 1e-12);
    assert!((params.offset_for(1) - 0.5).abs() < 1e-12);
    assert!((params.offset_for(2) - 0.8).abs() < 1e-12);
  }

  #[test]
  fn single_pass_wraps_one_square_at_tool_radius() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let params = IsolationParams { tool_diameter: 1.0, passes: 1, ..Default::default() };
    let (p, c) = silent();
    let out = isolate(&backend, &copper, &params, &p, &c).expect("isolate");
    assert_eq!(out.len(), 1, "one solid square, one pass => one ring");
    // 10mm square offset outward by radius 0.5 => bounds roughly [-5.5, 5.5].
    let (x0, y0, x1, y1) = bounds(&out.rings[0].geometry);
    assert!(x0 < -5.4 && y0 < -5.4 && x1 > 5.4 && y1 > 5.4, "expected outward wrap, got {:?}", (x0, y0, x1, y1));
  }

  #[test]
  fn n_passes_produce_n_rings_at_the_right_spacing() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let params = IsolationParams { tool_diameter: 1.0, passes: 3, overlap: 0.0, ..Default::default() };
    let (p, c) = silent();
    let out = isolate(&backend, &copper, &params, &p, &c).expect("isolate");
    assert_eq!(out.len(), 3, "three passes over one square => three rings");
    // Each pass is tagged with the documented offset, and the ring's half-width grows by that offset.
    for (pass, ring) in out.rings.iter().enumerate() {
      assert_eq!(ring.pass, pass);
      let expected = params.offset_for(pass);
      assert!((ring.offset - expected).abs() < 1e-12);
      let (x0, _, x1, _) = bounds(&ring.geometry);
      let half_width = (x1 - x0) / 2.0;
      assert!((half_width - (5.0 + expected)).abs() < 0.05, "pass {pass}: half {half_width} vs {}", 5.0 + expected);
    }
  }

  #[test]
  fn milling_direction_flips_ring_winding() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let (p, c) = silent();

    let climb = isolate(
      &backend,
      &copper,
      &IsolationParams { tool_diameter: 1.0, direction: MillingDirection::Climb, ..Default::default() },
      &p,
      &c,
    )
    .expect("climb");
    let conv = isolate(
      &backend,
      &copper,
      &IsolationParams { tool_diameter: 1.0, direction: MillingDirection::Conventional, ..Default::default() },
      &p,
      &c,
    )
    .expect("conv");

    let ls = |ring: &IsolationRing<RingPath>| {
      LineString(ring.geometry.points.iter().map(|p| Coord { x: p.x, y: p.y }).collect())
    };
    assert_eq!(ls(&climb.rings[0]).winding_order(), Some(WindingOrder::CounterClockwise));
    assert_eq!(ls(&conv.rings[0]).winding_order(), Some(WindingOrder::Clockwise));
  }

  #[test]
  fn copper_with_hole_isolates_both_edges() {
    let backend = DefaultBackend::new();
    // An annular pad: 10mm outer square with a 4mm square hole.
    let outer = LineString(vec![
      Coord { x: -5.0, y: -5.0 },
      Coord { x: 5.0, y: -5.0 },
      Coord { x: 5.0, y: 5.0 },
      Coord { x: -5.0, y: 5.0 },
      Coord { x: -5.0, y: -5.0 },
    ]);
    let hole = LineString(vec![
      Coord { x: -2.0, y: -2.0 },
      Coord { x: 2.0, y: -2.0 },
      Coord { x: 2.0, y: 2.0 },
      Coord { x: -2.0, y: 2.0 },
      Coord { x: -2.0, y: -2.0 },
    ]);
    let copper = MultiPolygon::new(vec![Polygon::new(outer, vec![hole])]);
    let params = IsolationParams { tool_diameter: 1.0, passes: 1, ..Default::default() };
    let (p, c) = silent();
    let out = isolate(&backend, &copper, &params, &p, &c).expect("isolate");
    // One exterior ring (CCW for climb) plus one hole ring (CW). The hole ring carries the reversed winding tag.
    assert_eq!(out.len(), 2, "annular pad isolates on both edges");
    assert!(out.rings.iter().any(|r| r.winding == WindingDirection::Ccw), "outer edge present");
    assert!(out.rings.iter().any(|r| r.winding == WindingDirection::Cw), "inner edge present");
  }

  #[test]
  fn disjoint_polygons_each_get_a_ring() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(-20.0, 0.0, 2.0), square(20.0, 0.0, 2.0)]);
    let params = IsolationParams { tool_diameter: 0.5, passes: 1, ..Default::default() };
    let (p, c) = silent();
    let out = isolate(&backend, &copper, &params, &p, &c).expect("isolate");
    assert_eq!(out.len(), 2, "two far-apart pads => two independent rings");
  }

  #[test]
  fn combine_unions_overlapping_same_pass_rings() {
    let backend = DefaultBackend::new();
    // Two pads 1mm apart edge-to-edge; a 2mm tool (radius 1.0) offset makes their rings overlap and merge.
    let copper = MultiPolygon::new(vec![square(-1.5, 0.0, 1.0), square(1.5, 0.0, 1.0)]);
    let params_sep = IsolationParams { tool_diameter: 2.0, passes: 1, combine: false, ..Default::default() };
    let params_comb = IsolationParams { combine: true, ..params_sep };
    let (p, c) = silent();
    let separate = isolate(&backend, &copper, &params_sep, &p, &c).expect("separate");
    let combined = isolate(&backend, &copper, &params_comb, &p, &c).expect("combined");
    assert_eq!(separate.len(), 2, "uncombined keeps a retracing ring per pad");
    assert_eq!(combined.len(), 1, "combine merges the overlapping rings into one");
  }

  #[test]
  fn thin_gap_prunes_without_panicking() {
    let backend = DefaultBackend::new();
    // A tiny 0.1mm feature with a 2mm tool: the outward offset is fine, but nothing should panic and the ring set
    // stays well-defined. (The collapse hazard is inward offsets; this guards the pipeline stays robust.)
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 0.05)]);
    let params = IsolationParams { tool_diameter: 2.0, passes: 2, ..Default::default() };
    let (p, c) = silent();
    let out = isolate(&backend, &copper, &params, &p, &c).expect("isolate must not panic");
    assert!(!out.is_empty());
  }

  #[test]
  fn invalid_params_are_rejected() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 1.0)]);
    let (p, c) = silent();
    let bad_tool = IsolationParams { tool_diameter: 0.0, ..Default::default() };
    let bad_passes = IsolationParams { passes: 0, ..Default::default() };
    let bad_overlap = IsolationParams { overlap: 1.0, ..Default::default() };
    assert!(isolate(&backend, &copper, &bad_tool, &p, &c).is_err());
    assert!(isolate(&backend, &copper, &bad_passes, &p, &c).is_err());
    assert!(isolate(&backend, &copper, &bad_overlap, &p, &c).is_err());
  }

  #[test]
  fn cancelled_isolation_returns_error() {
    let backend = DefaultBackend::new();
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 1.0)]);
    let params = IsolationParams::default();
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(isolate(&backend, &copper, &params, &ProgressReporter::silent(), &cancel).is_err());
  }

  #[test]
  fn arc_isolation_produces_arc_rings() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let params = IsolationParams { tool_diameter: 1.0, passes: 2, ..Default::default() };
    let (p, c) = silent();
    let out = isolate_arc(&copper, &params, &p, &c).expect("arc isolate");
    assert_eq!(out.len(), 2, "two passes => two arc rings for one square");
    assert!(!out.rings[0].geometry.vertices.is_empty());
  }

  #[test]
  fn arc_isolation_honours_milling_direction() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let (p, c) = silent();
    let climb = isolate_arc(
      &copper,
      &IsolationParams { tool_diameter: 1.0, direction: MillingDirection::Climb, ..Default::default() },
      &p,
      &c,
    )
    .expect("climb");
    let conv = isolate_arc(
      &copper,
      &IsolationParams { tool_diameter: 1.0, direction: MillingDirection::Conventional, ..Default::default() },
      &p,
      &c,
    )
    .expect("conv");
    assert_eq!(arc_vertex_winding(&climb.rings[0].geometry), WindingDirection::Ccw);
    assert_eq!(arc_vertex_winding(&conv.rings[0].geometry), WindingDirection::Cw);
  }

  #[test]
  fn reverse_arc_is_an_involution() {
    let ring = ArcPolyline {
      vertices: vec![
        ArcVertex { x: 0.0, y: 0.0, bulge: 0.2 },
        ArcVertex { x: 4.0, y: 0.0, bulge: -0.1 },
        ArcVertex { x: 4.0, y: 3.0, bulge: 0.5 },
        ArcVertex { x: 0.0, y: 3.0, bulge: 0.0 },
      ],
    };
    let twice = reverse_arc(&reverse_arc(&ring));
    assert_eq!(twice, ring, "reversing a closed arc polyline twice restores it exactly");
  }
}
