//! Panelization — array a source object into a grid of `rows x cols`, producing one combined object.
//!
//! Provenance: FlatCAM's panelize tool (see `docs/eitri-porting-plan.md` §7.7). Copies are translated on a grid
//! (via [`eitri_core::Affine`]) and merged (via the boolean union). The one FlatCAM gotcha this reproduces
//! explicitly is **gap vs pitch**: a [`Spacing::Gap`] leaves a fixed clear band between copies (so the grid step is
//! `source_extent + gap`), whereas a [`Spacing::Pitch`] fixes the centre-to-centre step directly. Panelization is
//! object-kind agnostic — [`panel_offsets`] yields the per-cell translation, and thin wrappers apply it to a copper
//! `MultiPolygon` ([`panelize_multipolygon`]) or to drill points ([`panelize_points`]).

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use geo_types::MultiPolygon;
use rayon::prelude::*;

use eitri_core::{Affine, CancelToken, Error, ProgressEvent, ProgressReporter, Result};
use eitri_geo::{GeoBackend, apply_affine, bounds};

use crate::optimize::Point;

/// Grid spacing along one axis: either a clear gap between copies, or a fixed centre-to-centre pitch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Spacing {
  /// A clear band of this width (millimetres) between adjacent copies; the grid step is `source_extent + gap`.
  Gap(f64),
  /// The centre-to-centre step (millimetres) between adjacent copies, independent of the source extent.
  Pitch(f64),
}

impl Spacing {
  /// The grid step for a source of the given `extent` (its width or height along this axis).
  fn step(self, extent: f64) -> f64 {
    match self {
      Spacing::Gap(g) => extent + g,
      Spacing::Pitch(p) => p,
    }
  }
}

/// A panelization specification: a `rows x cols` grid with X and Y spacing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PanelSpec {
  /// Number of rows (Y direction); must be at least one.
  pub rows: usize,
  /// Number of columns (X direction); must be at least one.
  pub cols: usize,
  /// Horizontal spacing between columns.
  pub x: Spacing,
  /// Vertical spacing between rows.
  pub y: Spacing,
}

impl PanelSpec {
  /// Validate the grid dimensions.
  fn validate(&self) -> Result<()> {
    if self.rows == 0 || self.cols == 0 {
      return Err(Error::InvalidGeometry("panelization needs at least one row and column".to_string()));
    }
    Ok(())
  }
}

/// The per-cell translation offsets for `spec` given a source spanning `(width, height)`. Row-major: the first
/// offset is `(0, 0)`, then across the columns, then up the rows.
pub fn panel_offsets(spec: &PanelSpec, width: f64, height: f64) -> Vec<(f64, f64)> {
  let step_x = spec.x.step(width);
  let step_y = spec.y.step(height);
  let mut offsets = Vec::with_capacity(spec.rows * spec.cols);
  for r in 0..spec.rows {
    for col in 0..spec.cols {
      offsets.push((col as f64 * step_x, r as f64 * step_y));
    }
  }
  offsets
}

/// Panelize a copper `source` into a combined `MultiPolygon`: translate a copy to each grid cell and union them all.
/// Independent copies are translated in parallel. Progress advances per cell; cancellation is polled up front.
pub fn panelize_multipolygon<B>(
  source: &MultiPolygon<f64>,
  spec: &PanelSpec,
  backend: &B,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<MultiPolygon<f64>>
where
  B: GeoBackend + Sync,
{
  spec.validate()?;
  progress.emit(ProgressEvent::Started { label: "panelize".to_string() });
  cancel.check()?;

  let Some((x0, y0, x1, y1)) = bounds(source) else {
    progress.emit(ProgressEvent::Finished);
    return Ok(MultiPolygon::new(Vec::new()));
  };
  let offsets = panel_offsets(spec, x1 - x0, y1 - y0);
  let total = offsets.len() as u64;

  // Translate each copy in parallel, advancing progress as each cell actually completes (not in a separate loop
  // afterwards, which left a bound UI motionless then snapped it to 100%). Cancellation is polled per cell too.
  let counter = Arc::new(AtomicU64::new(0));
  let copies: Vec<MultiPolygon<f64>> = offsets
    .par_iter()
    .map(|&(dx, dy)| -> Result<MultiPolygon<f64>> {
      cancel.check()?;
      let copy = apply_affine(source, Affine::translate(dx, dy));
      let done = counter.fetch_add(1, Ordering::Relaxed) + 1;
      progress.advance(done, total);
      Ok(copy)
    })
    .collect::<Result<Vec<_>>>()?;
  let all: Vec<_> = copies.into_iter().flat_map(|mp| mp.0).collect();
  let merged = backend.union_all(&all)?;

  progress.emit(ProgressEvent::Finished);
  Ok(merged)
}

/// Panelize a set of drill `points` (or any point set) by replicating them across the grid. `width`/`height` are the
/// source extent used to resolve [`Spacing::Gap`] into a step; pass the drill pattern's bounding size.
pub fn panelize_points(points: &[Point], spec: &PanelSpec, width: f64, height: f64) -> Result<Vec<Point>> {
  spec.validate()?;
  let offsets = panel_offsets(spec, width, height);
  let mut out = Vec::with_capacity(points.len() * offsets.len());
  for (dx, dy) in offsets {
    for p in points {
      out.push(Point::new(p.x + dx, p.y + dy));
    }
  }
  Ok(out)
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;
  use geo::algorithm::area::Area;
  use geo_types::{Coord, LineString, Polygon};

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

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  #[test]
  fn gap_spacing_steps_by_extent_plus_gap() {
    // A 10x8 source in a 2x2 grid with a 3mm gap: steps are 10+3 = 13 in x and 8+3 = 11 in y.
    let spec = PanelSpec { rows: 2, cols: 2, x: Spacing::Gap(3.0), y: Spacing::Gap(3.0) };
    let offsets = panel_offsets(&spec, 10.0, 8.0);
    assert_eq!(offsets, vec![(0.0, 0.0), (13.0, 0.0), (0.0, 11.0), (13.0, 11.0)]);
  }

  #[test]
  fn pitch_spacing_steps_by_pitch_regardless_of_extent() {
    // The same grid but a fixed 20mm pitch ignores the source extent.
    let spec = PanelSpec { rows: 2, cols: 2, x: Spacing::Pitch(20.0), y: Spacing::Pitch(20.0) };
    let offsets = panel_offsets(&spec, 10.0, 8.0);
    assert_eq!(offsets, vec![(0.0, 0.0), (20.0, 0.0), (0.0, 20.0), (20.0, 20.0)]);
  }

  #[test]
  fn panelize_multipolygon_makes_disjoint_copies_with_summed_area() {
    let source = MultiPolygon::new(vec![square(0.0, 0.0, 2.0)]); // 4x4 = area 16
    let spec = PanelSpec { rows: 2, cols: 2, x: Spacing::Gap(5.0), y: Spacing::Gap(5.0) };
    let (p, c) = silent();
    let panel = panelize_multipolygon(&source, &spec, &DefaultBackend::new(), &p, &c).expect("panelize");
    assert_eq!(panel.0.len(), 4, "a 2x2 gapped panel is four disjoint copies");
    assert!((panel.unsigned_area() - 64.0).abs() < 1e-6, "four 16mm^2 copies => 64mm^2, got {}", panel.unsigned_area());
  }

  #[test]
  fn panelize_points_replicates_the_pattern() {
    let points = vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)];
    let spec = PanelSpec { rows: 1, cols: 2, x: Spacing::Pitch(10.0), y: Spacing::Pitch(10.0) };
    let out = panelize_points(&points, &spec, 2.0, 2.0).expect("points");
    assert_eq!(out.len(), 4, "two points across two columns => four holes");
    assert!(out.contains(&Point::new(10.0, 0.0)) && out.contains(&Point::new(11.0, 0.0)), "second column shifted by pitch");
  }

  #[test]
  fn zero_grid_is_rejected() {
    let spec = PanelSpec { rows: 0, cols: 2, x: Spacing::Gap(1.0), y: Spacing::Gap(1.0) };
    assert!(panelize_points(&[Point::new(0.0, 0.0)], &spec, 1.0, 1.0).is_err());
  }

  #[test]
  fn progress_advances_once_per_cell_reaching_the_full_count() {
    // Finding #7: progress used to be advanced in a separate loop AFTER the whole parallel translate finished, so a
    // bound UI saw no movement then a single snap to 100%. Advancing inside the map yields one incremental event per
    // completed cell, covering 1..=total exactly once and reaching the full count.
    let source = MultiPolygon::new(vec![square(0.0, 0.0, 2.0)]);
    let spec = PanelSpec { rows: 2, cols: 3, x: Spacing::Gap(1.0), y: Spacing::Gap(1.0) };
    let (reporter, rx) = ProgressReporter::channel();
    panelize_multipolygon(&source, &spec, &DefaultBackend::new(), &reporter, &CancelToken::new()).expect("panelize");
    drop(reporter); // close the channel so the drain terminates
    let mut dones: Vec<u64> = rx
      .iter()
      .filter_map(|e| match e {
        ProgressEvent::Advanced { done, total } => {
          assert_eq!(total, 6, "every advance reports the full cell count as total");
          Some(done)
        }
        _ => None,
      })
      .collect();
    dones.sort_unstable();
    assert_eq!(dones, (1..=6).collect::<Vec<_>>(), "one advance per cell, covering 1..=total exactly once");
  }

  #[test]
  fn cancelled_panelize_returns_error() {
    let source = MultiPolygon::new(vec![square(0.0, 0.0, 2.0)]);
    let spec = PanelSpec { rows: 2, cols: 2, x: Spacing::Gap(1.0), y: Spacing::Gap(1.0) };
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(panelize_multipolygon(&source, &spec, &DefaultBackend::new(), &ProgressReporter::silent(), &cancel).is_err());
  }
}
