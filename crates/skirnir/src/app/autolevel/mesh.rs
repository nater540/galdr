//! The height-map mesh + bilinear interpolation — a warped-surface model for pre-stream Z correction.
//!
//! A [`Mesh`] is a rectangular grid of probed Z DELTAS over the work XY plane, used to shift a program's Z so a
//! cut follows a non-flat surface (warped stock, un-tram'd copper). It mirrors OpenCNCPilot / ioSender's
//! `HeightMap` (MIT), reimplemented clean-room in Rust:
//!
//! - **Grid by spacing.** The operator gives a bounding box and a target point SPACING; the point counts are
//!   `ceil(range/spacing) + 1` (min 2), and the EFFECTIVE spacing is re-derived as `range/(count−1)` so the nodes
//!   land exactly on the bounds. See [`Mesh::from_spacing`].
//! - **Deltas from the first probed point.** Each stored value is `Z − Z₀`, where `Z₀` is the first point probed
//!   (the acquisition wizard's reference). So a perfectly flat surface stores all-zeros and the correction is the
//!   identity — the datum is whatever the operator already set; the mesh only follows the *deviation*.
//! - **Bilinear interpolation inside; a fail-safe LIFT outside.** A query within the grid is bilinearly
//!   interpolated from the four surrounding nodes; a query OUTSIDE the grid returns [`Mesh::max_height`] (the
//!   greatest delta), so the correction lifts the tool over an unprobed region rather than plunging into it.
//!
//! Pure — no egui, no I/O — so the whole model unit-tests without a window or hardware. This is Part B1 of the
//! probing plan; the acquisition wizard (B2) fills the deltas and the correction pass (C) reads them back.

/// The float tolerance (mm) by which a query coordinate may exceed the grid bounds and still be treated as ON the
/// edge (interpolated) rather than out-of-grid (lifted). ~1 nm — far below any real feature or probe resolution,
/// so it only ever catches floating-point overshoot of an endpoint that mathematically lands on the boundary.
const EDGE_TOLERANCE_MM: f64 = 1e-6;

/// A rectangular height-map: a grid of probed Z deltas over the work XY plane, with the metadata needed to place
/// and interpolate them. `serde` so it persists with the profile (Part B4); the fields are public so the
/// acquisition wizard fills them and the correction pass reads them without a wide method surface.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mesh {
  /// The grid's minimum work-X (mm) — the X of column 0.
  pub min_x: f64,
  /// The grid's minimum work-Y (mm) — the Y of row 0.
  pub min_y: f64,
  /// The grid's maximum work-X (mm) — the X of column `nx − 1`.
  pub max_x: f64,
  /// The grid's maximum work-Y (mm) — the Y of row `ny − 1`.
  pub max_y: f64,
  /// The EFFECTIVE X spacing (mm) between adjacent columns: `(max_x − min_x)/(nx − 1)`. Re-derived from the
  /// counts so the nodes land exactly on the bounds (it may differ slightly from the requested spacing).
  pub grid_x: f64,
  /// The effective Y spacing (mm) between adjacent rows: `(max_y − min_y)/(ny − 1)`.
  pub grid_y: f64,
  /// The number of columns (X nodes), always `≥ 2`.
  pub nx: usize,
  /// The number of rows (Y nodes), always `≥ 2`.
  pub ny: usize,
  /// The probed Z DELTAS (mm), row-major (`z[iy*nx + ix]`), each `Z − Z₀` relative to the first probed point.
  /// All-zero on a fresh grid (and on a perfectly flat surface), so the correction starts as the identity.
  pub z: Vec<f64>,
  /// The greatest delta in [`Self::z`] (never negative — the first point's own delta is `0`). The value returned
  /// for an out-of-grid query, so a query outside the probed region LIFTS the tool rather than plunging it.
  pub max_height: f64,
  /// Which work-coordinate system the mesh was probed under (`0` = the active WCS). Carried so a correction can
  /// warn on a WCS mismatch; set by the acquisition wizard.
  pub wcs_index: usize,
}

impl Mesh {
  /// Build a fresh (all-zero) mesh over the bounding box `[min, max]` with a target point `spacing`, deriving the
  /// point counts as `ceil(range/spacing) + 1` (min 2) and re-deriving the EFFECTIVE spacing as `range/(count−1)`
  /// so the nodes land exactly on the bounds. A non-positive spacing or a zero-length axis degenerates to the
  /// minimum 2 points (spacing `0`) for that axis rather than dividing by zero. `wcs_index` defaults to `0`; the
  /// acquisition wizard sets it. The deltas start all-zero (a flat surface), so an unprobed mesh is the identity.
  pub fn from_spacing(min: (f64, f64), max: (f64, f64), spacing: (f64, f64)) -> Self {
    let (min_x, min_y) = min;
    let (max_x, max_y) = max;
    let nx = axis_count(max_x - min_x, spacing.0);
    let ny = axis_count(max_y - min_y, spacing.1);
    Mesh {
      min_x,
      min_y,
      max_x,
      max_y,
      grid_x: effective_spacing(max_x - min_x, nx),
      grid_y: effective_spacing(max_y - min_y, ny),
      nx,
      ny,
      z: vec![0.0; nx * ny],
      max_height: 0.0,
      wcs_index: 0,
    }
  }

  /// The flat (row-major) index of node `(ix, iy)`. Callers stay within `ix < nx`, `iy < ny`.
  pub fn index(&self, ix: usize, iy: usize) -> usize {
    iy * self.nx + ix
  }

  /// The work-XY (mm) of node `(ix, iy)`: `(min_x + ix·grid_x, min_y + iy·grid_y)`. Used by the acquisition wizard
  /// to drive the probe to each node and by tests to place readings.
  pub fn point_xy(&self, ix: usize, iy: usize) -> (f64, f64) {
    (self.min_x + ix as f64 * self.grid_x, self.min_y + iy as f64 * self.grid_y)
  }

  /// Record the delta `delta` (mm, `Z − Z₀`) at node `(ix, iy)` and refresh [`Self::max_height`]. Out-of-range
  /// indices are ignored (a defensive no-op). `max_height` is updated INCREMENTALLY (`max(max_height, delta)`) —
  /// O(1) per point rather than an O(N) rescan of the whole grid, so a full acquisition is O(N) not O(N²). Since
  /// `max_height`'s only use is the out-of-grid fail-safe LIFT, a value left slightly high after a rare
  /// value-lowering re-probe is still safe (it lifts more, never plunges); a fresh acquisition resets it to `0`.
  pub fn set_delta(&mut self, ix: usize, iy: usize, delta: f64) {
    if ix >= self.nx || iy >= self.ny {
      return;
    }
    let idx = self.index(ix, iy);
    self.z[idx] = delta;
    // `max_height` starts ≥ 0 (floored) and only ever rises here, so the ≥ 0 floor is preserved without a rescan.
    self.max_height = self.max_height.max(delta);
  }

  /// Recompute [`Self::max_height`] authoritatively from the whole delta grid (the greatest delta, floored at `0`).
  /// The O(1) incremental update in [`Self::set_delta`] only ever RAISES `max_height`, so a re-probe that LOWERS a
  /// node can leave it stale-high; the acquisition wizard calls this ONCE on completion to pin the exact value. The
  /// single O(N) pass at the end keeps a full acquisition O(N) overall rather than the old O(N²) per-point rescan.
  pub fn recompute_max_height(&mut self) {
    self.max_height = self.z.iter().copied().fold(0.0, f64::max);
  }

  /// Bilinearly interpolate the Z delta at work-XY `(x, y)`. A point INSIDE the grid (bounds inclusive, within a
  /// small float tolerance) is interpolated from the four surrounding nodes; a point genuinely OUTSIDE returns
  /// [`Self::max_height`] — the fail-safe lift, so the correction never plunges the tool into an unprobed region.
  /// A degenerate axis (zero length, `grid == 0`) contributes no interpolation along it (the single column/row's
  /// value is used).
  pub fn interpolate(&self, x: f64, y: f64) -> f64 {
    // Outside the probed rectangle: lift by the greatest delta rather than guess/plunge (ioSender's MaxHeight). A
    // small tolerance keeps a coordinate that lands ON the boundary but floating-point-OVERSHOOTS it (e.g. a
    // subdivided endpoint computed as max_x + 1e-12) from spuriously lifting — it interpolates the edge instead
    // (the cell locator then clamps the fractional position to the last cell). A genuinely out-of-grid point
    // (beyond the tolerance) still lifts.
    if x < self.min_x - EDGE_TOLERANCE_MM
      || x > self.max_x + EDGE_TOLERANCE_MM
      || y < self.min_y - EDGE_TOLERANCE_MM
      || y > self.max_y + EDGE_TOLERANCE_MM
    {
      return self.max_height;
    }
    let (ix0, tx) = cell(x - self.min_x, self.grid_x, self.nx);
    let (iy0, ty) = cell(y - self.min_y, self.grid_y, self.ny);
    let z00 = self.z[self.index(ix0, iy0)];
    let z10 = self.z[self.index(ix0 + 1, iy0)];
    let z01 = self.z[self.index(ix0, iy0 + 1)];
    let z11 = self.z[self.index(ix0 + 1, iy0 + 1)];
    // Interpolate along X on each of the two rows, then along Y between them.
    let z0 = z00 * (1.0 - tx) + z10 * tx;
    let z1 = z01 * (1.0 - tx) + z11 * tx;
    z0 * (1.0 - ty) + z1 * ty
  }

  /// Whether the surface is flat within `eps` (mm): the peak-to-valley of the deltas (`max − min`) is `≤ eps`. A
  /// flat surface has all-equal deltas (all-zero after the delta-from-first-point subtraction), so a small `eps`
  /// means the mesh is effectively the identity and correction can be skipped. An empty grid is trivially flat.
  pub fn is_flat_within(&self, eps: f64) -> bool {
    let Some(&first) = self.z.first() else {
      return true;
    };
    let mut lo = first;
    let mut hi = first;
    for &v in &self.z {
      lo = lo.min(v);
      hi = hi.max(v);
    }
    hi - lo <= eps
  }
}

/// The node count along one axis for a `range` and target `spacing`: `ceil(range/spacing) + 1`, floored at 2. A
/// non-positive `spacing` or a `range ≤ 0` degenerates to the minimum 2 nodes (rather than dividing by zero or
/// producing a 1-node grid that cannot interpolate).
fn axis_count(range: f64, spacing: f64) -> usize {
  if spacing <= 0.0 || range <= 0.0 {
    return 2;
  }
  ((range / spacing).ceil() as usize + 1).max(2)
}

/// The effective spacing along one axis: `range/(count − 1)`. `count ≥ 2` by construction, so the divisor is
/// `≥ 1`; a zero-length axis yields `0` (a degenerate but valid single-position axis).
fn effective_spacing(range: f64, count: usize) -> f64 {
  range / (count - 1) as f64
}

/// Locate the interpolation cell for an offset `off` (distance from the axis min) given the effective `grid`
/// spacing and node `count`. Returns `(i0, t)`: the lower node index (clamped to `[0, count−2]`) and the
/// fractional position `t ∈ [0, 1]` within the cell. A degenerate axis (`grid ≤ 0`) collapses to cell 0 with
/// `t = 0`, so the single column/row's value is used without a divide-by-zero.
fn cell(off: f64, grid: f64, count: usize) -> (usize, f64) {
  if grid <= 0.0 {
    return (0, 0.0);
  }
  let f = off / grid;
  let last = count - 2; // the highest valid lower-node index (cell spans i0..=i0+1).
  let i0 = (f.floor() as usize).min(last);
  let t = (f - i0 as f64).clamp(0.0, 1.0);
  (i0, t)
}

#[cfg(test)]
mod tests {
  use super::*;

  fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
  }

  #[test]
  fn spacing_derives_counts_by_ceil_plus_one_and_re_derives_effective_spacing() {
    // range 10, spacing 3 → ceil(10/3)+1 = 4+1 = 5 nodes; effective spacing 10/4 = 2.5 (lands on the bounds).
    let m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (3.0, 3.0));
    assert_eq!((m.nx, m.ny), (5, 5));
    assert!(close(m.grid_x, 2.5) && close(m.grid_y, 2.5), "effective spacing must be range/(count-1)");
    // The last node lands exactly on the max bound.
    assert!(close(m.point_xy(m.nx - 1, m.ny - 1).0, 10.0));
    assert!(close(m.point_xy(m.nx - 1, m.ny - 1).1, 10.0));
    // A fresh grid is all-zero (identity) with a zero lift.
    assert!(m.z.iter().all(|&v| v == 0.0));
    assert_eq!(m.max_height, 0.0);
    assert_eq!(m.z.len(), m.nx * m.ny);
  }

  #[test]
  fn an_evenly_dividing_spacing_lands_exact_counts() {
    // range 10, spacing 2 → ceil(5)+1 = 6 nodes; effective spacing 10/5 = 2.0.
    let m = Mesh::from_spacing((0.0, 0.0), (10.0, 8.0), (2.0, 2.0));
    assert_eq!((m.nx, m.ny), (6, 5));
    assert!(close(m.grid_x, 2.0) && close(m.grid_y, 2.0));
  }

  #[test]
  fn counts_floor_at_two_and_a_zero_length_axis_degenerates() {
    // A spacing larger than the range still yields the minimum 2 nodes (ceil(0.1)+1 = 2), spacing = the full range.
    let m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (100.0, 100.0));
    assert_eq!((m.nx, m.ny), (2, 2));
    assert!(close(m.grid_x, 10.0) && close(m.grid_y, 10.0));
    // A zero-length axis (min == max) also floors at 2 nodes, with a zero effective spacing (degenerate but valid).
    let flat = Mesh::from_spacing((5.0, 0.0), (5.0, 10.0), (2.0, 2.0));
    assert_eq!(flat.nx, 2);
    assert_eq!(flat.grid_x, 0.0);
    // Interpolating on the degenerate axis must not divide by zero — the single column's value is used.
    assert!(close(flat.interpolate(5.0, 5.0), 0.0));
  }

  #[test]
  fn a_non_positive_spacing_degenerates_rather_than_dividing_by_zero() {
    let m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (0.0, -1.0));
    assert_eq!((m.nx, m.ny), (2, 2), "a non-positive spacing must fall back to the minimum grid");
  }

  #[test]
  fn bilinear_is_exact_at_the_nodes() {
    // A 3×3 grid over [0,10]² (spacing 5 → 3 nodes, grid 5). Fill distinct deltas and read them back at the nodes.
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (5.0, 5.0));
    assert_eq!((m.nx, m.ny), (3, 3));
    // A simple tilted plane delta = 0.1·ix + 0.2·iy so every node is distinct and midpoints are predictable.
    for iy in 0..m.ny {
      for ix in 0..m.nx {
        m.set_delta(ix, iy, 0.1 * ix as f64 + 0.2 * iy as f64);
      }
    }
    for iy in 0..m.ny {
      for ix in 0..m.nx {
        let (x, y) = m.point_xy(ix, iy);
        assert!(close(m.interpolate(x, y), 0.1 * ix as f64 + 0.2 * iy as f64), "node ({ix},{iy}) must read exactly");
      }
    }
  }

  #[test]
  fn bilinear_is_the_average_at_cell_center_and_edge_midpoints() {
    // Corners of the single cell [0,10]² carry 0, 2, 4, 6 (row-major z00,z10,z01,z11).
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0)); // 2×2, one cell.
    m.set_delta(0, 0, 0.0);
    m.set_delta(1, 0, 2.0);
    m.set_delta(0, 1, 4.0);
    m.set_delta(1, 1, 6.0);
    // Center = mean of the four corners = 3.
    assert!(close(m.interpolate(5.0, 5.0), 3.0));
    // Bottom edge midpoint = mean of z00,z10 = 1; left edge midpoint = mean of z00,z01 = 2.
    assert!(close(m.interpolate(5.0, 0.0), 1.0));
    assert!(close(m.interpolate(0.0, 5.0), 2.0));
    // The far corner reads its own node exactly (t = 1 on both axes).
    assert!(close(m.interpolate(10.0, 10.0), 6.0));
  }

  #[test]
  fn out_of_grid_returns_max_height_the_fail_safe_lift() {
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    m.set_delta(0, 0, 0.0);
    m.set_delta(1, 0, 0.5);
    m.set_delta(0, 1, -0.3);
    m.set_delta(1, 1, 0.9);
    // max_height is the greatest delta (0.9) — floored at 0, so a mostly-below surface still never plunges.
    assert!(close(m.max_height, 0.9));
    for (x, y) in [(-1.0, 5.0), (11.0, 5.0), (5.0, -1.0), (5.0, 11.0), (100.0, 100.0)] {
      assert!(close(m.interpolate(x, y), 0.9), "out-of-grid ({x},{y}) must lift by max_height");
    }
  }

  #[test]
  fn max_height_is_floored_at_zero_so_a_dished_surface_never_plunges_outside() {
    // Every non-reference node dips BELOW the first point (all deltas ≤ 0). The out-of-grid lift must still be 0
    // (never negative), so a query outside the dish holds Z rather than driving deeper.
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    m.set_delta(0, 0, 0.0); // the reference point.
    m.set_delta(1, 0, -0.5);
    m.set_delta(0, 1, -0.4);
    m.set_delta(1, 1, -0.9);
    assert_eq!(m.max_height, 0.0, "with all deltas ≤ 0 the lift floors at 0, never negative");
    assert!(close(m.interpolate(-1.0, -1.0), 0.0));
  }

  #[test]
  fn a_float_overshoot_at_the_edge_interpolates_the_edge_not_the_out_of_grid_lift() {
    // A tilted mesh so the edge value differs from the out-of-grid lift, making the bug observable. delta = 0.1·ix.
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0)); // 2×2, grid 10.
    m.set_delta(0, 0, 0.0);
    m.set_delta(1, 0, 1.0); // edge x=10 → delta 1.0.
    m.set_delta(0, 1, 0.0);
    m.set_delta(1, 1, 1.0);
    assert!(close(m.max_height, 1.0));
    // EXACTLY on the max edge interpolates the edge column (1.0), and a hair BEYOND it (a subdivided endpoint's
    // float overshoot) must ALSO interpolate the edge, NOT spuriously lift.
    assert!(close(m.interpolate(10.0, 5.0), 1.0), "exactly on the max edge must interpolate the edge value");
    assert!(close(m.interpolate(10.0 + 1e-9, 5.0), 1.0), "a float overshoot at the edge must interpolate, not lift");
    // A point GENUINELY outside (well beyond the tolerance) still lifts by max_height.
    assert!(close(m.interpolate(11.0, 5.0), m.max_height), "a genuinely out-of-grid point still lifts");
  }

  #[test]
  fn recompute_max_height_pins_the_exact_max_after_a_lowered_delta() {
    // The acquisition-completion recompute: `set_delta`'s O(1) incremental max only rises, so a re-probe that
    // LOWERS the current-max node leaves `max_height` stale-high. `recompute_max_height` pins the exact grid max.
    let mut m = Mesh::from_spacing((0.0, 0.0), (30.0, 0.0), (10.0, 10.0)); // 4×2.
    for (ix, d) in [(0, 0.0), (1, 0.5), (2, 0.1), (3, 0.3)] {
      m.set_delta(ix, 0, d);
    }
    assert!(close(m.max_height, 0.5), "the running max is 0.5 after the fill");
    m.set_delta(1, 0, -0.2); // lower the former max node.
    assert!(close(m.max_height, 0.5), "the incremental update leaves it stale-high (safe but imprecise)");
    m.recompute_max_height();
    assert!(close(m.max_height, 0.3), "the authoritative recompute pins the exact max (now 0.3)");
  }

  #[test]
  fn set_delta_tracks_the_running_max_incrementally() {
    // The O(1) incremental max must still equal the greatest delta after a fill (correctness preserved vs the old
    // O(N) rescan), and a later LOWER value leaves it conservatively high (safe — the out-of-grid lift only rises).
    let mut m = Mesh::from_spacing((0.0, 0.0), (30.0, 0.0), (10.0, 10.0)); // 4×2.
    for (ix, d) in [(0, 0.0), (1, 0.3), (2, 0.1), (3, 0.5)] {
      m.set_delta(ix, 0, d);
    }
    assert!(close(m.max_height, 0.5), "max_height tracks the running max incrementally");
    // Lowering a node does not drop max_height (conservative: a slightly-high lift is fail-safe, never plunges).
    m.set_delta(3, 0, -0.2);
    assert!(close(m.max_height, 0.5), "a value-lowering re-probe leaves the lift conservatively high (safe)");
  }

  #[test]
  fn a_flat_surface_is_the_identity_everywhere_and_reads_as_flat() {
    // An all-zero mesh (flat, or freshly probed on a flat plate) interpolates 0 everywhere: the correction is the
    // identity — the datum the operator set is untouched.
    let m = Mesh::from_spacing((0.0, 0.0), (20.0, 20.0), (5.0, 5.0));
    for (x, y) in [(0.0, 0.0), (7.3, 12.1), (20.0, 20.0), (10.0, 3.0)] {
      assert!(close(m.interpolate(x, y), 0.0));
    }
    assert!(m.is_flat_within(1e-6), "an all-zero mesh is flat");
  }

  #[test]
  fn is_flat_within_measures_the_peak_to_valley_of_the_deltas() {
    let mut m = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    m.set_delta(0, 0, 0.0);
    m.set_delta(1, 0, 0.02);
    m.set_delta(0, 1, -0.01);
    m.set_delta(1, 1, 0.03);
    // Peak-to-valley = 0.03 − (−0.01) = 0.04.
    assert!(!m.is_flat_within(0.02), "0.04 pk-pk is not flat within 0.02");
    assert!(m.is_flat_within(0.05), "0.04 pk-pk is flat within 0.05");
  }
}
