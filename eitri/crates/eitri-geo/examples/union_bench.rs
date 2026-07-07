//! Rough throughput bench for the heaviest import step: `union_all` over thousands of polygons (the Gerber
//! `unary_union` that assembles copper). Run with:
//!
//! ```sh
//! cargo run -p eitri-geo --release --example union_bench -- 5000
//! ```
//!
//! Prints the polygon count, the wall-clock time, and the merged area/ring count so the result is sanity-checked,
//! not just timed. This is deliberately a standalone example (not a `#[test]`) so timing never gates the suite.

use std::time::Instant;

use eitri_geo::{DefaultBackend, GeoBackend};

use geo::algorithm::area::Area;
use geo_types::{Coord, LineString, Polygon};

/// Build a `count`-polygon grid of overlapping unit squares. Neighbours overlap (pitch < side) so `union_all`
/// does real merging work rather than trivially concatenating disjoint shapes — closer to dense copper.
fn overlapping_grid(count: usize) -> Vec<Polygon<f64>> {
  let side = 1.0;
  let pitch = 0.8;
  let cols = (count as f64).sqrt().ceil() as usize;
  let mut polys = Vec::with_capacity(count);
  for i in 0..count {
    let (r, c) = (i / cols, i % cols);
    let (x0, y0) = (c as f64 * pitch, r as f64 * pitch);
    polys.push(Polygon::new(
      LineString(vec![
        Coord { x: x0, y: y0 },
        Coord { x: x0 + side, y: y0 },
        Coord { x: x0 + side, y: y0 + side },
        Coord { x: x0, y: y0 + side },
        Coord { x: x0, y: y0 },
      ]),
      vec![],
    ));
  }
  polys
}

fn main() {
  let count: usize = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(5000);
  let backend = DefaultBackend::new();
  let polys = overlapping_grid(count);

  let start = Instant::now();
  let merged = backend.union_all(&polys).expect("union_all");
  let elapsed = start.elapsed();

  println!("union_all over {count} overlapping polygons: {elapsed:.3?}");
  println!("  merged into {} polygon(s), area {:.2}", merged.0.len(), merged.unsigned_area());
}
