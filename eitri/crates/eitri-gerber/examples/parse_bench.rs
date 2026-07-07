//! Rough timing for the Gerber import path — the heaviest step is the final `union_all` over every primitive (the
//! copper assembly). Generates a grid of `N` pad flashes and times a full parse. Run with:
//!
//! ```sh
//! cargo run -p eitri-gerber --release --example parse_bench -- 2000
//! ```
//!
//! Standalone example (not a `#[test]`) so timing never gates the suite.

use std::fmt::Write as _;
use std::time::Instant;

use eitri_core::{CancelToken, ProgressReporter};
use eitri_gerber::parse_gerber;

/// Build a Gerber source with `count` circle-pad flashes on a grid (0.6 mm pitch, 0.8 mm pads — adjacent pads
/// overlap so the assembly does real union work, merging them rather than concatenating disjoint shapes).
fn grid_source(count: usize) -> String {
  let cols = (count as f64).sqrt().ceil() as usize;
  let mut s = String::from("%FSLAX46Y46*%\n%MOMM*%\n%ADD10C,0.800000*%\nD10*\n");
  for i in 0..count {
    let (r, c) = (i / cols, i % cols);
    // 0.6 mm pitch in the 4.6 integer-coordinate encoding (mm * 1e6).
    let (x, y) = (c as i64 * 600_000, r as i64 * 600_000);
    let _ = writeln!(s, "X{x}Y{y}D03*");
  }
  s.push_str("M02*\n");
  s
}

fn main() {
  let count: usize = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(2000);
  let source = grid_source(count);

  let start = Instant::now();
  let image = parse_gerber(&source, &ProgressReporter::silent(), &CancelToken::new()).expect("parse");
  let elapsed = start.elapsed();

  println!("parse + assemble {count} pad flashes: {elapsed:.3?}");
  println!("  copper: {} polygon(s)", image.copper.0.len());
}
