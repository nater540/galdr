//! Travel-path optimization — ordering a set of stops to minimize non-cutting rapid moves.
//!
//! FlatCAM leaned on Google OR-Tools for drill-path TSP; that dependency is a porting liability (heavy, non-Rust).
//! Eitri replaces it with a self-contained heuristic — nearest-neighbour seeding plus a 2-opt / Or-opt improvement
//! pass — behind the [`TravelOptimizer`] trait so a stronger solver can drop in later without touching callers. See
//! `docs/eitri-porting-plan.md` §7.6.
//!
//! Stops are **directed**: a point drill enters and leaves at the same coordinate, but a routed slot (or an
//! isolation ring entered at a chosen start) enters at one endpoint and leaves at another, and can be traversed
//! either way. The optimizer therefore orders stops *and* picks each stop's traversal direction, and it costs only
//! the rapid hops *between* stops — the in-stop traversal (cutting a slot, following a ring) is invariant under
//! ordering, so it never enters the objective.

use eitri_core::CancelToken;
use eitri_core::Result;

/// A 2D point in millimetres.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
  /// X coordinate (millimetres).
  pub x: f64,
  /// Y coordinate (millimetres).
  pub y: f64,
}

impl Point {
  /// Construct a point.
  pub fn new(x: f64, y: f64) -> Point {
    Point { x, y }
  }

  /// Euclidean distance to another point.
  pub fn distance_to(self, other: Point) -> f64 {
    (self.x - other.x).hypot(self.y - other.y)
  }
}

/// A stop on a travel tour: the tool enters at `entry` and leaves at `exit`. For a point drill `entry == exit`; for
/// a slot or a routed segment they differ, and the stop may be traversed reversed (see [`Routed::reversed`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stop {
  /// Where the tool arrives when this stop is traversed forward.
  pub entry: Point,
  /// Where the tool leaves when this stop is traversed forward.
  pub exit: Point,
}

impl Stop {
  /// A zero-length stop (a point drill): entry and exit coincide.
  pub fn point(p: Point) -> Stop {
    Stop { entry: p, exit: p }
  }

  /// A directed stop (a slot / routed segment) traversed from `entry` to `exit` when forward.
  pub fn segment(entry: Point, exit: Point) -> Stop {
    Stop { entry, exit }
  }

  /// The point the tool arrives at, given the traversal direction.
  fn arrival(&self, reversed: bool) -> Point {
    if reversed { self.exit } else { self.entry }
  }

  /// The point the tool departs from, given the traversal direction.
  fn departure(&self, reversed: bool) -> Point {
    if reversed { self.entry } else { self.exit }
  }
}

/// One stop placed in a tour: which input stop, and whether it is traversed with entry/exit swapped. `reversed` is
/// meaningless for point stops (entry == exit) and is left `false` for them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Routed {
  /// Index into the input `stops` slice.
  pub index: usize,
  /// Whether this stop is entered at its `exit` and left at its `entry`.
  pub reversed: bool,
}

/// Total rapid travel of a tour: the sum of the hops from `start` into the first stop and between successive stops.
/// The in-stop traversal length is deliberately excluded — it does not depend on the ordering.
pub fn tour_travel(stops: &[Stop], start: Point, tour: &[Routed]) -> f64 {
  let mut total = 0.0;
  let mut cursor = start;
  for routed in tour {
    let stop = &stops[routed.index];
    total += cursor.distance_to(stop.arrival(routed.reversed));
    cursor = stop.departure(routed.reversed);
  }
  total
}

/// Orders a set of stops to reduce total rapid travel. Implementors return a permutation of the input as a `Vec` of
/// [`Routed`] (index + traversal direction). CAM operations depend only on this trait, so the ordering strategy is
/// swappable without touching them.
pub trait TravelOptimizer {
  /// Order `stops`, beginning from `start`. The returned tour references every input stop exactly once.
  fn order(&self, stops: &[Stop], start: Point) -> Vec<Routed>;
}

/// Greedy nearest-neighbour ordering: from the current position, repeatedly take the unused stop whose nearer
/// endpoint is closest, orienting it so that endpoint is the entry. Fast (`O(n^2)`) and a good seed for 2-opt.
#[derive(Debug, Clone, Copy, Default)]
pub struct NearestNeighbor;

impl TravelOptimizer for NearestNeighbor {
  fn order(&self, stops: &[Stop], start: Point) -> Vec<Routed> {
    let mut used = vec![false; stops.len()];
    let mut tour = Vec::with_capacity(stops.len());
    let mut cursor = start;
    for _ in 0..stops.len() {
      let mut best: Option<(usize, bool, f64)> = None;
      for (i, stop) in stops.iter().enumerate() {
        if used[i] {
          continue;
        }
        let d_fwd = cursor.distance_to(stop.entry);
        let d_rev = cursor.distance_to(stop.exit);
        let (reversed, dist) = if d_rev < d_fwd { (true, d_rev) } else { (false, d_fwd) };
        if best.is_none_or(|(_, _, bd)| dist < bd) {
          best = Some((i, reversed, dist));
        }
      }
      // `best` is always `Some` here: the loop runs exactly for the count of still-unused stops.
      if let Some((i, reversed, _)) = best {
        used[i] = true;
        tour.push(Routed { index: i, reversed });
        cursor = stops[i].departure(reversed);
      }
    }
    tour
  }
}

/// Nearest-neighbour seeding followed by a 2-opt + Or-opt local-search improvement pass. Each round sweeps all
/// segment-reversal (2-opt) and single-stop relocation (Or-1) moves and applies every strictly-improving one;
/// rounds repeat until a full sweep changes nothing or `max_rounds` is reached. Both move families use `O(1)`
/// incremental delta evaluation, so a round is `O(n^2)`.
#[derive(Debug, Clone, Copy)]
pub struct TwoOpt {
  /// Upper bound on improvement rounds; the search also stops early once a round yields no change.
  pub max_rounds: usize,
}

/// Improving moves below this magnitude (millimetres of saved travel) are ignored, so floating-point noise cannot
/// spin the search or flip an orientation for no real gain.
const IMPROVE_EPS: f64 = 1e-9;

impl Default for TwoOpt {
  fn default() -> TwoOpt {
    TwoOpt { max_rounds: 32 }
  }
}

impl TwoOpt {
  /// A 2-opt optimizer with the default round cap.
  pub fn new() -> TwoOpt {
    TwoOpt::default()
  }

  /// Improve an existing `tour` in place-of-return, without reseeding. Exposed so a caller that already has an
  /// ordering (from another optimizer or a previous run) can refine it.
  pub fn improve(&self, stops: &[Stop], start: Point, tour: &[Routed]) -> Vec<Routed> {
    let mut tour = tour.to_vec();
    for _ in 0..self.max_rounds {
      let mut changed = false;
      changed |= self.two_opt_sweep(stops, start, &mut tour);
      changed |= self.or_opt_sweep(stops, start, &mut tour);
      if !changed {
        break;
      }
    }
    tour
  }

  /// The feed point into tour position `pos`: `start` for the first stop, otherwise the previous stop's departure.
  fn feed(stops: &[Stop], start: Point, tour: &[Routed], pos: usize) -> Point {
    if pos == 0 {
      start
    } else {
      let prev = tour[pos - 1];
      stops[prev.index].departure(prev.reversed)
    }
  }

  /// One 2-opt sweep: for every sub-range `[i..=j]`, reversing it flips the visiting order and each stop's
  /// traversal direction. Only the two boundary hops change, so the delta is computed in `O(1)`.
  fn two_opt_sweep(&self, stops: &[Stop], start: Point, tour: &mut [Routed]) -> bool {
    let n = tour.len();
    let mut changed = false;
    for i in 0..n {
      for j in (i + 1)..n {
        let a = Self::feed(stops, start, tour, i);
        let ri = tour[i];
        let rj = tour[j];
        let in_i = stops[ri.index].arrival(ri.reversed);
        // After reversal the slot at `i` becomes old `j` flipped, so its arrival is old `j`'s departure.
        let new_in = stops[rj.index].departure(rj.reversed);
        let mut before = a.distance_to(in_i);
        let mut after = a.distance_to(new_in);
        if j + 1 < n {
          let out_j = stops[rj.index].departure(rj.reversed);
          let c = {
            let next = tour[j + 1];
            stops[next.index].arrival(next.reversed)
          };
          // After reversal the slot at `j` becomes old `i` flipped, so its departure is old `i`'s arrival.
          let new_out = stops[ri.index].arrival(ri.reversed);
          before += out_j.distance_to(c);
          after += new_out.distance_to(c);
        }
        if after + IMPROVE_EPS < before {
          tour[i..=j].reverse();
          for routed in &mut tour[i..=j] {
            routed.reversed = !routed.reversed;
          }
          changed = true;
        }
      }
    }
    changed
  }

  /// One Or-1 sweep: try relocating each single stop to every other gap, in either orientation, accepting the first
  /// strictly-improving move. Delta is the removal saving plus the insertion cost, both `O(1)`.
  fn or_opt_sweep(&self, stops: &[Stop], start: Point, tour: &mut Vec<Routed>) -> bool {
    let n = tour.len();
    let mut changed = false;
    for i in 0..n {
      let moved = tour[i];
      // Cost removed by lifting the stop out of position `i` and closing the gap between its neighbours.
      let left = Self::feed(stops, start, tour, i);
      let arr_i = stops[moved.index].arrival(moved.reversed);
      let dep_i = stops[moved.index].departure(moved.reversed);
      let right = if i + 1 < n {
        let next = tour[i + 1];
        Some(stops[next.index].arrival(next.reversed))
      } else {
        None
      };
      let removal_gain = match right {
        Some(r) => left.distance_to(arr_i) + dep_i.distance_to(r) - left.distance_to(r),
        None => left.distance_to(arr_i),
      };

      // The tour with `i` lifted out; insertion gaps index into this reduced sequence.
      let reduced: Vec<Routed> = tour.iter().enumerate().filter(|(k, _)| *k != i).map(|(_, r)| *r).collect();
      let mut best: Option<(usize, bool, f64)> = None;
      for gap in 0..=reduced.len() {
        let u = if gap == 0 { start } else { stops[reduced[gap - 1].index].departure(reduced[gap - 1].reversed) };
        let v = reduced.get(gap).map(|next| stops[next.index].arrival(next.reversed));
        let base = v.map(|vp| u.distance_to(vp)).unwrap_or(0.0);
        for reversed in [false, true] {
          let arr = stops[moved.index].arrival(reversed);
          let dep = stops[moved.index].departure(reversed);
          let insert_cost = match v {
            Some(vp) => u.distance_to(arr) + dep.distance_to(vp) - base,
            None => u.distance_to(arr),
          };
          let delta = insert_cost - removal_gain;
          if delta < -IMPROVE_EPS && best.is_none_or(|(_, _, bd)| delta < bd) {
            best = Some((gap, reversed, delta));
          }
        }
      }
      if let Some((gap, reversed, _)) = best {
        let mut next = reduced;
        next.insert(gap, Routed { index: moved.index, reversed });
        *tour = next;
        changed = true;
      }
    }
    changed
  }
}

impl TravelOptimizer for TwoOpt {
  fn order(&self, stops: &[Stop], start: Point) -> Vec<Routed> {
    let seed = NearestNeighbor.order(stops, start);
    self.improve(stops, start, &seed)
  }
}

/// Order `stops` with `optimizer`, checking `cancel` first so a large tour stays interruptible. This is the seam
/// CAM operations call, keeping the cancel-check in one place.
pub fn order_stops<O: TravelOptimizer>(
  optimizer: &O,
  stops: &[Stop],
  start: Point,
  cancel: &CancelToken,
) -> Result<Vec<Routed>> {
  cancel.check()?;
  Ok(optimizer.order(stops, start))
}

#[cfg(test)]
mod tests {
  use super::*;

  fn pts(coords: &[(f64, f64)]) -> Vec<Stop> {
    coords.iter().map(|&(x, y)| Stop::point(Point::new(x, y))).collect()
  }

  /// Travel of the identity ordering (input order, all forward) — the baseline every optimizer must beat or match.
  fn identity_travel(stops: &[Stop], start: Point) -> f64 {
    let tour: Vec<Routed> = (0..stops.len()).map(|index| Routed { index, reversed: false }).collect();
    tour_travel(stops, start, &tour)
  }

  #[test]
  fn nearest_neighbor_visits_every_stop_once() {
    let stops = pts(&[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]);
    let tour = NearestNeighbor.order(&stops, Point::new(0.0, 0.0));
    let mut seen: Vec<usize> = tour.iter().map(|r| r.index).collect();
    seen.sort_unstable();
    assert_eq!(seen, vec![0, 1, 2, 3]);
  }

  #[test]
  fn nearest_neighbor_beats_a_scrambled_order() {
    // A ring of points fed in a deliberately criss-crossing order; NN should undo most of the crossing.
    let stops = pts(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0), (5.0, 5.0)]);
    let start = Point::new(0.0, 0.0);
    let nn = NearestNeighbor.order(&stops, start);
    assert!(tour_travel(&stops, start, &nn) <= identity_travel(&stops, start));
  }

  #[test]
  fn two_opt_strictly_improves_a_crossing_tour() {
    // The classic 2-opt win: four corners fed in an order that crosses the square's diagonals.
    let stops = pts(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)]);
    let start = Point::new(0.0, 0.0);
    let base = identity_travel(&stops, start);
    let improved = tour_travel(&stops, start, &TwoOpt::new().order(&stops, start));
    assert!(improved < base, "2-opt should shorten a crossing tour: {improved} !< {base}");
  }

  #[test]
  fn two_opt_never_worsens_the_seed() {
    // Over a scrambled grid, the improver must be monotone: never longer than the nearest-neighbour seed.
    let stops = pts(&[
      (0.0, 0.0), (3.0, 7.0), (8.0, 1.0), (2.0, 2.0), (9.0, 9.0), (1.0, 6.0), (6.0, 3.0), (4.0, 8.0),
    ]);
    let start = Point::new(0.0, 0.0);
    let seed = NearestNeighbor.order(&stops, start);
    let improved = TwoOpt::new().improve(&stops, start, &seed);
    assert!(tour_travel(&stops, start, &improved) <= tour_travel(&stops, start, &seed) + IMPROVE_EPS);
  }

  #[test]
  fn two_opt_beats_input_order_on_scrambled_points() {
    // Acceptance: a strict improvement over the input ordering on a scrambled set.
    let stops = pts(&[
      (0.0, 0.0), (10.0, 1.0), (1.0, 9.0), (9.0, 8.0), (2.0, 1.0), (8.0, 2.0), (1.0, 8.0), (9.0, 1.0), (5.0, 5.0),
    ]);
    let start = Point::new(0.0, 0.0);
    let base = identity_travel(&stops, start);
    let improved = tour_travel(&stops, start, &TwoOpt::new().order(&stops, start));
    assert!(improved < base, "expected strict improvement: {improved} !< {base}");
  }

  #[test]
  fn reversed_slot_endpoints_are_honoured_in_travel() {
    // A slot from (0,0)->(2,0) placed after a stop at (10,0): entering it reversed (at its (2,0) exit) is nearer,
    // so the optimizer should flip it and the tour cost should reflect the reversed endpoints.
    let stops = vec![Stop::point(Point::new(10.0, 0.0)), Stop::segment(Point::new(0.0, 0.0), Point::new(2.0, 0.0))];
    let start = Point::new(10.0, 0.0);
    let tour = TwoOpt::new().order(&stops, start);
    // The slot (index 1) should be visited reversed: arrive at (2,0), leave at (0,0).
    let slot = tour.iter().find(|r| r.index == 1).expect("slot in tour");
    assert!(slot.reversed, "nearer endpoint of the slot should become the entry");
  }

  #[test]
  fn or_opt_relocates_a_stranded_stop() {
    // Points along a line at x=0,1,2,3,4 but fed with the x=2 point last, forcing a back-and-forth the Or-opt
    // relocation should remove by slotting it into the middle.
    let stops = pts(&[(0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (4.0, 0.0), (2.0, 0.0)]);
    let start = Point::new(0.0, 0.0);
    let base = identity_travel(&stops, start);
    let improved = tour_travel(&stops, start, &TwoOpt::new().order(&stops, start));
    // Optimal is the straight sweep 0->1->2->3->4 with travel 4.0; the improver must reach it.
    assert!((improved - 4.0).abs() < 1e-6, "expected the straight sweep (4.0), got {improved} (base {base})");
  }

  #[test]
  fn empty_input_yields_empty_tour() {
    let stops: Vec<Stop> = Vec::new();
    assert!(NearestNeighbor.order(&stops, Point::new(0.0, 0.0)).is_empty());
    assert!(TwoOpt::new().order(&stops, Point::new(0.0, 0.0)).is_empty());
  }

  #[test]
  fn order_stops_bails_out_when_cancelled() {
    let stops = pts(&[(0.0, 0.0), (1.0, 0.0)]);
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(order_stops(&NearestNeighbor, &stops, Point::new(0.0, 0.0), &cancel).is_err());
  }
}
