//! Drilling — turn Excellon hits into an ordered, travel-optimized, per-tool operation plan.
//!
//! For each tool the hits are collected and ordered to minimize rapid travel (via [`crate::optimize`]), grouped so
//! tool changes are minimized (tools run in ascending number order and the tool position carries across groups).
//! Slots are carried as routed segments, not points. The per-tool drill parameters (depth, feed, retract, peck,
//! dwell) ride along as **data** — this crate produces the operation model, not the peck/drill-cycle G-code, which
//! is `eitri-gcode`'s job (Phase 4). See `docs/eitri-porting-plan.md` §7.5. Provenance: FlatCAM's
//! `CNCjob.generate_from_excellon`.

use std::collections::BTreeMap;

use eitri_core::{CancelToken, ProgressEvent, ProgressReporter, Result};
use eitri_excellon::{DrillHit, ExcellonImage};

use crate::optimize::{Point, Routed, Stop, TravelOptimizer, order_stops};

/// Machining parameters for a drill tool, carried through to Phase 4 as data. Depths/heights are millimetres,
/// feed is millimetres per minute, dwell is seconds. This crate does not act on them beyond attaching them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrillParams {
  /// Final cut depth below the work surface (millimetres, a positive magnitude).
  pub depth: f64,
  /// Plunge feed rate (millimetres per minute).
  pub feed: f64,
  /// Retract height above the work surface between hits (millimetres).
  pub retract: f64,
  /// Peck depth per plunge (millimetres); `None` drills in a single plunge.
  pub peck: Option<f64>,
  /// Dwell at the bottom of the hole (seconds); `None` for no dwell.
  pub dwell: Option<f64>,
}

impl Default for DrillParams {
  fn default() -> DrillParams {
    DrillParams { depth: 1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None }
  }
}

/// Configuration for a drilling operation: the default parameters, optional per-tool overrides (keyed by tool
/// number), and the machine position the tour starts from.
#[derive(Debug, Clone, Default)]
pub struct DrillConfig {
  /// Parameters applied to any tool without an override.
  pub defaults: DrillParams,
  /// Per-tool parameter overrides, keyed by Excellon tool number.
  pub overrides: BTreeMap<u32, DrillParams>,
  /// The starting machine position for travel optimization (millimetres).
  pub start: Point,
}

impl DrillConfig {
  /// The parameters that apply to `tool` — its override if present, else the defaults.
  fn params_for(&self, tool: u32) -> DrillParams {
    self.overrides.get(&tool).copied().unwrap_or(self.defaults)
  }
}

/// One move in a drill plan: a point drill, or a slot routed from `from` to `to`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrillMove {
  /// Plunge-drill at a point.
  Drill {
    /// Hole centre (millimetres).
    at: Point,
  },
  /// Plunge and route a slot along a segment.
  Slot {
    /// Segment start (millimetres).
    from: Point,
    /// Segment end (millimetres).
    to: Point,
  },
}

impl DrillMove {
  /// Where the tool leaves this move — a point for a drill, the far endpoint for a routed slot. Used to chain the
  /// running machine position from one move (and one tool group) to the next.
  fn exit(&self) -> Point {
    match self {
      DrillMove::Drill { at } => *at,
      DrillMove::Slot { to, .. } => *to,
    }
  }
}

/// The ordered drilling plan for one tool: its number, diameter, machining parameters, and travel-optimized moves.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolDrillPlan {
  /// Excellon tool number.
  pub tool: u32,
  /// Tool diameter (millimetres); `0.0` if the file referenced a tool it never defined.
  pub diameter: f64,
  /// Machining parameters attached to this tool (Phase 4 consumes these).
  pub params: DrillParams,
  /// The hits for this tool, ordered to minimize rapid travel.
  pub moves: Vec<DrillMove>,
}

/// A complete drilling plan: one [`ToolDrillPlan`] per tool that has hits, in ascending tool-number order so tool
/// changes are minimized.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct DrillPlan {
  /// Per-tool plans, ascending by tool number.
  pub tools: Vec<ToolDrillPlan>,
}

impl DrillPlan {
  /// Total number of moves across all tools.
  pub fn move_count(&self) -> usize {
    self.tools.iter().map(|t| t.moves.len()).sum()
  }
}

/// The kind of a hit, kept alongside its [`Stop`] so the optimizer's ordering can be mapped back to a
/// direction-aware [`DrillMove`].
#[derive(Debug, Clone, Copy)]
enum HitKind {
  Drill(Point),
  Slot(Point, Point),
}

/// Build an ordered, per-tool drill plan from `image`, ordering each tool's hits with `optimizer` to minimize
/// rapid travel. Tools run in ascending number order and the machine position carries from one tool group into the
/// next, so cross-tool travel is minimized too. Progress advances per hit; cancellation is polled per tool group.
pub fn plan_drilling<O: TravelOptimizer>(
  image: &ExcellonImage,
  config: &DrillConfig,
  optimizer: &O,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<DrillPlan> {
  progress.emit(ProgressEvent::Started { label: "drilling".to_string() });

  // Group hits by tool number. `BTreeMap` keeps tools ascending, which is the tool-change-minimizing order.
  let mut by_tool: BTreeMap<u32, Vec<HitKind>> = BTreeMap::new();
  for hit in &image.hits {
    let kind = match *hit {
      DrillHit::Drill { x, y, .. } => HitKind::Drill(Point::new(x, y)),
      DrillHit::Slot { start, end, .. } => HitKind::Slot(Point::new(start.0, start.1), Point::new(end.0, end.1)),
    };
    by_tool.entry(hit.tool()).or_default().push(kind);
  }

  let total = image.hits.len() as u64;
  let mut done = 0u64;
  let mut cursor = config.start;
  let mut tools = Vec::with_capacity(by_tool.len());

  for (tool, kinds) in by_tool {
    cancel.check()?;
    let stops: Vec<Stop> = kinds
      .iter()
      .map(|k| match k {
        HitKind::Drill(p) => Stop::point(*p),
        HitKind::Slot(a, b) => Stop::segment(*a, *b),
      })
      .collect();

    let tour = order_stops(optimizer, &stops, cursor, cancel)?;
    let mut moves = Vec::with_capacity(tour.len());
    for Routed { index, reversed } in &tour {
      let mv = match kinds[*index] {
        HitKind::Drill(p) => DrillMove::Drill { at: p },
        HitKind::Slot(a, b) => {
          if *reversed { DrillMove::Slot { from: b, to: a } } else { DrillMove::Slot { from: a, to: b } }
        }
      };
      moves.push(mv);
    }
    if let Some(last) = moves.last() {
      cursor = last.exit();
    }

    done += kinds.len() as u64;
    progress.advance(done, total);

    let diameter = image.tools.get(&tool).map(|t| t.diameter).unwrap_or(0.0);
    tools.push(ToolDrillPlan { tool, diameter, params: config.params_for(tool), moves });
  }

  progress.emit(ProgressEvent::Finished);
  Ok(DrillPlan { tools })
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::optimize::{NearestNeighbor, TwoOpt};
  use eitri_excellon::Tool;

  /// A pass-through optimizer that keeps the input order — a trivial second impl that proves the trait seam.
  struct IdentityOrder;
  impl TravelOptimizer for IdentityOrder {
    fn order(&self, stops: &[Stop], _start: Point) -> Vec<Routed> {
      (0..stops.len()).map(|index| Routed { index, reversed: false }).collect()
    }
  }

  fn image(tools: &[(u32, f64)], hits: Vec<DrillHit>) -> ExcellonImage {
    use eitri_excellon::{NumberFormat};
    let mut table = BTreeMap::new();
    for &(n, d) in tools {
      table.insert(n, Tool { diameter: d });
    }
    ExcellonImage { tools: table, hits, format: NumberFormat::default_for(eitri_core::Unit::Millimeters) }
  }

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  /// Total rapid travel of a whole plan, from `start` through each move's endpoints, for test assertions.
  fn plan_travel(plan: &DrillPlan, start: Point) -> f64 {
    let mut total = 0.0;
    let mut cursor = start;
    for tool in &plan.tools {
      for mv in &tool.moves {
        let (entry, exit) = match mv {
          DrillMove::Drill { at } => (*at, *at),
          DrillMove::Slot { from, to } => (*from, *to),
        };
        total += cursor.distance_to(entry);
        cursor = exit;
      }
    }
    total
  }

  #[test]
  fn hits_are_grouped_by_tool_in_ascending_order() {
    let img = image(
      &[(1, 0.8), (2, 1.2)],
      vec![
        DrillHit::Drill { tool: 2, x: 0.0, y: 0.0 },
        DrillHit::Drill { tool: 1, x: 1.0, y: 0.0 },
        DrillHit::Drill { tool: 2, x: 2.0, y: 0.0 },
        DrillHit::Drill { tool: 1, x: 3.0, y: 0.0 },
      ],
    );
    let (p, c) = silent();
    let plan = plan_drilling(&img, &DrillConfig::default(), &NearestNeighbor, &p, &c).expect("plan");
    assert_eq!(plan.tools.len(), 2);
    assert_eq!(plan.tools[0].tool, 1, "tool 1 group first");
    assert_eq!(plan.tools[1].tool, 2, "tool 2 group second");
    assert_eq!(plan.tools[0].moves.len(), 2);
    assert_eq!(plan.tools[1].moves.len(), 2);
    assert!((plan.tools[0].diameter - 0.8).abs() < 1e-12);
    assert!((plan.tools[1].diameter - 1.2).abs() < 1e-12);
  }

  #[test]
  fn a_slot_is_carried_as_a_route_segment() {
    let img = image(&[(1, 1.0)], vec![DrillHit::Slot { tool: 1, start: (0.0, 0.0), end: (5.0, 0.0) }]);
    let (p, c) = silent();
    let plan = plan_drilling(&img, &DrillConfig::default(), &NearestNeighbor, &p, &c).expect("plan");
    match plan.tools[0].moves[0] {
      DrillMove::Slot { from, to } => {
        assert_eq!(from, Point::new(0.0, 0.0));
        assert_eq!(to, Point::new(5.0, 0.0));
      }
      other => panic!("expected a slot move, got {other:?}"),
    }
  }

  #[test]
  fn params_attach_defaults_and_overrides() {
    let img = image(&[(1, 0.8), (2, 1.2)], vec![
      DrillHit::Drill { tool: 1, x: 0.0, y: 0.0 },
      DrillHit::Drill { tool: 2, x: 1.0, y: 0.0 },
    ]);
    let mut overrides = BTreeMap::new();
    overrides.insert(2, DrillParams { depth: 3.0, feed: 250.0, retract: 5.0, peck: Some(0.5), dwell: Some(0.2) });
    let config = DrillConfig { defaults: DrillParams::default(), overrides, start: Point::new(0.0, 0.0) };
    let (p, c) = silent();
    let plan = plan_drilling(&img, &config, &NearestNeighbor, &p, &c).expect("plan");
    assert_eq!(plan.tools[0].params, DrillParams::default(), "tool 1 uses defaults");
    assert_eq!(plan.tools[1].params.peck, Some(0.5), "tool 2 uses its override");
    assert!((plan.tools[1].params.feed - 250.0).abs() < 1e-12);
  }

  #[test]
  fn two_opt_ordering_beats_the_input_order_on_scrambled_hits() {
    // A single tool with hits fed in a deliberately criss-crossing order.
    let img = image(&[(1, 0.8)], vec![
      DrillHit::Drill { tool: 1, x: 0.0, y: 0.0 },
      DrillHit::Drill { tool: 1, x: 10.0, y: 1.0 },
      DrillHit::Drill { tool: 1, x: 1.0, y: 9.0 },
      DrillHit::Drill { tool: 1, x: 9.0, y: 8.0 },
      DrillHit::Drill { tool: 1, x: 2.0, y: 1.0 },
      DrillHit::Drill { tool: 1, x: 8.0, y: 2.0 },
      DrillHit::Drill { tool: 1, x: 1.0, y: 8.0 },
      DrillHit::Drill { tool: 1, x: 9.0, y: 1.0 },
    ]);
    let start = Point::new(0.0, 0.0);
    let config = DrillConfig { start, ..Default::default() };
    let (p, c) = silent();
    let identity = plan_drilling(&img, &config, &IdentityOrder, &p, &c).expect("identity");
    let optimized = plan_drilling(&img, &config, &TwoOpt::new(), &p, &c).expect("optimized");
    let base = plan_travel(&identity, start);
    let improved = plan_travel(&optimized, start);
    assert!(improved < base, "2-opt drill order should strictly beat input order: {improved} !< {base}");
  }

  #[test]
  fn the_optimizer_is_swappable() {
    // The identity impl proves the seam: the same operation runs against any TravelOptimizer without change.
    let img = image(&[(1, 0.8)], vec![
      DrillHit::Drill { tool: 1, x: 0.0, y: 0.0 },
      DrillHit::Drill { tool: 1, x: 5.0, y: 0.0 },
      DrillHit::Drill { tool: 1, x: 2.0, y: 0.0 },
    ]);
    let (p, c) = silent();
    let plan = plan_drilling(&img, &DrillConfig::default(), &IdentityOrder, &p, &c).expect("plan");
    let xs: Vec<f64> = plan.tools[0].moves.iter().map(|m| match m {
      DrillMove::Drill { at } => at.x,
      DrillMove::Slot { from, .. } => from.x,
    }).collect();
    assert_eq!(xs, vec![0.0, 5.0, 2.0], "identity keeps input order");
  }

  #[test]
  fn empty_image_yields_empty_plan() {
    let img = image(&[(1, 0.8)], Vec::new());
    let (p, c) = silent();
    let plan = plan_drilling(&img, &DrillConfig::default(), &NearestNeighbor, &p, &c).expect("plan");
    assert!(plan.tools.is_empty());
    assert_eq!(plan.move_count(), 0);
  }

  #[test]
  fn cancelled_drilling_returns_error() {
    let img = image(&[(1, 0.8)], vec![DrillHit::Drill { tool: 1, x: 0.0, y: 0.0 }]);
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(plan_drilling(&img, &DrillConfig::default(), &NearestNeighbor, &ProgressReporter::silent(), &cancel)
      .is_err());
  }
}
