//! End-to-end example: turn the `starter-*` KiCad fixtures into a complete, controller-ready CAM job.
//!
//! This drives the whole Eitri pipeline through the public [`Session`] API — the same surface the Rhai console and
//! the egui front-end sit on — against the hand-authored fixtures in `eitri/fixtures/`. It produces the five jobs a
//! real two-sided PCB needs and writes each as a grbl-conformant G-code file, plus the saved project JSON:
//!
//! 1. **Front-copper isolation** — `starter-F_Cu.gbr` isolated with a 0.2 mm tool.
//! 2. **Back-copper isolation** — `starter-B_Cu.gbr` mirrored about the board centre-line (the two-sided flip) and
//!    isolated, so the bottom side machines correctly after a left-right flip on the fixture.
//! 3. **Plated drilling** — `starter-PTH.drl` drilled with a travel-optimised plan.
//! 4. **Non-plated drilling** — `starter-NPTH.drl` (mounting holes) drilled the same way.
//! 5. **Board cutout** — a tabbed profile routed around the `starter-Edge_Cuts.gbr` outline (the rectangle is derived
//!    from the parsed outline's bounds, so the cut tracks the real board edge).
//!
//! Run it from the eitri workspace:
//!
//! ```sh
//! cargo run -p eitri-script --example starter_board            # writes into eitri/target/starter-example/
//! cargo run -p eitri-script --example starter_board -- /tmp/out
//! ```

use std::error::Error;
use std::path::{Path, PathBuf};

use eitri_core::{CancelToken, ProgressReporter};
use eitri_gcode::{DrillJob, IsolationJob, check_grbl_conformance};
use eitri_geo::bounds;
use eitri_gerber::parse_gerber;
use eitri_project::{
  CutoutOutlineSpec, CutoutSpec, DirectionSpec, DrillSpec, IsolationSpec, MirrorLineSpec, ObjectId, TabPlacementSpec,
};
use eitri_script::Session;
use geo_types::Coord;

/// The five toolpath jobs this example emits, paired with the file each is written to.
struct Emitted {
  job: ObjectId,
  filename: &'static str,
}

fn main() -> Result<(), Box<dyn Error>> {
  let fixtures = fixtures_dir();
  let out_dir = output_dir();
  std::fs::create_dir_all(&out_dir)?;

  println!("Eitri starter-board example");
  println!("  fixtures: {}", fixtures.display());
  println!("  output:   {}\n", out_dir.display());

  // One session holds the whole project: every opened input and every emitted job, all under undo, all persistable.
  let mut session = Session::new("starter");

  // --- Stock & datum (work-zero) ------------------------------------------------------------------------------------
  // Open the board outline and fit the stock to it — a 1.6 mm FR-4 blank the size of the board. That seeds a
  // bottom-left / top-surface datum, so every emitted program is posted relative to the board's bottom-left corner
  // (grbl coordinates land ON the board, near 0,0, not in KiCad's page frame). On the machine you zero there.
  let edge = session.open_gerber(fixtures.join("gerber/starter-Edge_Cuts.gbr"))?;
  session.fit_stock_to(edge, 1.6, false)?;
  let (board_min, board_max) = board_outline_bounds(&fixtures)?;
  let (dx, dy, dz) = session.work_origin();
  println!(
    "  stock:    {:.1} x {:.1} x 1.6 mm; datum bottom-left/top -> native ({dx:.3}, {dy:.3}, {dz:.3})\n",
    board_max.x - board_min.x,
    board_max.y - board_min.y
  );

  // --- Front copper -------------------------------------------------------------------------------------------------
  // Open the top-copper Gerber and isolate it with a fine V-bit. A single 0.2 mm pass traces every copper island; the
  // shipped `grbl` dialect is the default, so the emitted job is Skirnir-ready with no extra wiring.
  let f_cu = session.open_gerber(fixtures.join("gerber/starter-F_Cu.gbr"))?;
  let front = session.isolate(f_cu, isolation_spec(), copper_job("front copper"))?;

  // --- Back copper (two-sided flip) ---------------------------------------------------------------------------------
  // The board's outline gives us the centre-line to mirror the bottom copper about. Milling a two-sided board means
  // flipping the stock left-to-right on the fixture, so the bottom artwork must be mirrored in X first; isolating the
  // mirrored geometry then cuts correctly once the board is flipped.
  let centre_x = 0.5 * (board_min.x + board_max.x);
  let b_cu = session.open_gerber(fixtures.join("gerber/starter-B_Cu.gbr"))?;
  let b_cu_flipped = session.mirror(b_cu, MirrorLineSpec::Vertical(centre_x))?;
  let back = session.isolate(b_cu_flipped, isolation_spec(), copper_job("back copper (mirrored)"))?;

  // --- Drilling -----------------------------------------------------------------------------------------------------
  // Plated and non-plated holes are separate Excellon files; each becomes its own travel-optimised drilling program.
  // (This starter board's NPTH file is a valid but empty drill file — no non-plated holes — so its program has no
  // hits; the pipeline handles that gracefully and the report flags it.)
  let pth = session.open_excellon(fixtures.join("excellon/starter-PTH.drl"))?;
  let pth_job = session.drill(pth, drill_spec(), drill_job("plated holes"))?;
  let npth = session.open_excellon(fixtures.join("excellon/starter-NPTH.drl"))?;
  let npth_job = session.drill(npth, drill_spec(), drill_job("non-plated holes"))?;

  // --- Board cutout -------------------------------------------------------------------------------------------------
  // Route the profile around the real board rectangle, leaving four holding tabs so the part stays put until it is
  // snapped out. The cut goes the full board thickness in shallow passes.
  let cutout = session.cutout(cutout_spec(board_min, board_max), cutout_job())?;

  // --- Write G-code + save the project ------------------------------------------------------------------------------
  let emitted = [
    Emitted { job: front, filename: "1-front-copper.gcode" },
    Emitted { job: back, filename: "2-back-copper.gcode" },
    Emitted { job: pth_job, filename: "3-drill-plated.gcode" },
    Emitted { job: npth_job, filename: "4-drill-nonplated.gcode" },
    Emitted { job: cutout, filename: "5-board-cutout.gcode" },
  ];

  println!("Generated jobs:");
  for e in &emitted {
    let gcode = session.write_gcode(e.job)?;
    let path = out_dir.join(e.filename);
    session.write_gcode_to(e.job, &path)?;
    report(&session, e, &gcode);
  }

  let project_path = out_dir.join("starter.eitri.json");
  session.save_project_to(&project_path)?;
  println!("\nProject ({} objects) saved to {}", session.len(), project_path.display());
  println!("Done — {} G-code files written to {}", emitted.len(), out_dir.display());
  Ok(())
}

/// Print one line per emitted job: its name, line count, and grbl conformance verdict — failing loudly if the shipped
/// postprocessor ever emits something the Skirnir/grblHAL contract rejects.
fn report(session: &Session, e: &Emitted, gcode: &str) {
  let name = session.object_name(e.job).unwrap_or_else(|_| "?".to_string());
  let lines = gcode.lines().count();
  let violations = check_grbl_conformance(gcode);
  let verdict = if violations.is_empty() {
    "grbl-conformant".to_string()
  } else {
    format!("{} CONFORMANCE VIOLATION(S)", violations.len())
  };
  // A program with no `G1` has no cutting/plunging moves (e.g. an empty drill file) — call that out so an empty
  // output never reads as a silent failure.
  let note = if gcode.contains("G1 ") { "" } else { "  (no cutting moves)" };
  println!("  {:<30} {:>4} lines  [{}]  -> {}{}", name, lines, verdict, e.filename, note);
}

/// A 0.2 mm single-pass isolation, climb-milled with same-pass rings combined so shared boundary is not retraced.
fn isolation_spec() -> IsolationSpec {
  IsolationSpec { tool_diameter: 0.2, passes: 1, overlap: 0.0, combine: true, direction: DirectionSpec::Climb }
}

/// Copper-isolation cutting parameters: a shallow single-pass cut just through the copper, with a named header.
fn copper_job(name: &str) -> IsolationJob {
  IsolationJob {
    cut_depth: 0.15,
    pass_depth: 0.15,
    cut_feed: 120.0,
    plunge_feed: 60.0,
    travel_z: 2.0,
    spindle_rpm: 10_000.0,
    name: Some(name.to_string()),
  }
}

/// A drill through a 1.6 mm board with 0.6 mm pecking to clear chips; retract to 2 mm between hits. `depth` is a
/// positive magnitude below the surface: the emitter negates it to `Z-1.8` (and the peck cycle needs it positive).
fn drill_spec() -> DrillSpec {
  DrillSpec { depth: 1.8, feed: 100.0, retract: 2.0, peck: Some(0.6), dwell: None }
}

/// Cross-tool drilling settings: a 3 mm rapid height and the spindle speed, with a named header.
fn drill_job(name: &str) -> DrillJob {
  DrillJob { travel_z: 3.0, spindle_rpm: 10_000.0, name: Some(name.to_string()) }
}

/// A 1 mm end-mill profile with four 3 mm holding tabs, cut conventionally around the board rectangle.
fn cutout_spec(min: Coord<f64>, max: Coord<f64>) -> CutoutSpec {
  CutoutSpec {
    tool_diameter: 1.0,
    tab_width: 3.0,
    tabs: TabPlacementSpec::Count(4),
    margin: 0.0,
    direction: DirectionSpec::Conventional,
    outline: CutoutOutlineSpec::Rectangle { min, max },
  }
}

/// Route the full board thickness in 0.6 mm passes, lifting well clear between the tab gaps.
fn cutout_job() -> IsolationJob {
  IsolationJob {
    cut_depth: 1.7,
    pass_depth: 0.6,
    cut_feed: 200.0,
    plunge_feed: 80.0,
    travel_z: 3.0,
    spindle_rpm: 10_000.0,
    name: Some("board cutout".to_string()),
  }
}

/// Parse the `Edge_Cuts` outline Gerber and return its `(min, max)` bounds — the real board rectangle the cutout and
/// the two-sided mirror centre-line are both derived from.
fn board_outline_bounds(fixtures: &Path) -> Result<(Coord<f64>, Coord<f64>), Box<dyn Error>> {
  let source = std::fs::read_to_string(fixtures.join("gerber/starter-Edge_Cuts.gbr"))?;
  let image = parse_gerber(&source, &ProgressReporter::silent(), &CancelToken::new())?;
  let (min_x, min_y, max_x, max_y) =
    bounds(&image.copper).ok_or("Edge_Cuts outline produced no geometry")?;
  Ok((Coord { x: min_x, y: min_y }, Coord { x: max_x, y: max_y }))
}

/// The repo's `eitri/fixtures/` directory, resolved relative to this crate so the example runs from anywhere.
fn fixtures_dir() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures")
}

/// The output directory: the first CLI argument, or `eitri/target/starter-example/` by default.
fn output_dir() -> PathBuf {
  match std::env::args().nth(1) {
    Some(arg) => PathBuf::from(arg),
    None => Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/starter-example"),
  }
}
