//! The Rhai binding over the typed [`Session`] command API.
//!
//! This layer is deliberately thin: every command is a one-line adapter that borrows the shared session, calls the
//! matching [`Session`] method, and maps its [`ScriptError`] onto a Rhai runtime error. All correctness lives in
//! [`crate::session`]; nothing here makes a CAM decision, so a bug is either in the (separately tested) engine or in
//! a trivial argument shuffle.
//!
//! ## Shape
//!
//! Rhai custom types must be `Clone`, and a [`Session`] is not (its postprocessor registry holds trait objects), so
//! the session is shared through a [`ScriptSession`] newtype over `Rc<RefCell<Session>>`. Scripts address it as a
//! variable (`cam` by convention) and call methods on it; the caller keeps a clone of the handle and reads the
//! mutated session back through the shared cell after the script runs.
//!
//! ## Ergonomics
//!
//! Object ids cross into scripts as plain integers (Rhai's `INT`). Command parameters are built by typed constructor
//! functions (`isolation_spec`, `cut_job`, …) rather than free-form maps: they mirror the Rust `*Spec` types exactly,
//! validate in one place (a bad milling-direction name is a clean script error, not a silent default), and keep each
//! CAM-op method to two or three arguments. Enumerations that would otherwise need their own constructors — milling
//! direction, fill strategy, mirror axis — are passed as strings and parsed centrally.
//!
//! ## Cancellation
//!
//! [`eval`] wires the engine's operation callback to the session's [`eitri_core::CancelToken`], so cancelling the
//! token (from a UI or another thread) aborts the running script at the next operation boundary — the Rhai idiom for
//! stopping a long script.

use std::cell::{Ref, RefCell, RefMut};
use std::rc::Rc;

use rhai::{Dynamic, Engine, EvalAltResult, Position, Scope};

use eitri_project::{
  BoundarySpec, CutoutOutlineSpec, CutoutSpec, DirectionSpec, DrillSpec, IsolationSpec, MirrorLineSpec, NonCopperSpec,
  ObjectId, PaintSpec, PaintStrategySpec, PanelizeSpec, SpacingSpec, TabPlacementSpec,
};
use eitri_gcode::{DrillJob, IsolationJob};
use geo_types::Coord;

use crate::error::{Result, ScriptError};
use crate::session::Session;

/// A shared, script-addressable handle to a [`Session`]. Cloning shares the same underlying session (an `Rc` bump),
/// so the caller can hand one clone to a script and read the mutations back through another.
#[derive(Clone)]
pub struct ScriptSession(Rc<RefCell<Session>>);

impl ScriptSession {
  /// Wrap a session for scripting.
  pub fn new(session: Session) -> ScriptSession {
    ScriptSession(Rc::new(RefCell::new(session)))
  }

  /// Immutably borrow the underlying session (to inspect results after a script runs).
  pub fn borrow(&self) -> Ref<'_, Session> {
    self.0.borrow()
  }

  /// Mutably borrow the underlying session.
  pub fn borrow_mut(&self) -> RefMut<'_, Session> {
    self.0.borrow_mut()
  }
}

/// Map a command [`Result`] onto a Rhai runtime error so a failure surfaces as a catchable script error, never a
/// panic.
fn rhai<T>(result: Result<T>) -> std::result::Result<T, Box<EvalAltResult>> {
  result.map_err(|e| Box::new(EvalAltResult::ErrorRuntime(e.to_string().into(), Position::NONE)))
}

/// Turn a script integer into an [`ObjectId`], rejecting negatives cleanly.
fn oid(id: i64) -> Result<ObjectId> {
  if id < 0 {
    return Err(ScriptError::InvalidArgument(format!("object id must be non-negative, got {id}")));
  }
  Ok(ObjectId(id as u64))
}

/// Parse a milling-direction word.
fn direction(word: &str) -> Result<DirectionSpec> {
  match word.to_ascii_lowercase().as_str() {
    "climb" => Ok(DirectionSpec::Climb),
    "conventional" | "conv" => Ok(DirectionSpec::Conventional),
    other => Err(ScriptError::InvalidArgument(format!(
      "unknown milling direction '{other}' (use 'climb' or 'conventional')"
    ))),
  }
}

/// Parse a paint fill-strategy word (raster defaults to a 0° scan angle; use `paint_spec_raster` for an angle).
fn strategy(word: &str) -> Result<PaintStrategySpec> {
  match word.to_ascii_lowercase().as_str() {
    "concentric" => Ok(PaintStrategySpec::Concentric),
    "seed" => Ok(PaintStrategySpec::Seed),
    "raster" => Ok(PaintStrategySpec::Raster { angle_deg: 0.0 }),
    other => Err(ScriptError::InvalidArgument(format!(
      "unknown paint strategy '{other}' (use 'concentric', 'seed' or 'raster')"
    ))),
  }
}

/// Parse a mirror-axis word into a [`MirrorLineSpec`] at `value`.
fn mirror_line(axis: &str, value: f64) -> Result<MirrorLineSpec> {
  match axis.to_ascii_lowercase().as_str() {
    "vertical" | "x" => Ok(MirrorLineSpec::Vertical(value)),
    "horizontal" | "y" => Ok(MirrorLineSpec::Horizontal(value)),
    other => Err(ScriptError::InvalidArgument(format!(
      "unknown mirror axis '{other}' (use 'vertical' or 'horizontal')"
    ))),
  }
}

/// Build a fully-registered Rhai engine exposing the [`Session`] command surface. Cancellation is **not** wired here
/// (it needs a specific session's token); use [`eval`] for the common path, or call [`Engine::on_progress`] yourself.
pub fn new_engine() -> Engine {
  let mut engine = Engine::new();

  // Register the custom types by name so runtime type errors read legibly.
  engine.register_type_with_name::<ScriptSession>("Session");
  engine.register_type_with_name::<IsolationSpec>("IsolationSpec");
  engine.register_type_with_name::<DrillSpec>("DrillSpec");
  engine.register_type_with_name::<PaintSpec>("PaintSpec");
  engine.register_type_with_name::<NonCopperSpec>("NonCopperSpec");
  engine.register_type_with_name::<CutoutSpec>("CutoutSpec");
  engine.register_type_with_name::<PanelizeSpec>("PanelizeSpec");
  engine.register_type_with_name::<IsolationJob>("CutJob");
  engine.register_type_with_name::<DrillJob>("DrillJob");

  register_spec_builders(&mut engine);
  register_open_import(&mut engine);
  register_cam_ops(&mut engine);
  register_output_and_management(&mut engine);

  engine
}

/// Register the parameter-builder functions that construct the typed `*Spec`/job values.
fn register_spec_builders(engine: &mut Engine) {
  engine.register_fn(
    "isolation_spec",
    |tool_diameter: f64, passes: i64, overlap: f64, combine: bool, dir: &str| {
      rhai(direction(dir).map(|direction| IsolationSpec {
        tool_diameter,
        passes: passes.max(0) as usize,
        overlap,
        combine,
        direction,
      }))
    },
  );

  engine.register_fn("drill_spec", |depth: f64, feed: f64, retract: f64| DrillSpec {
    depth,
    feed,
    retract,
    peck: None,
    dwell: None,
  });

  engine.register_fn(
    "paint_spec",
    |tool_diameter: f64, overlap: f64, margin: f64, dir: &str, finish_pass: bool, strat: &str| {
      rhai((|| {
        Ok(PaintSpec {
          tool_diameter,
          overlap,
          margin,
          direction: direction(dir)?,
          finish_pass,
          strategy: strategy(strat)?,
        })
      })())
    },
  );

  engine.register_fn("noncopper_bbox_spec", |margin: f64, paint: PaintSpec| NonCopperSpec {
    boundary: BoundarySpec::BoundingBox { margin },
    paint,
  });

  engine.register_fn(
    "cutout_rect_spec",
    |tool_diameter: f64, tab_width: f64, tabs: i64, margin: f64, dir: &str, min_x: f64, min_y: f64, max_x: f64,
     max_y: f64| {
      rhai(direction(dir).map(|direction| CutoutSpec {
        tool_diameter,
        tab_width,
        tabs: TabPlacementSpec::Count(tabs.max(0) as usize),
        margin,
        direction,
        outline: CutoutOutlineSpec::Rectangle {
          min: Coord { x: min_x, y: min_y },
          max: Coord { x: max_x, y: max_y },
        },
      }))
    },
  );

  engine.register_fn("panelize_gap_spec", |rows: i64, cols: i64, x_gap: f64, y_gap: f64| PanelizeSpec {
    rows: rows.max(1) as usize,
    cols: cols.max(1) as usize,
    x: SpacingSpec::Gap(x_gap),
    y: SpacingSpec::Gap(y_gap),
  });

  engine.register_fn(
    "cut_job",
    |cut_depth: f64, pass_depth: f64, cut_feed: f64, plunge_feed: f64, travel_z: f64, spindle_rpm: f64| {
      IsolationJob { cut_depth, pass_depth, cut_feed, plunge_feed, travel_z, spindle_rpm, name: None }
    },
  );

  engine.register_fn("drill_job", |travel_z: f64, spindle_rpm: f64| DrillJob {
    travel_z,
    spindle_rpm,
    name: None,
  });
}

/// Register the open/import commands.
fn register_open_import(engine: &mut Engine) {
  engine.register_fn("open_gerber_str", |cam: &mut ScriptSession, name: &str, source: &str| {
    rhai(cam.borrow_mut().open_gerber_str(name, source)).map(|id| id.0 as i64)
  });
  engine.register_fn("open_excellon_str", |cam: &mut ScriptSession, name: &str, source: &str| {
    rhai(cam.borrow_mut().open_excellon_str(name, source)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_svg_str", |cam: &mut ScriptSession, name: &str, source: &str| {
    rhai(cam.borrow_mut().import_svg_str(name, source)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_dxf_str", |cam: &mut ScriptSession, name: &str, source: &str| {
    rhai(cam.borrow_mut().import_dxf_str(name, source)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_gcode_str", |cam: &mut ScriptSession, name: &str, source: &str| {
    rhai(cam.borrow_mut().import_gcode_str(name, source)).map(|id| id.0 as i64)
  });

  // Path convenience wrappers.
  engine.register_fn("open_gerber", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow_mut().open_gerber(path)).map(|id| id.0 as i64)
  });
  engine.register_fn("open_excellon", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow_mut().open_excellon(path)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_svg", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow_mut().import_svg(path)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_dxf", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow_mut().import_dxf(path)).map(|id| id.0 as i64)
  });
  engine.register_fn("import_gcode", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow_mut().import_gcode(path)).map(|id| id.0 as i64)
  });
}

/// Register the CAM operations.
fn register_cam_ops(engine: &mut Engine) {
  engine.register_fn("isolate", |cam: &mut ScriptSession, source: i64, spec: IsolationSpec, job: IsolationJob| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().isolate(id, spec, job)).map(|id| id.0 as i64)
  });
  engine.register_fn("drill", |cam: &mut ScriptSession, source: i64, spec: DrillSpec, job: DrillJob| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().drill(id, spec, job)).map(|id| id.0 as i64)
  });
  engine.register_fn("paint", |cam: &mut ScriptSession, source: i64, spec: PaintSpec, job: IsolationJob| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().paint(id, spec, job)).map(|id| id.0 as i64)
  });
  engine.register_fn("noncopper", |cam: &mut ScriptSession, source: i64, spec: NonCopperSpec, job: IsolationJob| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().noncopper(id, spec, job)).map(|id| id.0 as i64)
  });
  engine.register_fn("cutout", |cam: &mut ScriptSession, spec: CutoutSpec, job: IsolationJob| {
    rhai(cam.borrow_mut().cutout(spec, job)).map(|id| id.0 as i64)
  });
  engine.register_fn("panelize", |cam: &mut ScriptSession, source: i64, spec: PanelizeSpec| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().panelize(id, spec)).map(|id| id.0 as i64)
  });
  engine.register_fn("mirror", |cam: &mut ScriptSession, source: i64, axis: &str, value: f64| {
    let id = rhai(oid(source))?;
    let line = rhai(mirror_line(axis, value))?;
    rhai(cam.borrow_mut().mirror(id, line)).map(|id| id.0 as i64)
  });
  engine.register_fn("translate", |cam: &mut ScriptSession, source: i64, dx: f64, dy: f64| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().translate(id, dx, dy)).map(|id| id.0 as i64)
  });
  engine.register_fn("scale", |cam: &mut ScriptSession, source: i64, sx: f64, sy: f64| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().scale(id, sx, sy)).map(|id| id.0 as i64)
  });
  engine.register_fn("rotate", |cam: &mut ScriptSession, source: i64, degrees: f64| {
    let id = rhai(oid(source))?;
    rhai(cam.borrow_mut().rotate(id, degrees)).map(|id| id.0 as i64)
  });
}

/// Register G-code output, object management, persistence, and dialect selection.
fn register_output_and_management(engine: &mut Engine) {
  engine.register_fn("write_gcode", |cam: &mut ScriptSession, job: i64| {
    let id = rhai(oid(job))?;
    rhai(cam.borrow().write_gcode(id))
  });
  engine.register_fn("write_gcode_to", |cam: &mut ScriptSession, job: i64, path: &str| {
    let id = rhai(oid(job))?;
    rhai(cam.borrow().write_gcode_to(id, path))
  });

  engine.register_fn("set_dialect", |cam: &mut ScriptSession, name: &str| rhai(cam.borrow_mut().set_dialect(name)));

  engine.register_fn("object_count", |cam: &mut ScriptSession| cam.borrow().len() as i64);
  engine.register_fn("object_name", |cam: &mut ScriptSession, id: i64| {
    let id = rhai(oid(id))?;
    rhai(cam.borrow().object_name(id))
  });
  engine.register_fn("id_of", |cam: &mut ScriptSession, name: &str| {
    cam.borrow().id_of(name).map(|id| id.0 as i64).ok_or_else(|| {
      Box::new(EvalAltResult::ErrorRuntime(format!("no object named '{name}'").into(), Position::NONE))
    })
  });
  engine.register_fn("rename", |cam: &mut ScriptSession, id: i64, name: &str| {
    let id = rhai(oid(id))?;
    rhai(cam.borrow_mut().rename(id, name))
  });
  engine.register_fn("delete", |cam: &mut ScriptSession, id: i64| {
    let id = rhai(oid(id))?;
    rhai(cam.borrow_mut().delete(id))
  });
  engine.register_fn("undo", |cam: &mut ScriptSession| cam.borrow_mut().undo());
  engine.register_fn("redo", |cam: &mut ScriptSession| cam.borrow_mut().redo());

  engine.register_fn("save_project", |cam: &mut ScriptSession| rhai(cam.borrow().save_project()));
  engine.register_fn("save_project_to", |cam: &mut ScriptSession, path: &str| {
    rhai(cam.borrow().save_project_to(path))
  });
  engine.register_fn("save_tool_db", |cam: &mut ScriptSession| rhai(cam.borrow().save_tool_db()));
  engine.register_fn("load_tool_db", |cam: &mut ScriptSession, json: &str| rhai(cam.borrow_mut().load_tool_db(json)));
}

/// Run `script` against `session`, wiring the engine's operation callback to the session's cancellation token so a
/// cancel aborts the running script. The session is exposed to the script as the variable `cam`. Returns the script's
/// final value.
pub fn eval(session: &ScriptSession, script: &str) -> std::result::Result<Dynamic, Box<EvalAltResult>> {
  let mut engine = new_engine();
  let cancel = session.borrow().cancel_token();
  // Called at every operation boundary; returning `Some(..)` aborts the script with `ErrorTerminated`.
  engine.on_progress(move |_ops| if cancel.is_cancelled() { Some(Dynamic::UNIT) } else { None });

  let mut scope = Scope::new();
  scope.push("cam", session.clone());
  engine.eval_with_scope::<Dynamic>(&mut scope, script)
}
