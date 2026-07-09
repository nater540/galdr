//! [`Session`] — the typed Rust command API over the whole Eitri engine.
//!
//! This is the load-bearing interface the plan (§10) calls for: a plain, fully unit-testable Rust surface that opens
//! and imports fabrication input, runs every CAM operation, writes G-code, manages the object collection with undo,
//! persists projects, and reads the tool database — with **no** dependency on any scripting runtime. The Rhai binding
//! ([`crate::bindings`]) is a thin adapter over exactly these methods; everything correctness-critical lives here so
//! it can be tested directly in Rust.
//!
//! ## Shape
//!
//! A session owns an undo-tracked [`ObjectCollection`] (via [`History`]), a [`ToolDatabase`], a postprocessor
//! [`Registry`], and the default geometry backend, plus the framework-agnostic [`ProgressReporter`]/[`CancelToken`]
//! seams every long command threads. Commands come in three shapes:
//! - **Open / import** eagerly parse their input so the object is immediately usable by a CAM op, and cache the
//!   parsed image alongside the persisted source.
//! - **CAM ops** read a source object's geometry, run the (cancellable) operation, and commit the result as a new
//!   object — a [`CncJobObject`] for the toolpath ops (isolate/drill/paint/non-copper/cutout), a [`GeometryObject`]
//!   for the geometry-producing ops (panelize/mirror/transform).
//! - **Object management / persistence / tool DB** mutate or read the collection and its neighbours.
//!
//! ## File I/O boundary
//!
//! The command core is **source-string based** and pure: `open_gerber_str`, `write_gcode`, `save_project`, etc. take
//! and return strings and never touch the filesystem, which is what makes the whole API unit-testable without a
//! temp dir. Path-based convenience wrappers (`open_gerber`, `write_gcode_to`, `save_project_to`, `load_project`)
//! sit on top for script/CLI ergonomics and are the only methods that do fs I/O.

use std::collections::BTreeMap;
use std::path::Path;

use eitri_cam::{DrillConfig, FilmParams, Point, TwoOpt};
use eitri_core::{Affine, CancelToken, ProgressReporter, Unit};
use eitri_gcode::{DrillJob, IsolationJob, Origin, Postprocessor, Program, Registry, emit_drilling, emit_isolation};
use eitri_geo::DefaultBackend;
use eitri_project::{
  CamOperation, CncJobObject, DEFAULT_HISTORY_LIMIT, DrillSpec, ExcellonObject, GeometryObject, GeometryOrigin,
  GerberObject, History, JobEmission,
  ImportFormat, IsolationSpec, JobOrigin, MirrorLineSpec, NonCopperSpec, Object, ObjectCollection, ObjectId,
  ObjectKind, ObjectMeta, ObjectPayload, PaintSpec, PanelizeSpec, Project, Stock, ToolDatabase, ToolEntry, ToolId,
  WorkSetup, load_project, save_project,
};
use eitri_project::{CutoutSpec, load_tool_db, save_tool_db};
use eitri_excellon::{DrillHit, ExcellonImage, parse_excellon};
use eitri_gerber::parse_gerber;
use eitri_import::{ImportedGeometry, SvgOptions, import_dxf, import_gcode, import_svg};
use geo_types::MultiPolygon;

use crate::error::{Result, ScriptError};

/// The registry name of the default (grblHAL / Skirnir-conformant) postprocessor dialect.
pub const DEFAULT_DIALECT: &str = "grbl";

/// The working state a script or CLI drives: an undo-tracked project, a tool database, the postprocessor registry,
/// the geometry backend, and the progress/cancel seams.
pub struct Session {
  /// The project name, carried onto the persisted document.
  name: String,
  /// The undo-tracked object collection.
  history: History,
  /// The user-global tool library defaults are read from.
  tools: ToolDatabase,
  /// The postprocessor dialects G-code can be emitted with.
  registry: Registry,
  /// The default geometry backend every CAM op runs against.
  backend: DefaultBackend,
  /// The dialect CAM ops emit with; a registry key, defaulting to [`DEFAULT_DIALECT`].
  dialect: String,
  /// Where long ops push progress; silent by default.
  progress: ProgressReporter,
  /// The cooperative cancellation flag long ops poll.
  cancel: CancelToken,
}

impl Session {
  /// A fresh, empty session named `name`, with a default tool database, the built-in postprocessor dialects, a
  /// silent progress reporter, and a not-yet-cancelled token.
  pub fn new(name: impl Into<String>) -> Session {
    Session {
      name: name.into(),
      history: History::new(ObjectCollection::new()),
      tools: ToolDatabase::new(),
      registry: Registry::with_builtins(),
      backend: DefaultBackend::new(),
      dialect: DEFAULT_DIALECT.to_string(),
      progress: ProgressReporter::silent(),
      cancel: CancelToken::new(),
    }
  }

  // --- Session-level accessors -------------------------------------------------------------------------------------

  /// The project name.
  pub fn name(&self) -> &str {
    &self.name
  }

  /// Rename the project.
  pub fn set_name(&mut self, name: impl Into<String>) {
    self.name = name.into();
  }

  /// A clone of the session's cancellation token. Hold it (or hand it to another thread / a UI) to request that an
  /// in-flight command stop; the command bails with [`eitri_core::Error::Cancelled`], surfaced as [`ScriptError`].
  pub fn cancel_token(&self) -> CancelToken {
    self.cancel.clone()
  }

  /// Replace the progress reporter (e.g. with one wired to a UI channel via [`ProgressReporter::channel`]).
  pub fn set_progress(&mut self, progress: ProgressReporter) {
    self.progress = progress;
  }

  /// Replace the cancellation token. Cancellation is one-way on a token, so a caller that cancelled an in-flight
  /// command installs a fresh token here before issuing the next one — otherwise every later command would bail
  /// immediately against the still-cancelled flag.
  pub fn set_cancel(&mut self, cancel: CancelToken) {
    self.cancel = cancel;
  }

  /// The postprocessor registry, to register additional dialects.
  pub fn registry_mut(&mut self) -> &mut Registry {
    &mut self.registry
  }

  /// The dialect CAM ops currently emit with.
  pub fn dialect(&self) -> &str {
    &self.dialect
  }

  /// Choose the postprocessor dialect subsequent CAM ops emit with. Errors if no dialect of that name is registered.
  pub fn set_dialect(&mut self, name: impl Into<String>) -> Result<()> {
    let name = name.into();
    if self.registry.get(&name).is_none() {
      return Err(ScriptError::UnknownDialect(name));
    }
    self.dialect = name;
    Ok(())
  }

  // --- Stock / datum / work-zero -----------------------------------------------------------------------------------

  /// The current work-zero (datum) offset `(x, y, z)`, in the board's native frame; `(0.0, 0.0, 0.0)` is the native
  /// (unshifted) frame. Every CAM op posts its G-code relative to this point.
  pub fn work_origin(&self) -> (f64, f64, f64) {
    self.setup().origin
  }

  /// The XY datum offset — a convenience over [`Session::work_origin`] for callers that only need the plane.
  pub fn datum(&self) -> (f64, f64) {
    let (x, y, _) = self.setup().origin;
    (x, y)
  }

  /// The current stock (material block) + work-zero setup, or `None` for the native frame.
  pub fn stock(&self) -> Option<Stock> {
    self.setup().stock
  }

  /// The undo-tracked work-setup (datum / stock).
  fn setup(&self) -> &WorkSetup {
    self.history.setup()
  }

  /// Commit a new work-setup as one undoable edit that also flags every rebuildable posted job stale (its G-code
  /// baked the old datum in). A no-op — no undo entry, no staleness — when the setup is unchanged. When `coalesce` is
  /// set the change folds into the current undo entry rather than pushing its own: the caller asserts it continues an
  /// in-progress gesture (a stock-spinner drag re-commits every frame; the open-time auto-fit folds into the import).
  /// Discrete setup actions pass `coalesce = false` so they stay independently undoable (see
  /// [`eitri_project::History::commit_setup`]).
  fn commit_setup(&mut self, setup: WorkSetup, coalesce: bool) {
    if *self.setup() == setup {
      return;
    }
    self.history.commit_setup(coalesce, |collection, current| {
      *current = setup;
      flag_all_jobs_stale(collection);
    });
  }

  /// Set (or clear) the job's stock, deriving the work-zero from it. `None` reverts to the native frame. This is the
  /// primary datum control: the UI edits a [`Stock`] and commits it here, and every subsequent CAM op posts relative
  /// to the stock's datum corner and Z reference. One undoable edit; `coalesce` folds a live stock-spinner drag's
  /// per-frame re-commits into a single entry (see [`Session::commit_setup`]).
  pub fn set_stock(&mut self, stock: Option<Stock>, coalesce: bool) {
    self.commit_setup(WorkSetup::from_stock(stock), coalesce);
  }

  /// The axis-aligned XY bounds `(min_x, min_y, max_x, max_y)` of an object's geometry — what the UI auto-fits a
  /// stock to. Errors if the object has no geometry, or is a kind without bounds (an Excellon / CNC job).
  pub fn object_bounds(&self, id: ObjectId) -> Result<(f64, f64, f64, f64)> {
    let region = self.region_of(id)?;
    eitri_geo::bounds(&region)
      .ok_or_else(|| ScriptError::InvalidArgument("object has no geometry to bound".to_string()))
  }

  /// Auto-fit the stock to `reference`'s bounds (footprint = the board's bounding box) with the given material
  /// `thickness`, a bottom-left datum, and a top Z reference — the one-call setup a freshly-loaded board seeds.
  /// `coalesce` folds this into the current undo entry: the open-time auto-fit passes `true` so opening a board is a
  /// single undo step (the import), while the manual "Fit Stock" button passes `false` for its own entry.
  pub fn fit_stock_to(&mut self, reference: ObjectId, thickness: f64, coalesce: bool) -> Result<()> {
    let bounds = self.object_bounds(reference)?;
    self.set_stock(Some(Stock::fit(bounds, thickness)), coalesce);
    Ok(())
  }

  /// Post CAM output relative to an explicit native-frame point (Z0 at the surface), clearing any stock — the
  /// "set origin here" primitive. One undoable edit.
  pub fn set_datum_point(&mut self, x: f64, y: f64) {
    self.commit_setup(WorkSetup::point(x, y), false);
  }

  /// Revert to the native (source / EDA plot) coordinate frame, clearing any stock. One undoable edit.
  pub fn clear_datum(&mut self) {
    self.set_stock(None, false);
  }

  /// Resolve `origin` against the bounds of `reference` and set an XY datum from it (no stock). Kept for scripting;
  /// the UI uses [`Session::set_stock`]. Errors if the reference has no geometry.
  pub fn set_datum(&mut self, origin: JobOrigin, reference: ObjectId) -> Result<()> {
    let bounds = self.object_bounds(reference)?;
    let (x, y) = origin.resolve(bounds);
    self.set_datum_point(x, y);
    Ok(())
  }

  // --- Object management -------------------------------------------------------------------------------------------

  /// The ids of every object, in display order.
  pub fn object_ids(&self) -> Vec<ObjectId> {
    self.collection().iter().map(|o| o.meta.id).collect()
  }

  /// The number of objects.
  pub fn len(&self) -> usize {
    self.collection().len()
  }

  /// Whether the session holds no objects.
  pub fn is_empty(&self) -> bool {
    self.collection().is_empty()
  }

  /// Borrow an object by id.
  pub fn object(&self, id: ObjectId) -> Result<&Object> {
    self.collection().get(id).ok_or(ScriptError::UnknownObject(id.0))
  }

  /// The kind of an object.
  pub fn kind(&self, id: ObjectId) -> Result<ObjectKind> {
    Ok(self.object(id)?.kind())
  }

  /// The name of an object.
  pub fn object_name(&self, id: ObjectId) -> Result<String> {
    Ok(self.object(id)?.meta.name.clone())
  }

  /// The id of the object with `name`, if any.
  pub fn id_of(&self, name: &str) -> Option<ObjectId> {
    self.collection().by_name(name).map(|o| o.meta.id)
  }

  /// Rename an object (one undoable edit). Errors on an unknown id or a name collision.
  pub fn rename(&mut self, id: ObjectId, new_name: impl Into<String>) -> Result<()> {
    let new_name = new_name.into();
    self.history.edit(|c| c.rename(id, new_name)).map_err(Into::into)
  }

  /// Set an object's visibility flag (one undoable edit; setting the current value is a no-op that pushes no undo
  /// snapshot). Errors on an unknown id. Visibility is display metadata the UI's tree/canvas honour; it persists
  /// with the project like the rest of [`ObjectMeta`].
  pub fn set_visible(&mut self, id: ObjectId, visible: bool) -> Result<()> {
    if self.object(id)?.meta.visible == visible {
      return Ok(());
    }
    self.history.edit(|c| {
      if let Some(object) = c.get_mut(id) {
        object.meta.visible = visible;
      }
    });
    Ok(())
  }

  /// Delete an object (one undoable edit). Errors if no object has that id.
  pub fn delete(&mut self, id: ObjectId) -> Result<()> {
    let removed = self.history.edit(|c| c.remove(id));
    if removed.is_some() { Ok(()) } else { Err(ScriptError::UnknownObject(id.0)) }
  }

  /// Move an object to a new position in the display order (clamped). One undoable edit.
  pub fn reorder(&mut self, id: ObjectId, index: usize) -> Result<()> {
    self.history.edit(|c| c.reorder(id, index)).map_err(Into::into)
  }

  /// Create an empty named group (one undoable edit).
  pub fn create_group(&mut self, name: impl Into<String>) -> Result<()> {
    let name = name.into();
    self.history.edit(|c| c.create_group(name)).map_err(Into::into)
  }

  /// Add an object to a group (one undoable edit).
  pub fn add_to_group(&mut self, group: &str, id: ObjectId) -> Result<()> {
    let group = group.to_string();
    self.history.edit(|c| c.add_to_group(&group, id)).map_err(Into::into)
  }

  /// The member ids of a group, in add order.
  pub fn group_members(&self, group: &str) -> Option<Vec<ObjectId>> {
    self.collection().group_members(group).map(|m| m.to_vec())
  }

  /// Every group as `(name, members)`, in creation order — the snapshot the UI renders as project-tree folders.
  pub fn groups(&self) -> Vec<(String, Vec<ObjectId>)> {
    self.collection().groups().map(|g| (g.name.clone(), g.members.clone())).collect()
  }

  /// The name of the group an object belongs to, if any (an object is in at most one group).
  pub fn group_of(&self, id: ObjectId) -> Option<String> {
    self.collection().group_of(id).map(|g| g.name.clone())
  }

  /// Whether an object is a freshly imported SOURCE — a Gerber, an Excellon, or geometry imported from a file — the
  /// kinds that auto-join the import group. Generated geometry (panelize/mirror/transform) and CNC jobs are not.
  pub fn is_imported_source(&self, id: ObjectId) -> bool {
    match self.object(id) {
      Ok(object) => match &object.payload {
        ObjectPayload::Gerber(_) | ObjectPayload::Excellon(_) => true,
        ObjectPayload::Geometry(geometry) => matches!(geometry.origin, GeometryOrigin::Imported { .. }),
        ObjectPayload::CncJob(_) => false,
      },
      Err(_) => false,
    }
  }

  /// The name of the managed group that every imported source object auto-joins, so a board's layers move together
  /// as one locked set on the stock.
  pub const IMPORT_GROUP: &'static str = "Imported";

  /// Add a freshly imported source object to the managed [`Session::IMPORT_GROUP`] (created on first import). Folded
  /// into the CURRENT undo entry (the import that produced `id`) via [`History::amend`], so one undo removes both the
  /// object and its group membership rather than leaving an orphan step.
  pub fn add_to_import_group(&mut self, id: ObjectId) -> Result<()> {
    self.object(id)?; // Validate before mutating.
    self.history.amend(|c| {
      if !c.has_group(Session::IMPORT_GROUP) {
        let _ = c.create_group(Session::IMPORT_GROUP);
      }
      let _ = c.add_to_group(Session::IMPORT_GROUP, id);
    });
    Ok(())
  }

  /// Move the whole group containing `anchor` (or just `anchor`, if it is ungrouped) by `(dx, dy)` millimetres on the
  /// stock, and flag every CNC job derived from a moved object stale so the UI can offer a rebuild. `new_edit` starts
  /// a fresh undo entry — pass `true` for the first step of a drag and `false` for the rest, so the whole drag
  /// coalesces into one undoable move.
  pub fn move_group(&mut self, anchor: ObjectId, dx: f64, dy: f64, new_edit: bool) -> Result<()> {
    self.object(anchor)?; // Validate before mutating.
    let members = self.move_set(anchor);
    let shift = Affine::translate(dx, dy);
    // One pass over the collection per drag frame: compose the shift onto each member and flag every dependent job
    // stale in the same walk. (Routing each member through `set_object_placement` would re-scan the whole collection
    // once per member — N passes per frame on a multi-layer grouped drag; docs review #9.) This still flags staleness
    // on every placement path, which is the invariant #2 established — `set_placement`/`translate_object` keep using
    // the `set_object_placement` choke point.
    let mutate = move |c: &mut ObjectCollection| {
      for object in c.iter_mut() {
        if members.contains(&object.meta.id) {
          object.meta.placement = object.meta.placement.then(shift);
        }
        if let ObjectPayload::CncJob(job) = &mut object.payload
          && job.source.is_some_and(|src| members.contains(&src))
          && is_rebuildable(job)
        {
          job.stale = true;
        }
      }
    };
    if new_edit {
      self.history.edit(mutate);
    } else {
      self.history.amend(mutate);
    }
    Ok(())
  }

  /// The ids that move together with `anchor`: the members of its group, or just `anchor` if it is ungrouped. The
  /// set [`Session::move_group`] repositions, exposed so the UI's drag fast-path can shift exactly the same objects
  /// in its cached scene without rebuilding it (docs follow-up #1).
  pub fn move_set(&self, anchor: ObjectId) -> Vec<ObjectId> {
    match self.collection().group_of(anchor) {
      Some(group) => group.members.clone(),
      None => vec![anchor],
    }
  }

  /// Whether there is an edit to undo.
  pub fn can_undo(&self) -> bool {
    self.history.can_undo()
  }

  /// Whether there is an edit to redo.
  pub fn can_redo(&self) -> bool {
    self.history.can_redo()
  }

  /// Undo the last edit, returning whether anything changed.
  pub fn undo(&mut self) -> bool {
    self.history.undo()
  }

  /// Redo the last undone edit, returning whether anything changed.
  pub fn redo(&mut self) -> bool {
    self.history.redo()
  }

  // --- Open / import (source-string core) --------------------------------------------------------------------------

  /// Open Gerber source as a new object, parsing it eagerly so it is ready for a CAM op and caching the parsed
  /// copper alongside the persisted source. Returns the new object's id.
  pub fn open_gerber_str(&mut self, name: impl Into<String>, source: impl Into<String>) -> Result<ObjectId> {
    let source = source.into();
    let image = parse_gerber(&source, &self.progress, &self.cancel).map_err(eitri_core::Error::from)?;
    let mut object = GerberObject::new(source);
    object.image = Some(image);
    self.add(name.into(), ObjectPayload::Gerber(object))
  }

  /// Open Excellon source as a new object, parsing it eagerly (with format inference) and caching the drill hits.
  pub fn open_excellon_str(&mut self, name: impl Into<String>, source: impl Into<String>) -> Result<ObjectId> {
    let source = source.into();
    let image = parse_excellon(&source, None, &self.progress, &self.cancel).map_err(eitri_core::Error::from)?;
    let mut object = ExcellonObject::new(source);
    object.image = Some(image);
    self.add(name.into(), ObjectPayload::Excellon(object))
  }

  /// Import SVG source as a new geometry object, flattening curves at the shared chord tolerance.
  pub fn import_svg_str(&mut self, name: impl Into<String>, source: impl Into<String>) -> Result<ObjectId> {
    let source: String = source.into();
    let import = import_svg(&source, &SvgOptions::default())?;
    let origin = GeometryOrigin::Imported { format: ImportFormat::Svg, source: source.into() };
    self.add_imported_geometry(name.into(), &import.geometry, origin)
  }

  /// Import DXF source as a new geometry object.
  pub fn import_dxf_str(&mut self, name: impl Into<String>, source: impl Into<String>) -> Result<ObjectId> {
    let source: String = source.into();
    let import = import_dxf(&source)?;
    let origin = GeometryOrigin::Imported { format: ImportFormat::Dxf, source: source.into() };
    self.add_imported_geometry(name.into(), &import.geometry, origin)
  }

  /// Import existing G-code as a new geometry object: the cut moves are walked back into open polylines (a plunge
  /// leaves no XY trace, so the recovered contours match the source), which is what a re-post or preview consumes.
  pub fn import_gcode_str(&mut self, name: impl Into<String>, source: impl Into<String>) -> Result<ObjectId> {
    let source: String = source.into();
    let preview = import_gcode(&source)?;
    let geometry = GeometryObject {
      polygons: Vec::new(),
      polylines: preview.cut_polylines(),
      origin: GeometryOrigin::Imported { format: ImportFormat::Gcode, source: source.into() },
    };
    self.add(name.into(), ObjectPayload::Geometry(geometry))
  }

  // --- Open / import (path convenience wrappers) -------------------------------------------------------------------

  /// Open a Gerber file, naming the object after the file stem.
  pub fn open_gerber(&mut self, path: impl AsRef<Path>) -> Result<ObjectId> {
    let (name, source) = read_named(path)?;
    self.open_gerber_str(name, source)
  }

  /// Open an Excellon file, naming the object after the file stem.
  pub fn open_excellon(&mut self, path: impl AsRef<Path>) -> Result<ObjectId> {
    let (name, source) = read_named(path)?;
    self.open_excellon_str(name, source)
  }

  /// Import an SVG file, naming the object after the file stem.
  pub fn import_svg(&mut self, path: impl AsRef<Path>) -> Result<ObjectId> {
    let (name, source) = read_named(path)?;
    self.import_svg_str(name, source)
  }

  /// Import a DXF file, naming the object after the file stem.
  pub fn import_dxf(&mut self, path: impl AsRef<Path>) -> Result<ObjectId> {
    let (name, source) = read_named(path)?;
    self.import_dxf_str(name, source)
  }

  /// Import a G-code file, naming the object after the file stem.
  pub fn import_gcode(&mut self, path: impl AsRef<Path>) -> Result<ObjectId> {
    let (name, source) = read_named(path)?;
    self.import_gcode_str(name, source)
  }

  // --- CAM operations ----------------------------------------------------------------------------------------------

  /// Isolate a copper/geometry `source` into cut rings and emit them as a CNC job. Returns the job's id.
  pub fn isolate(&mut self, source: ObjectId, spec: IsolationSpec, job: IsolationJob) -> Result<ObjectId> {
    let program = self.isolation_program(source, &spec, &job)?;
    let base = self.job_name(source, "isolation")?;
    self.store_program(program, Some(source), CamOperation::Isolation(spec), &base, Some(JobEmission::Isolation(job)))
  }

  /// Drill an Excellon `source`, ordering hits to minimize rapid travel, and emit the drilling program. `spec` is
  /// applied to every tool as its default parameters.
  pub fn drill(&mut self, source: ObjectId, spec: DrillSpec, job: DrillJob) -> Result<ObjectId> {
    let program = self.drill_program(source, &spec, &job)?;
    let base = self.job_name(source, "drilling")?;
    self.store_program(program, Some(source), CamOperation::Drilling(spec), &base, Some(JobEmission::Drill(job)))
  }

  /// Area-clear (paint) a copper/geometry `source` with the chosen fill strategy and emit the toolpaths as a job.
  pub fn paint(&mut self, source: ObjectId, spec: PaintSpec, job: IsolationJob) -> Result<ObjectId> {
    let program = self.paint_program(source, &spec, &job)?;
    let base = self.job_name(source, "paint")?;
    self.store_program(program, Some(source), CamOperation::Paint(spec), &base, Some(JobEmission::Isolation(job)))
  }

  /// Clear all non-copper within a boundary around a Gerber/geometry `source`, painting the negative region, and emit
  /// the toolpaths as a job.
  pub fn noncopper(&mut self, source: ObjectId, spec: NonCopperSpec, job: IsolationJob) -> Result<ObjectId> {
    let program = self.noncopper_program(source, &spec, &job)?;
    let base = self.job_name(source, "noncopper")?;
    self.store_program(program, Some(source), CamOperation::NonCopper(spec), &base, Some(JobEmission::Isolation(job)))
  }

  /// Route a board cutout around the outline the `spec` owns (a rectangle or a hand-drawn silhouette), leaving
  /// holding tabs, and emit the profile as a job. The outline is self-contained, so there is no source object.
  pub fn cutout(&mut self, spec: CutoutSpec, job: IsolationJob) -> Result<ObjectId> {
    let program = self.cutout_program(&spec, &job)?;
    self.store_program(program, None, CamOperation::Cutout(spec), "cutout", Some(JobEmission::Isolation(job)))
  }

  /// Recompute a CNC job in place from its stored operation spec + emission parameters, against the CURRENT (placed)
  /// source geometry and datum — the "rebuild" action. The job keeps its id, name, and source back-reference; only
  /// its G-code is refreshed and its stale flag cleared. One undoable edit. Errors if `id` is not a CNC job, if it
  /// predates rebuild support (no stored emission), or if its operation cannot be rebuilt (panelize/mirror produce
  /// geometry, not jobs).
  pub fn rebuild_job(&mut self, id: ObjectId) -> Result<()> {
    let (operation, emission, source) = match &self.object(id)?.payload {
      ObjectPayload::CncJob(job) => (job.operation.clone(), job.emission.clone(), job.source),
      other => return Err(self.wrong_kind(id, "a CNC job", other)),
    };
    let Some(emission) = emission else {
      return Err(ScriptError::InvalidArgument(
        "this job predates rebuild support and cannot be recalculated".to_string(),
      ));
    };
    let program = self.rebuild_program(&operation, &emission, source)?;
    let dialect = self.dialect.clone();
    let (ox, oy, _) = self.setup().origin;
    let origin = (ox, oy);
    let cnc = CncJobObject::from_program(&program, dialect, source, operation, origin, Some(emission));
    self.history.edit(|c| {
      if let Some(object) = c.get_mut(id) {
        object.payload = ObjectPayload::CncJob(cnc);
      }
    });
    Ok(())
  }

  /// Recompute the G-code [`Program`] for a stored operation + emission against the current source geometry — the
  /// shared body behind [`Session::rebuild_job`]. Dispatches to the same per-op compute the public ops use.
  fn rebuild_program(
    &self,
    operation: &CamOperation,
    emission: &JobEmission,
    source: Option<ObjectId>,
  ) -> Result<Program> {
    let src = || source.ok_or_else(|| ScriptError::InvalidArgument("this job's source is gone; cannot rebuild".into()));
    match (operation, emission) {
      (CamOperation::Isolation(spec), JobEmission::Isolation(job)) => self.isolation_program(src()?, spec, job),
      (CamOperation::Drilling(spec), JobEmission::Drill(job)) => self.drill_program(src()?, spec, job),
      (CamOperation::Paint(spec), JobEmission::Isolation(job)) => self.paint_program(src()?, spec, job),
      (CamOperation::NonCopper(spec), JobEmission::Isolation(job)) => self.noncopper_program(src()?, spec, job),
      (CamOperation::Cutout(spec), JobEmission::Isolation(job)) => self.cutout_program(spec, job),
      _ => Err(ScriptError::InvalidArgument("this operation cannot be rebuilt".to_string())),
    }
  }

  /// Compute the isolation G-code for a placed `source` — shared by [`Session::isolate`] and rebuild.
  fn isolation_program(&self, source: ObjectId, spec: &IsolationSpec, job: &IsolationJob) -> Result<Program> {
    let region = self.region_of(source)?;
    let toolpaths = eitri_cam::isolate(&self.backend, &region, &spec.to_params(), &self.progress, &self.cancel)?;
    let post = self.post()?;
    Ok(emit_isolation(&toolpaths, job, self.emit_origin(), post, Some(tool_note(spec.tool_diameter))))
  }

  /// Compute the drilling G-code for a placed Excellon `source` — shared by [`Session::drill`] and rebuild.
  fn drill_program(&self, source: ObjectId, spec: &DrillSpec, job: &DrillJob) -> Result<Program> {
    let image = self.excellon_image_of(source)?;
    let config = DrillConfig { defaults: spec.to_params(), overrides: BTreeMap::new(), start: Point::new(0.0, 0.0) };
    let plan = eitri_cam::plan_drilling(&image, &config, &TwoOpt::new(), &self.progress, &self.cancel)?;
    // A drilling job runs several bits; summarize their sizes in the header (each also gets a per-tool-change note).
    let tool = (!plan.tools.is_empty()).then(|| {
      let diameters: Vec<f64> = plan.tools.iter().map(|t| t.diameter).collect();
      drill_tools_note(&diameters)
    });
    let post = self.post()?;
    Ok(emit_drilling(&plan, job, self.emit_origin(), post, tool))
  }

  /// Compute the paint (area-clear) G-code for a placed `source` — shared by [`Session::paint`] and rebuild.
  fn paint_program(&self, source: ObjectId, spec: &PaintSpec, job: &IsolationJob) -> Result<Program> {
    let region = self.region_of(source)?;
    let strategy = spec.strategy();
    let result =
      eitri_cam::paint(&region, &spec.to_params(), strategy.as_ref(), &self.backend, &self.progress, &self.cancel)?;
    let post = self.post()?;
    Ok(emit_isolation(&result.toolpaths(), job, self.emit_origin(), post, Some(tool_note(spec.tool_diameter))))
  }

  /// Compute the non-copper-clearing G-code for a placed `source` — shared by [`Session::noncopper`] and rebuild.
  fn noncopper_program(&self, source: ObjectId, spec: &NonCopperSpec, job: &IsolationJob) -> Result<Program> {
    let region = self.region_of(source)?;
    let strategy = spec.strategy();
    let result = eitri_cam::clear_noncopper(
      &region,
      &spec.to_boundary(),
      &spec.to_params(),
      strategy.as_ref(),
      &self.backend,
      &self.progress,
      &self.cancel,
    )?;
    let post = self.post()?;
    Ok(emit_isolation(&result.toolpaths(), job, self.emit_origin(), post, Some(tool_note(spec.paint.tool_diameter))))
  }

  /// Compute the board-cutout G-code from the spec's self-contained outline — shared by [`Session::cutout`] and
  /// rebuild.
  fn cutout_program(&self, spec: &CutoutSpec, job: &IsolationJob) -> Result<Program> {
    let result =
      eitri_cam::cutout(&spec.to_outline(), &spec.to_params(), &self.backend, &self.progress, &self.cancel)?;
    let post = self.post()?;
    Ok(emit_isolation(&result.toolpaths(), job, self.emit_origin(), post, Some(tool_note(spec.tool_diameter))))
  }

  /// Panelize a copper/geometry `source` into an `rows × cols` grid, returning a **geometry** object holding the
  /// merged panel (panelization arrays geometry; it is not itself a toolpath op).
  pub fn panelize(&mut self, source: ObjectId, spec: PanelizeSpec) -> Result<ObjectId> {
    let region = self.region_of(source)?;
    let merged =
      eitri_cam::panelize_multipolygon(&region, &spec.to_params(), &self.backend, &self.progress, &self.cancel)?;
    let base = self.job_name(source, "panel")?;
    self.store_geometry(merged.0, Vec::new(), GeometryOrigin::Generated, &base)
  }

  /// Mirror a copper/geometry `source` about a line for two-sided alignment, returning a **geometry** object.
  pub fn mirror(&mut self, source: ObjectId, line: MirrorLineSpec) -> Result<ObjectId> {
    let region = self.region_of(source)?;
    let mirrored = eitri_cam::mirror_multipolygon(&region, line.to_mirror_line());
    let base = self.job_name(source, "mirror")?;
    self.store_geometry(mirrored.0, Vec::new(), GeometryOrigin::Generated, &base)
  }

  /// Apply an affine `transform` to a copper/geometry `source`, returning a new **geometry** object. The
  /// convenience wrappers [`Session::translate`] / [`Session::scale`] / [`Session::rotate`] build common transforms.
  pub fn transform(&mut self, source: ObjectId, transform: Affine) -> Result<ObjectId> {
    let region = self.region_of(source)?;
    let out = eitri_cam::edit::transform(&region, transform);
    let base = self.job_name(source, "xform")?;
    self.store_geometry(out.0, Vec::new(), GeometryOrigin::Generated, &base)
  }

  /// Translate a `source` by `(dx, dy)` millimetres into a new geometry object.
  pub fn translate(&mut self, source: ObjectId, dx: f64, dy: f64) -> Result<ObjectId> {
    self.transform(source, Affine::translate(dx, dy))
  }

  /// Scale a `source` by `(sx, sy)` about the origin into a new geometry object.
  pub fn scale(&mut self, source: ObjectId, sx: f64, sy: f64) -> Result<ObjectId> {
    self.transform(source, Affine::scale(sx, sy))
  }

  /// Rotate a `source` by `degrees` about the origin into a new geometry object.
  pub fn rotate(&mut self, source: ObjectId, degrees: f64) -> Result<ObjectId> {
    self.transform(source, Affine::rotate(degrees.to_radians()))
  }

  /// Set an object's [`ObjectMeta::placement`] — its position/orientation on the stock — as one undoable edit.
  /// Unlike [`Session::transform`], which bakes an affine into a NEW geometry object, this moves the object in place;
  /// every CAM op reads the placed geometry, so a move repositions the object's toolpaths on the next rebuild.
  pub fn set_placement(&mut self, id: ObjectId, placement: Affine) -> Result<()> {
    let current = self.object(id)?.meta.placement; // Validate the id before taking an undo snapshot; also the no-op check.
    // A placement must be a rigid motion (translate + rotate). A scale/shear would move a filled region and a set of
    // drill centers by different amounts — `place_region` transforms the whole polygon, `place_hits` only the hit
    // centers — so the emitted CAM program and the canvas preview would silently disagree (docs follow-up #4).
    if !placement.is_rigid() {
      return Err(ScriptError::InvalidArgument("a placement must be a rigid transform (translate/rotate only)".into()));
    }
    // A placement that does not actually change anything (a net-zero drag, `translate_object(id, 0, 0)`) must not push
    // an undo entry or flag dependent jobs stale — the rebuilt G-code would be byte-identical (docs review #5).
    if current == placement {
      return Ok(());
    }
    self.history.edit(|c| set_object_placement(c, id, placement));
    Ok(())
  }

  /// Translate an object by `(dx, dy)` millimetres on the stock, composing onto its current placement (one undoable
  /// edit). Positive `dx`/`dy` move it right/up in the work frame.
  pub fn translate_object(&mut self, id: ObjectId, dx: f64, dy: f64) -> Result<()> {
    let placement = self.object(id)?.meta.placement;
    self.set_placement(id, placement.then(Affine::translate(dx, dy)))
  }

  /// Render a copper/geometry `source` as a positive or negative photo-film SVG (see [`eitri_cam::film_svg`]).
  /// Film is vector *output*, not a toolpath, so nothing is committed to the collection — the caller writes the
  /// returned document wherever it wants. Uses the NATIVE region: film is 1:1 artwork, so the board's placement on the
  /// CNC stock must not offset it (and two-sided top/bottom films must share the native origin to register) —
  /// docs review #2.
  pub fn film_svg(&self, source: ObjectId, params: &FilmParams) -> Result<String> {
    let region = self.source_region(source)?;
    eitri_cam::film_svg(&region, params, &self.backend).map_err(Into::into)
  }

  // --- G-code output -----------------------------------------------------------------------------------------------

  /// The rendered G-code of a CNC-job object, as one newline-terminated string. Errors if the id is not a CNC job.
  pub fn write_gcode(&self, job: ObjectId) -> Result<String> {
    match &self.object(job)?.payload {
      ObjectPayload::CncJob(cnc) => Ok(cnc.render()),
      other => Err(self.wrong_kind(job, "a CNC job", other)),
    }
  }

  /// Write a CNC-job object's G-code to `path`.
  pub fn write_gcode_to(&self, job: ObjectId, path: impl AsRef<Path>) -> Result<()> {
    let text = self.write_gcode(job)?;
    let path = path.as_ref();
    std::fs::write(path, text).map_err(|e| ScriptError::io(path.display().to_string(), e))
  }

  // --- Project persistence -----------------------------------------------------------------------------------------

  /// Serialize the current project to versioned JSON.
  pub fn save_project(&self) -> Result<String> {
    let setup = self.setup();
    let project = Project {
      name: self.name.clone(),
      collection: self.collection().clone(),
      origin: setup.origin,
      stock: setup.stock,
    };
    save_project(&project).map_err(Into::into)
  }

  /// Write the current project to `path` as versioned JSON.
  pub fn save_project_to(&self, path: impl AsRef<Path>) -> Result<()> {
    let json = self.save_project()?;
    let path = path.as_ref();
    std::fs::write(path, json).map_err(|e| ScriptError::io(path.display().to_string(), e))
  }

  /// Build a session from a project JSON string, re-deriving Gerber/Excellon geometry so the loaded objects are
  /// immediately usable by CAM ops. The tool database and dialect start at their defaults (the tool DB is a
  /// separate, user-global file).
  pub fn load_project_str(json: &str) -> Result<Session> {
    let mut project = load_project(json)?;
    let progress = ProgressReporter::silent();
    let cancel = CancelToken::new();
    project.hydrate(&progress, &cancel)?;
    let setup = WorkSetup { origin: project.origin, stock: project.stock };
    Ok(Session {
      name: project.name,
      history: History::with_document(project.collection, setup, DEFAULT_HISTORY_LIMIT),
      tools: ToolDatabase::new(),
      registry: Registry::with_builtins(),
      backend: DefaultBackend::new(),
      dialect: DEFAULT_DIALECT.to_string(),
      progress,
      cancel,
    })
  }

  /// Load a session from a project file on disk.
  pub fn load_project(path: impl AsRef<Path>) -> Result<Session> {
    let path = path.as_ref();
    let json = std::fs::read_to_string(path).map_err(|e| ScriptError::io(path.display().to_string(), e))?;
    Session::load_project_str(&json)
  }

  // --- Tool database -----------------------------------------------------------------------------------------------

  /// The tool database.
  pub fn tools(&self) -> &ToolDatabase {
    &self.tools
  }

  /// The tool database, mutably.
  pub fn tools_mut(&mut self) -> &mut ToolDatabase {
    &mut self.tools
  }

  /// Add a tool to the database, returning its stable id.
  pub fn add_tool(&mut self, entry: ToolEntry) -> ToolId {
    self.tools.add(entry)
  }

  /// The isolation spec a tool seeds (its diameter plus the tool's isolation defaults). Errors on an unknown id.
  pub fn tool_isolation_spec(&self, id: ToolId) -> Result<IsolationSpec> {
    Ok(self.tool(id)?.isolation_spec())
  }

  /// The drilling spec a tool seeds. Errors on an unknown id.
  pub fn tool_drill_spec(&self, id: ToolId) -> Result<DrillSpec> {
    Ok(self.tool(id)?.drill_spec())
  }

  /// Serialize the tool database to versioned JSON.
  pub fn save_tool_db(&self) -> Result<String> {
    save_tool_db(&self.tools).map_err(Into::into)
  }

  /// Replace the session's tool database with one loaded from JSON.
  pub fn load_tool_db(&mut self, json: &str) -> Result<()> {
    self.tools = load_tool_db(json)?;
    Ok(())
  }

  // --- Private helpers ---------------------------------------------------------------------------------------------

  /// The current object collection.
  fn collection(&self) -> &ObjectCollection {
    self.history.current()
  }

  /// The active postprocessor, or [`ScriptError::UnknownDialect`] if it has somehow been removed.
  fn post(&self) -> Result<&dyn Postprocessor> {
    self.registry.get(&self.dialect).ok_or_else(|| ScriptError::UnknownDialect(self.dialect.clone()))
  }

  /// The datum as the emitter's [`Origin`] type, subtracted from every emitted coordinate.
  fn emit_origin(&self) -> Origin {
    let (x, y, z) = self.setup().origin;
    Origin::with_z(x, y, z)
  }

  /// Borrow a tool by id.
  fn tool(&self, id: ToolId) -> Result<&ToolEntry> {
    self.tools.get(id).ok_or_else(|| ScriptError::InvalidArgument(format!("no tool with id {id}")))
  }

  /// The copper/geometry region of a source object in its **native** frame — the artwork as authored, ignoring where
  /// the board was arranged on the stock. Gerber yields its copper (re-parsing on the rare chance the cache is empty);
  /// Geometry yields its polygons; anything else is a [`ScriptError::WrongKind`]. This is what placement-independent
  /// outputs (photo film) must use; CAM toolpaths use [`Session::region_of`], which places it.
  fn source_region(&self, id: ObjectId) -> Result<MultiPolygon<f64>> {
    let object = self.object(id)?;
    match &object.payload {
      ObjectPayload::Gerber(gerber) => match &gerber.image {
        Some(image) => Ok(image.copper.clone()),
        None => Ok(parse_gerber(&gerber.source, &self.progress, &self.cancel).map_err(eitri_core::Error::from)?.copper),
      },
      ObjectPayload::Geometry(geometry) => Ok(MultiPolygon::new(geometry.polygons.clone())),
      other => Err(self.wrong_kind(id, "a Gerber or Geometry object", other)),
    }
  }

  /// The copper/geometry region of a source object, **placed** by the object's [`ObjectMeta::placement`] so a moved
  /// board's toolpaths land where the operator arranged it on the stock — the CAM path. Placement-independent
  /// consumers (film export) must use [`Session::source_region`] instead so a stock move never leaks into the artwork.
  fn region_of(&self, id: ObjectId) -> Result<MultiPolygon<f64>> {
    let placement = self.object(id)?.meta.placement;
    Ok(place_region(self.source_region(id)?, placement))
  }

  /// The parsed drill image of an Excellon source, cloned (re-parsing if the cache is empty) and **placed** by the
  /// object's [`ObjectMeta::placement`], so a moved board drills where it was arranged on the stock.
  fn excellon_image_of(&self, id: ObjectId) -> Result<ExcellonImage> {
    let object = self.object(id)?;
    let mut image = match &object.payload {
      ObjectPayload::Excellon(excellon) => match &excellon.image {
        Some(image) => image.clone(),
        None => {
          parse_excellon(&excellon.source, None, &self.progress, &self.cancel).map_err(eitri_core::Error::from)?
        }
      },
      other => return Err(self.wrong_kind(id, "an Excellon object", other)),
    };
    place_hits(&mut image, object.meta.placement);
    Ok(image)
  }

  /// Wrap an emitted program as a CNC job and commit it as one undoable edit. `emission` is the machine intent
  /// (feeds/depths/spindle) persisted alongside the operation spec so the job can later be rebuilt.
  fn store_program(
    &mut self,
    program: Program,
    source: Option<ObjectId>,
    operation: CamOperation,
    base_name: &str,
    emission: Option<JobEmission>,
  ) -> Result<ObjectId> {
    let dialect = self.dialect.clone();
    let (ox, oy, _) = self.setup().origin;
    let origin = (ox, oy);
    let cnc = CncJobObject::from_program(&program, dialect, source, operation, origin, emission);
    let name = self.unique_name(base_name);
    self.add(name, ObjectPayload::CncJob(cnc))
  }

  /// Commit generated geometry as a new geometry object (one undoable edit).
  fn store_geometry(
    &mut self,
    polygons: Vec<geo_types::Polygon<f64>>,
    polylines: Vec<geo_types::LineString<f64>>,
    origin: GeometryOrigin,
    base_name: &str,
  ) -> Result<ObjectId> {
    let name = self.unique_name(base_name);
    self.add(name, ObjectPayload::Geometry(GeometryObject { polygons, polylines, origin }))
  }

  /// Commit an imported geometry preview as a new geometry object.
  fn add_imported_geometry(
    &mut self,
    name: String,
    geometry: &ImportedGeometry,
    origin: GeometryOrigin,
  ) -> Result<ObjectId> {
    let object = GeometryObject::from_imported(geometry, origin);
    self.add(name, ObjectPayload::Geometry(object))
  }

  /// Add an object under an explicit name as one undoable edit, pre-checking name uniqueness so a rejected add never
  /// leaves a wasted undo snapshot.
  fn add(&mut self, name: String, payload: ObjectPayload) -> Result<ObjectId> {
    if self.collection().by_name(&name).is_some() {
      return Err(ScriptError::Project(eitri_project::ProjectError::DuplicateName(name)));
    }
    let meta = ObjectMeta::new(name, Unit::Millimeters);
    self.history.edit(|c| c.add(meta, payload)).map_err(Into::into)
  }

  /// A `<source-name>-<suffix>` base name for a derived object, falling back to just the suffix if the source has
  /// somehow gone missing.
  fn job_name(&self, source: ObjectId, suffix: &str) -> Result<String> {
    let source_name = self.object_name(source)?;
    Ok(format!("{source_name}-{suffix}"))
  }

  /// `base`, or `base-2`, `base-3`, … — the first form not already taken, so derived objects never collide.
  fn unique_name(&self, base: &str) -> String {
    if self.collection().by_name(base).is_none() {
      return base.to_string();
    }
    let mut n = 2u32;
    loop {
      let candidate = format!("{base}-{n}");
      if self.collection().by_name(&candidate).is_none() {
        return candidate;
      }
      n += 1;
    }
  }

  /// Build a [`ScriptError::WrongKind`] naming what was required and what the object actually is.
  fn wrong_kind(&self, id: ObjectId, expected: &str, actual: &ObjectPayload) -> ScriptError {
    let actual = match actual {
      ObjectPayload::Gerber(_) => ObjectKind::Gerber,
      ObjectPayload::Excellon(_) => ObjectKind::Excellon,
      ObjectPayload::Geometry(_) => ObjectKind::Geometry,
      ObjectPayload::CncJob(_) => ObjectKind::CncJob,
    };
    ScriptError::WrongKind { id: id.0, expected: expected.to_string(), actual }
  }
}

/// Read a file to a string, deriving a display name from its stem. Returns `(name, source)`.
fn read_named(path: impl AsRef<Path>) -> Result<(String, String)> {
  let path = path.as_ref();
  let source = std::fs::read_to_string(path).map_err(|e| ScriptError::io(path.display().to_string(), e))?;
  let name = path
    .file_stem()
    .and_then(|s| s.to_str())
    .map(str::to_string)
    .unwrap_or_else(|| path.display().to_string());
  Ok((name, source))
}

/// A human-readable single-tool note for a G-code header comment (e.g. `"tool diameter 0.200 mm"`), so the operator
/// can see what cutter a program expects at a glance.
fn tool_note(diameter: f64) -> String {
  format!("tool diameter {diameter:.3} mm")
}

/// The most bytes a header note may occupy, leaving ample room within the grblHAL 256-byte line limit for the comment
/// delimiters and any prefix once it is emitted as a `(...)` line (the wire contract; see [`drill_tools_note`]).
const HEADER_NOTE_MAX: usize = 200;

/// A drilling-header note listing the bit sizes. A few tools are listed in full; a board with many distinct sizes
/// would overrun the grblHAL line limit as one comment, so it falls back to a count + range summary that a strict
/// grblHAL 1.1f receiver will still accept (docs review #4).
fn drill_tools_note(diameters: &[f64]) -> String {
  let sizes: Vec<String> = diameters.iter().map(|d| format!("{d:.3}")).collect();
  let full = format!("drill tools: {} mm", sizes.join(", "));
  if full.len() <= HEADER_NOTE_MAX {
    return full;
  }
  let min = diameters.iter().copied().fold(f64::INFINITY, f64::min);
  let max = diameters.iter().copied().fold(f64::NEG_INFINITY, f64::max);
  format!("drill tools: {} sizes, {min:.3}-{max:.3} mm", diameters.len())
}

/// Set an object's [`ObjectMeta::placement`] and flag every CNC job derived from it stale — the single
/// placement-write choke point. Routing both [`Session::set_placement`] and [`Session::move_group`] through here
/// means "source moved → dependent jobs need a rebuild" cannot be forgotten by a new call site. A missing id is a
/// silent no-op (callers validate the id before taking the undo snapshot).
fn set_object_placement(collection: &mut ObjectCollection, id: ObjectId, placement: Affine) {
  if let Some(object) = collection.get_mut(id) {
    object.meta.placement = placement;
  }
  flag_dependent_jobs_stale(collection, id);
}

/// Whether a CNC job can be rebuilt in place — only jobs that stored their emission parameters can. A legacy job
/// (loaded from a project written before the `emission` field existed) cannot, so flagging it stale would light a ⟳
/// rebuild badge that [`Session::rebuild_job`] then refuses, leaving it permanently and un-actionably "out of date"
/// (docs review #4). Such jobs are left un-flagged: the operator must re-post them by hand regardless.
fn is_rebuildable(job: &CncJobObject) -> bool {
  job.emission.is_some()
}

/// Flag every rebuildable CNC job whose source is `moved` stale, so its posted G-code (which baked in the source's
/// old position) no longer silently passes for current — the UI shows the ⟳ rebuild badge.
fn flag_dependent_jobs_stale(collection: &mut ObjectCollection, moved: ObjectId) {
  for object in collection.iter_mut() {
    if let ObjectPayload::CncJob(job) = &mut object.payload
      && job.source == Some(moved)
      && is_rebuildable(job)
    {
      job.stale = true;
    }
  }
}

/// Flag *every* rebuildable posted CNC job stale — used when a project-level input every job derives from changes (the
/// datum / stock), since each job's G-code already subtracted the old work-zero (docs follow-up #3).
fn flag_all_jobs_stale(collection: &mut ObjectCollection) {
  for object in collection.iter_mut() {
    if let ObjectPayload::CncJob(job) = &mut object.payload
      && is_rebuildable(job)
    {
      job.stale = true;
    }
  }
}

/// Apply an object's placement to its copper/geometry region. The identity placement (an unmoved object) is returned
/// untouched, so an unplaced board's geometry is bit-for-bit what it was before placement existed.
fn place_region(region: MultiPolygon<f64>, placement: Affine) -> MultiPolygon<f64> {
  if placement == Affine::IDENTITY {
    region
  } else {
    eitri_cam::edit::transform(&region, placement)
  }
}

/// Apply an object's placement to every drill/slot hit of an Excellon image in place. A no-op for the identity
/// placement, so an unmoved drill file's hits are unchanged.
fn place_hits(image: &mut ExcellonImage, placement: Affine) {
  if placement == Affine::IDENTITY {
    return;
  }
  for hit in &mut image.hits {
    match hit {
      DrillHit::Drill { x, y, .. } => (*x, *y) = placement.apply(*x, *y),
      DrillHit::Slot { start, end, .. } => {
        *start = placement.apply(start.0, start.1);
        *end = placement.apply(end.0, end.1);
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_cam::FilmKind;
  use eitri_gcode::check_grbl_conformance;
  use eitri_project::{
    CutoutOutlineSpec, DatumCorner, DirectionSpec, DrillDefaults, IsolationDefaults, JobOrigin, PaintStrategySpec,
    SpacingSpec, TabPlacementSpec,
  };

  const GERBER: &str = include_str!("../../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
  const EXCELLON: &str = include_str!("../../../fixtures/synthetic/excellon/metric_leading.drl");
  const SVG: &str = include_str!("../../../fixtures/synthetic/svg/shapes.svg");
  const DXF: &str = include_str!("../../../fixtures/synthetic/dxf/entities.dxf");

  fn iso_spec() -> IsolationSpec {
    IsolationSpec { tool_diameter: 0.2, passes: 1, overlap: 0.0, combine: false, direction: DirectionSpec::Climb }
  }

  fn drill_spec() -> DrillSpec {
    DrillSpec { depth: 1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None }
  }

  /// A session with the two-pad Gerber already opened as `"top"`.
  fn session_with_gerber() -> (Session, ObjectId) {
    let mut s = Session::new("board");
    let g = s.open_gerber_str("top", GERBER).expect("open gerber");
    (s, g)
  }

  // --- Open / import ------------------------------------------------------------------------------------------------

  #[test]
  fn open_gerber_adds_a_hydrated_object() {
    let (s, g) = session_with_gerber();
    assert_eq!(s.len(), 1);
    assert_eq!(s.kind(g).unwrap(), ObjectKind::Gerber);
    // The copper cache is populated at open, so a CAM op can read it without a re-parse.
    assert!(!s.region_of(g).unwrap().0.is_empty(), "opened gerber exposes copper");
  }

  #[test]
  fn open_excellon_adds_a_drill_object() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    assert_eq!(s.kind(d).unwrap(), ObjectKind::Excellon);
    assert!(!s.excellon_image_of(d).unwrap().hits.is_empty());
  }

  #[test]
  fn duplicate_object_name_is_rejected_without_polluting_history() {
    let (mut s, _) = session_with_gerber();
    let before = s.can_undo();
    let err = s.open_gerber_str("top", GERBER).unwrap_err();
    assert!(matches!(err, ScriptError::Project(eitri_project::ProjectError::DuplicateName(_))));
    // The rejected add must not have pushed an undo snapshot.
    assert_eq!(s.can_undo(), before);
    assert_eq!(s.len(), 1);
  }

  #[test]
  fn import_svg_and_gcode_produce_geometry_objects() {
    let mut s = Session::new("art");
    let svg = s.import_svg_str("logo", SVG).unwrap();
    assert_eq!(s.kind(svg).unwrap(), ObjectKind::Geometry);
    let dxf = s.import_dxf_str("frame", DXF).unwrap();
    assert_eq!(s.kind(dxf).unwrap(), ObjectKind::Geometry);
    // A tiny hand-written program: rapid, plunge, one cut. The cut becomes a recovered polyline.
    let gc = "G21\nG90\nG0 X0 Y0\nG1 Z-1 F100\nG1 X5 Y0 F200\nG1 X5 Y5\n";
    let g = s.import_gcode_str("repost", gc).unwrap();
    let region = &s.object(g).unwrap().payload;
    match region {
      ObjectPayload::Geometry(geo) => assert!(!geo.polylines.is_empty(), "g-code cut recovered as a polyline"),
      _ => panic!("expected geometry"),
    }
  }

  // --- CAM ops ------------------------------------------------------------------------------------------------------

  #[test]
  fn isolate_produces_a_conformant_cnc_job() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    assert_eq!(s.kind(job).unwrap(), ObjectKind::CncJob);
    let gcode = s.write_gcode(job).unwrap();
    assert!(gcode.contains("G1"), "isolation program cuts");
    assert!(check_grbl_conformance(&gcode).is_empty(), "isolation g-code is grbl-conformant");
    // The job records its provenance and operation for regeneration.
    match &s.object(job).unwrap().payload {
      ObjectPayload::CncJob(c) => {
        assert_eq!(c.source, Some(g));
        assert!(matches!(c.operation, CamOperation::Isolation(_)));
        assert_eq!(c.dialect, "grbl");
      }
      _ => unreachable!(),
    }
  }

  #[test]
  fn drilling_produces_a_conformant_cnc_job() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    let job = s.drill(d, drill_spec(), DrillJob::default()).unwrap();
    let gcode = s.write_gcode(job).unwrap();
    assert!(check_grbl_conformance(&gcode).is_empty(), "drilling g-code is grbl-conformant");
  }

  #[test]
  fn paint_and_cutout_produce_conformant_jobs() {
    let (mut s, g) = session_with_gerber();
    let paint_spec = PaintSpec {
      tool_diameter: 0.5,
      overlap: 0.3,
      margin: 0.0,
      direction: DirectionSpec::Climb,
      finish_pass: false,
      strategy: PaintStrategySpec::Concentric,
    };
    let painted = s.paint(g, paint_spec, IsolationJob::default()).unwrap();
    assert!(check_grbl_conformance(&s.write_gcode(painted).unwrap()).is_empty());

    let cutout_spec = CutoutSpec {
      tool_diameter: 2.0,
      tab_width: 3.0,
      tabs: TabPlacementSpec::Count(4),
      margin: 0.0,
      direction: DirectionSpec::Conventional,
      outline: CutoutOutlineSpec::Rectangle {
        min: geo_types::Coord { x: -2.0, y: -2.0 },
        max: geo_types::Coord { x: 12.0, y: 2.0 },
      },
    };
    let cut = s.cutout(cutout_spec, IsolationJob::default()).unwrap();
    assert!(check_grbl_conformance(&s.write_gcode(cut).unwrap()).is_empty());
  }

  #[test]
  fn panelize_and_mirror_produce_geometry_objects() {
    let (mut s, g) = session_with_gerber();
    let panel = s
      .panelize(g, PanelizeSpec { rows: 2, cols: 2, x: SpacingSpec::Gap(5.0), y: SpacingSpec::Gap(5.0) })
      .unwrap();
    assert_eq!(s.kind(panel).unwrap(), ObjectKind::Geometry);
    let mirrored = s.mirror(g, MirrorLineSpec::Vertical(5.0)).unwrap();
    assert_eq!(s.kind(mirrored).unwrap(), ObjectKind::Geometry);
  }

  #[test]
  fn transform_translates_geometry() {
    let (mut s, g) = session_with_gerber();
    let moved = s.translate(g, 100.0, 0.0).unwrap();
    // The moved copy's bounds are shifted +100 in X relative to the source.
    let src = eitri_geo::bounds(&s.region_of(g).unwrap()).unwrap();
    let dst = eitri_geo::bounds(&s.region_of(moved).unwrap()).unwrap();
    assert!((dst.0 - (src.0 + 100.0)).abs() < 1e-6, "translated copy shifted in X");
  }

  // --- Wrong-kind guards --------------------------------------------------------------------------------------------

  #[test]
  fn isolating_a_drill_file_is_a_wrong_kind_error() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    let err = s.isolate(d, iso_spec(), IsolationJob::default()).unwrap_err();
    match err {
      ScriptError::WrongKind { id, actual, .. } => {
        assert_eq!(id, d.0);
        assert_eq!(actual, ObjectKind::Excellon);
      }
      other => panic!("expected WrongKind, got {other:?}"),
    }
  }

  #[test]
  fn asking_for_gcode_of_a_gerber_is_a_wrong_kind_error() {
    let (s, g) = session_with_gerber();
    assert!(matches!(s.write_gcode(g), Err(ScriptError::WrongKind { .. })));
  }

  #[test]
  fn unknown_object_id_is_reported() {
    let s = Session::new("empty");
    assert!(matches!(s.object(ObjectId(999)), Err(ScriptError::UnknownObject(999))));
  }

  // --- Object management + undo -------------------------------------------------------------------------------------

  #[test]
  fn rename_delete_and_undo_round_trip() {
    let (mut s, g) = session_with_gerber();
    s.rename(g, "primary").unwrap();
    assert_eq!(s.object_name(g).unwrap(), "primary");
    assert_eq!(s.id_of("primary"), Some(g));

    s.delete(g).unwrap();
    assert_eq!(s.len(), 0);
    assert!(s.undo(), "undo restores the deleted object");
    assert_eq!(s.len(), 1);
    // Undo again reverts the rename.
    assert!(s.undo());
    assert_eq!(s.object_name(g).unwrap(), "top");
    assert!(s.redo(), "redo re-applies the rename");
    assert_eq!(s.object_name(g).unwrap(), "primary");
  }

  #[test]
  fn deleting_an_unknown_id_errors() {
    let mut s = Session::new("empty");
    assert!(matches!(s.delete(ObjectId(7)), Err(ScriptError::UnknownObject(7))));
  }

  #[test]
  fn set_visible_flips_the_flag_undoably_and_rejects_unknown_ids() {
    let (mut s, g) = session_with_gerber();
    assert!(s.object(g).unwrap().meta.visible, "objects start visible");

    s.set_visible(g, false).unwrap();
    assert!(!s.object(g).unwrap().meta.visible, "the flag lands on the object");
    assert!(s.undo(), "hiding is one undoable edit");
    assert!(s.object(g).unwrap().meta.visible, "undo restores visibility");
    assert!(s.redo());
    assert!(!s.object(g).unwrap().meta.visible);

    assert!(matches!(s.set_visible(ObjectId(999), true), Err(ScriptError::UnknownObject(999))));
  }

  #[test]
  fn set_visible_to_the_current_value_is_a_no_op_that_pushes_no_undo_snapshot() {
    let (mut s, g) = session_with_gerber();
    // Drain the open's own undo entry so the history state is unambiguous.
    while s.undo() {}
    assert!(s.redo());
    let before = s.can_undo();
    s.set_visible(g, true).unwrap();
    assert_eq!(s.can_undo(), before, "a no-op toggle must not pollute the undo history");
  }

  #[test]
  fn film_svg_renders_a_positive_film_from_a_gerber_and_rejects_wrong_kinds() {
    let (mut s, g) = session_with_gerber();
    let svg = s.film_svg(g, &FilmParams::default()).expect("a positive film renders");
    assert!(svg.starts_with("<svg") && svg.contains("<path"), "the copper is drawn: {svg}");

    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    assert!(matches!(s.film_svg(d, &FilmParams::default()), Err(ScriptError::WrongKind { .. })));

    let bad = FilmParams { scale: 0.0, ..FilmParams::default() };
    assert!(s.film_svg(g, &bad).is_err(), "invalid film params surface as an error");
  }

  #[test]
  fn film_svg_negative_uses_the_backend_difference() {
    let (s, g) = session_with_gerber();
    let params = FilmParams { kind: FilmKind::Negative, ..FilmParams::default() };
    let svg = s.film_svg(g, &params).expect("a negative film renders");
    // The negative draws the frame minus the copper, so its path reaches the viewBox corner.
    assert!(svg.contains("M 0.0000 0.0000"), "the negative's frame reaches the film corner: {svg}");
  }

  #[test]
  fn film_svg_ignores_the_board_placement_on_the_stock() {
    // Film is 1:1 artwork: arranging the board on the CNC stock must not offset the exported film, so top/bottom
    // films still share the native origin and register when overlaid (docs review #2).
    let (mut s, g) = session_with_gerber();
    let native = s.film_svg(g, &FilmParams::default()).expect("native film renders");
    s.translate_object(g, 25.0, 40.0).unwrap(); // arrange the board far across the stock
    let after_move = s.film_svg(g, &FilmParams::default()).expect("film still renders after the move");
    assert_eq!(native, after_move, "the film is byte-identical regardless of where the board sits on the stock");
  }

  #[test]
  fn drill_tools_note_lists_a_few_tools_but_caps_many_within_the_line_limit() {
    // A handful of tools list in full.
    let few = drill_tools_note(&[0.8, 1.0, 1.2]);
    assert_eq!(few, "drill tools: 0.800, 1.000, 1.200 mm");
    assert!(few.len() <= HEADER_NOTE_MAX);
    // Many distinct sizes would overrun the grblHAL 256-byte line as one comment, so it summarizes instead of
    // emitting an unbounded list that a strict receiver rejects with error:15 (docs review #4).
    let many: Vec<f64> = (0..60).map(|i| 0.5 + i as f64 * 0.05).collect();
    let note = drill_tools_note(&many);
    assert!(note.len() <= HEADER_NOTE_MAX, "the note stays within the header budget: {} bytes", note.len());
    assert!(note.contains("60 sizes"), "it summarizes the count when it cannot list them all: {note}");
    assert!(note.contains("0.500") && note.contains("3.450"), "and the size range: {note}");
  }

  #[test]
  fn groups_track_membership() {
    let (mut s, g) = session_with_gerber();
    s.create_group("copper").unwrap();
    s.add_to_group("copper", g).unwrap();
    assert_eq!(s.group_members("copper"), Some(vec![g]));
  }

  // --- Dialect ------------------------------------------------------------------------------------------------------

  #[test]
  fn unknown_dialect_is_rejected_and_generic_switches_output() {
    let (mut s, g) = session_with_gerber();
    assert!(matches!(s.set_dialect("nonesuch"), Err(ScriptError::UnknownDialect(_))));
    s.set_dialect("generic").unwrap();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    match &s.object(job).unwrap().payload {
      ObjectPayload::CncJob(c) => assert_eq!(c.dialect, "generic"),
      _ => unreachable!(),
    }
  }

  // --- Threading ----------------------------------------------------------------------------------------------------

  #[test]
  fn a_session_can_move_to_a_worker_thread_and_come_back_with_its_result() {
    // The GUI's op-execution contract: long CAM commands run OFF the UI thread by moving the whole `Session` into a
    // worker (so `&mut self` commands need no locking), then handing it back over a channel with the outcome. This
    // both asserts `Session: Send` at compile time and exercises the actual round trip on a real op.
    let (s, g) = session_with_gerber();
    let worker = std::thread::spawn(move || {
      let mut s = s;
      let result = s.isolate(g, iso_spec(), IsolationJob::default());
      (s, result)
    });
    let (s, result) = worker.join().expect("the worker must not panic");
    let job = result.expect("isolation succeeds on the fixture gerber");
    assert_eq!(s.kind(job).unwrap(), ObjectKind::CncJob, "the returned session carries the committed job");
  }

  // --- Cancellation -------------------------------------------------------------------------------------------------

  #[test]
  fn a_cancelled_token_bails_a_cam_op() {
    let (mut s, g) = session_with_gerber();
    s.cancel_token().cancel();
    let err = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap_err();
    assert!(matches!(err, ScriptError::Engine(eitri_core::Error::Cancelled)), "cancel surfaces as Engine(Cancelled)");
  }

  #[test]
  fn a_fresh_cancel_token_recovers_a_session_after_a_cancel() {
    // Cancellation is one-way on a token, so a GUI that cancels one op must be able to install a FRESH token or
    // every later command on the same session would bail immediately — the recovery seam `set_cancel` provides.
    let (mut s, g) = session_with_gerber();
    s.cancel_token().cancel();
    assert!(s.isolate(g, iso_spec(), IsolationJob::default()).is_err(), "the poisoned token still bails the op");
    s.set_cancel(CancelToken::new());
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).expect("a fresh token un-poisons the session");
    assert_eq!(s.kind(job).unwrap(), ObjectKind::CncJob);
  }

  // --- Persistence --------------------------------------------------------------------------------------------------

  #[test]
  fn project_save_load_round_trip_is_usable() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let saved_gcode = s.write_gcode(job).unwrap();
    let json = s.save_project().unwrap();

    let reloaded = Session::load_project_str(&json).unwrap();
    assert_eq!(reloaded.len(), 2, "gerber + job survive the round-trip");
    // The reloaded Gerber is hydrated, so it can be isolated again to the same conformant output.
    let g2 = reloaded.id_of("top").expect("gerber name preserved");
    assert!(!reloaded.region_of(g2).unwrap().0.is_empty(), "reloaded gerber is hydrated");
    // The persisted job renders the same g-code it was saved with.
    let job2 = reloaded.id_of(&reloaded.object_name(job).unwrap()).unwrap();
    assert_eq!(reloaded.write_gcode(job2).unwrap(), saved_gcode);
  }

  // --- Datum / work-zero --------------------------------------------------------------------------------------------

  /// The first `G0 X.. Y..` rapid (a ring start) of a program, as `(x, y)`; the safe-height `G0 Z..` rapids are
  /// skipped since they carry no X.
  fn first_xy(gcode: &str) -> (f64, f64) {
    for line in gcode.lines() {
      if let Some(rest) = line.strip_prefix("G0 X") {
        let mut parts = rest.split(" Y");
        let x = parts.next().and_then(|s| s.parse::<f64>().ok());
        let y = parts.next().and_then(|s| s.parse::<f64>().ok());
        if let (Some(x), Some(y)) = (x, y) {
          return (x, y);
        }
      }
    }
    panic!("no `G0 X.. Y..` rapid in program:\n{gcode}");
  }

  #[test]
  fn set_datum_shifts_every_coordinate_by_the_resolved_offset() {
    let (mut s, g) = session_with_gerber();
    let native_job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let (nx, ny) = first_xy(&s.write_gcode(native_job).unwrap());

    // A bottom-left datum resolves to the copper's (min_x, min_y); the emitter subtracts it from every coordinate.
    s.set_datum(JobOrigin::Bounds(DatumCorner::BottomLeft), g).unwrap();
    let (ox, oy) = s.datum();
    assert_eq!((ox, oy), eitri_geo::bounds(&s.region_of(g).unwrap()).map(|b| (b.0, b.1)).unwrap());

    let shifted_job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let shifted = s.write_gcode(shifted_job).unwrap();
    let (sx, sy) = first_xy(&shifted);
    assert!(
      (sx - (nx - ox)).abs() < 1e-3 && (sy - (ny - oy)).abs() < 1e-3,
      "datum must shift the ring start by exactly the offset: native ({nx},{ny}) − ({ox},{oy}) got ({sx},{sy})"
    );
    assert!(check_grbl_conformance(&shifted).is_empty(), "datum-shifted output stays grbl-conformant");
  }

  #[test]
  fn a_job_records_the_datum_it_was_posted_with() {
    // The job stores the emit origin so a preview (which re-imports the datum-shifted G-code) can add it back and
    // draw the toolpath over the native-frame source geometry.
    let (mut s, g) = session_with_gerber();
    s.set_datum(JobOrigin::Bounds(DatumCorner::BottomLeft), g).unwrap();
    let datum = s.datum();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    match &s.object(job).unwrap().payload {
      ObjectPayload::CncJob(c) => assert_eq!(c.origin, datum, "the job records its emit origin"),
      _ => unreachable!(),
    }
  }

  #[test]
  fn fit_stock_seeds_a_datum_and_posts_relative_to_the_board_corner() {
    use eitri_project::{DatumCorner, ZReference};
    let (mut s, g) = session_with_gerber();
    let native_job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let native = first_xy(&s.write_gcode(native_job).unwrap());

    s.fit_stock_to(g, 1.6, false).unwrap();
    let stock = s.stock().expect("stock is set");
    assert_eq!(stock.thickness, 1.6);
    assert_eq!(stock.datum, DatumCorner::BottomLeft);
    assert_eq!(stock.z_ref, ZReference::Top);
    // The work origin is the board's bottom-left corner, Z0 at the surface.
    let (ox, oy, oz) = s.work_origin();
    assert_eq!((ox, oy), eitri_geo::bounds(&s.region_of(g).unwrap()).map(|b| (b.0, b.1)).unwrap());
    assert_eq!(oz, 0.0);

    let shifted_job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let shifted = first_xy(&s.write_gcode(shifted_job).unwrap());
    assert!((shifted.0 - (native.0 - ox)).abs() < 1e-3, "stock datum shifts the ring start by the board corner");
  }

  #[test]
  fn a_bottom_z_reference_lifts_the_emitted_z_by_the_thickness() {
    use eitri_project::ZReference;
    let (mut s, g) = session_with_gerber();
    s.fit_stock_to(g, 1.6, false).unwrap();
    let mut stock = s.stock().unwrap();
    stock.z_ref = ZReference::Bottom;
    s.set_stock(Some(stock), false);
    assert_eq!(s.work_origin().2, -1.6, "work Z0 sits one thickness below the surface");
    let job = s.isolate(g, iso_spec(), IsolationJob { cut_depth: 0.1, pass_depth: 0.1, ..IsolationJob::default() }).unwrap();
    let gcode = s.write_gcode(job).unwrap();
    assert!(gcode.contains("G1 Z1.5000"), "a 0.1 mm cut with Z0 at the stock bottom is at +1.5:\n{gcode}");
  }

  #[test]
  fn stock_persists_through_save_and_load() {
    use eitri_project::DatumCorner;
    let (mut s, g) = session_with_gerber();
    s.fit_stock_to(g, 1.6, false).unwrap();
    let json = s.save_project().unwrap();
    let reloaded = Session::load_project_str(&json).unwrap();
    let stock = reloaded.stock().expect("stock round-trips");
    assert_eq!(stock.thickness, 1.6);
    assert_eq!(stock.datum, DatumCorner::BottomLeft);
    assert_eq!(reloaded.work_origin(), s.work_origin(), "the resolved work origin round-trips too");
  }

  #[test]
  fn datum_point_round_trips_and_clears() {
    let (mut s, _) = session_with_gerber();
    assert_eq!(s.datum(), (0.0, 0.0), "a fresh session is in the native frame");
    s.set_datum_point(3.0, -4.0);
    assert_eq!(s.datum(), (3.0, -4.0));
    s.clear_datum();
    assert_eq!(s.datum(), (0.0, 0.0));
  }

  #[test]
  fn datum_persists_through_save_and_load() {
    let (mut s, _) = session_with_gerber();
    s.set_datum_point(12.5, -7.25);
    let json = s.save_project().unwrap();
    let reloaded = Session::load_project_str(&json).unwrap();
    assert_eq!(reloaded.datum(), (12.5, -7.25), "the datum round-trips with the project");
  }

  #[test]
  fn set_datum_against_a_drill_reference_is_a_wrong_kind_error() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    assert!(s.set_datum(JobOrigin::Bounds(DatumCorner::Center), d).is_err(), "drills have no region to bound");
  }

  #[test]
  fn changing_the_datum_flags_posted_jobs_stale_and_undoes_together() {
    // #3: the datum is a project-level input every job baked in, so moving it must stale every posted job — and the
    // whole thing is ONE undoable edit that reverts the datum and the staleness in lockstep.
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    assert!(!is_stale(&s, job), "a freshly posted job is up to date");

    s.set_datum_point(5.0, 5.0);
    assert!(is_stale(&s, job), "changing the work-zero stales every posted job");
    assert_eq!(s.datum(), (5.0, 5.0), "the datum moved");

    assert!(s.undo(), "the datum change is one undoable edit");
    assert_eq!(s.datum(), (0.0, 0.0), "undo restores the native datum");
    assert!(!is_stale(&s, job), "and clears the staleness it flagged — datum and stale revert together");
  }

  #[test]
  fn committing_an_unchanged_setup_is_a_no_op_that_pushes_no_undo_entry() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    assert!(!s.can_redo());
    s.set_datum_point(0.0, 0.0); // identical to the native frame already in force
    assert!(!is_stale(&s, job), "a no-op datum commit does not stale jobs");
    assert!(s.undo(), "the only undoable edit is the isolate, not a phantom datum change");
    // Undoing the isolate removes the job entirely; a phantom datum entry would have been undone first instead.
    assert!(s.object(job).is_err(), "one undo stepped back to before the job, proving no datum entry sat on top");
  }

  #[test]
  fn a_coalesced_stock_drag_is_one_undo_entry() {
    // A stock spinner re-commits every frame of a drag: the first frame starts a fresh entry (coalesce=false) and the
    // rest fold in (coalesce=true), so the whole drag is a single undo step (docs review #1).
    let (mut s, g) = session_with_gerber();
    s.fit_stock_to(g, 1.0, false).unwrap(); // drag start
    for thickness in 2..=5 {
      s.fit_stock_to(g, thickness as f64, true).unwrap(); // continuation frames fold in
    }
    assert_eq!(s.stock().map(|st| st.thickness), Some(5.0), "the last drag value is in force");
    assert!(s.undo(), "one undo reverts the entire coalesced drag");
    assert!(s.stock().is_none(), "a single undo returns to no stock, not one thickness step back");
  }

  #[test]
  fn discrete_setup_changes_are_independent_undo_entries() {
    // The over-coalescing fix (docs review #1): distinct deliberate setup actions (coalesce=false) must NOT merge,
    // even back-to-back with no object edit between, so one undo reverts exactly one action.
    let (mut s, _g) = session_with_gerber();
    s.set_datum_point(3.0, 3.0);
    s.set_datum_point(7.0, 7.0); // a separate discrete action right after — its own entry, not a coalesce
    assert_eq!(s.datum(), (7.0, 7.0));
    assert!(s.undo(), "undo the second datum change");
    assert_eq!(s.datum(), (3.0, 3.0), "reverts only the later change, not both");
  }

  // --- Placement (move on the stock) --------------------------------------------------------------------------------

  #[test]
  fn translate_object_moves_in_place_and_is_undoable() {
    let (mut s, id) = session_with_gerber();
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::IDENTITY, "a fresh object sits at its native origin");
    s.translate_object(id, 3.0, -2.0).unwrap();
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::translate(3.0, -2.0), "the move composes onto identity");
    // A second translate composes onto the first rather than replacing it.
    s.translate_object(id, 1.0, 0.0).unwrap();
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::translate(4.0, -2.0), "moves accumulate");
    assert!(s.undo(), "the move is one undoable edit");
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::translate(3.0, -2.0), "undo steps back one move");
  }

  #[test]
  fn translate_object_flags_dependent_jobs_stale() {
    // The sibling of `move_group`: repositioning a source through `translate_object` must also stale the jobs posted
    // from it, so no placement-write path leaves a job silently out of date (docs follow-up #2).
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    assert!(!is_stale(&s, job), "a freshly posted job is up to date");
    s.translate_object(g, 4.0, -1.0).unwrap();
    assert!(is_stale(&s, job), "translating the source stales its dependent job");
  }

  #[test]
  fn set_placement_flags_dependent_jobs_stale() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    s.set_placement(g, Affine::translate(7.0, 2.0)).unwrap();
    assert!(is_stale(&s, job), "an explicit set_placement stales the dependent job too");
  }

  #[test]
  fn a_no_op_placement_pushes_no_undo_entry_and_flags_nothing_stale() {
    // A net-zero move (translate_object(id, 0, 0), or set_placement to the current value) must not push an undo entry
    // or falsely flag toolpaths stale — the rebuilt G-code would be byte-identical (docs review #5).
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    assert!(!s.can_redo());
    s.translate_object(g, 0.0, 0.0).unwrap();
    assert!(!is_stale(&s, job), "a zero translate does not stale the dependent job");
    s.set_placement(g, Affine::IDENTITY).unwrap(); // still the current placement — another no-op
    assert!(!is_stale(&s, job), "re-setting the current placement is a no-op too");
    // The only undoable edit is the isolate; a phantom placement entry would be undone first.
    assert!(s.undo(), "undo steps back to before the job");
    assert!(s.object(job).is_err(), "one undo reached pre-job state, proving no no-op placement entry sat on top");
  }

  #[test]
  fn set_placement_rejects_a_non_rigid_transform() {
    // A scale/shear placement would move a region and its drill centers by different amounts, so CAM and preview
    // would diverge — the placement seam only accepts rigid motions (docs follow-up #4).
    let (mut s, g) = session_with_gerber();
    let err = s.set_placement(g, Affine::scale(2.0, 2.0)).unwrap_err();
    assert!(matches!(err, ScriptError::InvalidArgument(_)), "a scale is rejected, not applied: {err:?}");
    assert_eq!(s.object(g).unwrap().meta.placement, Affine::IDENTITY, "the rejected placement never mutated the object");
  }

  #[test]
  fn a_moved_source_region_is_placed_before_a_cam_op_sees_it() {
    // The CAM path reads geometry through `region_of`, which must return the PLACED region — so isolating a moved
    // board emits a toolpath shifted by the same vector. We prove it end-to-end through the rendered G-code bounds.
    let (mut s, id) = session_with_gerber();
    let native = s.isolate(id, iso_spec(), IsolationJob::default()).unwrap();
    let native_gcode = s.write_gcode(native).unwrap();
    s.translate_object(id, 25.0, 10.0).unwrap();
    let moved = s.isolate(id, iso_spec(), IsolationJob::default()).unwrap();
    let moved_gcode = s.write_gcode(moved).unwrap();
    let (nx, ny) = first_xy(&native_gcode);
    let (mx, my) = first_xy(&moved_gcode);
    assert!((mx - (nx + 25.0)).abs() < 1e-3, "the moved toolpath's first ring X shifts by dx: {nx} -> {mx}");
    assert!((my - (ny + 10.0)).abs() < 1e-3, "and its first ring Y by dy: {ny} -> {my}");
  }

  #[test]
  fn a_moved_drill_source_is_placed_before_drilling() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    let native = s.drill(d, drill_spec(), DrillJob::default()).unwrap();
    let native_gcode = s.write_gcode(native).unwrap();
    s.translate_object(d, -5.0, 8.0).unwrap();
    let moved = s.drill(d, drill_spec(), DrillJob::default()).unwrap();
    let moved_gcode = s.write_gcode(moved).unwrap();
    let (nx, ny) = first_xy(&native_gcode);
    let (mx, my) = first_xy(&moved_gcode);
    assert!((mx - (nx - 5.0)).abs() < 1e-3, "drilled hits shift by dx: {nx} -> {mx}");
    assert!((my - (ny + 8.0)).abs() < 1e-3, "and by dy: {ny} -> {my}");
  }

  // --- Groups & group moves -----------------------------------------------------------------------------------------

  /// Whether a CNC job is currently flagged stale.
  fn is_stale(s: &Session, job: ObjectId) -> bool {
    matches!(&s.object(job).unwrap().payload, ObjectPayload::CncJob(j) if j.stale)
  }

  #[test]
  fn moving_a_group_moves_every_member_and_flags_dependent_jobs_stale() {
    let mut s = Session::new("board");
    let top = s.open_gerber_str("top", GERBER).unwrap();
    let drills = s.open_excellon_str("drills", EXCELLON).unwrap();
    s.create_group("board").unwrap();
    s.add_to_group("board", top).unwrap();
    s.add_to_group("board", drills).unwrap();
    let job = s.isolate(top, iso_spec(), IsolationJob::default()).unwrap();
    assert!(!is_stale(&s, job), "a freshly posted job is up to date");

    s.move_group(top, 12.0, -3.0, true).unwrap();
    // Grabbing ONE layer moves the whole registered board.
    assert_eq!(s.object(top).unwrap().meta.placement, Affine::translate(12.0, -3.0), "the grabbed layer moves");
    assert_eq!(s.object(drills).unwrap().meta.placement, Affine::translate(12.0, -3.0), "its sibling moves too");
    assert!(is_stale(&s, job), "a job whose source moved is now stale");
  }

  #[test]
  fn moving_an_ungrouped_object_moves_only_it() {
    let mut s = Session::new("board");
    let a = s.open_gerber_str("a", GERBER).unwrap();
    let b = s.open_gerber_str("b", GERBER).unwrap();
    s.move_group(a, 5.0, 0.0, true).unwrap();
    assert_eq!(s.object(a).unwrap().meta.placement, Affine::translate(5.0, 0.0), "the ungrouped object moves");
    assert_eq!(s.object(b).unwrap().meta.placement, Affine::IDENTITY, "an unrelated object stays put");
  }

  #[test]
  fn a_coalesced_drag_is_one_undoable_move() {
    let (mut s, id) = session_with_gerber();
    s.move_group(id, 1.0, 0.0, true).unwrap(); // gesture start: pushes one snapshot
    s.move_group(id, 1.0, 0.0, false).unwrap(); // continuation: no new snapshot
    s.move_group(id, 1.0, 0.0, false).unwrap();
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::translate(3.0, 0.0), "the drag accumulated");
    assert!(s.undo(), "the whole drag is one undoable move");
    assert_eq!(s.object(id).unwrap().meta.placement, Affine::IDENTITY, "one undo reverts the entire drag");
  }

  #[test]
  fn imported_layers_auto_join_one_locked_group_and_move_together() {
    let mut s = Session::new("board");
    let top = s.open_gerber_str("top", GERBER).unwrap();
    s.add_to_import_group(top).unwrap();
    let drills = s.open_excellon_str("drills", EXCELLON).unwrap();
    s.add_to_import_group(drills).unwrap();
    assert_eq!(s.group_of(top).as_deref(), Some(Session::IMPORT_GROUP), "the first import creates the group");
    assert_eq!(s.group_of(drills).as_deref(), Some(Session::IMPORT_GROUP), "later imports join it");
    assert_eq!(s.groups().len(), 1, "there is exactly one managed import group");

    s.move_group(drills, 8.0, 8.0, true).unwrap();
    let moved = s.object(top).unwrap().meta.placement;
    assert_eq!(moved, Affine::translate(8.0, 8.0), "grabbing the drills moves the copper too");
  }

  // --- Rebuild ------------------------------------------------------------------------------------------------------

  #[test]
  fn rebuild_recomputes_gcode_after_a_move_and_clears_stale() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let before = first_xy(&s.write_gcode(job).unwrap());
    s.move_group(g, 20.0, 5.0, true).unwrap();
    assert!(is_stale(&s, job), "moving the source staled the job");

    s.rebuild_job(job).unwrap();
    assert!(!is_stale(&s, job), "rebuild clears the stale flag");
    let after = first_xy(&s.write_gcode(job).unwrap());
    assert!((after.0 - (before.0 + 20.0)).abs() < 1e-3, "rebuilt g-code tracks the moved source in X: {after:?}");
    assert!((after.1 - (before.1 + 5.0)).abs() < 1e-3, "and in Y");
  }

  #[test]
  fn rebuild_keeps_the_jobs_identity_and_source() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let name = s.object_name(job).unwrap();
    s.rebuild_job(job).unwrap();
    assert_eq!(s.object_name(job).unwrap(), name, "rebuild is in place: same id keeps the same name");
    match &s.object(job).unwrap().payload {
      ObjectPayload::CncJob(j) => assert_eq!(j.source, Some(g), "the source back-reference survives a rebuild"),
      _ => panic!("still a CNC job"),
    }
  }

  #[test]
  fn a_persisted_job_can_still_be_rebuilt_after_save_load() {
    // The emission params must round-trip so a job loaded from disk is still rebuildable.
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let name = s.object_name(job).unwrap();
    let json = s.save_project().unwrap();
    let mut reloaded = Session::load_project_str(&json).unwrap();
    let job2 = reloaded.id_of(&name).unwrap();
    reloaded.rebuild_job(job2).expect("the persisted emission enables a rebuild after reload");
    assert!(!reloaded.write_gcode(job2).unwrap().is_empty(), "the rebuilt job has g-code");
  }

  #[test]
  fn rebuilding_a_non_job_is_a_wrong_kind_error() {
    let (mut s, g) = session_with_gerber();
    assert!(s.rebuild_job(g).is_err(), "a Gerber source is not a rebuildable CNC job");
  }

  // --- Tool notes in the G-code header ------------------------------------------------------------------------------

  #[test]
  fn isolation_gcode_notes_the_tool_diameter_in_the_header() {
    let (mut s, g) = session_with_gerber();
    let job = s.isolate(g, iso_spec(), IsolationJob::default()).unwrap();
    let gcode = s.write_gcode(job).unwrap();
    let head: String = gcode.lines().take(5).collect::<Vec<_>>().join("\n");
    assert!(head.contains("tool diameter 0.200 mm"), "the header must note the isolation tool size:\n{head}");
  }

  #[test]
  fn drilling_gcode_summarizes_tool_sizes_in_the_header() {
    let mut s = Session::new("board");
    let d = s.open_excellon_str("drills", EXCELLON).unwrap();
    let job = s.drill(d, drill_spec(), DrillJob::default()).unwrap();
    let gcode = s.write_gcode(job).unwrap();
    let head: String = gcode.lines().take(5).collect::<Vec<_>>().join("\n");
    assert!(head.contains("drill tools:"), "the drill header must summarize its tool sizes:\n{head}");
  }

  // --- Tool database ------------------------------------------------------------------------------------------------

  #[test]
  fn tool_db_seeds_specs_and_round_trips() {
    let mut s = Session::new("board");
    let id = s.add_tool(ToolEntry {
      id: ToolId(0),
      name: "0.2mm v-bit".to_string(),
      diameter: eitri_core::Length::from_mm(0.2),
      isolation: IsolationDefaults::default(),
      drilling: DrillDefaults::default(),
    });
    let spec = s.tool_isolation_spec(id).unwrap();
    assert!((spec.tool_diameter - 0.2).abs() < 1e-9);

    let json = s.save_tool_db().unwrap();
    let mut s2 = Session::new("other");
    s2.load_tool_db(&json).unwrap();
    assert_eq!(s2.tools().len(), 1);
    assert!(s2.tool_isolation_spec(id).is_ok());
  }
}
