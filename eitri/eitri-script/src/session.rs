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
use eitri_gcode::{DrillJob, IsolationJob, Postprocessor, Program, Registry, emit_drilling, emit_isolation};
use eitri_geo::DefaultBackend;
use eitri_project::{
  CamOperation, CncJobObject, DrillSpec, ExcellonObject, GeometryObject, GeometryOrigin, GerberObject, History,
  ImportFormat, IsolationSpec, MirrorLineSpec, NonCopperSpec, Object, ObjectCollection, ObjectId, ObjectKind,
  ObjectMeta, ObjectPayload, PaintSpec, PanelizeSpec, Project, ToolDatabase, ToolEntry, ToolId, load_project,
  save_project,
};
use eitri_project::{CutoutSpec, load_tool_db, save_tool_db};
use eitri_excellon::{ExcellonImage, parse_excellon};
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
    let region = self.region_of(source)?;
    let toolpaths = eitri_cam::isolate(&self.backend, &region, &spec.to_params(), &self.progress, &self.cancel)?;
    let program = {
      let post = self.post()?;
      emit_isolation(&toolpaths, &job, post)
    };
    let base = self.job_name(source, "isolation")?;
    self.store_program(program, Some(source), CamOperation::Isolation(spec), &base)
  }

  /// Drill an Excellon `source`, ordering hits to minimize rapid travel, and emit the drilling program. `spec` is
  /// applied to every tool as its default parameters.
  pub fn drill(&mut self, source: ObjectId, spec: DrillSpec, job: DrillJob) -> Result<ObjectId> {
    let image = self.excellon_image_of(source)?;
    let config = DrillConfig { defaults: spec.to_params(), overrides: BTreeMap::new(), start: Point::new(0.0, 0.0) };
    let plan = eitri_cam::plan_drilling(&image, &config, &TwoOpt::new(), &self.progress, &self.cancel)?;
    let program = {
      let post = self.post()?;
      emit_drilling(&plan, &job, post)
    };
    let base = self.job_name(source, "drilling")?;
    self.store_program(program, Some(source), CamOperation::Drilling(spec), &base)
  }

  /// Area-clear (paint) a copper/geometry `source` with the chosen fill strategy and emit the toolpaths as a job.
  pub fn paint(&mut self, source: ObjectId, spec: PaintSpec, job: IsolationJob) -> Result<ObjectId> {
    let region = self.region_of(source)?;
    let strategy = spec.strategy();
    let result = eitri_cam::paint(
      &region,
      &spec.to_params(),
      strategy.as_ref(),
      &self.backend,
      &self.progress,
      &self.cancel,
    )?;
    let toolpaths = result.toolpaths();
    let program = {
      let post = self.post()?;
      emit_isolation(&toolpaths, &job, post)
    };
    let base = self.job_name(source, "paint")?;
    self.store_program(program, Some(source), CamOperation::Paint(spec), &base)
  }

  /// Clear all non-copper within a boundary around a Gerber/geometry `source`, painting the negative region, and emit
  /// the toolpaths as a job.
  pub fn noncopper(&mut self, source: ObjectId, spec: NonCopperSpec, job: IsolationJob) -> Result<ObjectId> {
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
    let toolpaths = result.toolpaths();
    let program = {
      let post = self.post()?;
      emit_isolation(&toolpaths, &job, post)
    };
    let base = self.job_name(source, "noncopper")?;
    self.store_program(program, Some(source), CamOperation::NonCopper(spec), &base)
  }

  /// Route a board cutout around the outline the `spec` owns (a rectangle or a hand-drawn silhouette), leaving
  /// holding tabs, and emit the profile as a job. The outline is self-contained, so there is no source object.
  pub fn cutout(&mut self, spec: CutoutSpec, job: IsolationJob) -> Result<ObjectId> {
    let result =
      eitri_cam::cutout(&spec.to_outline(), &spec.to_params(), &self.backend, &self.progress, &self.cancel)?;
    let toolpaths = result.toolpaths();
    let program = {
      let post = self.post()?;
      emit_isolation(&toolpaths, &job, post)
    };
    self.store_program(program, None, CamOperation::Cutout(spec), "cutout")
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

  /// Render a copper/geometry `source` as a positive or negative photo-film SVG (see [`eitri_cam::film_svg`]).
  /// Film is vector *output*, not a toolpath, so nothing is committed to the collection — the caller writes the
  /// returned document wherever it wants.
  pub fn film_svg(&self, source: ObjectId, params: &FilmParams) -> Result<String> {
    let region = self.region_of(source)?;
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
    let project = Project { name: self.name.clone(), collection: self.collection().clone() };
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
    Ok(Session {
      name: project.name,
      history: History::new(project.collection),
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

  /// Borrow a tool by id.
  fn tool(&self, id: ToolId) -> Result<&ToolEntry> {
    self.tools.get(id).ok_or_else(|| ScriptError::InvalidArgument(format!("no tool with id {id}")))
  }

  /// The copper/geometry region of a source object, as a `MultiPolygon`. Gerber yields its copper (re-parsing on the
  /// rare chance the cache is empty); Geometry yields its polygons; anything else is a [`ScriptError::WrongKind`].
  fn region_of(&self, id: ObjectId) -> Result<MultiPolygon<f64>> {
    match &self.object(id)?.payload {
      ObjectPayload::Gerber(gerber) => match &gerber.image {
        Some(image) => Ok(image.copper.clone()),
        None => {
          let image =
            parse_gerber(&gerber.source, &self.progress, &self.cancel).map_err(eitri_core::Error::from)?;
          Ok(image.copper)
        }
      },
      ObjectPayload::Geometry(geometry) => Ok(MultiPolygon::new(geometry.polygons.clone())),
      other => Err(self.wrong_kind(id, "a Gerber or Geometry object", other)),
    }
  }

  /// The parsed drill image of an Excellon source, cloned (re-parsing if the cache is empty).
  fn excellon_image_of(&self, id: ObjectId) -> Result<ExcellonImage> {
    match &self.object(id)?.payload {
      ObjectPayload::Excellon(excellon) => match &excellon.image {
        Some(image) => Ok(image.clone()),
        None => {
          let image = parse_excellon(&excellon.source, None, &self.progress, &self.cancel)
            .map_err(eitri_core::Error::from)?;
          Ok(image)
        }
      },
      other => Err(self.wrong_kind(id, "an Excellon object", other)),
    }
  }

  /// Wrap an emitted program as a CNC job and commit it as one undoable edit.
  fn store_program(
    &mut self,
    program: Program,
    source: Option<ObjectId>,
    operation: CamOperation,
    base_name: &str,
  ) -> Result<ObjectId> {
    let dialect = self.dialect.clone();
    let cnc = CncJobObject::from_program(&program, dialect, source, operation);
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

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_cam::FilmKind;
  use eitri_gcode::check_grbl_conformance;
  use eitri_project::{
    CutoutOutlineSpec, DirectionSpec, DrillDefaults, IsolationDefaults, PaintStrategySpec, SpacingSpec,
    TabPlacementSpec,
  };

  const GERBER: &str = include_str!("../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
  const EXCELLON: &str = include_str!("../../fixtures/synthetic/excellon/metric_leading.drl");
  const SVG: &str = include_str!("../../fixtures/synthetic/svg/shapes.svg");
  const DXF: &str = include_str!("../../fixtures/synthetic/dxf/entities.dxf");

  fn iso_spec() -> IsolationSpec {
    IsolationSpec { tool_diameter: 0.2, passes: 1, overlap: 0.0, combine: false, direction: DirectionSpec::Climb }
  }

  fn drill_spec() -> DrillSpec {
    DrillSpec { depth: -1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None }
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
