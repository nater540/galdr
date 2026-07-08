//! The object/document model: an [`Object`] is shared [`ObjectMeta`] plus a variant [`ObjectPayload`].
//!
//! FlatCAM modelled its objects as a Qt class hierarchy (`FlatCAMGerber`, `FlatCAMExcellon`, `FlatCAMGeometry`,
//! `FlatCAMCNCjob`). Rust models the same closed set better as an enum with match-based dispatch (plan §9, §14): the
//! metadata every object shares lives in one struct, and each variant carries only its own payload.
//!
//! ## What is persisted vs re-derived
//!
//! The payloads are deliberately split by *whether the data is cheap and authoritative or heavy and derivable*:
//! - **Gerber / Excellon** persist their **embedded source text** and re-parse the copper/drill geometry on load
//!   (see [`crate::hydrate`]). The source is an order of magnitude smaller than the assembled `MultiPolygon`, it is
//!   the ground truth, and a deterministic re-parse means a later parser fix improves already-saved projects.
//! - **Geometry** persists its **materialized `geo-types` geometry** directly (via `geo-types`' own `serde`) — a
//!   generated or hand-edited shape has no source file to re-derive from, so the geometry *is* the truth.
//! - **CncJob** persists the **rendered G-code lines** — that text is the deliverable and is cheap to store, so
//!   there is no need to persist the intermediate toolpath geometry; the operation parameters are kept so the job
//!   can be regenerated from its parent object.

use crate::serde_ext;
use eitri_cam::{
  Boundary, Concentric, CutoutOutline, CutoutParams, DrillParams, IsolationParams, MillingDirection, MirrorLine,
  PanelSpec, PaintParams, PaintStrategy, Point, Raster, Seed, Spacing, TabPlacement,
};
use eitri_core::{Affine, Unit};
use eitri_geo::JoinType;
use eitri_gcode::{DrillJob, IsolationJob, Program};
use eitri_gerber::GerberImage;
use eitri_excellon::ExcellonImage;
use eitri_import::ImportedGeometry;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::id::ObjectId;

/// A project object: shared metadata plus a type-specific payload.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Object {
  /// Name, units, placement, visibility — everything common to every object kind.
  pub meta: ObjectMeta,
  /// The variant-specific data.
  pub payload: ObjectPayload,
}

impl Object {
  /// The kind discriminant, for match-free callers (UI icons, filtering) that only need the variant.
  pub fn kind(&self) -> ObjectKind {
    match &self.payload {
      ObjectPayload::Gerber(_) => ObjectKind::Gerber,
      ObjectPayload::Excellon(_) => ObjectKind::Excellon,
      ObjectPayload::Geometry(_) => ObjectKind::Geometry,
      ObjectPayload::CncJob(_) => ObjectKind::CncJob,
    }
  }
}

/// Metadata shared by every object variant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectMeta {
  /// Stable, unique handle assigned by the owning [`crate::ObjectCollection`]. Never reused within a collection.
  pub id: ObjectId,
  /// Human-facing name; unique within a collection and the key for name-based lookup.
  pub name: String,
  /// The unit the object was authored in (all stored coordinates are millimetres regardless — this is display intent).
  #[serde(with = "serde_ext::unit")]
  pub units: Unit,
  /// Placement transform applied to the object's geometry in world space.
  #[serde(with = "serde_ext::affine")]
  pub placement: Affine,
  /// Whether the object is shown in the (future) canvas / project tree.
  pub visible: bool,
  /// Free-form operator notes; empty by default.
  #[serde(default)]
  pub notes: String,
}

impl ObjectMeta {
  /// A metadata block with sensible defaults: identity placement, visible, no notes. The id is filled in by the
  /// collection at insertion time, so callers pass [`ObjectId`]`(0)` as a placeholder.
  pub fn new(name: impl Into<String>, units: Unit) -> ObjectMeta {
    ObjectMeta {
      id: ObjectId(0),
      name: name.into(),
      units,
      placement: Affine::IDENTITY,
      visible: true,
      notes: String::new(),
    }
  }
}

/// The discriminant of an [`ObjectPayload`], without any data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObjectKind {
  /// A parsed Gerber layer.
  Gerber,
  /// A parsed Excellon drill program.
  Excellon,
  /// Vector geometry (imported or generated).
  Geometry,
  /// A generated CNC job (rendered G-code).
  CncJob,
}

/// The variant-specific payload of an [`Object`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ObjectPayload {
  /// A Gerber layer: embedded source, copper re-derived on load.
  Gerber(GerberObject),
  /// An Excellon drill program: embedded source, hits re-derived on load.
  Excellon(ExcellonObject),
  /// Vector geometry, persisted directly.
  Geometry(GeometryObject),
  /// A CNC job: rendered G-code plus the operation that produced it.
  CncJob(CncJobObject),
}

/// A Gerber layer. The RS-274X source is authoritative and persisted; the parsed [`GerberImage`] is a re-derivable
/// cache that is not written to disk and is `None` until [`crate::Project::hydrate`] re-parses the source.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GerberObject {
  /// The embedded RS-274X source text (shared cheaply for undo snapshots).
  pub source: Arc<str>,
  /// The parsed copper/aperture model — re-derived from `source`, never serialized.
  #[serde(skip)]
  pub image: Option<GerberImage>,
}

impl GerberObject {
  /// Wrap Gerber source with an empty (un-hydrated) parse cache.
  pub fn new(source: impl Into<Arc<str>>) -> GerberObject {
    GerberObject { source: source.into(), image: None }
  }
}

/// An Excellon drill program. The source is authoritative and persisted; the parsed [`ExcellonImage`] is a
/// re-derivable cache, not serialized, and `None` until hydration re-parses it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExcellonObject {
  /// The embedded Excellon source text.
  pub source: Arc<str>,
  /// The parsed tool table / hits — re-derived from `source`, never serialized.
  #[serde(skip)]
  pub image: Option<ExcellonImage>,
}

impl ExcellonObject {
  /// Wrap Excellon source with an empty (un-hydrated) parse cache.
  pub fn new(source: impl Into<Arc<str>>) -> ExcellonObject {
    ExcellonObject { source: source.into(), image: None }
  }
}

/// Vector geometry, persisted directly as `geo-types` polygons and polylines. Unlike Gerber/Excellon there is no
/// source to re-derive from — generated or edited geometry *is* the authoritative data — so it is serialized in full.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometryObject {
  /// Closed regions.
  pub polygons: Vec<Polygon<f64>>,
  /// Open paths.
  pub polylines: Vec<LineString<f64>>,
  /// Where the geometry came from (provenance only; the geometry above is authoritative).
  pub origin: GeometryOrigin,
}

impl GeometryObject {
  /// Build a geometry object from an [`ImportedGeometry`] preview, recording the import provenance.
  pub fn from_imported(geometry: &ImportedGeometry, origin: GeometryOrigin) -> GeometryObject {
    GeometryObject {
      polygons: geometry.polygons.clone(),
      polylines: geometry.polylines.clone(),
      origin,
    }
  }
}

/// Provenance of a [`GeometryObject`]: generated within Eitri, or imported from an external vector format.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GeometryOrigin {
  /// Produced by an Eitri operation or hand-editing — no external source.
  Generated,
  /// Imported from a vector file; the original source is kept as provenance so it can be re-imported.
  Imported {
    /// The source format.
    format: ImportFormat,
    /// The original file text (kept for re-import; the materialized geometry above is what edits mutate).
    source: Arc<str>,
  },
}

/// The vector formats [`eitri_import`] can bring in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImportFormat {
  /// Scalable Vector Graphics.
  Svg,
  /// AutoCAD DXF.
  Dxf,
  /// Existing G-code, walked back into geometry.
  Gcode,
}

/// A generated CNC job. The rendered G-code lines are the persisted deliverable; the [`CamOperation`] records the
/// parameters that produced them so the job can be regenerated from its parent object.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CncJobObject {
  /// The rendered G-code lines (shared cheaply for undo snapshots).
  pub gcode: Arc<[String]>,
  /// The postprocessor dialect that rendered the G-code (e.g. `"grblHAL"`).
  pub dialect: String,
  /// The object this job was generated from, if any (a stable back-reference into the same collection).
  pub source: Option<ObjectId>,
  /// The operation and parameters that produced the job.
  pub operation: CamOperation,
  /// The datum (work-zero) offset, in the board's native frame, this job was posted with: the emitter already
  /// subtracted it from the stored `gcode`, so a preview that re-imports the G-code adds it back to place the
  /// toolpath over the native-frame source. `(0.0, 0.0)` for a job posted in the native frame. `#[serde(default)]`
  /// so jobs written before the datum existed load as native.
  #[serde(default)]
  pub origin: (f64, f64),
  /// Whether the job is out of date with its source — set when the source object is moved (or otherwise changed)
  /// after the job was posted, so the UI can flag it for a rebuild. `#[serde(default)]` so a freshly posted job (and
  /// any project written before this field existed) loads as up to date.
  #[serde(default)]
  pub stale: bool,
  /// The emission parameters (depths, feeds, spindle, travel height) the job was posted with, kept alongside the
  /// [`CamOperation`] spec so a rebuild fully reproduces the G-code — not just the geometry. `#[serde(default)]` (so
  /// jobs written before this field existed load as `None`); a `None` job cannot be rebuilt from stored params alone.
  #[serde(default)]
  pub emission: Option<JobEmission>,
}

/// The emission (cutting) parameters a CNC job was posted with, tagged by which emitter produced it, so a rebuild can
/// re-run the exact same job — feeds, depths, spindle and all — against the (possibly moved) source. Paired
/// with the [`CamOperation`] spec: the spec is the geometry intent, this is the machine intent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum JobEmission {
  /// An isolation-style contour job (isolation, paint, non-copper, cutout all emit through [`IsolationJob`]).
  Isolation(IsolationJob),
  /// A drilling job.
  Drill(DrillJob),
}

impl CncJobObject {
  /// Capture an emitted [`Program`]'s rendered lines as a persisted CNC job posted with datum `origin`.
  pub fn from_program(
    program: &Program,
    dialect: impl Into<String>,
    source: Option<ObjectId>,
    operation: CamOperation,
    origin: (f64, f64),
    emission: Option<JobEmission>,
  ) -> CncJobObject {
    CncJobObject {
      gcode: Arc::from(program.lines().to_vec()),
      dialect: dialect.into(),
      source,
      operation,
      origin,
      stale: false,
      emission,
    }
  }

  /// The G-code rendered back to a single string with newline terminators.
  pub fn render(&self) -> String {
    let mut out = String::new();
    for line in self.gcode.iter() {
      out.push_str(line);
      out.push('\n');
    }
    out
  }
}

/// A CAM operation plus the operator-facing parameters that produced an object. These are the curated *intent*
/// knobs the UI edits and scripting sets — not the full internal `eitri-cam` params (corner joins, miter limits and
/// the like are filled from defaults when the operation is re-run). The enum is `#[non_exhaustive]` so later phases
/// can add further operations without a breaking change.
///
/// The input *region* an operation clears or profiles (a copper layer, a source geometry) is not duplicated here — it
/// is reached through [`CncJobObject::source`] by [`ObjectId`], mirroring the Gerber/Excellon "persist source and
/// re-derive" contract. Only geometry the operation's *parameters* genuinely own — an explicit non-copper frame
/// ([`BoundarySpec::Region`]), a hand-drawn cutout outline ([`CutoutOutlineSpec::Geometry`]), or explicit alignment
/// holes — is serialized inline via `geo-types`' `serde`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum CamOperation {
  /// Isolation routing.
  Isolation(IsolationSpec),
  /// Drilling.
  Drilling(DrillSpec),
  /// Area clearing (paint) of a source region with a chosen fill strategy.
  Paint(PaintSpec),
  /// Non-copper clearing: paint everything within a boundary that is *not* copper.
  NonCopper(NonCopperSpec),
  /// Board cutout / profile routing with holding tabs.
  Cutout(CutoutSpec),
  /// Panelization: array a source object into a grid.
  Panelize(PanelizeSpec),
  /// Two-sided alignment: a mirror line plus registration holes.
  TwoSided(TwoSidedSpec),
}

impl CamOperation {
  /// The single cutting-tool diameter (millimetres) this operation uses, for a UI/header summary. `None` for
  /// operations without one single tool: drilling runs several bits sized by the source drill file, and
  /// panelize/two-sided produce geometry, not a toolpath.
  pub fn tool_diameter(&self) -> Option<f64> {
    match self {
      CamOperation::Isolation(spec) => Some(spec.tool_diameter),
      CamOperation::Paint(spec) => Some(spec.tool_diameter),
      CamOperation::NonCopper(spec) => Some(spec.paint.tool_diameter),
      CamOperation::Cutout(spec) => Some(spec.tool_diameter),
      CamOperation::Drilling(_) | CamOperation::Panelize(_) | CamOperation::TwoSided(_) => None,
    }
  }
}

/// Operator-facing isolation parameters. Maps to [`IsolationParams`] via [`IsolationSpec::to_params`], which fills
/// the corner-join details from sensible defaults.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IsolationSpec {
  /// Isolation tool diameter (millimetres).
  pub tool_diameter: f64,
  /// Number of concentric passes.
  pub passes: usize,
  /// Fraction each pass overlaps the previous, in `[0, 1)`.
  pub overlap: f64,
  /// Union overlapping same-pass rings so the cutter does not retrace shared boundary.
  pub combine: bool,
  /// Climb vs conventional milling (sets the ring winding).
  pub direction: DirectionSpec,
}

impl IsolationSpec {
  /// Expand to full [`IsolationParams`], filling the corner-join style with the isolation default (round joins).
  pub fn to_params(&self) -> IsolationParams {
    IsolationParams {
      tool_diameter: self.tool_diameter,
      passes: self.passes,
      overlap: self.overlap,
      combine: self.combine,
      direction: self.direction.into(),
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }
}

/// Operator-facing drilling parameters. Maps to [`DrillParams`] via [`DrillSpec::to_params`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DrillSpec {
  /// Total drill depth as a positive magnitude below the surface (millimetres); the emitter negates it to a negative
  /// Z, so a positive value drills down (matching the isolation `cut_depth` and `eitri-cam`'s `DrillParams::depth`).
  pub depth: f64,
  /// Plunge feed rate (mm/min).
  pub feed: f64,
  /// Retract height between hits (millimetres).
  pub retract: f64,
  /// Peck increment (millimetres); `None` drills in one plunge.
  pub peck: Option<f64>,
  /// Dwell at the bottom of each hole (seconds); `None` for no dwell.
  pub dwell: Option<f64>,
}

impl DrillSpec {
  /// Expand to full [`DrillParams`]. Depth is normalized to a positive magnitude here — the single choke point every
  /// drill emission passes through — so a stray negative (a legacy persisted spec, a hand-built Rhai/script spec)
  /// can never reach the negating emitter and air-drill above the stock.
  pub fn to_params(&self) -> DrillParams {
    DrillParams {
      depth: self.depth.abs(),
      feed: self.feed,
      retract: self.retract,
      peck: self.peck,
      dwell: self.dwell,
    }
  }
}

/// Milling direction as a project-owned enum, mapped to [`MillingDirection`] on expansion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DirectionSpec {
  /// Climb milling.
  Climb,
  /// Conventional milling.
  Conventional,
}

impl From<DirectionSpec> for MillingDirection {
  fn from(spec: DirectionSpec) -> MillingDirection {
    match spec {
      DirectionSpec::Climb => MillingDirection::Climb,
      DirectionSpec::Conventional => MillingDirection::Conventional,
    }
  }
}

/// Operator-facing area-clearing (paint) parameters. Maps to [`PaintParams`] via [`PaintSpec::to_params`] (filling
/// the round-join defaults) and to a boxed [`PaintStrategy`] via [`PaintSpec::strategy`]. The region to clear is the
/// job's source object, not a field here.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PaintSpec {
  /// Clearing tool diameter (millimetres).
  pub tool_diameter: f64,
  /// Fraction each pass overlaps the previous, in `[0, 1)`.
  pub overlap: f64,
  /// Inset from the region boundary before filling (millimetres, `>= 0`).
  pub margin: f64,
  /// Climb vs conventional milling (sets the closed-ring winding).
  pub direction: DirectionSpec,
  /// Append a boundary-following finishing pass after the fill.
  pub finish_pass: bool,
  /// Which fill pattern to use.
  pub strategy: PaintStrategySpec,
}

impl PaintSpec {
  /// Expand to full [`PaintParams`], filling the corner-join style with the paint default (round joins).
  pub fn to_params(&self) -> PaintParams {
    PaintParams {
      tool_diameter: self.tool_diameter,
      overlap: self.overlap,
      margin: self.margin,
      direction: self.direction.into(),
      finish_pass: self.finish_pass,
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }

  /// The fill strategy as a boxed [`PaintStrategy`] trait object, ready to hand to [`eitri_cam::paint`].
  pub fn strategy(&self) -> Box<dyn PaintStrategy> {
    self.strategy.to_strategy()
  }
}

/// The fill pattern for an area-clearing operation, mirroring `eitri-cam`'s [`PaintStrategy`] implementors.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PaintStrategySpec {
  /// Inward-offset rings, emitted outer → inner.
  Concentric,
  /// The same rings emitted inner → outer, growing from a seed.
  Seed,
  /// Parallel scan-line (raster) fill at `angle_deg` from the X axis.
  Raster {
    /// Scan-line angle in degrees.
    angle_deg: f64,
  },
}

impl PaintStrategySpec {
  /// Build the concrete boxed [`PaintStrategy`] this spec names.
  pub fn to_strategy(&self) -> Box<dyn PaintStrategy> {
    match self {
      PaintStrategySpec::Concentric => Box::new(Concentric),
      PaintStrategySpec::Seed => Box::new(Seed),
      PaintStrategySpec::Raster { angle_deg } => Box::new(Raster::at_angle(*angle_deg)),
    }
  }
}

/// Operator-facing non-copper-clearing parameters: the boundary to clear within plus the paint pass that clears it.
/// The copper being cleared is the job's source object; only an explicit boundary [`BoundarySpec::Region`] is owned
/// geometry and serialized inline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NonCopperSpec {
  /// How the outer frame that bounds the clearing is defined.
  pub boundary: BoundarySpec,
  /// The paint pass (strategy + tool) that clears the non-copper region.
  pub paint: PaintSpec,
}

impl NonCopperSpec {
  /// The clearing [`Boundary`] for `eitri-cam`.
  pub fn to_boundary(&self) -> Boundary {
    self.boundary.to_boundary()
  }

  /// The paint [`PaintParams`] for the clearing pass.
  pub fn to_params(&self) -> PaintParams {
    self.paint.to_params()
  }

  /// The paint [`PaintStrategy`] for the clearing pass.
  pub fn strategy(&self) -> Box<dyn PaintStrategy> {
    self.paint.strategy()
  }
}

/// How a non-copper clearing's outer frame is defined, mirroring `eitri-cam`'s [`Boundary`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BoundarySpec {
  /// The copper bounding box expanded outward by `margin` millimetres on every side.
  BoundingBox {
    /// Outward expansion of the copper bounding box (millimetres, `>= 0`).
    margin: f64,
  },
  /// An explicit boundary region the operator owns (a traced outline, a hull) — serialized inline.
  Region(MultiPolygon<f64>),
}

impl BoundarySpec {
  /// Map to `eitri-cam`'s [`Boundary`].
  pub fn to_boundary(&self) -> Boundary {
    match self {
      BoundarySpec::BoundingBox { margin } => Boundary::BoundingBox { margin: *margin },
      BoundarySpec::Region(region) => Boundary::Region(region.clone()),
    }
  }
}

/// Operator-facing cutout (board-profile) parameters. Maps to [`CutoutParams`] via [`CutoutSpec::to_params`] and to a
/// [`CutoutOutline`] via [`CutoutSpec::to_outline`]. The outline is genuinely owned by the operation (an explicit
/// rectangle, or a hand-drawn geometry) and is serialized inline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CutoutSpec {
  /// Routing tool diameter (millimetres).
  pub tool_diameter: f64,
  /// Width of the uncut gap left at each tab (millimetres, `>= 0`).
  pub tab_width: f64,
  /// Tab placement around each cut ring.
  pub tabs: TabPlacementSpec,
  /// Extra outward offset beyond the tool radius (millimetres, `>= 0`).
  pub margin: f64,
  /// Climb vs conventional milling (sets the cut-ring winding).
  pub direction: DirectionSpec,
  /// The board outline to cut around.
  pub outline: CutoutOutlineSpec,
}

impl CutoutSpec {
  /// Expand to full [`CutoutParams`], filling the corner-join style with the cutout default (round joins).
  pub fn to_params(&self) -> CutoutParams {
    CutoutParams {
      tool_diameter: self.tool_diameter,
      tab_width: self.tab_width,
      tabs: self.tabs.to_placement(),
      margin: self.margin,
      direction: self.direction.into(),
      join: JoinType::Round,
      miter_limit: 2.0,
    }
  }

  /// The board [`CutoutOutline`] for `eitri-cam`.
  pub fn to_outline(&self) -> CutoutOutline {
    self.outline.to_outline()
  }
}

/// Where holding tabs are placed around each cut ring, mirroring `eitri-cam`'s [`TabPlacement`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TabPlacementSpec {
  /// `n` evenly-spaced tabs, the first centred at the ring start.
  Count(usize),
  /// Tabs centred at these fractions of the ring perimeter (each in `[0, 1)`).
  AtFractions(Vec<f64>),
}

impl TabPlacementSpec {
  /// Map to `eitri-cam`'s [`TabPlacement`].
  pub fn to_placement(&self) -> TabPlacement {
    match self {
      TabPlacementSpec::Count(n) => TabPlacement::Count(*n),
      TabPlacementSpec::AtFractions(fracs) => TabPlacement::AtFractions(fracs.clone()),
    }
  }
}

/// The board outline to cut around, mirroring `eitri-cam`'s [`CutoutOutline`]. The [`CutoutOutlineSpec::Geometry`]
/// variant owns its silhouette and is serialized inline; rectangle corners are stored as `geo-types` [`Coord`]s.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CutoutOutlineSpec {
  /// A simple axis-aligned rectangle from `min` to `max` (millimetres).
  Rectangle {
    /// Lower-left corner.
    min: Coord<f64>,
    /// Upper-right corner.
    max: Coord<f64>,
  },
  /// A geometry-derived outline; its merged silhouette exteriors are each cut.
  Geometry(MultiPolygon<f64>),
}

impl CutoutOutlineSpec {
  /// Map to `eitri-cam`'s [`CutoutOutline`], turning stored corners into cam [`Point`]s.
  pub fn to_outline(&self) -> CutoutOutline {
    match self {
      CutoutOutlineSpec::Rectangle { min, max } => {
        CutoutOutline::Rectangle { min: Point::new(min.x, min.y), max: Point::new(max.x, max.y) }
      }
      CutoutOutlineSpec::Geometry(geometry) => CutoutOutline::Geometry(geometry.clone()),
    }
  }
}

/// Operator-facing panelization parameters. Maps to [`PanelSpec`] via [`PanelizeSpec::to_params`]. The source object
/// being arrayed is the job's source, not a field here.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PanelizeSpec {
  /// Number of rows (Y direction); at least one.
  pub rows: usize,
  /// Number of columns (X direction); at least one.
  pub cols: usize,
  /// Horizontal spacing between columns.
  pub x: SpacingSpec,
  /// Vertical spacing between rows.
  pub y: SpacingSpec,
}

impl PanelizeSpec {
  /// Map to `eitri-cam`'s [`PanelSpec`].
  pub fn to_params(&self) -> PanelSpec {
    PanelSpec { rows: self.rows, cols: self.cols, x: self.x.into(), y: self.y.into() }
  }
}

/// Grid spacing along one axis, mirroring `eitri-cam`'s [`Spacing`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SpacingSpec {
  /// A clear band of this width (millimetres) between copies; the step is `source_extent + gap`.
  Gap(f64),
  /// The centre-to-centre step (millimetres) between copies, independent of the source extent.
  Pitch(f64),
}

impl From<SpacingSpec> for Spacing {
  fn from(spec: SpacingSpec) -> Spacing {
    match spec {
      SpacingSpec::Gap(g) => Spacing::Gap(g),
      SpacingSpec::Pitch(p) => Spacing::Pitch(p),
    }
  }
}

/// Operator-facing two-sided-alignment parameters: the mirror line for the bottom layer plus the registration holes
/// to drill on both faces. The base hole centres are owned by the operation and stored as `geo-types` [`Coord`]s.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TwoSidedSpec {
  /// The axis the flip mirrors about.
  pub mirror: MirrorLineSpec,
  /// The registration-hole centres the operator placed (millimetres).
  pub alignment_holes: Vec<Coord<f64>>,
  /// Registration-drill diameter (millimetres).
  pub hole_diameter: f64,
}

impl TwoSidedSpec {
  /// The mirror line as `eitri-cam`'s [`MirrorLine`].
  pub fn to_mirror_line(&self) -> MirrorLine {
    self.mirror.to_mirror_line()
  }

  /// The alignment-hole centres as cam [`Point`]s, ready for [`eitri_cam::alignment_holes`].
  pub fn alignment_hole_points(&self) -> Vec<Point> {
    self.alignment_holes.iter().map(|c| Point::new(c.x, c.y)).collect()
  }
}

/// The axis a two-sided flip mirrors about, mirroring `eitri-cam`'s [`MirrorLine`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MirrorLineSpec {
  /// Reflect about the vertical line `x = value` (a left-right flip).
  Vertical(f64),
  /// Reflect about the horizontal line `y = value` (a top-bottom flip).
  Horizontal(f64),
}

impl MirrorLineSpec {
  /// Map to `eitri-cam`'s [`MirrorLine`].
  pub fn to_mirror_line(&self) -> MirrorLine {
    match self {
      MirrorLineSpec::Vertical(x) => MirrorLine::Vertical(*x),
      MirrorLineSpec::Horizontal(y) => MirrorLine::Horizontal(*y),
    }
  }
}
