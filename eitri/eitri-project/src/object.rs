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
use eitri_cam::{DrillParams, IsolationParams, MillingDirection};
use eitri_core::{Affine, Unit};
use eitri_geo::JoinType;
use eitri_gcode::Program;
use eitri_gerber::GerberImage;
use eitri_excellon::ExcellonImage;
use eitri_import::ImportedGeometry;
use geo_types::{LineString, Polygon};
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
}

impl CncJobObject {
  /// Capture an emitted [`Program`]'s rendered lines as a persisted CNC job.
  pub fn from_program(
    program: &Program,
    dialect: impl Into<String>,
    source: Option<ObjectId>,
    operation: CamOperation,
  ) -> CncJobObject {
    CncJobObject {
      gcode: Arc::from(program.lines().to_vec()),
      dialect: dialect.into(),
      source,
      operation,
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
/// can add paint / cutout / panelize operations without a breaking change.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum CamOperation {
  /// Isolation routing.
  Isolation(IsolationSpec),
  /// Drilling.
  Drilling(DrillSpec),
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
  /// Total drill depth (millimetres, negative into the stock as the emitter expects).
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
  /// Expand to full [`DrillParams`].
  pub fn to_params(&self) -> DrillParams {
    DrillParams {
      depth: self.depth,
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
