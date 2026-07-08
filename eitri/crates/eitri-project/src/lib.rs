//! `eitri-project` — the object/document model, versioned persistence, undo, and tool database.
//!
//! This is the integration crate: it holds the outputs of every other Eitri crate as a coherent, serializable
//! project. FlatCAM modelled objects as a Qt class hierarchy and persisted them as pickled Python; Eitri replaces
//! both with an [`Object`] enum dispatched by `match` and a versioned `serde` schema (plan §9, §14).
//!
//! The pieces:
//! - [`Object`] / [`ObjectCollection`] — the object model and the ordered, named, groupable set behind a project tree.
//! - [`Project`] plus [`save_project`] / [`load_project`] — the versioned JSON document and its migration seam.
//! - [`History`] — snapshot-based undo/redo over the collection.
//! - [`ToolDatabase`] plus [`save_tool_db`] / [`load_tool_db`] — a user-global tool library the CAM layer reads from.
//!
//! ## Persisted vs re-derived
//!
//! To keep the on-disk schema small, stable, and self-owned, geometry that can be regenerated is not stored: Gerber
//! and Excellon objects persist their embedded source and re-parse on load ([`Project::hydrate`]); CNC jobs persist
//! their rendered G-code (the deliverable) plus the parameters that produced it. Only vector [`GeometryObject`]s,
//! which have no source to re-derive from, store their `geo-types` geometry directly. See [`object`] for the full
//! rationale.

#![forbid(unsafe_code)]

pub mod collection;
pub mod datum;
pub mod document;
pub mod error;
pub mod history;
pub mod id;
pub mod object;
pub mod project;
mod serde_ext;
pub mod tooldb;

pub use collection::{Group, ObjectCollection};
pub use datum::{DatumCorner, JobOrigin, Stock, ZReference};
pub use document::{SCHEMA_VERSION, load_project, save_project};
pub use error::{ProjectError, Result};
pub use history::{DEFAULT_HISTORY_LIMIT, History};
pub use id::{ObjectId, ToolId};
pub use object::{
  BoundarySpec, CamOperation, CncJobObject, CutoutOutlineSpec, CutoutSpec, DirectionSpec, DrillSpec, ExcellonObject,
  GeometryObject, GeometryOrigin, GerberObject, ImportFormat, IsolationSpec, JobEmission, MirrorLineSpec,
  NonCopperSpec, Object,
  ObjectKind, ObjectMeta, ObjectPayload, PaintSpec, PaintStrategySpec, PanelizeSpec, SpacingSpec, TabPlacementSpec,
  TwoSidedSpec,
};
pub use project::Project;
pub use tooldb::{
  DrillDefaults, IsolationDefaults, TOOL_DB_SCHEMA_VERSION, ToolDatabase, ToolEntry, load_tool_db, save_tool_db,
};
