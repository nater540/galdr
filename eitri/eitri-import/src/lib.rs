//! `eitri-import` — SVG / DXF / G-code import into Eitri geometry.
//!
//! FlatCAM accepts several additive input formats beyond Gerber/Excellon: SVG and DXF vector art (imported as
//! geometry for engraving/cutting) and existing G-code (opened to visualize and re-post). This crate reproduces
//! those import paths (plan §6), producing **parser-local** preview types — flattened `geo_types` geometry and a
//! motion preview — deliberately *not* the Phase-7 `eitri-project` object/serde schema, which stays deferred.
//!
//! Three entry points, each returning recovered geometry beside a loud list of anything it skipped:
//! - [`import_svg`] — SVG via `usvg` (normalized tree, resolved transforms), curves flattened at the shared chord
//!   tolerance; closed subpaths become polygons, open ones polylines.
//! - [`import_dxf`] / [`import_dxf_reader`] — DXF via the `dxf` crate: `LINE`/`LWPOLYLINE`/`POLYLINE`/`ARC`/
//!   `CIRCLE`, with bulge and arc flattening; unsupported entities (e.g. `SPLINE`) are reported, never dropped.
//! - [`import_gcode`] — G-code walked back into a [`GcodePreview`] via the shared [`eitri_gcode::lex`] lexer.
//!
//! All geometry flattening routes through `eitri-geo` so imported precision matches native Eitri geometry.

#![forbid(unsafe_code)]

pub mod diagnostic;
pub mod dxf;
pub mod gcode;
pub mod geometry;
pub mod svg;

pub use diagnostic::Skipped;
pub use dxf::{DxfImport, import_dxf, import_dxf_reader};
pub use gcode::{GcodePreview, MotionKind, Point3, PreviewMove, import_gcode};
pub use geometry::ImportedGeometry;
pub use svg::{SvgImport, SvgOptions, import_svg};
