//! Integration tests for `eitri-project`: the object model, versioned persistence, hydration, undo, and tool DB.
//!
//! These exercise only the public API (black-box), which doubles as a check that the crate is ergonomic to drive.

use eitri_cam::{
  Boundary, CutoutOutline, CutoutParams, MillingDirection, MirrorLine, PanelSpec, PaintParams, Point, Spacing,
  TabPlacement,
};
use eitri_core::{CancelToken, ProgressReporter, Unit};
use eitri_gcode::{OutputFormat, Program};
use eitri_geo::DefaultBackend;
use eitri_project::{
  BoundarySpec, CamOperation, CncJobObject, CutoutOutlineSpec, CutoutSpec, DirectionSpec, DrillDefaults, DrillSpec,
  ExcellonObject, GeometryObject, GeometryOrigin, GerberObject, History, ImportFormat, IsolationDefaults,
  IsolationSpec, MirrorLineSpec, NonCopperSpec, ObjectKind, ObjectMeta, ObjectPayload, PaintSpec, PaintStrategySpec,
  PanelizeSpec, Project, ProjectError, SpacingSpec, TabPlacementSpec, ToolDatabase, ToolEntry, TwoSidedSpec,
  load_project, load_tool_db, save_project, save_tool_db,
};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

// A tiny real Gerber and Excellon, embedded from the live fixture corpus so hydration re-parses the same bytes the
// dedicated parser tests use.
const GERBER_SRC: &str = include_str!("../../../fixtures/synthetic/gerber/coords_trailing.gbr");
const EXCELLON_SRC: &str = include_str!("../../../fixtures/synthetic/excellon/large_drill.drl");

fn square() -> Polygon<f64> {
  let ring = LineString::from(vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]);
  Polygon::new(ring, vec![])
}

fn polyline() -> LineString<f64> {
  LineString::from(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 0.5)])
}

fn isolation_op() -> CamOperation {
  CamOperation::Isolation(IsolationSpec {
    tool_diameter: 0.2,
    passes: 2,
    overlap: 0.1,
    combine: true,
    direction: DirectionSpec::Climb,
  })
}

/// Build a project holding one of each object kind, with a CNC job that references the Gerber it was cut from.
fn sample_project() -> Project {
  let mut project = Project::new("sample");
  let gerber = project
    .collection
    .add(ObjectMeta::new("top-copper", Unit::Millimeters), ObjectPayload::Gerber(GerberObject::new(GERBER_SRC)))
    .expect("add gerber");
  project
    .collection
    .add(ObjectMeta::new("drills", Unit::Millimeters), ObjectPayload::Excellon(ExcellonObject::new(EXCELLON_SRC)))
    .expect("add excellon");
  project
    .collection
    .add(
      ObjectMeta::new("outline", Unit::Millimeters),
      ObjectPayload::Geometry(GeometryObject {
        polygons: vec![square()],
        polylines: vec![polyline()],
        origin: GeometryOrigin::Imported { format: ImportFormat::Svg, source: "<svg/>".into() },
      }),
    )
    .expect("add geometry");
  project
    .collection
    .add(
      ObjectMeta::new("isolate-job", Unit::Millimeters),
      ObjectPayload::CncJob(CncJobObject {
        gcode: std::sync::Arc::from(vec!["G21".to_string(), "G0 X0 Y0".to_string(), "M2".to_string()]),
        dialect: "grblHAL".to_string(),
        source: Some(gerber),
        operation: isolation_op(),
        origin: (0.0, 0.0),
        stale: false,
        emission: None,
      }),
    )
    .expect("add cncjob");
  project
}

#[test]
fn collection_allocates_stable_unique_ids_and_enforces_names() {
  let mut project = Project::new("p");
  let a = project.collection.add(ObjectMeta::new("a", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  let b = project.collection.add(ObjectMeta::new("b", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  assert_ne!(a, b, "ids are distinct");
  assert_eq!(project.collection.get(a).unwrap().meta.name, "a");
  assert_eq!(project.collection.by_name("b").unwrap().meta.id, b);

  // Duplicate names are rejected.
  let dup = project.collection.add(ObjectMeta::new("a", Unit::Millimeters), ObjectPayload::Geometry(empty_geo()));
  assert!(matches!(dup, Err(ProjectError::DuplicateName(n)) if n == "a"));

  // Removing an object does not recycle its id.
  project.collection.remove(a);
  let c = project.collection.add(ObjectMeta::new("c", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  assert_ne!(c, a);
  assert_ne!(c, b);
}

#[test]
fn rename_is_stable_and_unique() {
  let mut project = Project::new("p");
  let a = project.collection.add(ObjectMeta::new("a", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  project.collection.add(ObjectMeta::new("b", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();

  // Renaming onto an existing name fails; id is unchanged after a successful rename.
  assert!(matches!(project.collection.rename(a, "b"), Err(ProjectError::DuplicateName(_))));
  project.collection.rename(a, "a2").unwrap();
  assert_eq!(project.collection.get(a).unwrap().meta.name, "a2");
  assert_eq!(project.collection.by_name("a2").unwrap().meta.id, a);
}

#[test]
fn reorder_moves_display_order_without_changing_ids() {
  let mut project = Project::new("p");
  let a = project.collection.add(ObjectMeta::new("a", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  let b = project.collection.add(ObjectMeta::new("b", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  let c = project.collection.add(ObjectMeta::new("c", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  project.collection.reorder(c, 0).unwrap();
  let order: Vec<_> = project.collection.iter().map(|o| o.meta.id).collect();
  assert_eq!(order, vec![c, a, b]);
}

#[test]
fn groups_track_membership_by_id() {
  let mut project = Project::new("p");
  let a = project.collection.add(ObjectMeta::new("a", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  let b = project.collection.add(ObjectMeta::new("b", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap();
  project.collection.create_group("layer-1").unwrap();
  assert!(matches!(project.collection.create_group("layer-1"), Err(ProjectError::DuplicateGroup(_))));
  project.collection.add_to_group("layer-1", a).unwrap();
  project.collection.add_to_group("layer-1", b).unwrap();
  project.collection.add_to_group("layer-1", a).unwrap(); // idempotent re-add
  assert_eq!(project.collection.group_members("layer-1").unwrap(), &[a, b]);

  // Removing an object drops it from its groups.
  project.collection.remove(a);
  assert_eq!(project.collection.group_members("layer-1").unwrap(), &[b]);

  // Grouping an unknown id or naming an unknown group fails cleanly.
  assert!(matches!(project.collection.add_to_group("nope", b), Err(ProjectError::UnknownGroup(_))));
}

#[test]
fn kind_reflects_payload() {
  let project = sample_project();
  assert_eq!(project.collection.by_name("top-copper").unwrap().kind(), ObjectKind::Gerber);
  assert_eq!(project.collection.by_name("drills").unwrap().kind(), ObjectKind::Excellon);
  assert_eq!(project.collection.by_name("outline").unwrap().kind(), ObjectKind::Geometry);
  assert_eq!(project.collection.by_name("isolate-job").unwrap().kind(), ObjectKind::CncJob);
}

#[test]
fn round_trip_is_lossless_for_every_object_kind() {
  let project = sample_project();
  let json = save_project(&project).unwrap();
  let loaded = load_project(&json).unwrap();

  assert_eq!(loaded.name, "sample");
  assert_eq!(loaded.collection.len(), 4);

  // Gerber/Excellon source survives; parse caches are empty until hydration.
  let gerber = match &loaded.collection.by_name("top-copper").unwrap().payload {
    ObjectPayload::Gerber(g) => g,
    _ => panic!("expected gerber"),
  };
  assert_eq!(&*gerber.source, GERBER_SRC);
  assert!(gerber.image.is_none(), "parse cache is re-derived, not persisted");

  // Geometry round-trips through geo-types serde exactly.
  let geo = match &loaded.collection.by_name("outline").unwrap().payload {
    ObjectPayload::Geometry(g) => g,
    _ => panic!("expected geometry"),
  };
  assert_eq!(geo.polygons, vec![square()]);
  assert_eq!(geo.polylines, vec![polyline()]);

  // CNC job keeps its rendered lines, dialect, cross-object source link, and operation params.
  let job = match &loaded.collection.by_name("isolate-job").unwrap().payload {
    ObjectPayload::CncJob(j) => j,
    _ => panic!("expected cncjob"),
  };
  assert_eq!(&*job.gcode, &["G21".to_string(), "G0 X0 Y0".to_string(), "M2".to_string()]);
  assert_eq!(job.dialect, "grblHAL");
  assert_eq!(job.operation, isolation_op());
  let gerber_id = project.collection.by_name("top-copper").unwrap().meta.id;
  assert_eq!(job.source, Some(gerber_id), "cross-object id link survives the round trip");
}

#[test]
fn hydrate_re_derives_gerber_and_excellon_geometry() {
  let project = sample_project();
  let json = save_project(&project).unwrap();
  let mut loaded = load_project(&json).unwrap();

  loaded.hydrate(&ProgressReporter::silent(), &CancelToken::new()).unwrap();

  // The re-derived Gerber matches a direct parse of the same source.
  let expected = eitri_gerber::parse_gerber(GERBER_SRC, &ProgressReporter::silent(), &CancelToken::new()).unwrap();
  let gerber = match &loaded.collection.by_name("top-copper").unwrap().payload {
    ObjectPayload::Gerber(g) => g,
    _ => panic!("expected gerber"),
  };
  let image = gerber.image.as_ref().expect("hydrated");
  assert_eq!(image.copper, expected.copper);

  let excellon = match &loaded.collection.by_name("drills").unwrap().payload {
    ObjectPayload::Excellon(e) => e,
    _ => panic!("expected excellon"),
  };
  assert!(excellon.image.is_some(), "excellon hydrated");
}

#[test]
fn hydrate_honours_cancellation() {
  let mut project = sample_project();
  let cancel = CancelToken::new();
  cancel.cancel();
  let result = project.hydrate(&ProgressReporter::silent(), &cancel);
  assert!(matches!(result, Err(ProjectError::Engine(eitri_core::Error::Cancelled))));
}

#[test]
fn load_rejects_bad_or_unsupported_versions() {
  // Garbage JSON.
  assert!(matches!(load_project("not json"), Err(ProjectError::Deserialize(_))));

  // Missing version field.
  assert!(matches!(load_project("{}"), Err(ProjectError::Deserialize(_))));

  // A newer version we cannot read.
  let mut value: serde_json::Value = serde_json::from_str(&save_project(&sample_project()).unwrap()).unwrap();
  value["schema_version"] = serde_json::json!(999);
  let bumped = serde_json::to_string(&value).unwrap();
  assert!(matches!(
    load_project(&bumped),
    Err(ProjectError::UnsupportedVersion { found: 999, supported: 1 })
  ));

  // Version zero is invalid.
  value["schema_version"] = serde_json::json!(0);
  let zeroed = serde_json::to_string(&value).unwrap();
  assert!(matches!(load_project(&zeroed), Err(ProjectError::UnsupportedVersion { found: 0, supported: 1 })));
}

#[test]
fn cncjob_from_program_captures_rendered_lines() {
  let mut program = Program::new(OutputFormat::default());
  program.push("G21");
  program.push("G0 X1 Y1");
  let job = CncJobObject::from_program(&program, "grblHAL", None, isolation_op(), (0.0, 0.0), None);
  assert_eq!(&*job.gcode, &["G21".to_string(), "G0 X1 Y1".to_string()]);
  assert_eq!(job.render(), "G21\nG0 X1 Y1\n");
  assert!(!job.stale && job.emission.is_none(), "a fresh job is up to date with no captured emission");
}

#[test]
fn undo_redo_walks_edit_history() {
  let mut history = History::new(sample_project().collection);
  assert!(!history.can_undo());
  let start = history.current().len();

  let id = history.edit(|c| {
    c.add(ObjectMeta::new("extra", Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap()
  });
  assert_eq!(history.current().len(), start + 1);
  assert!(history.can_undo());

  assert!(history.undo());
  assert_eq!(history.current().len(), start);
  assert!(history.current().get(id).is_none());

  assert!(history.redo());
  assert_eq!(history.current().len(), start + 1);
  assert!(history.current().get(id).is_some());

  // A fresh edit clears the redo future.
  history.undo();
  history.edit(|c| c.create_group("g").unwrap());
  assert!(!history.can_redo());
}

#[test]
fn undo_limit_bounds_retained_snapshots() {
  let mut history = History::with_limit(Project::new("p").collection, 2);
  for i in 0..5 {
    history.edit(|c| {
      c.add(ObjectMeta::new(format!("o{i}"), Unit::Millimeters), ObjectPayload::Geometry(empty_geo())).unwrap()
    });
  }
  // Only two snapshots are retained, so only two undos are possible.
  assert!(history.undo());
  assert!(history.undo());
  assert!(!history.undo());
}

#[test]
fn tool_db_round_trips_and_seeds_cam_defaults() {
  let mut db = ToolDatabase::new();
  let id = db.add(ToolEntry {
    id: eitri_project::ToolId(0),
    name: "0.2mm end mill".to_string(),
    diameter: eitri_core::Length::from_mm(0.2),
    isolation: IsolationDefaults { passes: 3, overlap: 0.15, combine: true, direction: DirectionSpec::Conventional },
    drilling: DrillDefaults::default(),
  });

  // The tool DB hands back the same operator-facing specs that objects store.
  let iso = db.get(id).unwrap().isolation_spec();
  assert_eq!(iso.tool_diameter, 0.2);
  assert_eq!(iso.passes, 3);
  assert_eq!(iso.direction, DirectionSpec::Conventional);

  // A spec expands into full eitri-cam params, filling the join defaults.
  let params = iso.to_params();
  assert_eq!(params.tool_diameter, 0.2);
  assert_eq!(params.join, eitri_geo::JoinType::Round);
  assert_eq!(params.direction, eitri_cam::MillingDirection::Conventional);

  let drill_params = db.get(id).unwrap().drill_spec().to_params();
  assert_eq!(drill_params.depth, DrillDefaults::default().depth);

  // Round-trip through JSON.
  let json = save_tool_db(&db).unwrap();
  let loaded = load_tool_db(&json).unwrap();
  assert_eq!(loaded.len(), 1);
  let tool = loaded.by_name("0.2mm end mill").unwrap();
  assert_eq!(tool.diameter.as_mm(), 0.2);
  assert_eq!(tool.isolation.passes, 3);
}

#[test]
fn tool_db_update_replaces_an_entry_in_place_keeping_its_id_and_position() {
  let mut db = ToolDatabase::new();
  let first = db.add(ToolEntry {
    id: eitri_project::ToolId(0),
    name: "0.2mm end mill".to_string(),
    diameter: eitri_core::Length::from_mm(0.2),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });
  let second = db.add(ToolEntry {
    id: eitri_project::ToolId(0),
    name: "1.0mm drill".to_string(),
    diameter: eitri_core::Length::from_mm(1.0),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });

  // Update the FIRST entry: the id is preserved (even if the caller passed a placeholder), the list order is
  // unchanged, and the new fields land.
  let edited = ToolEntry {
    id: eitri_project::ToolId(0),
    name: "0.25mm end mill".to_string(),
    diameter: eitri_core::Length::from_mm(0.25),
    isolation: IsolationDefaults { passes: 2, ..IsolationDefaults::default() },
    drilling: DrillDefaults::default(),
  };
  assert!(db.update(first, edited), "updating an existing id succeeds");
  assert_eq!(db.len(), 2, "update replaces, never adds");
  let tool = db.get(first).expect("the id survives the update");
  assert_eq!(tool.name, "0.25mm end mill");
  assert_eq!(tool.isolation.passes, 2);
  assert_eq!(db.iter().next().map(|t| t.id), Some(first), "the entry keeps its list position");
  assert_eq!(db.get(second).map(|t| t.name.as_str()), Some("1.0mm drill"), "the neighbour is untouched");

  // An unknown id is refused without mutating anything.
  let stray = ToolEntry {
    id: eitri_project::ToolId(0),
    name: "ghost".to_string(),
    diameter: eitri_core::Length::from_mm(3.0),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  };
  assert!(!db.update(eitri_project::ToolId(999), stray), "an unknown id must be refused");
  assert_eq!(db.len(), 2);
}

#[test]
fn tool_db_rejects_unsupported_version() {
  let db = ToolDatabase::new();
  let mut value: serde_json::Value = serde_json::from_str(&save_tool_db(&db).unwrap()).unwrap();
  value["schema_version"] = serde_json::json!(7);
  assert!(matches!(
    load_tool_db(&serde_json::to_string(&value).unwrap()),
    Err(ProjectError::UnsupportedVersion { found: 7, supported: 1 })
  ));
}

#[test]
fn drill_spec_maps_peck_and_dwell() {
  let spec = DrillSpec { depth: 2.0, feed: 120.0, retract: 3.0, peck: Some(0.5), dwell: Some(0.2) };
  let params = spec.to_params();
  assert_eq!(params.peck, Some(0.5));
  assert_eq!(params.dwell, Some(0.2));
  assert_eq!(params.depth, 2.0);
  // Depth is a positive magnitude: a stray negative (legacy or hand-built spec) is folded so it never air-drills.
  let negative = DrillSpec { depth: -2.0, feed: 120.0, retract: 3.0, peck: None, dwell: None };
  assert_eq!(negative.to_params().depth, 2.0, "to_params normalizes a negative depth to its magnitude");
}

#[test]
fn loading_a_tool_db_folds_a_legacy_negative_drill_depth_positive() {
  // Simulate a database written under the old signed-Z convention (drill depth stored negative).
  let mut db = ToolDatabase::new();
  db.add(ToolEntry {
    id: eitri_project::ToolId(0),
    name: "legacy".to_string(),
    diameter: eitri_core::Length::from_mm(0.8),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults { depth: -1.8, feed: 100.0, retract: 2.0, peck: None, dwell: None },
  });
  let json = save_tool_db(&db).unwrap();
  let loaded = load_tool_db(&json).unwrap();
  let depth = loaded.iter().next().expect("one tool").drilling.depth;
  assert_eq!(depth, 1.8, "a legacy signed-Z depth is migrated to a positive magnitude on load");
}

#[test]
fn a_stock_reconciles_a_stale_stored_origin_on_load() {
  // The stock is authoritative: a project whose stored `origin` disagrees with `stock.origin()` (a hand-edit, or a
  // future writer that updates the stock without re-resolving) must load with the origin re-derived from the stock,
  // so the emitted G-code can never post at a frame the Setup panel doesn't show.
  let stock = eitri_project::Stock::fit((10.0, 20.0, 30.0, 50.0), 1.6);
  let mut project = Project::new("stock");
  project.stock = Some(stock);
  project.origin = (999.0, 999.0, 999.0); // deliberately stale
  let json = save_project(&project).unwrap();
  let loaded = load_project(&json).unwrap();
  assert_eq!(loaded.origin, stock.origin(), "load re-derives the origin from the stock");
  assert_eq!(loaded.origin, (10.0, 20.0, 0.0));
}

fn empty_geo() -> GeometryObject {
  GeometryObject { polygons: Vec::new(), polylines: Vec::new(), origin: GeometryOrigin::Generated }
}

// --- Phase-7 CAM-operation coverage (paint / non-copper / cutout / panelize / two-sided) -----------------------------

/// A rectangular `MultiPolygon` used where a spec genuinely owns a region (non-copper frame, geometry outline).
fn rect_region(x0: f64, y0: f64, x1: f64, y1: f64) -> MultiPolygon<f64> {
  let ring = LineString::from(vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]);
  MultiPolygon::new(vec![Polygon::new(ring, vec![])])
}

fn paint_spec() -> PaintSpec {
  PaintSpec {
    tool_diameter: 1.0,
    overlap: 0.25,
    margin: 0.5,
    direction: DirectionSpec::Climb,
    finish_pass: true,
    strategy: PaintStrategySpec::Raster { angle_deg: 30.0 },
  }
}

fn paint_op() -> CamOperation {
  CamOperation::Paint(paint_spec())
}

fn noncopper_op() -> CamOperation {
  CamOperation::NonCopper(NonCopperSpec {
    boundary: BoundarySpec::Region(rect_region(-1.0, -1.0, 11.0, 11.0)),
    paint: PaintSpec {
      tool_diameter: 0.8,
      overlap: 0.3,
      margin: 0.0,
      direction: DirectionSpec::Conventional,
      finish_pass: false,
      strategy: PaintStrategySpec::Concentric,
    },
  })
}

fn cutout_op() -> CamOperation {
  CamOperation::Cutout(CutoutSpec {
    tool_diameter: 2.0,
    tab_width: 3.0,
    tabs: TabPlacementSpec::AtFractions(vec![0.0, 0.25, 0.5, 0.75]),
    margin: 0.5,
    direction: DirectionSpec::Conventional,
    outline: CutoutOutlineSpec::Geometry(rect_region(0.0, 0.0, 30.0, 20.0)),
  })
}

fn panelize_op() -> CamOperation {
  CamOperation::Panelize(PanelizeSpec {
    rows: 2,
    cols: 3,
    x: SpacingSpec::Gap(4.0),
    y: SpacingSpec::Pitch(25.0),
  })
}

fn twosided_op() -> CamOperation {
  CamOperation::TwoSided(TwoSidedSpec {
    mirror: MirrorLineSpec::Vertical(15.0),
    alignment_holes: vec![Coord { x: 2.0, y: 3.0 }, Coord { x: 28.0, y: 3.0 }],
    hole_diameter: 3.2,
  })
}

/// Build a project whose collection holds one CNC job per new CAM-operation kind, so a lossless round trip proves all
/// five variants (and the geometry they own) survive persistence.
fn every_op_project() -> Project {
  let mut project = Project::new("ops");
  for (name, op) in [
    ("paint", paint_op()),
    ("noncopper", noncopper_op()),
    ("cutout", cutout_op()),
    ("panelize", panelize_op()),
    ("twosided", twosided_op()),
  ] {
    project
      .collection
      .add(
        ObjectMeta::new(name, Unit::Millimeters),
        ObjectPayload::CncJob(CncJobObject {
          gcode: std::sync::Arc::from(vec!["G21".to_string(), "M2".to_string()]),
          dialect: "grblHAL".to_string(),
          source: None,
          operation: op,
          origin: (0.0, 0.0),
          stale: false,
          emission: None,
        }),
      )
      .expect("add cncjob");
  }
  project
}

fn loaded_operation(project: &Project, name: &str) -> CamOperation {
  match &project.collection.by_name(name).unwrap().payload {
    ObjectPayload::CncJob(job) => job.operation.clone(),
    _ => panic!("expected a cncjob for {name}"),
  }
}

#[test]
fn every_new_cam_operation_round_trips_losslessly() {
  let project = every_op_project();
  let json = save_project(&project).unwrap();
  let loaded = load_project(&json).unwrap();

  assert_eq!(loaded.collection.len(), 5, "one job per new operation kind");
  assert_eq!(loaded_operation(&loaded, "paint"), paint_op());
  assert_eq!(loaded_operation(&loaded, "noncopper"), noncopper_op());
  assert_eq!(loaded_operation(&loaded, "cutout"), cutout_op());
  assert_eq!(loaded_operation(&loaded, "panelize"), panelize_op());
  assert_eq!(loaded_operation(&loaded, "twosided"), twosided_op());
}

#[test]
fn paint_spec_maps_to_paint_params_and_strategy() {
  let spec = paint_spec();
  assert_eq!(
    spec.to_params(),
    PaintParams {
      tool_diameter: 1.0,
      overlap: 0.25,
      margin: 0.5,
      direction: MillingDirection::Climb,
      finish_pass: true,
      join: eitri_geo::JoinType::Round,
      miter_limit: 2.0,
    }
  );
  // The Raster strategy fills a solid square with at least one open row; Concentric yields only closed rings.
  let region = rect_region(0.0, 0.0, 10.0, 10.0);
  let (progress, cancel) = (ProgressReporter::silent(), CancelToken::new());
  let raster = spec.strategy();
  let filled = eitri_cam::paint(&region, &spec.to_params(), raster.as_ref(), &DefaultBackend::new(), &progress, &cancel)
    .expect("raster paint");
  assert!(filled.paths.iter().any(|p| !p.is_closed()), "raster strategy produces open rows");

  let concentric = PaintStrategySpec::Concentric.to_strategy();
  let rings = eitri_cam::paint(
    &region,
    &PaintParams { finish_pass: false, ..spec.to_params() },
    concentric.as_ref(),
    &DefaultBackend::new(),
    &progress,
    &cancel,
  )
  .expect("concentric paint");
  assert!(rings.paths.iter().all(|p| p.is_closed()), "concentric strategy produces only closed rings");
}

#[test]
fn noncopper_spec_maps_to_boundary_and_paint_params() {
  let CamOperation::NonCopper(spec) = noncopper_op() else { panic!("expected non-copper") };
  assert_eq!(spec.to_boundary(), Boundary::Region(rect_region(-1.0, -1.0, 11.0, 11.0)));
  assert_eq!(spec.to_params().tool_diameter, 0.8);
  assert_eq!(spec.to_params().direction, MillingDirection::Conventional);

  // The bounding-box boundary variant maps across too.
  let bbox = BoundarySpec::BoundingBox { margin: 4.0 };
  assert_eq!(bbox.to_boundary(), Boundary::BoundingBox { margin: 4.0 });
}

#[test]
fn cutout_spec_maps_to_params_and_outline() {
  let CamOperation::Cutout(spec) = cutout_op() else { panic!("expected cutout") };
  assert_eq!(
    spec.to_params(),
    CutoutParams {
      tool_diameter: 2.0,
      tab_width: 3.0,
      tabs: TabPlacement::AtFractions(vec![0.0, 0.25, 0.5, 0.75]),
      margin: 0.5,
      direction: MillingDirection::Conventional,
      join: eitri_geo::JoinType::Round,
      miter_limit: 2.0,
    }
  );
  assert_eq!(spec.to_outline(), CutoutOutline::Geometry(rect_region(0.0, 0.0, 30.0, 20.0)));

  // The rectangle outline variant maps its owned corners into cam Points.
  let rect = CutoutOutlineSpec::Rectangle { min: Coord { x: 1.0, y: 2.0 }, max: Coord { x: 9.0, y: 8.0 } };
  assert_eq!(
    rect.to_outline(),
    CutoutOutline::Rectangle { min: Point::new(1.0, 2.0), max: Point::new(9.0, 8.0) }
  );
}

#[test]
fn panelize_spec_maps_gap_and_pitch() {
  let CamOperation::Panelize(spec) = panelize_op() else { panic!("expected panelize") };
  assert_eq!(
    spec.to_params(),
    PanelSpec { rows: 2, cols: 3, x: Spacing::Gap(4.0), y: Spacing::Pitch(25.0) }
  );
}

#[test]
fn twosided_spec_maps_mirror_line_and_alignment_holes() {
  let CamOperation::TwoSided(spec) = twosided_op() else { panic!("expected two-sided") };
  assert_eq!(spec.to_mirror_line(), MirrorLine::Vertical(15.0));
  assert_eq!(spec.alignment_hole_points(), vec![Point::new(2.0, 3.0), Point::new(28.0, 3.0)]);

  let horizontal = MirrorLineSpec::Horizontal(7.5);
  assert_eq!(horizontal.to_mirror_line(), MirrorLine::Horizontal(7.5));
}
