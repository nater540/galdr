//! Integration tests for `eitri-project`: the object model, versioned persistence, hydration, undo, and tool DB.
//!
//! These exercise only the public API (black-box), which doubles as a check that the crate is ergonomic to drive.

use eitri_core::{CancelToken, ProgressReporter, Unit};
use eitri_gcode::{OutputFormat, Program};
use eitri_project::{
  CamOperation, CncJobObject, DirectionSpec, DrillDefaults, DrillSpec, ExcellonObject, GeometryObject,
  GeometryOrigin, GerberObject, History, ImportFormat, IsolationDefaults, IsolationSpec, ObjectKind, ObjectMeta,
  ObjectPayload, Project, ProjectError, ToolDatabase, ToolEntry, load_project, load_tool_db, save_project,
  save_tool_db,
};
use geo_types::{LineString, Polygon};

// A tiny real Gerber and Excellon, embedded from the live fixture corpus so hydration re-parses the same bytes the
// dedicated parser tests use.
const GERBER_SRC: &str = include_str!("../../fixtures/synthetic/gerber/coords_trailing.gbr");
const EXCELLON_SRC: &str = include_str!("../../fixtures/synthetic/excellon/large_drill.drl");

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
  let job = CncJobObject::from_program(&program, "grblHAL", None, isolation_op());
  assert_eq!(&*job.gcode, &["G21".to_string(), "G0 X1 Y1".to_string()]);
  assert_eq!(job.render(), "G21\nG0 X1 Y1\n");
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
  let spec = DrillSpec { depth: -2.0, feed: 120.0, retract: 3.0, peck: Some(0.5), dwell: Some(0.2) };
  let params = spec.to_params();
  assert_eq!(params.peck, Some(0.5));
  assert_eq!(params.dwell, Some(0.2));
  assert_eq!(params.depth, -2.0);
}

fn empty_geo() -> GeometryObject {
  GeometryObject { polygons: Vec::new(), polylines: Vec::new(), origin: GeometryOrigin::Generated }
}
