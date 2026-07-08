//! The cached render model: everything the canvas paints, prepared FROM engine outputs once per collection
//! change, so the per-frame paint pass is a cheap walk over ready-made meshes and polylines.
//!
//! The boundary discipline: **all geometry comes from the engine.** Filled regions are tessellated by
//! [`eitri_geo::triangulate`] (ear-cut, holes honoured), drill hits become polygons via
//! [`eitri_excellon::ExcellonImage::hit_geometry`], and a CNC job's toolpath preview comes from
//! [`eitri_import::import_gcode`] over the job's own rendered G-code — the same importer the engine uses for
//! re-posting, so what the canvas shows *is* what the program does. This module only re-shapes those outputs
//! into flat, paint-friendly arrays and tracks their combined bounds (a min/max fold — bookkeeping, not
//! geometry).

use eitri_geo::TriangleMesh;
use eitri_import::MotionKind;
use eitri_project::{ObjectId, ObjectKind, ObjectPayload};
use eitri_script::Session;
use geo_types::{MultiPolygon, Polygon};

/// Axis-aligned bounds as `(min_x, min_y, max_x, max_y)`, in engine millimetres.
pub type Bounds = (f64, f64, f64, f64);

/// One object's paint-ready geometry.
#[derive(Debug, Clone)]
pub struct ObjectScene {
  /// The object's id (for selection highlighting).
  pub id: ObjectId,
  /// Its kind (selects the fill/stroke colours).
  pub kind: ObjectKind,
  /// Whether the object is shown ([`eitri_project::ObjectMeta::visible`]): a hidden object is neither painted
  /// nor click-pickable, and its extent is excluded from the fit-view bounds.
  pub visible: bool,
  /// The filled area (copper, drill capsules, geometry polygons) as an indexed triangle mesh.
  pub fill: TriangleMesh,
  /// The region's rings (exterior + holes), for the crisp outline pass over the fill.
  pub outlines: Vec<Vec<[f64; 2]>>,
  /// Open polylines (imported SVG/DXF strokes, recovered G-code contours on geometry objects).
  pub polylines: Vec<Vec<[f64; 2]>>,
  /// A CNC job's cutting polylines (engine-classified feed runs, XY-projected).
  pub cuts: Vec<Vec<[f64; 2]>>,
  /// A CNC job's rapid segments, as `(from, to)` XY pairs.
  pub rapids: Vec<([f64; 2], [f64; 2])>,
  /// This object's bounds, if it has any extent.
  pub bounds: Option<Bounds>,
}

/// The whole collection, paint-ready, plus the combined bounds the fit-view zooms to.
#[derive(Debug, Clone, Default)]
pub struct RenderScene {
  /// One entry per visible object, in display order (painted back-to-front in that order).
  pub objects: Vec<ObjectScene>,
  /// The union of every object's bounds.
  pub bounds: Option<Bounds>,
}

impl ObjectScene {
  /// An empty entry for an object, filled in by [`build_scene`].
  fn new(id: ObjectId, kind: ObjectKind, visible: bool) -> Self {
    ObjectScene {
      id,
      kind,
      visible,
      fill: TriangleMesh::default(),
      outlines: Vec::new(),
      polylines: Vec::new(),
      cuts: Vec::new(),
      rapids: Vec::new(),
      bounds: None,
    }
  }
}

impl RenderScene {
  /// The scene entry for an object id, if present.
  pub fn object(&self, id: ObjectId) -> Option<&ObjectScene> {
    self.objects.iter().find(|o| o.id == id)
  }
}

/// Build the paint-ready scene from the session's collection. Called only when the collection changes (an op
/// lands, undo/redo, delete) — never per frame.
pub fn build_scene(session: &Session) -> RenderScene {
  let mut scene = RenderScene::default();
  for id in session.object_ids() {
    let Ok(object) = session.object(id) else { continue };
    let mut entry = ObjectScene::new(id, object.kind(), object.meta.visible);
    match &object.payload {
      ObjectPayload::Gerber(gerber) => {
        if let Some(image) = &gerber.image {
          fill_region(&mut entry, &image.copper);
        }
      }
      ObjectPayload::Excellon(excellon) => {
        if let Some(image) = &excellon.image {
          // Each hit becomes its polygon (a circle for a drill, a capsule for a slot) via the ENGINE's
          // geometry; a hit whose tool is unknown yields None and is skipped, exactly as the CAM ops do.
          let mut polygons: Vec<Polygon<f64>> = Vec::new();
          for hit in &image.hits {
            if let Ok(Some(geometry)) = image.hit_geometry(hit) {
              polygons.extend(geometry.0);
            }
          }
          fill_region(&mut entry, &MultiPolygon::new(polygons));
        }
      }
      ObjectPayload::Geometry(geometry) => {
        fill_region(&mut entry, &MultiPolygon::new(geometry.polygons.clone()));
        for line in &geometry.polylines {
          let pts: Vec<[f64; 2]> = line.0.iter().map(|c| [c.x, c.y]).collect();
          extend_bounds(&mut entry.bounds, pts.iter().copied());
          entry.polylines.push(pts);
        }
      }
      ObjectPayload::CncJob(job) => {
        // The preview importer walks the job's own rendered G-code — the engine's classification of cut vs
        // rapid, not ours. A job that somehow fails to re-import previews as empty rather than wrong. The G-code
        // was posted with the job's datum subtracted, so add it back here to draw the toolpath over the
        // native-frame source geometry (a native job's origin is (0, 0), so this is a no-op for those).
        let (ox, oy) = job.origin;
        if let Ok(preview) = eitri_import::import_gcode(&job.render()) {
          for line in preview.cut_polylines() {
            let pts: Vec<[f64; 2]> = line.0.iter().map(|c| [c.x + ox, c.y + oy]).collect();
            extend_bounds(&mut entry.bounds, pts.iter().copied());
            entry.cuts.push(pts);
          }
          let mut seen_cut = false;
          for mv in &preview.moves {
            match mv.kind {
              // Before the first cut, every rapid is setup off the importer's assumed (0, 0) start — the leading
              // `G0 Z<safe>` and the synthetic hop to the first cut, both from the phantom origin. Drop those (they
              // are not part of the job and would stretch the fit out to frame empty space). Once cutting has begun,
              // every rapid is real between-ring travel and stays — including one that starts at (0, 0) because a
              // ring begins on the datum corner.
              MotionKind::Rapid => {
                if !seen_cut && mv.from.x == 0.0 && mv.from.y == 0.0 {
                  continue;
                }
                let (from, to) = ([mv.from.x + ox, mv.from.y + oy], [mv.to.x + ox, mv.to.y + oy]);
                extend_bounds(&mut entry.bounds, [from, to].into_iter());
                entry.rapids.push((from, to));
              }
              MotionKind::Cut => seen_cut = true,
            }
          }
        }
      }
    }
    // Only what is actually on screen participates in the fit-view union — a hidden board must not zoom the
    // camera out to frame something invisible.
    if entry.visible {
      merge_bounds(&mut scene.bounds, entry.bounds);
    }
    scene.objects.push(entry);
  }
  scene
}

/// Tessellate a filled region into the entry (mesh + ring outlines + bounds), all through engine geometry.
fn fill_region(entry: &mut ObjectScene, region: &MultiPolygon<f64>) {
  entry.fill = eitri_geo::triangulate(region);
  for polygon in &region.0 {
    for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
      let pts: Vec<[f64; 2]> = ring.0.iter().map(|c| [c.x, c.y]).collect();
      entry.outlines.push(pts);
    }
  }
  merge_bounds(&mut entry.bounds, eitri_geo::bounds(region));
}

/// Fold a set of points into an optional bounds accumulator.
fn extend_bounds(bounds: &mut Option<Bounds>, points: impl Iterator<Item = [f64; 2]>) {
  for [x, y] in points {
    merge_bounds(bounds, Some((x, y, x, y)));
  }
}

/// Merge `other` into `bounds`.
fn merge_bounds(bounds: &mut Option<Bounds>, other: Option<Bounds>) {
  let Some((ox0, oy0, ox1, oy1)) = other else { return };
  *bounds = Some(match *bounds {
    None => (ox0, oy0, ox1, oy1),
    Some((x0, y0, x1, y1)) => (x0.min(ox0), y0.min(oy0), x1.max(ox1), y1.max(oy1)),
  });
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_gcode::IsolationJob;
  use eitri_project::{DirectionSpec, IsolationSpec};

  const GERBER: &str = include_str!("../../../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
  const EXCELLON: &str = include_str!("../../../../fixtures/synthetic/excellon/metric_leading.drl");

  #[test]
  fn a_gerber_object_yields_a_filled_mesh_with_outlines_and_bounds() {
    let mut session = Session::new("fixture");
    let id = session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let scene = build_scene(&session);
    let entry = scene.object(id).expect("the gerber is in the scene");
    assert!(entry.fill.triangle_count() > 0, "copper must tessellate to triangles");
    assert!(!entry.outlines.is_empty(), "copper rings must be present for the outline pass");
    assert!(entry.bounds.is_some() && scene.bounds.is_some(), "a non-empty region has bounds");
    let (x0, y0, x1, y1) = entry.bounds.unwrap();
    assert!(x1 > x0 && y1 > y0, "bounds must have extent: {:?}", entry.bounds);
  }

  #[test]
  fn an_excellon_object_yields_drill_geometry_from_the_engine() {
    let mut session = Session::new("fixture");
    let id = session.open_excellon_str("fixture-drills", EXCELLON).expect("fixture opens");
    let scene = build_scene(&session);
    let entry = scene.object(id).expect("the drills are in the scene");
    assert!(entry.fill.triangle_count() > 0, "each hit becomes a filled circle/capsule mesh");
  }

  #[test]
  fn a_cnc_job_previews_cut_polylines_and_rapids_via_the_importer() {
    let mut session = Session::new("fixture");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let spec = IsolationSpec {
      tool_diameter: 0.2,
      passes: 1,
      overlap: 0.0,
      combine: false,
      direction: DirectionSpec::Climb,
    };
    let job = session.isolate(gerber, spec, IsolationJob::default()).expect("isolation succeeds");
    let scene = build_scene(&session);
    let entry = scene.object(job).expect("the job is in the scene");
    assert!(!entry.cuts.is_empty(), "an isolation job must preview cut contours");
    assert!(!entry.rapids.is_empty(), "and the rapid hops between rings");
    assert!(entry.fill.triangle_count() == 0 && entry.polylines.is_empty(), "a job is trails, not fills");
    // The job's preview must overlap the copper it isolates — a gross transform error would land it elsewhere.
    let copper = scene.objects.iter().find(|o| o.kind == ObjectKind::Gerber).unwrap().bounds.unwrap();
    let trails = entry.bounds.unwrap();
    assert!(trails.0 <= copper.2 && trails.2 >= copper.0, "the toolpath must span the copper in X");
    assert!(trails.1 <= copper.3 && trails.3 >= copper.1, "and in Y");
  }

  #[test]
  fn the_importers_synthetic_start_rapid_is_stripped_from_job_previews() {
    // The G-code preview importer assumes the machine starts at (0, 0), so every job preview begins with a
    // synthetic rapid from the origin to the first real move — a long diagonal to nowhere on a KiCad-frame
    // board. The scene must drop it (and keep it out of the bounds), while keeping the real between-ring hops.
    let mut session = Session::new("fixture");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let spec = IsolationSpec {
      tool_diameter: 0.2,
      passes: 1,
      overlap: 0.0,
      combine: false,
      direction: DirectionSpec::Climb,
    };
    let job = session.isolate(gerber, spec, IsolationJob::default()).expect("isolation succeeds");
    let scene = build_scene(&session);
    let entry = scene.object(job).expect("the job is in the scene");
    assert!(!entry.rapids.is_empty(), "the real between-ring rapids survive the strip");
    let (ox, oy) = (0.0, 0.0); // a native-frame job posts un-shifted, so gcode (0,0) is world (0,0).
    assert!(
      entry.rapids.iter().all(|(from, _)| *from != [ox, oy]),
      "no preview rapid may start at the importer's synthetic origin: {:?}",
      entry.rapids.first(),
    );
    // The job's bounds hug the copper it isolates — the origin must no longer stretch them to (0, 0). The
    // two-pad fixture's copper starts near x=1, so a bounds min at 0 would be the synthetic rapid leaking in.
    let copper = scene.objects.iter().find(|o| o.kind == ObjectKind::Gerber).unwrap().bounds.unwrap();
    let trails = entry.bounds.unwrap();
    assert!(trails.0 >= copper.0 - 1.0, "the preview bounds must hug the copper, not the machine origin");
  }

  #[test]
  fn an_empty_session_builds_an_empty_scene() {
    let scene = build_scene(&Session::new("empty"));
    assert!(scene.objects.is_empty());
    assert_eq!(scene.bounds, None);
  }

  #[test]
  fn a_hidden_object_is_marked_invisible_and_excluded_from_the_fit_bounds() {
    let mut session = Session::new("fixture");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("gerber opens");
    let drills = session.open_excellon_str("fixture-drills", EXCELLON).expect("drills open");
    session.set_visible(gerber, false).expect("hide the gerber");

    let scene = build_scene(&session);
    assert!(!scene.object(gerber).unwrap().visible, "the hidden object carries its flag into the scene");
    assert!(scene.object(drills).unwrap().visible, "the neighbour stays visible");
    // Fit-view bounds only frame what is actually on screen: the union must equal the drills' own bounds.
    assert_eq!(scene.bounds, scene.object(drills).unwrap().bounds, "hidden objects must not stretch the fit");
  }

  #[test]
  fn hiding_everything_leaves_no_fit_bounds() {
    let mut session = Session::new("fixture");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("gerber opens");
    session.set_visible(gerber, false).expect("hide it");
    let scene = build_scene(&session);
    assert_eq!(scene.bounds, None, "an all-hidden scene has nothing to fit to");
    assert_eq!(scene.objects.len(), 1, "the entry itself remains (the tree still lists it)");
  }

  #[test]
  fn scene_bounds_union_every_object() {
    let mut session = Session::new("fixture");
    session.open_gerber_str("fixture-top", GERBER).expect("gerber opens");
    session.open_excellon_str("fixture-drills", EXCELLON).expect("drills open");
    let scene = build_scene(&session);
    let union = scene.bounds.expect("two objects yield bounds");
    for object in &scene.objects {
      let Some((x0, y0, x1, y1)) = object.bounds else { continue };
      assert!(union.0 <= x0 && union.1 <= y0 && union.2 >= x1 && union.3 >= y1,
        "the scene bounds must contain every object's bounds");
    }
  }
}
