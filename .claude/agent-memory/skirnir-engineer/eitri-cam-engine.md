---
name: eitri-cam-engine
description: Eitri CAM/toolpath engine — nested workspace at eitri/, phased build, current phase status and key constraints
metadata:
  type: project
---

Eitri is a Rust CAM/toolpath engine inside the galdr repo — a clean-room reimplementation of FlatCAM's
functionality/algorithms (no FlatCAM code; named only as provenance). It lives as its OWN nested Cargo workspace at
`eitri/` (own `[workspace]` table), deliberately NOT in the root galdr members list, so the firmware/skirnir build is
untouched. Authoritative design: `docs/eitri-porting-plan.md` (phased build order in §13).

**Why:** decouple CAM from any UI and route all geometry through one swappable backend; keep the stack all-Rust.

**How to apply:** work phase-by-phase per the plan; build the two named crates for real, leave the rest as
placeholders. Trait-seams-first, TDD, 2-space indent, no unwrap/expect in lib code, `-D warnings` + clippy clean.

Status as of 2026-07-04 (each phase = one commit on its own `feat/eitri-phaseN` branch, NOT yet merged; each builds
on the previous branch tip; note Phase 3 STACKS onto `feat/eitri-phase2`, no new branch — user prefers not to
proliferate branches):
- Phase 1 DONE (`feat/eitri-phase1`, commit 353ce59): eitri-core (checked Length units, Affine transforms,
  precision incl. one INTEGER_SCALE=1e6, Error/Result, ProgressReporter+CancelToken) and eitri-geo (CAM-shaped
  GeoBackend trait; DefaultBackend = clipper2 offset + geo/i_overlay booleans + DP simplify; cavalier arc offset;
  geos parity backend is a feature-gated compiling stub).
- Phase 2 DONE (`feat/eitri-phase2`, commit 082f8b2): eitri-gerber (RS-274X → aperture table + copper MultiPolygon)
  and eitri-excellon (drill → tool table + drill/slot hits); added `buffer_path` (open-path stroke) to eitri-geo.
- Phase 3 DONE (stacked on `feat/eitri-phase2`): eitri-cam isolation + drilling. Three modules, all TDD, 34 new
  tests (131→165), -D warnings + clippy --all-targets clean. Public API:
  - `optimize.rs`: `TravelOptimizer` trait (`order(&[Stop], start) -> Vec<Routed>`) over DIRECTED stops
    (`Stop{entry,exit}`; point drill entry==exit, slot differs & is reversible). `NearestNeighbor` (greedy) +
    `TwoOpt` (NN seed + 2-opt segment-reversal AND Or-1 relocation, both O(1) incremental delta). `tour_travel`
    costs ONLY inter-stop rapids (in-stop traversal is ordering-invariant). `order_stops` wraps a cancel-check.
    2-opt reversal flips each reversed stop's traversal flag — that's why stops are directed.
  - `isolation.rs`: `isolate<B:GeoBackend+Sync>` (polyline must-have) + `isolate_arc` (arc option). Pass n offset =
    radius + n*dia*(1-overlap) (`IsolationParams::offset_for`). MillingDirection→winding: Climb=Ccw, Conventional=Cw
    (centralized, swappable). combine=true unions SAME-PASS offsets across polygons (per-pass, preserves multi-pass
    structure) so overlapping rings from close traces merge. Holes handled (annular pad isolates both edges) in
    LINEAR mode only. rayon per-polygon offset fan-out; progress per polygon; cancel per polygon.
    ARC mode caveats (documented): exterior-boundary ONLY (no hole isolation), does NOT combine; but DOES honor
    milling direction via a bulge-correct `reverse_arc` (derived: nb[k]=-b[(n-1-k)%n], w[k]=v[(n-k)%n]; involution
    verified). offset_arc only offsets exterior — that's the source of the arc hole limitation.
  - `drill.rs`: `plan_drilling<O:TravelOptimizer>(image, &DrillConfig, &optimizer, ...) -> DrillPlan`. Groups hits by
    tool via BTreeMap (ascending = min tool changes), cursor carries across tool groups. `DrillMove::{Drill,Slot}`
    (slot from/to flips on reversed). `DrillParams` (depth/feed/retract/peck/dwell) carried as DATA only (Phase 4
    emits cycles). DrillConfig{defaults, overrides: BTreeMap<u32,DrillParams>, start}.
Phase 4 DONE (stacked on `feat/eitri-phase2`, commit 5e9f0fc, 2026-07-04): eitri-gcode NC generation. Split =
  dialect-agnostic EMITTER (decides motions) vs POSTPROCESSOR (renders text). Public API (all re-exported from
  eitri_gcode): `Postprocessor` trait (required `name`/`format`/`start_code`/`end_code`; DEFAULT RS-274 hooks
  `rapid`/`linear`/`arc`/`dwell`/`spindle_on`/`spindle_off`/`tool_change`/`comment` — a 2nd dialect only overrides
  frame hooks); hook data `JobContext`/`Spindle`/`ToolChange`/`Axes`/`ArcMove`; `Registry` (name→dialect,
  with_builtins). Impls: `GrblHal` (default, grblHAL/Skirnir-conformant) + `Generic` (semicolon/CRLF/3-dec/M30, proves
  seam). `emit_isolation<G:CutRing>`/`emit_drilling`; `CutRing` impl'd for RingPath (linear) & ArcPolyline (arc→G2/G3).
  `depth_steps(total,step)` shared multi-depth. `check_grbl_conformance(text)->Vec<Violation>` public validator. Golden
  files eitri/fixtures/golden/*.nc (regen: EITRI_REGEN_GOLDEN=1). 165→214 tests. arc.rs `bulge_to_arc`: centre
  C = midpoint + ((1-b²)/(4b))·(-Δy,Δx), I/J = C-start; b>0→CCW→G3, b<0→CW→G2. Drilling: NO canned cycle (grbl lacks
  G81/G83) → peck expanded to manual plunge/retract moves.

  (Phase-3 note) Output types are TOOLPATH/OPERATION models (rings, ordered moves), NOT G-code — Phase 4 (eitri-gcode) still
  deferred, untouched. eitri-gerber/eitri-excellon added to eitri/ workspace.dependencies; rayon "1" added.

Phase 5 DONE (stacked on `feat/eitri-phase2`, commit 8c60acc, 2026-07-04): remaining eitri-cam ops. 214->280 tests, -D
  warnings + clippy --all-targets clean, root galdr untouched (eitri is a separate nested workspace). ALL fill/profile
  output reuses the Phase-4 emitter — NO parallel emitter.
  - REUSE MECHANISM: ops produce `IsolationToolpaths<RingPath>` via `IsolationToolpaths::from_paths(paths, winding)`;
    `emit_isolation` consumes them unchanged. Ring closed-ness = first==last (NO new geometry field); `RingPath` gained
    `open()`/`closed()`/`start()`/`end()`/`is_closed()` (is_closed rule = len<2 OR start≈end; a 2-pt distinct path is
    OPEN — the len<3 rule was WRONG, bit the raster test). Emitter got `CutRing::is_closed()` (delegates to RingPath's)
    + open-path multi-depth: between depth passes an OPEN path lifts to travel_z & rapids back to start; CLOSED rings
    byte-identical so isolation goldens unchanged.
  - eitri-geo new primitives (all geometry stays in eitri-geo; eitri-cam sees only geo_types): `apply_affine`/
    `apply_affine_polygon` (MapCoords + eitri-core Affine — ONE transform source of truth; mirror/neg-scale flips
    winding, renormalize downstream), `clip_lines` (geo BooleanOps::clip, raster spans, holes split lines), `bounds`,
    `contains_point` (geo Contains, raster connector test). Files: eitri-geo/src/{transform,region}.rs.
  - paint.rs (§7.2): `PaintStrategy: Sync` object-safe trait, `fill(region:&Polygon, params, backend:&(dyn GeoBackend+
    Sync), cancel)`. Strategies: `Concentric` (inward offset radius+n*step, outer->inner), `Seed` (concentric reversed),
    `Raster{angle_deg}` (rotate into scan frame, offset -radius, clip horizontal scan lines, boustrophedon connect —
    break to new path when link midpoints leave region via contains_point; rotate back). `paint()` insets by margin,
    rayon over disjoint regions, optional finish_pass (boundary rings). `PaintResult.toolpaths()`. `order_paths` reuses
    TravelOptimizer (closed=point stop, open=reversible segment).
  - noncopper.rs (§7.3): `Boundary::{BoundingBox{margin}|Region(mp)}`, `clear_region`=frame-copper, `clear_noncopper`=
    clear_region then paint. Pure composition.
  - cutout.rs (§7.4): `CutoutOutline::{Rectangle|Geometry}`, `TabPlacement::{Count(n)|AtFractions(Vec<f64> in [0,1))}`,
    outward offset radius+margin, `split_ring_at_tabs` (pub(crate), unit-tested): arc-length param, augmented-vertex +
    break-point list, start walk AFTER a gap edge so kept runs never straddle seam → open arcs preserve order/direction.
    Guards: no tabs/zero width->one closed ring; tabs>=perimeter->empty.
  - panelize.rs (§7.7): `Spacing::{Gap(g)|Pitch(p)}` — GAP step=extent+g, PITCH step=p (the gotcha). `panel_offsets`
    (pure), `panelize_multipolygon` (translate+union, rayon), `panelize_points`.
  - twosided.rs (§7.9): `MirrorLine::{Vertical(x)|Horizontal(y)}`+center helpers, `mirror_multipolygon`/`mirror_points`,
    `alignment_holes` (base ∪ mirror, dedup on-axis).
  - edit.rs (§7.10): thin uniform `transform`/`transform_points`/`buffer`/`simplify`/`join` over Affine + backend.
  - film.rs (§7.8): SHIPPED MINIMAL positive/negative SVG (`FilmKind`, scale/mirror/border, evenodd holes, Y flipped);
    negative = difference(bordered frame, copper). SVG READ side stays Phase 6. clippy: use writeln! not write!+\n.
  - Emit proof: eitri-gcode/tests/fill_and_cutout_emit.rs — paint+cutout .toolpaths() through emit_isolation, asserts
    check_grbl_conformance empty + open-path multi-depth lift/return. CAM ops tested with INLINE synthetic geometry
    (squares/annulus/L/U), no new fixture files (noted in fixtures/README.md).

Phase 6 DONE (stacked on `feat/eitri-phase2`, commit d527c0a, 2026-07-04): eitri-import (SVG/DXF/G-code) + the
  Phase-4-deferred G-code lexer. 280->330 tests, -D warnings + clippy --all-targets clean, root galdr untouched.
  - eitri-gcode LEXER now EXISTS (was deferred): `src/lex.rs`, re-exported `lex`/`lex_line`/`Line`/`Word`. Shared
    word model with the emitter (plan §8). `Word{letter(upper),value}`, `Line{words,comments,block_delete}` +
    `Line::value(letter)`/`has(letter)`. Handles ws-insensitivity, case-fold, `(...)`/`;` comments (unterminated
    paren = lenient to EOL), signed/`.5`/`5.` numbers, N/`/`block-delete/`%`; malformed = `Error::Parse`. `lex`
    drops fully-blank lines.
  - eitri-geo flatten.rs GAINED shared flatteners (all return points AFTER start, incl end): `flatten_cubic`/
    `flatten_quad` (adaptive de Casteljau, flatness = squared perp-dist of BOTH controls from chord vs
    CHORD_TOLERANCE_MM, depth cap 24), `flatten_arc(cx,cy,r,start,sweep signed)`, `flatten_bulge` (same sqrt-free
    centre as arc.rs bulge_to_arc). GOTCHA: de Casteljau midpoints ARE exact curve points, so the flatten-quality
    test must measure curve→polyline dist (sample curve, nearest polyline segment), NOT vertex→curve (that just
    measures your t-sampling resolution — cost a red failure).
  - eitri-core re-exports ADDED (additive): `GEOM_EPSILON_MM`, `MM_PER_INCH` at crate root.
  - eitri-import modules: diagnostic.rs (`Skipped{what,reason}` — skip LOUDLY, never silent-drop), geometry.rs
    (`ImportedGeometry{polygons,polylines}` + `push_subpath(points,closed)` dedup+auto-close, `bounds()`), svg.rs,
    dxf.rs, gcode.rs. Entry APIs: `import_svg(&str,&SvgOptions)->SvgImport`, `import_dxf(&str)`/`import_dxf_reader<R:
    Read>`->`DxfImport`, `import_gcode(&str)->GcodePreview`. Each result = geometry + `Vec<Skipped>` (gcode's is
    `preview.skipped`).
  - DEPS pinned: usvg = "0.47", dxf = "0.6" (both compile clean on this darwin host; usvg pulls a big tree ~image/
    rustybuzz/fontdb but builds). eitri-import Cargo.toml uses eitri-gcode via path (not workspace.deps), eitri-cam
    is a DEV-dep.
  - usvg 0.47 API FACTS (verified from registry src): `Tree::from_str(str,&Options)`; `Options{dpi,..Default}`;
    `tree.size().height()`; `tree.root().children()->&[Node]`; `Node::{Group(Box),Path(Box),Image,Text}`;
    `path.data()->&tiny_skia_path::Path` is LOCAL coords — MUST apply `path.abs_transform()` (Transform{sx,kx,ky,
    sy,tx,ty} f32; x'=sx*x+kx*y+tx). The data() doc saying "absolute" is MISLEADING (parser stores data raw +
    abs_transform separately — confirmed in converter.rs Path::new). `PathSegment` via `usvg::tiny_skia_path::
    PathSegment` (usvg re-exports the module). SvgOptions: flip_y default TRUE (Y-down SVG -> Y-up CAD, reflect
    about canvas height), scale default 1.0 (user unit = mm). undecodable <image> href is DROPPED by usvg (test
    needs a VALID 1x1 PNG data URI to get an Image node to skip).
  - dxf 0.6 API FACTS: entities code-GENERATED at build (see target/.../out/generated/entities.rs). `Drawing::load(
    &mut src.as_bytes())`. `drawing.entities()->&Entity`, `entity.specific: EntityType::{Line,Circle,Arc,LwPolyline,
    Polyline,Spline,ModelPoint(NOT Point),...}`. Line.p1/p2 (Point x,y,z f64). Arc angles in DEGREES CCW. LwPolyline
    .vertices[] {x,y,bulge} + `is_closed()`. Polyline `.vertices()` iter Vertex{location:Point,bulge} + `is_closed()`
    /`is_3d_polygon_mesh()`/`is_polyface_mesh()` (skip meshes loudly).
  - GcodePreview: `moves: Vec<PreviewMove{kind:MotionKind::{Rapid,Cut},from,to:Point3{x,y,z}}>` + `skipped`.
    `cut_polylines()` = contiguous Cut runs split at Rapids, XY-projected + consecutive-coincident-deduped, so a
    plunge (Z-only) leaves no XY trace and the recovered contour == the source ring. Modal walk: G90/91 abs/rel,
    G20/21 units->mm, G0-3 motion, arc centre from I/J (start-relative) or R (signed, major/minor via sign), G18/19
    or missing-centre arc = loud skip + straight cut. Emit<->import round-trip (square + golden .nc) in
    eitri-import/tests/roundtrip.rs; fixtures svg/shapes.svg + dxf/entities.dxf in tests/fixtures.rs.
  - eitri-project Object/serde schema STILL deferred (Phase 7) — flagged, not invented. Import outputs are
    parser-local geo_types previews only.

Phase 7 DONE (stacked on `feat/eitri-phase2`, commit 2474737, 2026-07-04): eitri-project = the integration crate.
  330->345 tests, -D warnings + clippy --all-targets clean, root galdr untouched. Modules: object/collection/
  document/project/history/tooldb/id/error/serde_ext (all pub except serde_ext). NO pickle, NO FlatCAM import.
  - Object model = ENUM not inheritance: `Object{meta:ObjectMeta, payload:ObjectPayload}`; ObjectMeta{id:ObjectId,
    name,units,placement:Affine,visible,notes}; ObjectPayload::{Gerber,Excellon,Geometry,CncJob}. `Object::kind()`
    -> ObjectKind. `ObjectMeta::new(name,units)` stamps id=ObjectId(0) placeholder (collection overwrites).
  - ObjectCollection: Vec-backed insertion order, monotonic never-reused u64 ids (next_id starts 1), unique-name
    enforce on add/rename (DuplicateName), by_name/get/get_mut/remove(also drops from groups)/reorder(clamped)/
    groups (create_group/add_to_group idempotent/group_members). Serde-derived directly (fields private).
  - SCHEMA DECISIONS (load-bearing, ratified-pending): (i) versioned envelope {schema_version:u32, name, collection},
    SCHEMA_VERSION=1; load = parse to Value -> read_version -> document::upgrade(version,value) [MIGRATION HOOK:
    match on version, current=identity, older arms migrate+recurse, else UnsupportedVersion] -> from_value. (ii) JSON
    (serde_json to_string_pretty) for v1 — human-readable; envelope is serializer-agnostic so binary later behind same
    gate. (iii) geometry via geo-types "serde" feature (enabled in workspace Cargo.toml — the ONLY upstream change;
    additive) — Polygon/LineString serialize {exterior:[{x,y}..],interiors:[]}. (iv) PERSISTED vs RE-DERIVED:
    Gerber/Excellon persist embedded `source:Arc<str>` + re-parse on load (image:Option<_> is #[serde(skip)], filled
    by Project::hydrate(progress,cancel) — excellon re-parsed with None override, caveat noted); CncJob persists
    rendered `gcode:Arc<[String]>` (the deliverable) + dialect + source:Option<ObjectId> + operation; Geometry stores
    geo-types directly (no source to re-derive). (v) undo = SNAPSHOT-based History (clone collection onto bounded
    past/future stacks, DEFAULT_HISTORY_LIMIT=64; Arc blobs make clone a refcount bump). (vi) tool DB =
    ToolDatabase{tools,next_id} own envelope TOOL_DB_SCHEMA_VERSION=1, ToolEntry{id,name,diameter:Length,isolation:
    IsolationDefaults,drilling:DrillDefaults}, seeds IsolationSpec/DrillSpec via isolation_spec()/drill_spec().
  - KEY DECOUPLING: eitri-project OWNS the whole schema. serde_ext.rs pins wire shape of eitri-core Unit("mm"/"inch"),
    Affine([f64;6]), Length(bare mm) via `#[serde(with=...)]` adapters — NO serde derives added to eitri-core/cam/geo
    (envelope version is the stability contract, not field-level DTO isolation). CAM params stored as project-owned
    CURATED specs: CamOperation::{Isolation(IsolationSpec),Drilling(DrillSpec)} #[non_exhaustive]; IsolationSpec.
    to_params() fills join=Round/miter=2.0, DrillSpec.to_params()->DrillParams. DirectionSpec->MillingDirection.
    Paint/cutout/etc ops NOT yet in CamOperation (extension point, geo-types-bearing params deferred).
    UPDATE (commit d484b30 on feat/eitri-phase2, 2026-07-04): CamOperation EXTENDED to all 5 remaining CAM ops, still
    SCHEMA_VERSION=1 (pre-release, additive, no migration arm): Paint(PaintSpec)+PaintStrategySpec{Concentric/Seed/
    Raster{angle_deg}} (to_params->PaintParams, strategy()->Box<dyn PaintStrategy>); NonCopper(NonCopperSpec{boundary:
    BoundarySpec{BoundingBox{margin}|Region(MultiPolygon)}, paint:PaintSpec}); Cutout(CutoutSpec) w/ CutoutOutlineSpec
    {Rectangle{min,max:Coord}|Geometry(MultiPolygon)}+TabPlacementSpec{Count|AtFractions} (to_params->CutoutParams,
    to_outline->CutoutOutline); Panelize(PanelizeSpec{rows,cols,x/y:SpacingSpec{Gap|Pitch}}->PanelSpec via From);
    TwoSided(TwoSidedSpec{mirror:MirrorLineSpec{Vertical|Horizontal}, alignment_holes:Vec<Coord>, hole_diameter})
    to_mirror_line()/alignment_hole_points(). PATTERN: owned points stored as geo_types::Coord (serde-enabled, {x,y});
    eitri_cam::Point is NOT serde so specs convert Coord->Point in to_*(). Input regions stay referenced by ObjectId
    (CncJobObject.source); only op-owned geometry (Region/Geometry outline/alignment holes) serialized inline. 345->351
    tests.
  - GOTCHAS: parse_gerber/parse_excellon return GerberError/ExcellonError -> bridge via `.map_err(eitri_core::Error::
    from)?` (both impl From into eitri_core::Error; ProjectError has #[from] eitri_core::Error as ::Engine). Affine is
    `Affine::IDENTITY` const (no identity()). JoinType is eitri_geo not eitri_cam. serde "rc" feature enabled for
    Arc<str>/Arc<[String]>. Deps: all 7 engine crates + geo-types + serde/serde_json + thiserror (workspace). Added
    eitri-gcode/eitri-import to workspace.dependencies.
  - Report to team-lead flagged 6 schema decisions for USER ratification (schema hard to change post-v1). Next: Phase
    8 eitri-script (typed command API), Phase 9 eitri-app (egui).

Phase 8 DONE (stacked on `feat/eitri-phase2`, commit a7c0b01, 2026-07-04): eitri-script = typed command API + Rhai
  binding. 351->375 tests, -D warnings + clippy --all-targets clean (both feature configs), root galdr untouched.
  Files: src/{error,session,bindings}.rs. rhai 1.25.1 pinned, DEFAULT FEATURES ONLY (no `sync`) — engine driven on one
  thread; cancellation crosses threads only via the already-Send+Sync eitri_core::CancelToken. Feature `scripting`
  (default-on) gates the whole Rhai layer via `dep:rhai`; the typed `Session` builds+tests with --no-default-features.
  tests/rhai.rs is `#![cfg(feature="scripting")]` so --no-default-features --all-targets stays clean.
  - `Session` (src/session.rs) = THE load-bearing API, zero rhai deps, 19 inline tests. Holds: name, History (undo-
    tracked ObjectCollection), ToolDatabase, Registry, DefaultBackend, dialect:String (="grbl"), ProgressReporter,
    CancelToken. Commands: open_{gerber,excellon}_str + import_{svg,dxf,gcode}_str (EAGER parse, cache image), path
    wrappers open_gerber(path)/… (name = file stem); isolate/drill/paint/noncopper/cutout -> CncJob; panelize/mirror/
    transform(+translate/scale/rotate) -> Geometry object; write_gcode(id)->String (+_to path); save_project()->String
    /save_project_to; Session::load_project_str/load_project (assoc ctor, hydrates); object mgmt rename/delete/reorder/
    create_group/add_to_group/undo/redo; tool DB add_tool/tool_isolation_spec/tool_drill_spec/save_tool_db/load_tool_db;
    set_dialect(name) validates against registry.
  - PATTERN (borrow-safe): CAM op = region_of(src) clones an OWNED MultiPolygon (drops the imm borrow) -> run cam op
    (imm borrows backend/progress/cancel) -> emit in a `{ let post=self.post()?; emit_*(...) }` block (post borrow ends)
    -> store_program/store_geometry via history.edit (mut borrow). NEVER run heavy CAM inside history.edit (borrow
    clash + one clean snapshot per commit). add() PRE-CHECKS dup name so a rejected add leaves no undo snapshot.
  - region_of: Gerber->image.copper (re-parse if cache None), Geometry->polygons; else WrongKind. cutout takes NO source
    (outline is self-contained in the spec). CncJobObject stores CamOperation::{Isolation/Drilling/Paint/NonCopper/
    Cutout}(spec) + dialect + source. drill uses TwoOpt::new() + DrillConfig{defaults=spec.to_params(), overrides:{},
    start:(0,0)}.
  - GOTCHAS (API reality vs the Phase-6 memory notes): import_svg/import_dxf/import_gcode ALL return eitri_core::Result
    (NOT bare SvgImport/DxfImport/GcodePreview) — just `?`, no map_err. parse_gerber/parse_excellon return their own
    GerberError/ExcellonError -> `.map_err(eitri_core::Error::from)?`. GrblHal registry name is "grbl" (Registry.get
    ("grbl")), NOT "grblHAL". Unit::Millimeters/Inches (not Mm). eitri_geo::bounds -> Option<(x0,y0,x1,y1)> tuple.
  - Rhai binding (src/bindings.rs): Session isn't Clone (Registry has trait objects) -> `ScriptSession(Rc<RefCell<
    Session>>)` newtype registered as rhai type "Session"; caller keeps an Rc-clone handle to read mutations back.
    Ids cross as i64 (rhai INT). Params = typed constructor fns (isolation_spec/drill_spec/paint_spec/cutout_rect_spec/
    noncopper_bbox_spec/panelize_gap_spec/cut_job/drill_job); enums (direction/strategy/mirror axis) passed as strings,
    parsed centrally -> clean InvalidArgument on a bad word. `fn rhai<T>(Result)->Result<T,Box<EvalAltResult>>` maps
    every ScriptError to EvalAltResult::ErrorRuntime (a catchable script error, never a panic). `eval(&ScriptSession,
    script)` wires engine.on_progress -> if cancel.is_cancelled() Some(UNIT) -> aborts with ErrorTerminated (the rhai
    cancellation idiom); cancel test PRE-cancels then runs `while true{}` (deterministic, cannot hang).
  - File-I/O decision reported: string-core (pure, testable) + thin path wrappers for CLI/script ergonomics. rhai
    features: default-only, justified above. Phase-7 schema NOT disturbed (eitri-script only READS the specs/CamOp).

Phase 10 DONE (stacked on `feat/eitri-phase2`, commit 445c766, 2026-07-05): eitri-geo GEOS Shapely-parity backend.
  Replaced the Phase-1 GeosBackend STUB with a real GeoBackend impl on the `geos` crate (11.1.1) against system GEOS
  3.14.1 (brew install geos — NOW INSTALLED on this darwin host; geos-config at /opt/homebrew/bin). Fully gated behind
  `--features geos`; DEFAULT build unchanged (532 passed). geos-feature: 46 lib + 5 parity + 11 doc, all green, clippy
  clean -D warnings both feature configs.
  - Cargo.toml feature now `geos = ["dep:geos", "geos/geo"]` — the geos crate's `geo` sub-feature pulls geo-types/wkt
    interop (adds `wkt` to Cargo.lock; geos/geos-sys/pkg-config already there from the stub).
  - offset = geos::BufferParams::builder() {end_cap Round, join JoinType->JoinStyle, mitre_limit=param, quadrant_segments
    =8 (SHAPELY_QUADRANT_SEGMENTS const)} + Geom::buffer_with_params(distance,&params). Shapely defaults = round/8/round-cap;
    mitre from caller. JoinType::Square -> GEOS Bevel (GEOS has NO squared join, only a Square *cap*). Normalizes winding
    CCW first (parity of preconditions w/ DefaultBackend). Negative distance = inward (native); collapse -> empty.
  - union_all = MultiPolygon(slice) -> Geom::unary_union (empty slice short-circuits). difference/intersection = Geom::
    difference/intersection. simplify = topology_preserve_simplify (Shapely preserve_topology=True; NOT raw DP simplify).
    normalize_winding = crate::winding::normalize (pure, shared w/ DefaultBackend & stub — backends can't disagree).
  - convert mod (private in geos_backend.rs): geos::Geometry::try_from(&Polygon/&MultiPolygon) in; geo_types::Geometry::
    try_from(&geos) out (geos crate pivots through WKT at full double precision, roundingPrecision=-1, loss-free ~16 dig).
    collect_polygons coerces Geometry->MultiPolygon: Polygon/MultiPolygon pass, GeometryCollection flattened, lower-dim
    (touching edges/points) DROPPED = areal-only trait contract (DefaultBackend drops same by type). geos_to_multipolygon
    RETAINs non-empty exteriors (GEOS `POLYGON EMPTY` from over-inset -> empty MultiPolygon, not a 1-elt holding an empty
    poly — the bug my first test caught). geos_to_polygon (for simplify) = Polygon | 1-elt MultiPolygon else InvalidGeometry.
    geos::Error -> Error::Geometry.
  - PARITY MEASURED (tests/geos_parity.rs, GeosBackend vs DefaultBackend on same inputs): booleans (union/diff/intersect)
    + inward offset EXACT (rel 0.0); ONLY divergence = outward round-join offset rel 1.6e-4 (~0.016%), Clipper2 round
    tessellation vs GEOS 8-quadrant. Tolerances: OFFSET_REL_TOL=1e-2, BOOLEAN_REL_TOL=1e-6. => shipping Clipper2/geo path
    validated as Shapely-faithful.
  - PRE-EXISTING DEFAULT-SUITE FAILURE (NOT mine, proven via git stash): `eitri-app` test
    `app::shell::tests::tool_db_add_edit_and_seed_round_trip_through_intents` (shell.rs:903) asserts app.tool_db.is_empty()
    but HEAD's seed commit (a73b197 "seed the tool library") made EitriApp::new -> tool_store::load() -> seed_library()
    non-empty when no ~/Library/Application Support/eitri/tools.json exists. Fails deterministically on any host w/o that
    file (incl CI). Flagged to team-lead; eitri-app owner should fix the test (seed-aware or construct empty DB). Full
    default suite = 532 passed, 1 failed (this).

Standing constraints/decisions (see [[eitri-geo-backend-decisions]] if written):
- Booleans go through `geo` (i_overlay 4.5.2 transitive), NOT a direct i_overlay pin — avoids a duplicate engine.
- `--features geos` now BUILDS+TESTS here: system GEOS 3.14.1 installed via `brew install geos`, geos crate 11.1.1
  compiles against it (no cmake needed — bottle + geos-config). Default build still needs no GEOS and is the shipping path.
- Winding is load-bearing: normalize (exterior CCW / holes CW) before offsetting or before feeding polygons-with-
  holes to union_all (nonzero fill), or holes get filled. Bit both phases.
- Deferred by design, do NOT invent until their phase: eitri-project Object/serde schema (Phase 7). The eitri-gcode
  postprocessor hook contract is now PINNED (Phase 4, above). The G-code read-back LEXER is deferred to Phase 6
  (eitri-import) — nothing consumes it yet; flagged in eitri-gcode lib.rs docs. STOP and flag on eitri-project schema.
- Phase 4 contract source of truth: `docs/eitri-gcode-skirnir-contract.md` (grblHAL 1.1f; arcs = G2/G3 IJK NEVER R
  NEVER linearized; F on G1/G2/G3 absent on G0; 3+ dec; LF; ≤256B; no N; end M2/M30; strict G/M allowlist).
- kicad-cli is unavailable on this host, so fixtures under `eitri/fixtures/` are hand-authored KiCad-style, not
  plotted from the repo board. Fixtures are LIVE: `eitri-{gerber,excellon}/tests/fixtures.rs` load them via
  `CARGO_MANIFEST_DIR/../fixtures/synthetic/{gerber,excellon}`; each fixture row in `fixtures/README.md` names the
  spec corner / finding it guards.

Phase 1-8 high-review fixes (commit f2012fb on `feat/eitri-phase2`, 2026-07-04) — 7 findings, TDD, 375->384 tests,
-D warnings + clippy clean (default AND eitri-script --no-default-features), root galdr untouched. Policy = handle
correctly OR fail loudly (Error::Unsupported/InvalidGeometry), never silently drop/mis-scale (CAM feeds a real mill).
- excellon parser.rs: (1) infer_format now recognizes legacy M71(metric)/M72(inch) unit codes (were ignored -> mm
  default -> 25.4x-wrong coords+diameters). Single-format model, so inference-scan handling suffices (main loop needs
  no per-line switch). (3) combined tool-DEFINE+coordinate line (T1C0.8X5Y5) now selects+drills the tail after the
  diameter, matching the already-fixed bare-SELECT path; only selects when a coord follows so header defines are
  undisturbed.
- gerber parser.rs: (2) incremental FS notation now REFUSED loudly at FS-set time (Notation::Incremental ->
  Unsupported), matching MI/OF/SF/AS. CHOICE=refuse (not implement): incremental is deprecated/never-emitted and the
  file already refuses all rare deprecated whole-image modes; lowest-risk correct-or-loud. parse_fs stays pure (its
  incremental-parse test unaffected).
- cam/paint.rs: (4) finishing pass (boundary_rings) now insets by tool radius (backend.offset -radius) instead of
  offset 0, so the cutter edge stays inside (was overcutting a full radius outside). boundary_rings now takes &params
  and returns Result. (6) boustrophedon link_inside replaced fixed 0.25/0.5/0.75 sampling with new
  eitri_geo::segment_within (region.rs, re-exported): rejects a link that PROPERLY crosses any ring edge (exterior or
  hole, via geo line_intersection is_proper) OR whose midpoint leaves the region -> catches a hole of ANY size the
  connector passes through. contains_point moved to paint's tests module (now test-only there).
- gcode/emit.rs: (5) peck rapid-return clamped `(-(prev - PECK_RAPID_CLEARANCE)).min(0.0)` so a sub-clearance peck
  never rapids above Z0 (was going positive -> tool into air then plunge through material).
- cam/panelize.rs: (7) progress now advances INSIDE the parallel map via AtomicU64 + per-cell cancel.check (was a
  separate post-loop after all work -> UI snap 0->100%). CAVEAT: this is the ONE finding without a strict fail-before
  test — old post-loop and new in-map code emit an IDENTICAL buffered mpsc event sequence (the defect is purely
  wall-clock timing of emission vs work), so its regression guards the incremental contract (one Advanced per cell,
  1..=total) rather than failing before. All 6 others verified fail-before by reverting each impl.
- REFUTED item isolation.rs:344 arc_vertex_winding left untouched (own review agreed not a bug). No new fixture files
  (all regressions use inline synthetic geometry/strings, consistent with prior phases).

Phase 2 code-review fixes (commit 9c3fe17 on `feat/eitri-phase2`, 2026-07-04) — 10 findings, TDD, all green
(104→131 tests, -D warnings + clippy clean):
- Shared robust interior point `eitri_geo::ring_interior_point` (geo's scanline `InteriorPoint`) replaces the
  mean-of-vertices probe at BOTH the gerber region-assembly and convert.rs offset hole-nesting sites.
- GOTCHA that cost real time: geo's `InteriorPoint` of a hole-free ring returns a DEEP/central point, so it does
  NOT fix a convex frame/donut outer whose centre lands in the hole (the even-odd depth of a frame needs a
  NEAR-BOUNDARY probe, which no single-ring interior point gives). The finding is specifically about CONCAVE rings
  whose vertex-mean falls OUTSIDE the ring — that IS fixed.
- BIGGER gotcha: the gerber region pipeline runs every primitive through `finish()`'s `union_all`
  (i_overlay `unary_union`), which re-derives fill globally and LAUNDERS intermediate `assemble_region` nesting
  errors — so post-union area/poly-count is NOT discriminating for the rep-point bug. The discriminating regression
  for region nesting is a UNIT test on the private `assemble_region` (assert disjoint solids stay 2 polys, no
  spurious hole), not an end-to-end fixture. The offset (convert.rs) site IS observable end-to-end.
- Also: `eitri_core::decode_zero_omitted` (shared zero-omission decode, both parsers map their own errors onto it);
  `eitri_geo::circle_polygon`/`arc_segment_count` hoisted (excellon dropped its fixed 48-seg circle);
  `CHORD_TOLERANCE_MM` now lives in `eitri_core::precision`.
- Correct-or-loud conversions in gerber: `%IPNEG` + MI/OF/SF/AS now `Unsupported` (were silently ignored);
  non-circular stroke `Unsupported` (`stroke_radius` now returns `Option`, `Some` only for Circle); arc with no
  I/J is new `GerberError::InvalidGeometry`; D01/D02/D03 are modal (bare coord line repeats last op, gated on
  `has_coord` so a bare `D10*` aperture-select never triggers a spurious op). Excellon: combined `T<n>X..Y..`
  line now selects AND drills.
