---
name: datum-finder
description: skirnir datum-finding probing (edge/corner/Z) — module layout, two-stage G38.3 latch, wiring seams
metadata:
  type: project
---

Phase 1 (Part A) of the tool-probing plan (`~/.claude/plans/zesty-enchanting-comet.md`): a unified datum
finder mirroring ioSender, built as pure state machines + shell orchestration + thin views panel. Lives in
`crates/skirnir/src/app/datum/` (`comp.rs`, `touch.rs`, `finder.rs`, `mod.rs`).

**Why / how to apply:** it is the NON-ROTARY sibling of the rotary center-finder — it was deliberately built by
mirroring `app/rotary_center.rs` (`WizardState`) + `app/rotary_probe.rs` (`rotary_safe_probe_lines`) +
`ShellRotaryCenter`. When extending it (Phases B/C heightmap reuse the two-stage Z touch), copy those idioms.

Key facts, non-obvious:
- **Two-stage `G38.3` latch** (`touch::touch_lines`): fast search → retract by `latch_distance` → SLOW re-probe
  over `max(latch·1.5, 2)` → `G90`. Uses `G38.3` (no-alarm) so a miss reports `[PRB:…:0]` instead of `ALARM:5`,
  letting a wizard chain touches. The KEPT reading is the SLOW pass; the shell arms `begin_probe` for the whole
  emitted sequence and the wizard reads the LAST resolved `[PRB:]` per touch.
- **Tip comp** (`comp::edge_coord = contact + Ø/2·af`) is applied to X/Y only, never Z. `af` is the approach-dir
  sign and is the WHOLE inside/outside story — an inside (pocket) corner just inverts both approach signs.
- **`Corner { af_x, af_y, inside }`** with consts `A/B/C/D` (ioSender outside map: A=(+X,+Y)…D=(+X,−Y)); `.inside()`
  inverts; `approach_x()/approach_y()` give the effective (inverted-when-inside) sign used for both the touch Dir
  and the comp.
- **`ProbeParams`** (serde+Copy+Default): xy_clearance, depth, probe_distance, latch_distance, probe_feed,
  latch_feed, rapids_feed, probe_diameter, offset. Persisted as `Prefs::datum_bench` with `#[serde(default)]`
  (NO version bump — same precedent as `rotary_bench`; PROFILE_VERSION stayed 1). Seeded into `UiState::datum_bench`.
- **WCS write** = `G10 L2 P0 <axis><machine-coord>` via new `intent::machine_offset_line` (the multi-axis L2 form,
  a sibling of `work_offset_line`'s L20 and `probe_flow::zero_z_line`'s single-axis L2). Datums are NOT persisted;
  only the bench params are.
- **VerifyProbe guard**: `shell::verify_probe_clear` refuses to start when `self.view.pins.probe` (Pn:P asserted).

Wiring seams touched (mirror these for future probe flows):
- `view_state.rs`: added `ProbeKind::Datum`.
- `shell.rs`: `DatumRun` struct + `datum: Option<DatumRun>` field; `ProbeOpSlot::Datum`; `datum_edge_start` /
  `datum_corner_start` / `datum_probe_next` / `datum_write_wcs` / `pump_datum` (copies of the rotary equivalents
  incl. the shared `probe_flow::await_action` lost-push fallback via `TouchFallback`); registered in the frame
  pump + repaint condition + `cancel_probe_ops_except` + `handle_intent` + `snapshot_prefs`.
- `intent.rs`: `DatumEdgeStart{axis,dir,params}`, `DatumCornerStart{corner,params}`, `DatumProbeNext`,
  `DatumWriteWcs`, `DatumCancel`.
- `views.rs`: `datum_finder()` panel (+ `corner_button`, `datum_run_readings`, `datum_bench_params`), threaded
  via `ShellPanelsData::datum`; `UiState` fields `datum_bench` + `datum_edge_axis/dir` + `datum_corner`.
- i18n: added `hdr-datum`/`datum-*`/`lbl-datum-*` keys to BOTH `assets/i18n/{en-US,sv-SE}.ftl` — a key-parity
  test (`i18n/mod.rs`) FAILS if a locale is missing any en-US key, so always add to both.

Loopback shell tests live in `shell.rs` (`datum_touch` helper mirrors `wizard_touch`). See [[engine-write-cancel-safety]],
[[probing-pipeline]], [[settings-and-overrides]], [[autolevel-mesh]].

---

## Phase 2 (Part B1/B4): autolevel Mesh + persistence — DONE

`crates/skirnir/src/app/autolevel/{mod,mesh}.rs`. `pub mod autolevel;` registered in app/mod.rs's NON-gui section
(mesh is egui-free and profile.rs references it, so it must compile in `--no-default-features` — verified).

`Mesh` (pub fields; Debug+Clone+PartialEq+serde): min/max_x/y, grid_x/y (EFFECTIVE = range/(count-1)), nx/ny (≥2),
z: Vec<f64> row-major `z[iy*nx+ix]` DELTAS from the first probed point, max_height (greatest delta, floored ≥0 =
out-of-grid lift), wcs_index. Methods: `from_spacing(min,max,spacing)` (counts = ceil(range/spacing)+1 min 2;
non-positive spacing / zero-range axis → 2 nodes, grid 0, no div-by-zero), `index`, `point_xy`, `set_delta`
(recomputes max_height over whole vec), `interpolate` (bilinear inside bounds-inclusive; OUT-OF-GRID → max_height
LIFT), `is_flat_within(eps)` (peak-to-valley max−min ≤ eps). Mirrors OpenCNCPilot/ioSender HeightMap (MIT).

Profile B4: PROFILE_VERSION bumped 1→2 (mesh genuinely grew the shape); `mesh: Option<Mesh>` with `#[serde(default)]`
so v1 files load with mesh None (v1 ≤ 2 accepted by the version gate). Convention note: the datum_bench field in
Phase 1 did NOT bump the version (serde-default alone); the mesh bump is because the plan reserved 1→2 for it — both
approaches coexist, `#[serde(default)]` is the actual back-compat mechanism. See [[datum-finder]].

---

## Phase 3 (Part C): pre-stream correction — DONE (pure)

`crates/skirnir/src/app/autolevel/{segment,correct}.rs` (24 tests). Registered in autolevel/mod.rs.

`correct_program(program: &[String], mesh: &Mesh, cfg: &CorrectionConfig) -> Result<Vec<String>, CorrectionError>`
— whole-program pure rewrite, driven by `cnc_kinematics::gcode::Parser` (SAME parser firmware/eta use).
CorrectionConfig{correct_rapids:bool=true}. CorrectionError{PositionUnknown, FrameShiftMidProgram, InverseTimeFeed,
NonXyArc, ArcRadiusForm} (thiserror). segment.rs: planar_distance, subdivision_count (ceil(dist/seg), ≥1, seg≤0→1),
ArcSpan{center,radius,start_angle,signed_sweep}::from_endpoints/length/point_at (f64 generalization of preview::flatten_arc).

KEY DESIGN DECISIONS (load-bearing — a reviewer/next-phase must know):
1. OUTPUT = re-emit corrected MOTION lines in canonical `G90 G21` absolute-mm (+ one `G90 G21 G94` header); pass
   every non-motion/non-hazard line through VERBATIM. Re-asserting G90 G21 per corrected move makes passed-through
   source modal words (G20/G91) HARMLESS — I do NOT strip source modal lines. This is the G91/G20 blind-spot fix.
2. SUBDIVISION uses PLANAR (XY) distance not 3D length (mesh varies only in XY; a pure-Z plunge → 1 seg).
   seg = min(grid_x,grid_y). G1 subdivided; G0 corrected-not-split; arcs → real G2/G3 sub-arcs, I/J recomputed as
   (center − sub_start) per sub-arc, Z ramped along helix + mesh at each sub-endpoint, last point snapped to exact endpoint.
3. STATE = cx,cy,pz: Option<f64> (pz == last_programmed_z == current programmed work Z). started_motion gates
   leading-vs-mid-body frame shift. Machine moves (G53/G28/G38) passthrough + invalidate_position. Pure-XY injects
   pz+mesh; NO-axis-word move (F-only) passes through untouched (ioSender #451); first-Z-unknown pure-XY → no Z.
4. R-form arc = i.is_none() && j.is_none() → ArcRadiusForm (parser Arc has no R field). Non-XY plane → NonXyArc.
   Parser Err → passthrough verbatim (parser leaves modal state intact on error).
GOTCHA: coords emitted at {:.3}; a test that parses a formatted coord back and checks geometry (point on circle)
needs ~2e-3 tolerance not 1e-6 (3-decimal rounding ≈5e-4/axis). Phase 4 (task 59) wires correct_program into
shell.rs::start_stream + corrected_program cache + ETA over the corrected total.

---

## Phase 4 (Part C5): pipeline wiring — DONE

Mostly written by team-lead; I fixed a compile break + a correctness gap + added tests. In shell.rs:
`autolevel_cache: Option<Arc<[String]>>`, `resolve_stream_program() -> Result<Arc<[String]>, String>` (off → source
verbatim; on+no-mesh → Err "no height map probed"; on+mesh → correct_program vs profile.mesh, cached), and
`invalidate_autolevel()`. Called from start_stream + simulate (ETA builds over the CORRECTED program so live per-line
remaining keys on the same total the stream acks). invalidate on open_program + AutolevelToggle. UiState:
autolevel_enabled + autolevel_cfg. Transport-bar toggle (views.rs ~1186) emits Intent::AutolevelToggle.

Gotchas fixed:
- egui has NO free-standing `SelectableLabel` widget TYPE in this version (0.34) — `egui::SelectableLabel::new(..)`
  won't compile. Use `egui::Button::new(txt).selected(bool)` for a toggle-look button, or `ui.selectable_label(sel, txt)`.
- AutolevelToggle must clear BOTH the autolevel_cache AND the stored simulation (self.clear_simulation()) — a sim
  over the source program shape misindexes the corrected stream's per-line ETA (source vs corrected line counts differ).
  open_program already clears both; the toggle originally cleared only the cache.
- SPLIT documented: Program tab + toolpath preview keep showing the SOURCE file; stream + ETA use the corrected
  (longer, subdivided) program. Progress fraction (acked/corrected_total) stays monotonic 0..1, but the program-tab
  executing-line highlight does NOT map source↔corrected line indices (known limitation, out of Phase-4 scope).
mesh lives on Profile.mesh (not ui) — Phase 5 acquisition must call invalidate_autolevel() after saving a new mesh.

---

## Phase 5 (Part B2/B3): heightmap acquisition wizard — DONE

Built entirely by me (team-lead had NOT started it; no collision). NEW pure file app/autolevel/acquire.rs (8 tests):
GridProbeParams (clearance_z/probe_feed/probe_depth/latch_distance/latch_feed/rapids_feed/probe_offset_x/y; serde+Copy),
grid_points_serpentine(&Mesh)→Vec<(ix,iy)> (COLUMN-major boustrophedon: even cols bottom→top, odd top→bottom), point_probe_lines(probe_xy,params)
→ [G53 clearance, G90 G0 work-XY, datum::touch_lines(Z,Neg) two-stage G38.3, G53 retract] (REUSES datum two-stage
touch by mapping GridProbeParams→datum::ProbeParams), MeshProbeState{mesh,order,cursor,z0,step} — first accepted Z =
z0, each stored value = Z−z0 (delta-from-first-point), :0 miss aborts fail-closed.

Shell wiring (mirrors DatumRun/pump_datum exactly): MeshProbeRun, mesh_probe field, ProbeOpSlot::Mesh, ProbeKind::Mesh,
mesh_probe_start (VerifyProbe-guarded, builds Mesh::from_spacing), mesh_probe_next, pump_mesh, finish_mesh_probe
(on Done: clone mesh → profile.mesh, invalidate_autolevel(), save_profile — THIS is the Phase-4 mesh-change
invalidation seam I flagged), mesh_clear. Intents: MeshProbeStart{params,min,max,spacing}/Next/Cancel/Clear.
Views: mesh_probe() panel + grid_point_counts + mesh_bench_params; ShellPanelsData.mesh_probe/has_saved_mesh; UiState
mesh_min/max/spacing/mesh_bench. i18n keys in BOTH .ftl. 3 loopback tests (full run persists + invalidates cache;
miss aborts w/o persist; VerifyProbe refuses on Pn:P).

Design notes: XY positioning is WORK-coord (G90 G0) not G53 — the mesh is a work-coordinate grid the correction
indexes by work XY; the plan's "G53 G0 X.." would need a WCO the pure builder lacks. probe_offset shifts the
commanded XY (spindle→probe). ApplySavedMesh intent from the plan was NOT added — redundant (autolevel toggle uses
profile.mesh directly; Clear is provided). Auto-grid-bounds-from-program = Phase 6. See [[datum-finder]].

## Phase 5 additions (team-lead's full B2/B3 spec) — DONE
Beyond the initial acquire.rs, added to fully meet the spec: (1) serpentine switched ROW→COLUMN-major (updated 3
tests); (2) `Intent::ApplySavedMesh` + shell apply_saved_mesh (arms autolevel + invalidate_autolevel + clear_simulation;
notice if no mesh); (3) auto-from-program-bounds button → new UiState::program_xy_bounds() reading toolpath_bounds;
(4) live grid/Z preview mesh_preview() (cool→warm dot grid by probed delta; new MeshProbeState::probed() accessor);
(5) GridProbeParams persisted as Prefs.grid_bench (#[serde(default)], no version bump — datum_bench precedent),
seeded into UiState.mesh_bench + snapshot_prefs. GridProbeParams keeps latch_distance/latch_feed (2 fields beyond the
spec's list) — required by the two-stage datum::touch_lines reuse. 761 lib tests green.

## Phase 6 (polish, #61) — in progress
DONE + green (766 lib tests): (1) correct_rapids toggle — Intent::SetCorrectRapids(bool) (sets autolevel_cfg +
invalidate_autolevel + clear_simulation), checkbox in mesh_probe panel (mesh-correct-rapids key both locales).
(2) WCS-mismatch warning — parse_gc_body now extracts active WCS (G54..G59→0..5) into ParserState.wcs; ViewState.active_wcs
tracks it from [GC:] (folds like current_tool; a [GC:] with NEITHER tool NOR wcs still echoes — updated the fold/echo
test); mesh_probe_start stamps mesh.wcs_index = view.active_wcs; shell.warn_on_wcs_mismatch() notices at start_stream when
mesh.wcs_index != active_wcs (warning, not a block). DEFERRED (explicitly "optional" in the task): G92 datum write mode
(semantic complexity — G92 is position-dependent, needs live machine pos, unlike the position-independent G10 L2 the
finder uses; flagged to team-lead), Measure-only datum mode (marginal — panel already shows the computed datum without
writing), reached-position/GotoMachinePosition check (speculative), standalone mesh file export/import (.map RON; profile
already autosaves the mesh). Auto grid-bounds + save/apply/clear mesh were already done in Phase 5.

## Phase 6 #0 — G21 units bug FIXED (hardware-safety)
correct.rs corrected-motion emit sites originally re-asserted only `G90`, NOT `G21` (despite the module doc
claiming G90 G21). A passed-through source `G20` then governed the mm-valued corrected coords → 25.4× inch scale →
crash. Fixed all THREE emit sites to `G90 G21 …`: linear_line, the pure-XY-first-move branch, and the arc branch
(`G90 G21 G17 G2/G3`). G21 is idempotent + all emitted values are already mm (+ fixes F units too). Regression test
`every_corrected_coordinate_line_re_asserts_g90_g21_so_a_stray_g20_cannot_govern` asserts EVERY emitted coordinate
line starts with `G90 G21`. Updated existing prefix-matcher tests (G90 G1/G0/G17 G3 → G90 G21 …). LESSON: the earlier
[[datum-finder]] Phase-3 note "re-emit in canonical G90 G21" was a DOC claim the code didn't honor until this fix; the
inch/mm test only checked the numeric value, never that the surviving G20 no longer governed — so it passed while the
bug lived. Prefix-assert the modal frame, not just the number. NOTE (unfixed, flagged to team-lead): acquire.rs
point_probe_lines emits `G90 G0 X Y` without G21 too — same class, out of the scoped fix. 767 lib tests green.

## Phase 6 — probe sequences made unit-safe (G21)
Same G20 class of bug in the AUTOMATED probe sequences of this feature: fixed by prepending a standalone `G21` as
the FIRST line of both (a) acquire.rs::point_probe_lines (its G53 clearance, G90 G0 work-XY node, and probe
distances are all unit-sensitive → a G20 machine mis-positions the grid probe 25.4×) and (b)
datum/touch.rs::touch_lines (the G91 G38.3 probe distances). G21 must precede EVEN the G53 clearance (G53 values are
still interpreted in current units). Idempotent; touch_lines' G21 is redundantly re-established inside point_probe_lines
(harmless). Updated exact-match/index tests (G21 shifts indices) + added mm-first assertions. rotary_probe.rs
DELIBERATELY untouched — same pre-existing pattern but predates this feature + is shipped/tested; flagged to
team-lead as a known consistency item for a future pass, NOT introduced by us. 768 lib tests green. Whole 6-phase
tool-probing feature (datum + heightmap + correction + pipeline + polish) COMPLETE + green, uncommitted.

## Code-review fixes (task #62) — 9 findings, all with reproducing tests, 776 lib green
1. arc Z-drop fail-open (correct.rs handle_arc): an arc with a Z word but no prior Z (z_start None) DROPPED the Z
   → helical cut silently flattened. Fixed: added (None, Some) arm → emit Z flat at z_end + mesh (don't drop).
2. **TWO-STAGE LATCH kept the FAST reading** (view_state ProbeOp) — THE big one. Latch armed before the whole
   touch, so the fast G38.3's [PRB:] resolved it first (fast, inaccurate reading kept). Fix: ProbeKind::expected_pushes()
   (Datum|Mesh=2, else 1); ProbeOp.remaining; resolve_probe is now count-based last-wins — a SUCCESS updates last +
   decrements remaining, resolves only at 0 (keeps the SLOW 2nd push); a FAILURE resolves immediately (miss on either
   pass aborts). No shell changes (begin_probe(kind) auto-sets it). Loopback helpers datum_touch/mesh_point now inject
   a JUNK fast [PRB:99...] then the real slow prb (proves discard). $#-poll fallback still works (counts as a push).
3. A-word sub-segment mis-sequencing (correct.rs handle_move): A was on the LAST sub-move only (jumped) AND emitted
   raw-incremental under G90 (wrong). Fix: track ca:Option<f64>, resolve_a() (absolute, NO unit scaling — A is deg),
   interpolate A across sub-moves ((Some,Some)→lerp; (None,Some)→endpoint-only).
4. mesh edge float-overshoot lift (mesh.rs interpolate): a boundary point at max_x+1e-12 spuriously lifted. Fixed:
   EDGE_TOLERANCE_MM=1e-6 slack on the out-of-grid guard (cell locator then clamps to the last cell).
5. $G console-echo regression (view_state): my Phase-6 WCS fold made WCS-bearing [GC:] lines suppress-echo, swallowing
   hand-typed $G. Fix: capture active_wcs ALWAYS but suppress echo ONLY on tool (original behavior).
6. stale simulation after mesh change (shell): finish_mesh_probe + mesh_clear invalidated the cache but not the sim.
   Fix: added clear_simulation() to both.
7. over-eager frame-shift rejection (correct.rs handle_coordinate): rejected a G10 L2/L20 targeting ANY WCS. Fix:
   track active_wcs (updated by SelectWcs); reject G10 L2/L20 only when index==active_wcs; G92/G92.1 always.
8. dead inside-corner clamp (finder.rs): corner_clearance/inside_clamped_clearance had no production consumer (corner
   auto-positioning deferred). REMOVED them + re-exports + test.
9. set_delta O(N²) (mesh.rs): rescanned whole z each call. Fix: incremental max_height.max(delta) — O(1), fail-safe
   (a lowered re-probe leaves it conservatively high, never plunges).

## Code-review fixes — RECONCILED to the detailed prescriptions (779 lib green)
The detailed review arrived after my first pass; reconciled the deltas:
- #1 arc Z-drop: first pass only handled absolute-Z-first (emit flat Z). Per prescription, an INCREMENTAL Z with no
  prior Z now fails CLOSED (Err PositionUnknown) matching the linear path; the absolute (None,Some) flat-Z arm stays.
- #7 frame-shift: my first pass fixed a DIFFERENT over-eagerness (WCS-targeting). The REAL reported bug was
  started_motion set by opening RAPIDS/re-anchor → a leading G0+G92 preamble wrongly refused. Fix: started_motion =
  !rapid && base_z.is_some(), computed BEFORE the cannot-place return (Test B's first G1 X10 Z-1 is cannot-place on
  Y but must still mark cutting-begun); arc gates on z_end.is_some(). KEPT the WCS-aware refinement too (complementary).
- #9: added Mesh::recompute_max_height() (O(N) authoritative) called in finish_mesh_probe, so a lowered re-probe
  can't leave max stale; set_delta stays O(1) incremental.
- #4: +exactly-at-max-edge assertion. #8: +reserved-for-deferred-corner-slide docs on ProbeParams.offset/xy_clearance.
- #2 DELIBERATE DEVIATION: prescription was SEQUENTIAL (send fast+retract, arm mid-sequence, send slow). That has an
  ASYNC RACE — shell sends are QUEUED, so arming right after queueing fast+retract happens before the fast [PRB:]
  comes back (seconds later, after motion), so the armed latch still catches the fast push. My COUNT-BASED approach
  (ProbeOp.remaining=expected_pushes, resolve on Nth success / immediate on failure) is race-free and satisfies the
  same test requirement (two [PRB:], slow kept). Flagged to team-lead.
