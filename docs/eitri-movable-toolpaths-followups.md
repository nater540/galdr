# Eitri movable-toolpaths — review follow-ups

Tracking note for the review findings on the movable-components / toolpaths-panel / rebuild / tool-note work
(commit `f1ec3d1`, branch `feat/eitri-movable-toolpaths`). The three confirmed defects below were fixed in that
branch; the rest are deeper follow-ups deferred out of the initial change and captured here so they are not lost.

Line numbers drift — each item names the owning function so it stays findable.

## Status (2026-07-09 follow-up pass)

**All items resolved** on this branch (test-first, `cargo test` + `clippy` clean under `-D warnings`): follow-ups
**#1–#5**, the xhigh-review fixes **R1–R9**, and minors **M1–M3**. See "Resolved", "Review fixes", and "Final
follow-ups (#5, M2)" below. The "Open follow-ups" and "Minor" sections are retained for the original problem
statements.

## Review fixes (xhigh multi-agent review of the pass)

A follow-up `/code-review xhigh` on the branch surfaced 10 defects. Eight were fixed (test-first, workspace green under
`-D warnings`); two were accepted as designed.

- **R1 — `commit_setup` over-coalesced discrete datum actions** (was: Fit-Stock-then-Set-Origin, or open-then-datum,
  merged into one undo step, so one Undo silently wiped the stock too). Coalescing is now **explicit**, not an implicit
  `LastEdit::Setup` marker: `History::commit_setup(coalesce, …)` folds only when the caller asserts an in-progress
  gesture. The Setup panel sets `coalesce` from the actual DragValue drag state (`dragged() && !drag_started()`), so a
  live stock-spinner drag is one entry while every discrete click (datum grid, Z-ref, typed size, Fit/Clear) is its
  own. `Intent::SetStock` carries the flag. Tests: `a_coalesced_stock_drag_is_one_undo_entry`,
  `discrete_setup_changes_are_independent_undo_entries`, and the `History` coalescing tests.
- **R2 — `region_of` leaked stock placement into film export.** Split into `source_region` (native) and `region_of`
  (placed); `film_svg` uses the native region so 1:1 artwork and two-sided top/bottom film registration never inherit
  the board's position on the stock. Test: `film_svg_ignores_the_board_placement_on_the_stock`.
- **R3 — drilling header note could exceed the grblHAL 256-byte line limit.** `drill_tools_note` lists a few tools in
  full but falls back to a count + range summary past `HEADER_NOTE_MAX`. Test:
  `drill_tools_note_lists_a_few_tools_but_caps_many_within_the_line_limit`.
- **R4 — legacy `emission=None` jobs got a permanent, un-actionable ⟳ badge.** `flag_all_jobs_stale` /
  `flag_dependent_jobs_stale` / `move_group` now skip non-rebuildable jobs (`is_rebuildable`), since `rebuild_job`
  refuses them anyway. (No test — constructing an `emission=None` job needs a serde_json dev-dep + fragile JSON
  surgery; the guard is trivial and commented.)
- **R5 — no unchanged-placement no-op guard.** `set_placement` now early-returns when the placement is unchanged (so a
  net-zero drag / `translate_object(id, 0, 0)` pushes no undo entry and flags nothing stale). Test:
  `a_no_op_placement_pushes_no_undo_entry_and_flags_nothing_stale`.
- **R8 — opening a board was two undo steps** (import, then auto-fit). The open-time auto-fit now folds into the
  import's undo entry via `coalesce`, restoring single-undo-per-open. Test:
  `opening_a_board_is_a_single_undo_step_even_with_auto_fit`.
- **R9 — `move_group` did N×O(M) collection passes per drag frame** (the #2 per-member choke-point routing). Restored
  to a single pass that moves members and flags dependent jobs in one walk; `set_placement`/`translate_object` keep
  the `set_object_placement` choke point, so #2's "every placement path flags staleness" invariant holds.
- **R7 — `History::amend` only debug-asserted the no-prior-snapshot misuse.** Added a release-build fallback: a
  first-call amend degrades to a real snapshot instead of an un-undoable mutation. Test:
  `a_coalesce_with_no_prior_entry_falls_back_to_a_fresh_snapshot` (the sibling `commit_setup` path).
- **Accepted as designed:** R6 (the drag stale-badge reconciles on `Intent::EndTranslate`, not per frame — egui
  delivers `drag_stopped` reliably).

## Final follow-ups (#5, M2)

- **#5 — a cutout now follows the board it profiles.** A cutout job is associated with its board via the existing
  `CncJobObject::source` back-reference (`Session::cutout(spec, job, board)`; the shell's `RunCutout` passes the
  selected object, and `OpRequest::Cutout` carries it). Whenever that board's placement changes, the cutout's stored
  outline is carried along by the same movement delta and the job is flagged stale, so a rebuild re-cuts the profile
  in register. This holds on **every** placement path: `move_group` carries it by the drag `shift` in its single
  pass, and `set_object_placement` (the `set_placement`/`translate_object` seam) computes the delta `old⁻¹ ∘ new` and
  applies it. `CutoutOutlineSpec::transform` does the geometry (rectangle corners or silhouette polygons). The outline
  is still self-owned (not re-derived from a source at rebuild), so a hand-drawn silhouette is preserved. Rebuild
  ignores `source` for cutouts, so the association is purely for carry + staleness. Note: the carry applies the full
  rigid delta, so a future rotation UI would rotate the outline too, though a rectangle outline would then skew (the
  drag only translates today). Tests: `moving_a_board_carries_and_stales_its_associated_cutout`,
  `translating_a_board_carries_its_associated_cutout`, `a_standalone_cutout_is_untouched_by_an_unrelated_board_move`,
  `rebuilding_a_carried_cutout_re_cuts_at_the_moved_position`.
- **M2 — one tool-diameter mapping.** The `*_program` builders no longer hand-pick `spec.tool_diameter` /
  `spec.paint.tool_diameter` for the header note; they take the note as a parameter, derived once by `header_note(&
  CamOperation)` from the single source of truth `CamOperation::tool_diameter()` (the same the panel readout uses).
  The public ops build the operation up front and pass the derived note; `rebuild_program` derives it from the stored
  operation. Drilling keeps its own multi-tool summary note. A new single-tool op now updates one place, not two.
  Test: `the_header_tool_note_is_derived_from_the_operation_for_single_tool_ops` (covers the non-copper indirection).

## Resolved (this pass)

- **#2 — single placement-write choke point.** `set_placement` and `move_group` now both route through a private
  `set_object_placement(collection, id, placement)` in `session.rs` that sets the placement *and* flags every
  dependent job (`source == id`) stale. `move_group` no longer hand-rolls its mutation, and `translate_object`
  (which composes onto `set_placement`) now stales dependents too. Tests: `translate_object_flags_dependent_jobs_stale`,
  `set_placement_flags_dependent_jobs_stale`.
- **#4 — rigid-only placement.** New `Affine::is_rigid()` (orthonormal linear part, positive determinant) in
  `eitri-core`; `set_placement` rejects a non-rigid (scale/shear/reflection) transform with a
  `ScriptError::InvalidArgument` before mutating, so `place_hits` (centers-only) and `place_region` (whole-polygon)
  can never diverge. Tests: `is_rigid_accepts_rotations_and_translations_and_rejects_scale_shear_reflection`,
  `set_placement_rejects_a_non_rigid_transform`.
- **#1 — drag no longer rebuilds the whole scene per frame.** New `RenderScene::translate_objects(ids, dx, dy)` shifts
  only the moved source entries' cached vertices/outlines/polylines/bounds (CNC-job previews are skipped — they redraw
  from posted G-code, staying put/stale until rebuild). The `TranslateGroup` intent takes this fast-path each frame;
  the authoritative `refresh_from_session` runs once on the new `Intent::EndTranslate` (emitted from `canvas.rs`
  `drag_stopped`). `Session::move_set(anchor)` is the shared "which ids move together" accessor used by both
  `move_group` and the shell. Tests: `translate_objects_shifts_cached_geometry_to_match_a_full_rebuild`,
  `translate_objects_leaves_cnc_job_previews_in_place`.
- **#3 — datum/stock changes are undoable and stale every posted job.** The work-setup (`origin` + `stock`) moved out
  of `Session` into the undo `History`: it now snapshots a document (`ObjectCollection` + `WorkSetup`), so a datum
  change is one undoable edit that flags every posted job stale and reverts both in lockstep. `History::commit_setup`
  coalesces a consecutive run of setup edits into one undo entry (a stock spinner re-commits every frame of a drag);
  the first commit after any object edit / undo starts a fresh entry. `Session::commit_setup` is the choke point (with
  an unchanged-setup no-op guard); the shell's setup handlers refresh the tree badges + undo state via a cheap
  `refresh_after_setup_change` that skips the scene rebuild (a datum change moves no geometry). The pump now
  auto-fits the stock *after* the import-group `amend` so the group membership still folds into the import entry, not
  the new stock entry. Tests: `changing_the_datum_flags_posted_jobs_stale_and_undoes_together`,
  `a_run_of_setup_commits_coalesces_into_one_undo_entry`, `a_datum_change_after_an_object_edit_starts_a_fresh_undo_entry`,
  `committing_an_unchanged_setup_is_a_no_op_...`, `changing_the_datum_flags_the_toolpath_row_stale_live`, and the
  `History` setup/coalescing tests.
- **M1 — one partition helper.** Extracted `shell::partition_from_session(session) -> (tree, toolpaths, groups)`; the
  shell, `tutorial_shots::synced_view`, and `snapshot_test::loaded_board` all call it, so the harnesses cannot drift
  from the live PROJECT/TOOLPATHS split.
- **M3 — `History::amend` misuse guard.** A `debug_assert!(self.can_undo())` catches a first-call amend (no snapshot to
  coalesce into) instead of silently corrupting undo. Tests in `history.rs`
  (`amend_without_a_prior_edit_panics_in_debug`).

## Fixed (in `f1ec3d1` follow-up)

- **Dragging a toolpath polluted undo.** `canvas::show` could grab a `CncJob` as the drag anchor (jobs are
  pickable via their cut trails); `move_group` then mutated the job's inert `meta.placement`, moving nothing on
  screen but committing a no-op undo entry each frame. Fixed with a pure `drag_anchor()` helper that only returns a
  movable *source* object.
- **Drags during a background op corrupted undo.** The canvas was not gated on `busy`, so a drag begun while an op
  ran off-thread latched `committed = true` even though the shell dropped the move (session away), and the resumed
  drag then called `history.amend` with no opening snapshot. `drag_anchor()` now returns `None` while `busy`.
- **Status bar undercounted.** `views::status_bar` counted `view.tree.len()` only; after the PROJECT/TOOLPATHS
  split that excludes every job. Now `view.tree.len() + view.toolpaths.len()`.

---

## Open follow-ups

### 5. Moving a board does not move or stale its cutout job (by-design gap, no UI signal)

**Where:** `session.rs` `move_group` + `cutout` (a cutout job has `source = None` and a frozen
`CutoutOutlineSpec`).

**Problem:** a cutout owns its outline (no source object), so `move_group` neither shifts it nor flags it stale when
the board it profiles is moved as part of a group.

**Failure scenario:** route a cutout around a placed board, then drag the board — copper and drills shift, the cutout
outline stays put and shows no stale badge, so the profile cut silently misregisters against the moved board.

**Proposed fix:** either move a group-associated cutout's outline with the group, or flag cutout jobs stale on any
move of the group they belong to, or at minimum surface a UI signal that a self-contained cutout may be out of
registration. Requires deciding how a cutout is associated with a board (it currently has no back-reference).

**RESOLVED (2026-07-09):** the cutout is now associated with its board via `CncJobObject::source`, and its stored
outline is carried by the board's movement delta on every placement path (with the job flagged stale). See
"Final follow-ups (#5, M2)" above for the implementation and tests.

---

## Minor / cleanup (low priority)

- **Tool-diameter knowledge in two places.** `CamOperation::tool_diameter()` (UI readout) and the per-op
  `tool_note(spec.tool_diameter)` calls in the `session.rs` `*_program` builders both encode "which spec field is the
  tool" (including the `NonCopper → spec.paint.tool_diameter` indirection). A new single-tool op must update both or
  the header note and the panel readout diverge. **RESOLVED (2026-07-09):** the header note is now derived once from
  `CamOperation` via `header_note()`; see "Final follow-ups (#5, M2)" above.

_(Not a divergence risk: `scene::placed` and `session::place_region` use different helper names
(`eitri_geo::apply_affine` vs `eitri_cam::edit::transform`) but both bottom out in `apply_affine`, so they cannot
produce different results — noted here only so a future reader does not re-flag it.)_
