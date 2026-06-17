---
name: coordinate-model-phase-b
description: Phase B coordinate-system/offset model — WCO semantics, where work->machine conversion happens, persistence record, $# / WPos / WCO wiring
metadata:
  type: project
---

Phase B of the grblHAL streaming work (G54-G59 / G92 / G28-G30 / TLO + WPos/WCO + `$#`). Built on Phase A
(see [[streaming-state-sharing]]). Landed on branch `firmware/core-pipeline-and-streaming`.

**Coordinate model** lives in `firmware-core::coords` (`CoordinateSystems`, `Copy`, host-tested, NaN-guarded).
Owns G54-G59 `[[f32;3];6]`, active WCS index, G92, TLO (Z scalar folded into WCO Z only), G28/G30 (machine
coords). The ONE relationship: `WCO = G54..59[active] + G92 + TLO`; `MPos = WPos + WCO`. grbl's TLO is INSIDE
WCO — never subtract it twice (the docs' `WPos_Z = MPos_Z - WCO_Z - TLO_Z` double-counts; we use single
subtraction). `set_g92_to_position` computes `g92 = machine - active_wcs - work` (independent of which WCS).

**Where work->machine happens**: the PLANNER still applies the offset for ABSOLUTE moves, but its
`work_offset_mm` was generalized from "G92 only" to the FULL active WCO. The consumer pushes the WCO via
`Planner::set_work_offset(wco)` after every coordinate change. `PlannerCommand::Move`/`Arc` gained a
`machine_coords: bool` (G53 one-shot) that bypasses the offset. The planner's old `SetCoordinateOffset`/
`apply_g92_offset`/`OffsetUpdated` are GONE — replaced by `PlannerCommand::Coordinate(CoordinateOp)` passthrough
-> `PlannerOutcome::Coordinate`. Incremental moves never get an offset (delta unchanged).

**Parser** (`gcode.rs`): new `CoordinateOp` enum (the parser emits these for G10/G54-59/G92/G92.1/G28.1/G30.1/
G43.1/G49). Fractional G-codes (28.1/30.1/43.1/92.1) dispatched in `apply_fractional_g_word` by `round(value*10)`
(`%10==0` -> integer path). `ModalState` gained `wcs` + `tlo_active` (for `$G`). G53 sets a one-shot
`acc.machine_coords` flag and does NOT claim the motion group. G10 P-word: `index = P-1` (P1=G54), absent/P0 =
active WCS. A bare WCS select emits `SelectWcs`; a select sharing a line with a move emits the MOVE, so the
consumer calls `sync_active_wcs(parser.state().wcs)` before planning.

**Consumer wiring** (`comms.rs`): `COORDINATES: BlockingMutex<Cell<CoordinateSystems>>` (sync, like `CONTROL`,
read by `status_responder`). `apply_coordinate_op` resolves set-to-position ops against the planner's COMMANDED
`position_mm()` (grbl `gc_state.position`, race-free vs LIVE_POSITION atomics), scales inch->mm at the boundary,
pushes WCO, marks `COORDINATES_DIRTY` for persistent ops only (G92/TLO are session-only, NOT persisted).
`WCO_REPORTER` (Cell of `protocol::WcoReporter`) drives the `WCO:` include cadence. `$#` -> `dump_ngc_parameters`
(11 lines via `ResponseWriter::ngc_parameter_line`). `$RST=#` -> `handle_restore_params` (clear_all + persist +
push WCO + reset cadence). Soft reset: `coords.clear_volatile()` (drops G92/TLO, keeps persistent) + re-push WCO
to the rebuilt planner + `reset_wco_reporter`.

**Status report**: `MachineSnapshot` gained `wco_mm`, `position_report` (`PositionReport::from_status_mask` off
`$10` bit0 = MPos vs WPos), `include_wco`. `WPos = MPos - WCO`. `$10` bit0 const is
`protocol::STATUS_MASK_MACHINE_POSITION`. WCO cadence: included on change, first-report-after-reset, and every
`WCO_REFRESH_PERIOD` (10) reports.

**Persistence**: SECOND NVS record. `NvsKey::Coordinates = 1` in `storage.rs` (`FlashCoordinateStore` mirrors
`FlashSettingsStore`, same flash/cache, `NVS_KEY_SLOTS` now 2). Proto `Coordinates` message in galdr-proto
(`coordinates_size`/`encode_coordinates_into`/`decode_coordinates`, `COORDINATES_MAX_LEN`). Framing in
`coords::wire` (magic "GdC1", same MAGIC|ver|len|payload|CRC32 layout as settings "GdS1"). New trait
`hal_traits::CoordinateStore`. Boot: `main.rs` loads via `coords::load_or_default` -> `comms::init_coordinates`
-> `comms::seed_planner_work_offset`. Coalesced flush (`flush_coordinates`) alongside settings flush at
burst-boundary / safety-interval / soft-reset.

**Hardware-boundary stub flagged**: `[PRB:..:0]` probe fields are zeros/flag-0 until Phase C probing (DOC-09);
`coordinate_report()` has a TODO there.

Tests: 264 host tests green in firmware-core (`cargo test -p firmware-core`); firmware Xtensa build clean (see
[[build-test-commands]] — run from inside `crates/firmware` with `source $HOME/export-esp.sh`). clippy on
firmware-core clean; firmware-bin clippy shows only the pre-existing intentional `drop(async_flash)` lint.
