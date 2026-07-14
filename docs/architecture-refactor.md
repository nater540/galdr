# Architecture Refactor Initiative

Tracking doc for the workspace-wide structural cleanup identified in the 2026-07-13 deep-dive
architecture review (4 domain agents + 2 adversarial "nemesis" verifiers). This doc is the running
ledger — update the status tables as work lands.

## Guiding principles

- **Structure only, behavior never.** These are mechanical splits and DRY extractions. No logic,
  timing, or protocol behavior changes. A refactor step is done only when the build is green and
  (where a host test harness exists) tests still pass.
- **Respect the layering.** `cnc-kinematics` and `firmware-core` stay `no_std` + esp-hal/embassy-free.
  Nothing that touches `esp-hal`, `embassy-*`, or `crate::crash` hardware types may move into them.
- **Incremental + verifiable.** Extract one concern at a time; build after each. Firmware builds via
  `just build` (Xtensa); host crates via `cargo test`.
- **Keep public paths stable.** Splitting a file into a directory module keeps `crate::x::Y` resolving
  by re-exporting (`pub(crate) use child::*;`) from the parent, so call sites don't churn.

## Findings that survived the nemesis pass

Severity/scope reflect the adversarial corrections (some findings were downgraded or rejected).

### Tier 1 — high value

| ID | Item | Status |
|----|------|--------|
| A1 | Split `firmware/src/comms.rs` (5,383 lines, **0 tests**) into a `comms/` module dir | **done** |
| A3 | Split `skirnir/src/app/views.rs` (~4,229 prod lines, 93 fns) into `views/` | **done** |
| A2 | Split `firmware-core/src/protocol.rs` (~2,721 prod lines) into `protocol/` (TMC-diag stays as `protocol/diag_types.rs` — see A2 plan) | **done** |
| A4 | Split `skirnir/src/app/shell.rs` (~2,663 prod lines, 73 methods) into `shell/` impl blocks | Not started |
| D1 | Extract grblHAL error/alarm/run-state constant tables into a shared `no_std` crate consumed by both `firmware-core` and `skirnir` | Not started |
| C1 | `protocol.rs:1488/1509` call the existing `write_axes_csv` helper instead of inlining the loop | Not started |
| B1 | Decompose `comms.rs plan_gcode_line` (181 lines → gate/drive_modal/dispatch) | **done** (with A1 Step 14) |
| A6/B2 | Decompose `main.rs main()` (369 lines) into per-phase init fns | Not started |
| B4 | Extract `handle_soft_reset`/`handle_override` from `dispatch_realtime` | **done** (with A1 Step 5) |

### Tier 2 — real, lower value / care

| ID | Item | Status |
|----|------|--------|
| C5 | skirnir wizard `pump_*` glue — `trait ProbeWizard`/effect-return (4 pumps, ~150 lines; `pump_probe_z` excluded) | Not started |
| C6 | skirnir `views.rs` egui idiom helpers (`dim_label`/`full_width_button`/`gated_action_button`/`param_row`) | Not started |
| C2 | `comms.rs` `enumerate_*`/`send_setting_description` loop family → generic (exclude `dump_settings`) | **done** (with A1 Step 6) |
| C4 | `settings.rs` fold parse/format into `SETTING_DESCRIPTORS` (4-way coupling) | Not started |
| B5 | `gcode.rs apply_g_word`/`apply_m_word` extract `set_plane/units/distance/feed_mode` | Not started |
| C7 | `comms.rs` `cell_update` helper for `BlockingMutex<Cell<T>>` idiom (~14 sites) | Not started (A1 done without it; standalone follow-up) |
| D2 | CI/host round-trip test: `SETTING_DESCRIPTORS` ↔ `settings.proto` lock-step | Not started |
| E1 | Move the **4 truly-pure** helpers (`axis_values_mm`, `units_scale`, `coolant_mask`, `parser_snapshot`) to firmware-core | Not started |

### Explicitly NOT doing (nemesis rejections / cautions)

- **B8 — REJECTED.** `toolbar_content_fingerprint` is a 7-line width-cache key, not a 215-line fn; the
  proposed `derive(Hash)` would defeat the cache.
- **E1 crash-formatter family + `dwell_duration`/`resolve_pause_signal`/`coordinate_report` — do NOT move
  to firmware-core.** They consume `embassy_*` / `crate::crash` (esp-hal-bound) types; moving them
  breaks the `no_std` constraint. Only the 4 pure helpers above are eligible.
- **B3/B7 (`motion.rs run`/`emit_burst`) — do NOT split for size.** Real-time RMT hot path,
  tightly-ordered start-all-then-poll-all sequence, mostly explanatory comments.

## Confirmed clean (no action — recorded so we don't re-litigate)

Shared motion/timing core (firmware `motion.rs` is a thin RMT/GPIO adapter; skirnir `eta.rs` drives the
same `cnc_kinematics::sim`) · single GCode parser · coordinate/WCS math single-sourced in `coords.rs` ·
TMC2209 codec single-sourced in `firmware-core` · skirnir UI↔engine `Intent`/`ViewState` boundary.

---

## A1 — `comms.rs` split plan

Target: `comms.rs` (parent, keeps the module docs + `mod`/re-export wiring) + a `comms/` directory of
concern modules. `comms.rs` currently declares ~60 statics and ~148 fns spanning 12 concerns. Extraction
order is chosen lowest-coupling-first so each step builds green.

Planned modules (Xtensa `just build` after each):

| Step | Module | Contents | Status |
|------|--------|----------|--------|
| 1 | `comms/state.rs` | The ~60 `static`/`const`/atomic declarations + their thin accessors/init fns; re-exported so `crate::comms::X` still resolves | **done** |
| 2 | `comms/crash_report.rs` | `format_*_report` family + `stuck_comms_stage_label` (pure crash-type → `Response` formatters) | **done** |
| 3 | `comms/rx.rs` | `usb_rx`, `line_assembler`, `frame_byte`, `trim_ascii` | **done** |
| 4 | `comms/tx.rs` | `usb_tx`, `write_response_polled`, wedge capture, `enqueue`/`ack`/`error` (+ boot crash-emit/replay cluster) | **done** |
| 5 | `comms/realtime.rs` | `dispatch_realtime` (+ extracted `handle_soft_reset`/`handle_override`, B4) | **done** |
| 6 | `comms/syscmd.rs` | `handle_system_command` + `enumerate_*`/`dump_*`/`send_*` (+ C2 generic) | **done** |
| 7 | `comms/probe.rs` | `run_probe_cycle`, `handle_probe`, `send_probe_report`, `probe_step_period_ticks` | **done** |
| 8 | `comms/jog.rs` | `handle_jog`, `cancel_jog_cycle`, `quiesce_executor`, `refresh_jog_state` | **done** |
| 9 | `comms/homing.rs` | `handle_home`, `run_homing_cycle` | **done** |
| 10 | `comms/program_flow.rs` | `program_end`, `run_program_pause`, `hold_until_resume`, `program_stop_cycle`, `run_dwell` | **done** |
| 11 | `comms/spindle_coolant.rs` | `spindle`/`coolant` tasks, `apply_spindle`, `sync_spindle_from_modal`, `inject_spin_up_dwell` | **done** |
| 12 | `comms/status.rs` | `status_responder`, `auto_report_task` (`parser_snapshot`/`coordinate_report` left in `syscmd`) | **done** |
| 13 | `comms/watchdog.rs` | `watchdog_feed`, `watchdog_heartbeat` (+ `provoke_executor_stall`, feed consts) | **done** |
| 14 | `comms/consumer.rs` | `comms_consumer`, `handle_line`, `plan_gcode_line` (+ B1 decomposition), `plan_command`, `reset_pipeline`, `apply_soft_reset` | **done** |

Notes:
- `#[embassy_executor::task]` fns keep working from submodules (the macro is path-agnostic); their
  spawns in `main.rs` reference them via the re-export, so `main` doesn't change.
- The `cell_update` helper (C7) lands in `comms/state.rs` alongside the `BlockingMutex<Cell<T>>` cells.
- B1 (`plan_gcode_line`) and B4 (`dispatch_realtime`) decompositions happen as their modules are extracted.

### Progress log

- 2026-07-13 — Doc created; A1 plan drafted; beginning Step 1 (`comms/state.rs`).
- 2026-07-13 — A1 Step 1 DONE. Moved the state block (`comms.rs` lines 81–931, `Line`/`Response` aliases through
  `init_planner`) verbatim into `comms/state.rs` (881 lines) behind `mod state;` + `pub(crate) use state::*;`;
  `comms.rs` 5,383 → 4,530 lines. Previously-private accessors (`control_state`, `coordinates`, `settings_snapshot`,
  `refresh_status_cfg`, the `StatusCfg` cache + fields, `WCO_REPORTER`/`OV_REPORTER`/`STATUS_CFG`, …) widened to
  `pub(crate)` for the glob re-export; no call sites changed. One outbound ref (`min_axis_max_rate`, still in
  `comms.rs`) reached via `use super::min_axis_max_rate;`. Pruned 7 now-unused `comms.rs` imports. `just build` +
  `just build --features capture-reset` both green, zero warnings. C7 (`cell_update`) deferred — its reset call sites
  are outside the moved block, so it belongs with the reporter/consumer steps rather than this mechanical move.
- 2026-07-13 — A1 Step 2 DONE. Moved the crash-report formatters (`comms.rs` lines 215–409 post-Step-1: the six
  `format_panic_report`/`format_crash_report`/`format_rmt_hang_report`/`format_usb_tx_stall_report`/
  `format_comms_stage_report` + `stuck_comms_stage_label`, a contiguous block) verbatim into `comms/crash_report.rs`
  (207 lines) behind `mod crash_report;` + `pub(crate) use crash_report::*;`; `comms.rs` 4,530 → 4,338 lines. The five
  `format_*` fns widened to `pub(crate)` for the glob (called by `maybe_emit_crash_report`, which stays); the
  `stuck_comms_stage_label` helper stays private (only `format_crash_report` calls it, and both moved). Only two
  imports needed (`firmware_core::protocol::ResponseWriter`, `use super::Response;`) — everything else was already
  fully-qualified `crate::crash::*` / `firmware_core::diag::*`. `format_usb_tx_stall_report`'s delegation to
  `UsbTxStall::verdict` is untouched. No `comms.rs` imports became unused; no call sites changed. `just build` +
  `just build --features capture-reset` both green, zero warnings.
- 2026-07-13 — A1 Step 3 DONE. Moved the RX path into `comms/rx.rs` (146 lines): the main trio (`comms.rs` post-Step-2
  lines 259–362: `usb_rx` task, `line_assembler` task, `frame_byte`) plus the tail pair (lines 4321–4338: the
  `USB_RX_ERROR_BACKOFF` const, which only `usb_rx` uses, and `trim_ascii`). `comms.rs` 4,338 → 4,217 lines. Behind
  `mod rx;` + `pub(crate) use rx::*;`. `usb_rx`/`line_assembler` were already `pub` (task-spawned from `main.rs` via the
  `comms::` re-export — confirmed unchanged); `trim_ascii` widened to `pub(crate)` (3 callers stay in `comms.rs`);
  `frame_byte` and `USB_RX_ERROR_BACKOFF` stay module-private in `rx.rs`. `dispatch_realtime` (Step 5) and `error`
  (Step 4) stay in `comms.rs`, widened to `pub(crate)` and reached from `rx.rs` via an explicit `use super::{…}`.
  Pruned 5 now-unused `comms.rs` imports (`UsbSerialJtagRx`, `embedded_io_async::Read`, `classify_realtime`,
  `EngineEvent`, `StreamEngine`). `just build` + `just build --features capture-reset` both green, zero warnings.
- 2026-07-13 — A1 Step 4 DONE. Moved the TX path into `comms/tx.rs` (479 lines) as three verbatim blocks from
  `comms.rs` (post-Step-3 numbering): the emit/crash cluster (85–218: `enqueue`, `send_banner`, `send_reset_reason`,
  the `CRASH_REPORT`/`RESET_REPORT` stashes, `maybe_emit_crash_report`, `take_pending_crash_report`,
  `take_pending_reset_report`), the write cluster (421–684: `USB_TX_TIMEOUT`/`USB_TX_PACKET_BYTES`,
  `write_response_polled`, `usb_tx`, `handle_usb_tx_wedge` ×2 cfg variants, `capture_usb_tx_stall_and_reset`), and the
  ack cluster (3906–3954: `ack`, `error`, `error_bare`). `comms.rs` 4,217 → 3,763 lines. Behind `mod tx;` +
  `pub(crate) use tx::*;`. **Crash-emit helpers came along** — they stash+emit through `enqueue`, so they couple to the
  tx path; the pure `crash_report::format_*` formatters (Step 2) stay put and are reached via `super::`. Widened
  `enqueue`/`ack`/`error_bare`/`take_pending_crash_report`/`take_pending_reset_report` to `pub(crate)` (many callers
  remain across the consumer/syscmd/probe/status handlers, all resolving via the re-export — no other file edited,
  `main.rs` spawns/boot calls of `usb_tx`/`send_banner`/`send_reset_reason`/`maybe_emit_crash_report` unchanged). The
  `CRASH_REPORT`/`RESET_REPORT` statics, `write_response_polled`, the two consts, and the wedge helpers stay
  module-private in `tx.rs` (used only within it). `tx.rs` uses `use super::*;` for the wide parent surface (state
  statics + `format_*`, several `capture-reset`-gated) plus explicit external imports; the `UsbTxStall::verdict`
  delegation is untouched. Pruned 6 now-unused `comms.rs` imports (`Cell`, `BlockingMutex`, `Instant`, `yield_now`,
  `UsbSerialJtagTx`, `Async`). `just build`, `--features capture-reset`, and `--features force-withhold` (the moved
  wedge cfg matrix) all green, zero warnings.
- 2026-07-13 — A1 Step 5 DONE (move + B4). (1) Pure move: `dispatch_realtime` + the `try_send_banner` best-effort
  reset banner it owns (its only caller) moved verbatim from `comms.rs` lines 110–274 into `comms/realtime.rs` behind
  `mod realtime;` + `pub(crate) use realtime::*;`. `dispatch_realtime` was already `pub(crate)` (`rx.rs`'s `usb_rx`
  calls it via the re-export — unchanged); `try_send_banner` stays private. `realtime.rs` uses `use super::*;` for the
  wide (~23-item, no-cfg) parent state/accessor surface + explicit `firmware_core::protocol::{ControlState,
  RealtimeCommand, ResponseWriter}` and `core::sync::atomic::Ordering`. Pruned `RealtimeCommand` from `comms.rs`
  (unused after the move). Build green. (2) B4 extract-method (separate change on the green move): the SoftReset arm →
  `fn handle_soft_reset()` and the Override arm body → `fn handle_override(byte: u8)`, both private, called from
  `dispatch_realtime`. VERIFIED byte-identical to the original inline arm bodies via `diff` against git HEAD (only the
  expected 4-space de-indent differs) — no signal/order/flush change. `realtime.rs` is 192 lines; `comms.rs`
  4,217 → 3,600. `just build` + `just build --features capture-reset` green after both the move and the extraction,
  zero warnings.
- 2026-07-13 — A1 Step 6 DONE (move + C2). (1) Pure move: `handle_system_command` + its `$`-query/dump/enumeration/
  build-info/parser-state/coordinate/NGC/startup/`$PBX`/`$RST` family (27 fns + `PB_CHUNK_BYTES`) moved verbatim into
  `comms/syscmd.rs` as FOUR ranges, carving out the interleaved homing pair (`handle_home`/`run_homing_cycle`, Step 9),
  the shared `send_message`, and the coordinate/reporter-cadence trio (`flush_coordinates`/`reset_wco_reporter`/
  `reset_ov_reporter`, consumed by the consumer/reset/status paths) — all stay in `comms.rs`, reached via the re-export.
  `comms.rs` 3,600 → 2,848 lines. Behind `mod syscmd;` + `pub(crate) use syscmd::*;`. `handle_system_command` widened
  to `pub(crate)` (called by the consumer's `handle_line`); six staying-callees widened to `pub(crate)`
  (`force_spindle_off`/`force_coolant_off`/`reset_pipeline`/`send_message`/`reset_wco_reporter`/`settings_write_blocked`),
  and `struct ConsumerState` widened to `pub(crate)` (it appears in the now-`pub(crate)` fn signatures). NOTE:
  `coordinate_report` and `parser_snapshot` moved (their only callers, `$#`/`$G`, are syscmd) — Step 12 will relocate
  them to `status.rs`. syscmd.rs uses `use super::*;` for the parent surface; a child module's glob also surfaces
  `comms.rs`'s own private `firmware_core` imports, so syscmd needs only the few names the glob leaves ambiguous — the
  rest are pruned. No `comms.rs` imports became unused (they now feed syscmd via the glob). (2) C2: the four
  `enumerate_*` + `send_setting_description` collapsed to one generic `async fn enqueue_lines<T>(items, fmt)` + thin
  call sites; same iteration order, same skip-on-format-failure, same `enqueue` calls (the generic body IS the original
  loop). `dump_settings` excluded (snapshots first + appends its own `\r\n`). `just build` + `--features capture-reset`
  green after both the move and C2, zero warnings.
- 2026-07-13 — A1 Step 7 DONE. Moved the `G38.x` probe cycle (a self-contained core-0 gating concern) verbatim from
  `comms.rs` lines 1886–2018 — `probe_step_period_ticks`, `run_probe_cycle`, `send_probe_report`, `handle_probe`, one
  contiguous block — into `comms/probe.rs` (149 lines) behind `mod probe;` + `pub(crate) use probe::*;`. `comms.rs`
  2,848 → 2,717 lines. Pure move (no `send_probe_report` idiom refactor, per the brief). `handle_probe` (consumer plan
  path) and `probe_step_period_ticks` (plan path builds the `ProbeRequest`) widened to `pub(crate)`; `run_probe_cycle`
  and `send_probe_report` stay private (probe-internal). Three staying-callees widened to `pub(crate)`
  (`apply_soft_reset`, `emit_alarm`, `units_scale`). probe.rs needed ONLY `use super::*;` — the child glob supplies the
  entire parent surface (probe signals/types, state accessors, `enqueue`/`ack`, firmware_core types via `comms.rs`'s
  own imports, and the staying helpers), so zero explicit imports and zero now-unused `comms.rs` imports. `just build`
  + `just build --features capture-reset` green, zero warnings, no >120-char lines (wrapped the `handle_probe`
  signature the `pub(crate)` prefix pushed to 126), no double-blank runs.
- 2026-07-13 — A1 Step 8 DONE. Moved the `$J=` jog concern + the shared executor-park primitive into `comms/jog.rs`
  (186 lines) as TWO verbatim blocks from `comms.rs` (post-Step-7 numbering): `handle_jog` + its doc (1900–1959), and
  the contiguous `refresh_jog_state`/`QuiesceOutcome`/`quiesce_executor`/`release_hold`/`cancel_jog_cycle` run
  (1988–2099). `comms.rs` 2,717 → 2,543 lines. Behind `mod jog;` + `pub(crate) use jog::*;`. All six moved items are
  called from `comms.rs` — `handle_jog`/`refresh_jog_state`/`cancel_jog_cycle` from the consumer (lines 584/618/226),
  `quiesce_executor`/`release_hold`/`QuiesceOutcome` from `program_stop_cycle` (Step 10) — so all six widened to
  `pub(crate)` for the glob re-export. The interleaved `current_soft_limits` (shared with `plan_command`) and
  `program_running` (shared with `wait_for_motion_idle`, Step 10) are NOT jog-only, so they STAY in `comms.rs`, reached
  by `jog.rs` via the `use super::*` glob (which surfaces the parent's private items to the child — no widening needed
  on either). `jog.rs` needed ONLY `use super::*;` (zero explicit imports, zero now-unused `comms.rs` imports).
  `just build` green first try, zero warnings; no capture-reset-gated code touched, no >120-char lines, no trailing
  whitespace, no double-blank runs.
- 2026-07-13 — A1 Step 9 DONE. Moved the `$H` homing cycle (one contiguous block, `comms.rs` post-Step-8 lines
  2133–2231: `handle_home` + `run_homing_cycle` with their docs) verbatim into `comms/homing.rs` (112 lines) behind
  `mod homing;` + `pub(crate) use homing::*;`. `comms.rs` 2,543 → 2,446 lines. `handle_home` is dispatched from the
  `$H` arm of `handle_system_command` in the sibling `syscmd.rs`, so it widened to `pub(crate)` for the re-export;
  `run_homing_cycle` is homing-internal (only `handle_home` calls it, both moved) and stays private. `homing.rs`
  needed ONLY `use super::*;` — the child glob supplies the whole parent surface (homing signals/statics, state
  accessors, `enqueue`/`ack`/`error`, the staying `reset_pipeline`/`apply_soft_reset`/`emit_alarm`, firmware_core
  types), so zero explicit imports and zero now-unused `comms.rs` imports. `just build` green first try, zero warnings;
  no capture-reset-gated code touched, no >120-char lines, no trailing whitespace, no double-blank runs.
- 2026-07-13 — A1 Step 10 DONE. Moved the program-flow control concern into `comms/program_flow.rs` (362 lines) as
  TWO verbatim blocks from `comms.rs` (post-Step-9 numbering): the dwell/pause machinery (1224–1470:
  `MOTION_IDLE_POLL`, `wait_for_motion_idle`, `run_dwell`, `program_end`, `PauseOutcome`, `run_program_pause`,
  `hold_until_resume`, `resolve_pause_signal`, `service_held_query`, `peeked_line_is_holdable_query`), and
  `program_stop_cycle` (1933–2031). `comms.rs` 2,446 → 2,101 lines. Behind `mod program_flow;` +
  `pub(crate) use program_flow::*;`. The consumer entry points widened to `pub(crate)` (`run_dwell` — also called by
  `inject_spin_up_dwell`, Step 11 — `program_end`, `PauseOutcome`, `run_program_pause`, `program_stop_cycle`); the
  pause/hold internals stay private (`wait_for_motion_idle`, `MOTION_IDLE_POLL`, `hold_until_resume`,
  `resolve_pause_signal`, `service_held_query`, `peeked_line_is_holdable_query` — all called only within the module).
  JUDGMENT: `dwell_duration` + `MAX_DWELL_S` are NOT program-flow-only (the spindle reverse-dwell shares
  `dwell_duration`), so they STAY in `comms.rs` (reached by `program_flow.rs`'s `run_dwell` via the `use super::*`
  glob); `service_held_query` (undocumented in the step list) is hold-internal so it came along. `program_flow.rs`
  needed ONLY `use super::*;` — zero explicit imports, zero now-unused `comms.rs` imports. `just build` green first try,
  zero warnings; no capture-reset-gated code touched, no genuinely-over-120-char lines (two 121-BYTE lines are 119/117
  chars — em-dashes), no trailing whitespace, no double-blank runs.
- 2026-07-13 — A1 Step 11 DONE. Moved the spindle + coolant drive into `comms/spindle_coolant.rs` (288 lines) as FOUR
  verbatim blocks from `comms.rs` (post-Step-10 numbering): the coolant force-off/modal/pack cluster (387–430:
  `force_spindle_off`, `force_coolant_off`, `sync_coolant_from_modal`, `coolant_mask`, `commanded_coolant`), the
  spindle modal/spin-up cluster (1108–1210: `sync_spindle_from_modal`, `dispatch_spindle`, `SpinUpInjection`,
  `inject_spin_up_dwell`), the two tasks (1227–1295: `spindle`, `coolant`), and the spindle apply cluster (1597–1648:
  `apply_spindle`, `complete_spindle_reverse`, `spindle_emergency_stop`, `commanded_spindle`). `comms.rs` 2,101 →
  1,832 lines. Behind `mod spindle_coolant;` + `pub(crate) use spindle_coolant::*;`. Consumer entry points widened to
  `pub(crate)` (`force_*` were already, `sync_coolant_from_modal`, `sync_spindle_from_modal`, `SpinUpInjection`,
  `inject_spin_up_dwell`); the `spindle`/`coolant` tasks + `commanded_coolant` stay `pub` (main spawns/reads them via
  the re-export); the apply/dispatch internals + `coolant_mask` stay private. JUDGMENT: `coolant_mask` IS coolant-only
  so it moved (with its sole caller `sync_coolant_from_modal`); `sync_coolant_from_modal` + `commanded_coolant` +
  `dispatch_spindle` + `SpinUpInjection` (unlisted but the coherent coolant/spin-up cluster, mirroring Step 10's
  `service_held_query`) came along. `axis_values_mm` is NOT spindle/coolant-only (the gcode dispatch uses it) so it
  STAYS in `comms.rs`, as do the shared `dwell_duration`/`MAX_DWELL_S`; all reached here via the `use super::*` glob.
  `spindle_coolant.rs` needed ONLY `use super::*;` — zero explicit imports, zero now-unused `comms.rs` imports. The
  `pub async fn spindle` fn (value ns) and the `use crate::spindle` module (type ns) coexist without collision. `just
  build` green first try, zero warnings; no capture-reset-gated code touched, no double-blank runs, no trailing
  whitespace; one 124-char line (`apply_spindle`'s `controller.apply(...)` call) is PRE-EXISTING verbatim in HEAD and a
  private un-widened fn, so it was relocated unchanged rather than reflowed (behavior-preserving move, not a regression).
- 2026-07-13 — A1 Step 12 DONE. Moved the `<...>` status reporting into `comms/status.rs` (181 lines) as ONE
  contiguous verbatim block from `comms.rs` (post-Step-11 lines 1574–1737: `status_responder` + `AUTO_REPORT_FLOOR_MS`
  + `auto_report_task` + `effective_auto_report_interval`, with their docs). `comms.rs` 1,832 → 1,670 lines. Behind
  `mod status;` + `pub(crate) use status::*;`. NO widening needed: both tasks are already `pub` (spawned from `main`
  via the re-export) and the two interval helpers are status-internal, staying private. DECISION (per the step note):
  `coordinate_report`/`parser_snapshot` were earmarked here but already moved to `syscmd.rs` in Step 6 (their only
  callers are the `$#`/`$G` handlers) — left in `syscmd`, relocating again would be churn for no gain. `min_axis_max_rate`
  is NOT status-task-owned (it derives the `StatusCfg` cache in `state.rs`, which reaches it via `use super::`), so it
  STAYS in `comms.rs`; likewise `reset_ov_reporter` (soft-reset/program-flow cadence reset) stays just above the moved
  block. `status.rs` needed ONLY `use super::*;` — zero explicit imports, zero now-unused `comms.rs` imports. The one
  `#[cfg(feature = "capture-reset")]` arm in `status_responder` moved verbatim. `just build` AND
  `just build --features capture-reset` both green first try, zero warnings; no >120-char lines, no trailing
  whitespace, no double-blank runs.
- 2026-07-13 — A1 Step 13 DONE. Moved the watchdog + executor-liveness concern into `comms/watchdog.rs` (315 lines) as
  ONE contiguous verbatim block from `comms.rs` (post-Step-12 lines 1084–1382): `WATCHDOG_FEED_INTERVAL`, the four
  `#[cfg(not(feature = "capture-reset"))]` tick consts (`CORE1_STALL_TICKS`, `COMMS_STALL_TICKS`, `RX_ACTIVE_TICKS`,
  `DEAD_ZONE_BACKSTOP_ARMED`), and the three tasks (`watchdog_feed` = not(capture-reset), `watchdog_heartbeat` =
  capture-reset, `provoke_executor_stall` = provoke-* gated), with their docs. `comms.rs` 1,670 → 1,373 lines. Behind
  `mod watchdog;` + `pub(crate) use watchdog::*;`. NO widening needed: the three tasks stay `pub` (spawned from `main`
  via the re-export), `WATCHDOG_FEED_INTERVAL` stays `pub(crate)` (documented intent, now used only within the module
  but kept verbatim), the tick consts stay private. JUDGMENT: `provoke_executor_stall` (a diag stall-provocation task,
  unlisted) came along — it exists solely to exercise the watchdog path and is spawned right beside it; the four feed
  consts are watchdog-internal so they moved too. `dwell_duration`/`MAX_DWELL_S` sit just above the block and STAY.
  `watchdog.rs` needed ONLY `use super::*;` — zero explicit imports, zero now-unused `comms.rs` imports; the
  fully-qualified `esp_hal::rtc_cntl::Rtc` signature resolves without an import. `just build`,
  `just build --features capture-reset`, AND `just build --features provoke-executor-stall` ALL green first try, zero
  warnings; no double-blank runs, no trailing whitespace; seven >120-char lines inside `watchdog_feed` are PRE-EXISTING
  verbatim in HEAD (a `pub` un-widened task) and were relocated unchanged rather than reflowed — behavior-preserving
  move, not a regression.
- 2026-07-13 — A1 Steps 8–13 COMPLETE (`comms.rs` 2,717 → 1,376 lines by true `wc`; the per-step `→ N` figures above
  are the extraction script's immediate post-removal counts and each omits that step's own 2-line `mod`/`use` wiring,
  added afterward). Remaining: Step 14 (`consumer.rs`, carries the B1 `plan_gcode_line` decomposition) — a separate
  focused pass. The `comms/` directory now holds 13 concern modules (`state` 881, `syscmd` 773, `tx` 479, `program_flow`
  362, `watchdog` 315, `spindle_coolant` 288, `crash_report` 207, `realtime` 192, `jog` 186, `status` 181, `probe` 149,
  `rx` 146, `homing` 112 lines); `comms.rs` retains the module docs + `mod`/re-export wiring, the consumer pipeline
  (`comms_consumer`/`handle_line`/`plan_*`/`reset_pipeline`/`apply_soft_reset`), the coordinate ops, and the
  cross-cutting helpers (`emit_alarm`, `send_message`, `units_scale`, `axis_values_mm`, `min_axis_max_rate`,
  `read_live_position`, `dwell_duration`, `current_soft_limits`, `program_running`, `strip_jog_prefix`, …).
- 2026-07-13 — A1 Step 14 DONE (move). Moved the parser → planner consumer pipeline into `comms/consumer.rs`
  (988 lines) as SEVEN verbatim top-level-item blocks carved out of the interleaved parent (post-Step-13
  numbering): `comms_consumer`+`flush_settings`+`apply_soft_reset` (143–367), `reset_pipeline` (396–494),
  `handle_line`+`plan_gcode_line`+`PlanResult` (542–815), `ERROR_PLANNER_UNINITIALIZED`+`plan_command`+
  `drive_pending_arc`+`handle_go_to_predefined` (828–1070), `apply_coordinate_op`+`sync_active_wcs`+
  `axis_values_mm` (1125–1217), `flush_coordinates` (1236–1258), and `QUEUE_FULL_RETRY`+`SETTINGS_FLUSH_SAFETY`
  (1366–1377). `comms.rs` 1,376 → 404 lines. Behind `mod consumer;` + `pub(crate) use consumer::*;`. Items are
  top-level so NO de-indent was needed. Widened FOUR to `pub(crate)` for the re-export glob: `plan_command` +
  `PlanResult` (the `spindle_coolant` sibling's `inject_spin_up_dwell` calls/matches them) and `sync_active_wcs`
  (the `program_flow` sibling's M30/reset paths call it — the one caller the compiler surfaced); `apply_soft_reset`
  /`reset_pipeline` were already `pub(crate)` (homing/probe/syscmd callers); `comms_consumer` stays `pub` (spawned
  from `main`). JUDGMENT — LEFT in `comms.rs` (the small shared-core + wiring parent): `ConsumerState`+`StartupLine`
  +`STARTUP_LINE_MAX` (the state type appears in 6 sibling handler signatures — genuinely shared), and the
  cross-concern helpers `emit_alarm`/`send_message`/`dwell_duration`+`MAX_DWELL_S`/`units_scale`/`strip_jog_prefix`/
  `current_soft_limits`/`program_running`/`read_live_position`/`planner_blocks_free`/`motion_idle`/
  `settings_write_blocked`/`refresh_planner_config`/`min_axis_max_rate`/`reset_wco_reporter`/`reset_ov_reporter` and
  the shared error consts `ERROR_HOLD_CODE`/`ERROR_LOCKED` (each still called by a non-consumer sibling). The moved
  code reaches all of these via `use super::*` — the child glob surfaces the parent's private items and its own
  `firmware_core` imports, so `consumer.rs` needed ZERO explicit imports and no `comms.rs` import became unused (all
  now feed the child via the glob). `just build` + `--features capture-reset` green, zero warnings (rustc ran clean
  through to link even under `-D warnings`); no >120-char lines introduced (the 6 in `consumer.rs` are PRE-EXISTING
  verbatim — em-dash byte-length), no trailing whitespace, no double-blank runs.
- 2026-07-13 — B1 DONE (pure extract-method on the green move). Decomposed `plan_gcode_line` (181 lines) into three
  private helpers, leaving it a thin orchestrator: `gate_line(&ConsumerState) -> Option<ControlState>` (the pre-parse
  error-hold / stale-`Jog`-refresh / alarm-sleep-jog lockout gating; `return;` → `return None;` glue), `drive_modal_
  outputs(parser, state, &parsed, control)` (the clean-parse `sync_active_wcs` + modal spindle/coolant drive; `parsed`
  taken by reference so the caller still owns it for the dispatch match), and `dispatch_plan_result(result, parser,
  state, flash)` (the 13-arm `PlanResult` outcome match, de-indented 4). The DOC-07 spin-up-dwell injection is KEPT
  inline in the `Ok(Some(command))` arm: it is per-command control flow with early returns and runs under a
  narrower condition than the modal drive (only `Ok(Some)` non-Check, not every clean parse), so folding it into
  `drive_modal_outputs` — as the brief's parenthetical suggested — would change WHEN it runs and turn a void
  side-effect into a control-flow-returning gate; that is a restructure, not a pure extract, so it stays put.
  VERIFIED byte-identical: each extracted body `diff`ed against the corresponding region of the pre-B1 `consumer.rs`
  (de-indent-normalized) — all three exact-match, only the expected extract-method glue differs (the two
  `return None;` and dispatch's 4-space de-indent). `just build` + `--features capture-reset` green after the
  decomposition, zero warnings. `consumer.rs` 988 → 1,036 lines.
- 2026-07-13 — **A1 COMPLETE.** `comms.rs` 5,383 → 404 lines (module docs + `mod`/re-export wiring + the send-boot/
  wedge-alarm entry points + `AUTO_REPORT_WAKE` + the ~18 cross-concern shared helpers/consts). The pipeline now
  spans 14 concern modules under `comms/`: `consumer` 1,036, `state` 881, `syscmd` 773, `tx` 479, `program_flow` 362,
  `watchdog` 315, `spindle_coolant` 288, `crash_report` 207, `realtime` 192, `jog` 186, `status` 181, `probe` 149,
  `rx` 146, `homing` 112 (5,307 total). All public paths (`crate::comms::X`) stayed stable via `pub(crate) use
  child::*;`; no file outside `comms.rs`/`comms/*` was edited across all 14 steps. The B1 decomposition (Tier-1) and
  the earlier B4/C2 decompositions landed alongside their module extractions; C7 (`cell_update`) and the E1 pure-helper
  moves remain as separate follow-ups.

---

## A3 — `skirnir/src/app/views.rs` split plan

Target: convert `app/views.rs` (5,014 lines; ~4,229 production, tests from 4,230) into an `app/views/` module
directory — **one view per file** under the single `mod views`. Host-compiled, so gate each extraction on BOTH
`cargo build -p skirnir` and `cargo test -p skirnir` (green after each — a stronger net than the firmware side had).

**Key difference from A1 — `super` re-scoping.** `views.rs` is itself a child of `app`, so inside it `super::X` means
`app::X`. Once a view moves into `app/views/<child>.rs`, `super` shifts to `views`. Verified: all 22 `super::<mod>`
references in views.rs are app-level siblings (overrides/datum/autolevel/view_state/intent/rotary_center/preview/
angle_sweep/setting_help/progress/settings_staging/mdi/dock_tiles/rotary_probe/badge/theme/settings_model/runout/
metrics/flip_verify/app_settings), plus exactly ONE `super::super::` (→ crate). So the mechanical rewrite rule for
moved bodies is uniform: **`super::` → `crate::app::`** and the lone **`super::super::` → `crate::`**. View-level
names (the shared types + sibling view fns) resolve via each child's `use super::*;` from `views/mod.rs`.

**External API to keep stable** (via `pub use child::*;` / shared types in `mod.rs`): `views::{toolbar, dock, toolpath,
settings, shell_panels, sanitize_baud, settings_action_needs_confirm, settings_discard_confirm}` and the types
`views::{UiState, EtaQualifier, SetupDialog, ShellPanelsData, PendingSettingsAction, RuntimeStyle}` +
`views::dock_rect_probe`. Callers: `shell.rs`, `dock_tiles.rs`, `snapshot_test.rs`, `app_settings.rs`, `mod.rs`.
Note module `toolbar` (type namespace) and fn `toolbar` (value namespace) coexist — `views::toolbar(...)` still calls the fn.

Planned layout (`app/views/`), one view per file:

| File | Contents | Status |
|------|----------|--------|
| `mod.rs` | module docs + `mod`/`pub use` wiring + shared view-level types (`UiState`+impls, `RuntimeStyle`, `DockTab`, `SetupDialog`, `EtaQualifier`, `ShellPanelsData`+impl, `SetupRunning`, `PendingSettingsAction`) + `sanitize_baud`/`forced_setup_dialog`/`toggle_setup_dialog` + the `#[cfg(test)]` module (widen tested private helpers to `pub(crate)`, or co-locate tests per view — agent judgment) | done |
| `widgets.rs` | shared render helpers: `header_bar`, `section_header`, `setup_dialog_header`, `tab_strip`, `header_title`, `chip_frame`, `paint_pride`, `button_row`, `toolbar_divider`, `dot`, `contained` | done |
| `shell_panels.rs` | `shell_panels`, `setup_menu`, `setup_menu_button`, `setup_dialog_windows` | done |
| `toolbar.rs` | `toolbar`, `toolbar_content_fingerprint`, `compact_tip`, `state_badge`, `transport_group`, `dock_tab_for_click`, `dock_toggle_label`, `ToolbarFit` | done |
| `dro.rs` | `dro`, `endstop_chips`, `endstop_chip`, `pos_toggle`, `big_axis_value` | done |
| `jog.rs` | `jog`, `step_selector`, `jog_enabled`/`jog_blank`/`jog_button`/`jog_z`/`jog_sense`/`emit_jog`, `JOG_STEPS` | done |
| `overrides.rs` | `overrides`, `override_slider`, `override_axis`, `slider_rect_probe` | done |
| `probe.rs` | `probe`, `probe_result` | done |
| `rotary_center.rs` | `rotary_center`, `rotary_bench_params`, `rotary_run_readings`, `rotary_z_datum_picker` | done |
| `datum_finder.rs` | `datum_finder`, `corner_button`, `datum_run_readings`, `datum_bench_params` | done |
| `mesh_probe.rs` | `mesh_probe`, `grid_point_counts`, `mesh_preview`, `lerp_color`, `mesh_bench_params` | done |
| `verify.rs` | `verify_measure`, `verify_done`, `verify_readings_table` | done |
| `dock.rs` | `dock`, `dock_collapse_toggle`, `dock_progress`, `dock_progress_separator`, `dock_eta_qualifier`, `eta_qualifier_text`, `program_follow_target`, `dock_rect_probe` | done |
| `console.rs` | `program_body`, `console_body`, `mdi_strip`, `should_submit_mdi`, `is_ok_noise`, `console_line_style`, `MDI_FIELD_MIN_W` | done |
| `status_bar.rs` | `status_bar`, `alarm_banner`, `tool_change_headline`, `tool_change_banner` | done |
| `toolpath.rs` | `toolpath`, `span_diagonal`, `program_min_z`, `is_moving_state`, `entered_run`, `update_live_overlay`, `draw_grid`, `parse_xy_path`, `toolpath_bounds`, `push_trail_point`, `gcode_words`, `Segment`, `TrailPoint`, `MARKER_*`/`MAX_TRAIL_POINTS` consts | done |
| `settings.rs` | `settings`, `settings_save_button`, `settings_refresh_button`, `setting_edit_should_stage`, `settings_action_needs_confirm`, `setting_tooltip_meta`, `settings_tooltip_ui`, `settings_list`, `settings_discard_confirm` | done |

Order: `mod.rs` scaffolding (types + wiring, build green with the views still inline) → `widgets.rs` (shared, used by
many) → then the leaf views one at a time. Preserve behavior exactly; no widget/layout/logic change. The nemesis
B8 caution applies: `toolbar_content_fingerprint` is a deliberate width-cache key — relocate verbatim, do NOT touch it.

### Progress log

- 2026-07-13 — **A3 COMPLETE.** `app/views.rs` (5,014 lines) → `app/views/mod.rs` (1,360 lines) + 16 one-view-per-file
  children under `app/views/` (5,096 lines total; the +82 over the original is 16×3-line child headers + the 34-line
  `mod`/`pub(crate) use` wiring). Children by line count: `toolpath` 476, `toolbar` 451, `console` 266, `settings` 262,
  `shell_panels` 256, `jog` 239, `rotary_center` 232, `overrides` 230, `dock` 208, `widgets` 208, `datum_finder` 201,
  `mesh_probe` 198, `dro` 172, `status_bar` 132, `verify` 131, `probe` 74. `mod.rs` retains the module docs, the top
  `use` block, the `mod`/`pub(crate) use` facade wiring, the shared view-level types (`RuntimeStyle`, `UiState`+both
  impls, `PendingSettingsAction`, `EtaQualifier`, `DockTab`, `SetupDialog`, `ShellPanelsData`+impl, `SetupRunning`),
  `sanitize_baud`/`forced_setup_dialog`/`toggle_setup_dialog`, the `BAUD_RANGE`/`DEFAULT_BAUD`/`FABULOUS_CLICKS`
  consts, and the full `#[cfg(test)]` module (verbatim, still `use super::*;`).
  - **`super` re-scoping.** The mechanical rule held: in relocated bodies `super::<app-sibling>` → `crate::app::<sibling>`
    (both inline type refs and fn-local `use` statements), and the lone nested `super::super::overrides` inside
    `slider_rect_probe` → `crate::app::overrides`. View-level names (shared types, sibling view fns, retained consts)
    resolve in each child via `use super::*;` — the child glob surfaces the parent's private items (incl. the top `use`
    re-exports of `egui`/`Palette`/`ViewState`/`BadgeState`/…), exactly the A1 pattern, so every child needed only the
    single `use super::*;` line and zero explicit external imports.
  - **Facade wiring.** `mod.rs` re-exports each child with `pub(crate) use child::*;` (NOT `pub use` — the crate has no
    out-of-crate consumers, and `pub use` warns when a child exposes only `pub(crate)` items, e.g. `widgets`). All the
    external paths (`views::{toolbar,dro,jog,overrides,probe,settings,dock,shell_panels,status_bar,transport_group,
    tool_change_headline,sanitize_baud,settings_action_needs_confirm,settings_discard_confirm}` + the types +
    `views::dock_rect_probe`/`views::slider_rect_probe`) keep resolving; no file outside `app/views/` was edited.
  - **Visibility widening.** Private items referenced across the new file boundary were widened to `pub(crate)`
    (auto-detected: an item referenced anywhere outside its home file — 39 genuine widenings, mostly the `widgets`
    helpers, the `toolpath` geometry/overlay helpers `parse_xy_path`/`toolpath_bounds`/`program_min_z`/`span_diagonal`/
    `is_moving_state`/`entered_run`/`update_live_overlay` + the `Segment`/`TrailPoint` types + `MAX_TRAIL_POINTS`/
    `TRAIL_MIN_STEP_FRACTION` consts, and the console/settings/dock helpers the test module pokes). Four struct fields
    the `#[cfg(test)]` module reads across the boundary were widened too: `Segment::{from,to,rapid}` and
    `TrailPoint::{pos,stroke_start}` (`TrailPoint::z` stayed private — no cross-file reader).
  - **Tests.** Kept whole in `mod.rs` (its `super::preview`/`super::super::*` refs resolve unchanged since `mod.rs` is
    still `app::views`); the only requirement was that the private helpers/types/fields it exercises be `pub(crate)` so
    the `pub(crate) use child::*;` facade re-surfaces them into `views` for the tests' `use super::*;`.
  - **Nemesis B8 honored.** `toolbar_content_fingerprint` relocated byte-identical (verified via `diff` against HEAD) —
    stayed a private `fn` in `toolbar.rs`, untouched.
  - **Verification.** `cargo build -p skirnir`, `cargo build -p skirnir --features gui`, and `cargo test -p skirnir`
    all green under `RUSTFLAGS="-D warnings"`; **789 tests pass** (785 + 3 + 1), identical to the pre-split baseline.
    The split was produced by a single deterministic partition script with a coverage assertion (every source line
    placed exactly once), so relocation is behavior-identical by construction — only path spelling and visibility
    prefixes changed.

---

## A2 — `firmware-core/src/protocol.rs` split plan

Target: split `protocol.rs` (4,881 lines; ~2,721 production, tests from 2722) into a `protocol/` module directory.
Pure `no_std` host-tested core → gate each step on `cargo test -p firmware-core` (fast). `firmware` consumes it
heavily (`firmware_core::protocol::{…}`), so ALL paths stay stable via `pub use child::*;` from `protocol/mod.rs`
(zero consumer edits expected), and a final Xtensa `just build` confirms the firmware side.

**Easier than A3:** protocol.rs is a top-level module (`pub mod protocol;` in lib.rs) using crate-absolute paths —
recon confirmed **zero `super::`** in it, so there is no rescoping to do; moved code keeps its `crate::` paths verbatim
and cross-cluster names resolve via each child's `use super::*;` + the `mod.rs` re-exports (A1 pattern).

**Decision — TMC-diag block STAYS in protocol (as `protocol/diag_types.rs`), NOT moved to `drivers/tmc2209/`.**
The nemesis suggested relocating it to shed manager.rs's one `crate::protocol::TmcProbeStage` ref. But recon shows
**`ResponseWriter` formats these types** (`driver_info(&DriverStatus)`, `driver_probe(&DriverProbe)`,
`driver_ioin_raw(&IoinRawCapture)`, the loopback formatter — 5+ refs), and `firmware/src/tmc.rs` imports the whole set
from `firmware_core::protocol::`. Moving the DTOs to `drivers/tmc2209/` would ADD a protocol→drivers formatter
dependency (heavier than the one ref it removes) and churn the firmware import paths. Keeping the DTOs next to the
formatters that serialize them is more cohesive and lower-risk — they are grbl **wire** DTOs, and the wire layer is
protocol. So they become a `protocol/` submodule; all `protocol::X` paths stay native and stable.

Planned submodules (`protocol/`), each `cargo test -p firmware-core` green after extraction:

| File | Contents | Status |
|------|----------|--------|
| `mod.rs` | module docs + `mod`/`pub use` wiring + shared cross-cutting consts (`RX_BUFFER_SIZE`, `BLOCK_BUFFER_SIZE`, `MAX_LINE_LEN`, `AXIS_COUNT`, `AXIS_LETTERS`, `VERSION`, `REFRESH_PERIOD`/`WCO_REFRESH_PERIOD`/`OV_REFRESH_PERIOD`, `RESPONSE_CAPACITY`, `NGC_PARAMETER_LINES`, `STATUS_MASK_MACHINE_POSITION`, `WCS_TAGS`) + the `#[cfg(test)]` module (widen tested privates to `pub(crate)`) | done |
| `realtime.rs` | `RealtimeCommand`, `classify_realtime` | done |
| `codes.rs` | `ErrorCode`, `ERROR_CODES`, `error_name`, the `ERROR_*` consts, `AlarmCode`+impl, `wedge_reset_alarm` | done |
| `state.rs` | `MachineState`+impl, `ControlState`+impl, `UnlockOutcome`, `CheckToggle` (most self-contained, ~500 lines) | done |
| `report.rs` | `PositionReport`+impl, `MachineSnapshot`+impl, `RefreshReporter`, `CoordinateReport`, `LastProbe`, `ProbeResponse`, `probe_response` (the report DTOs) | done |
| `response.rs` | `FmtError`, `ResponseWriter` (both impls — the formatting engine, incl. the `[DRIVER:]`/`$I+` diag formatters), `write_axes_csv`, `write_minimal_f32` | done |
| `stream.rs` | `LineEvent`, `PendingTerminator`, `LineReader`, `StreamEngine`, `EngineEvent` | done |
| `parser_state.rs` | `ParserMotion/Spindle/Plane/Coolant/Units/Distance/FeedMode` (+impls), `ParserSnapshot`+impl | done |
| `system_command.rs` | `SystemCommand` + `classify` | done |
| `overrides.rs` | `Overrides`+impl, `clamp_override`, `OVERRIDE_*` consts | done |
| `pins.rs` | `PinReport`+impl, `SIGNAL_CAPABILITIES` | done |
| `diag_types.rs` | the TMC-diag wire DTOs: `DriverStatus`, `TmcProbeStage`+impl, `DriverProbe`, `BusStats`, `BusErrorKind`, `RxErrorKind`, `IoStage`, `InitFailure`, `IoinRawCapture`, `TMC_LOOPBACK_PATTERN`, `LoopbackReport` | done |

Order: `mod.rs` scaffolding (shared consts + wiring) → leaf clusters (`realtime`, `codes`, `parser_state`, `pins`,
`overrides`, `diag_types`) → `state`, `stream`, `report` → `response` last (it depends on the most others). C1
(`status_report` → call `write_axes_csv`) is a SEPARATE Tier-1 item; do NOT fold it in — pure relocation only here.
DOC-07 caution: preserve every formatter byte-for-byte (the wire format is a host↔firmware contract).

### Progress log

- 2026-07-13 — **A2 COMPLETE.** `protocol.rs` (4,881 lines) → `protocol/mod.rs` (2,305 lines) + 11 cluster children under
  `protocol/` (2,632 lines across children). Executed as a single deterministic partition (the A3 pattern): a coverage-
  asserted script placed every production line 56–2721 into exactly one destination, so relocation is byte-identical by
  construction (verified — see below). Because `protocol.rs` was a top-level module using crate-absolute paths with **zero
  `super::`**, moved code kept its `crate::…` paths verbatim; there was no rescoping (unlike A3).
  - **Children by line count:** `response` 436, `diag_types` 470, `state` 375, `codes` 252, `report` 249, `parser_state`
    203, `stream` 193, `system_command` 148, `overrides` 131, `pins` 102, `realtime` 73. `mod.rs` retains the module docs,
    `#![allow(clippy::result_unit_err)]`, the `mod`/`pub use child::*;` wiring, the 13 shared cross-cutting consts
    (`RX_BUFFER_SIZE`/`BLOCK_BUFFER_SIZE`/`MAX_LINE_LEN`/`AXIS_COUNT`/`AXIS_LETTERS`/`VERSION`/`STATUS_MASK_MACHINE_POSITION`/
    `REFRESH_PERIOD`/`WCO_REFRESH_PERIOD`/`RESPONSE_CAPACITY`/`NGC_PARAMETER_LINES`/`WCS_TAGS`/`OV_REFRESH_PERIOD`), and the
    full `#[cfg(test)]` module (~2,160 lines).
  - **Imports.** The original file had only two top-level `use`s (`core::fmt::Write as _`, `heapless::{String, Vec}`); every
    other reference is fully-qualified `crate::…` inline, so no per-child re-imports were needed beyond `heapless`/`fmt`.
    Children that format got `use heapless::String;` (+ `use core::fmt::Write as _;` where they `write!`): `response`,
    `diag_types`; `pins` took `heapless::String` (uses `.push()`, no `write!`); `stream` took `heapless::Vec` (`LineReader`
    buffer). The other seven children reference sibling/parent surface only and use `use super::*;`. Five leaf children
    (`realtime`, `codes`, `parser_state`, `overrides`, `system_command`) proved fully self-contained — they reference nothing
    from the parent surface, so they carry **no** `use super::*;` at all (it would warn unused).
  - **`pub(crate)` widening.** Twelve private methods are called across the new file boundary (by `response.rs`'s formatters
    and/or the test module) and were widened to `pub(crate)` so the `pub use child::*;` facade re-surfaces them:
    `ParserMotion/Spindle/Plane/Coolant/Units/Distance/FeedMode::word` (6), `MachineState::token` + the two diag
    `TmcProbeStage/*::token` (glob), and the two `write_token` (`TmcProbeStage`, `InitFailure`). The illustrative candidates
    from the plan (`write_axes_csv`, `write_minimal_f32`, `clamp_override`, `PendingTerminator`, `WCS_TAGS`) turned out to be
    used **only within their own child** (or, for `WCS_TAGS`, only by `response` which reads the parent-private const via
    `use super::*;`), so none needed widening.
  - **Test module.** Kept whole and byte-identical in `mod.rs` (verified via `diff` against HEAD) except **two added import
    lines** in its existing `use` block — `use core::fmt::Write as _;` and `use heapless::String;` — since the file-level
    imports it had relied on were relocated to the children. No test was deleted, weakened, or reordered.
  - **Byte-identity proof.** A sorted multiset diff of all non-blank production lines (HEAD `protocol.rs` lines 1–2721 vs the
    union of the new files, tests excluded) shows the **only** differences are the 12 method signatures gaining `pub(crate)`
    and the dropped combined `use heapless::{String, Vec};` — i.e. every formatter (the host↔firmware wire contract) moved
    verbatim. C1 (`status_report` → `write_axes_csv` dedup) was **not** folded in, per the brief.
  - **Verification.** `cargo test -p firmware-core` **351 passed / 0 failed / 1 ignored** (+ the `probe_cycle` integration
    test 1 passed) — identical to the pre-split baseline — clean under `RUSTFLAGS="-D warnings"`. `cargo build` (host
    workspace: firmware-core + skirnir + galdr-proto) green under `-D warnings`. **`just build` (Xtensa firmware) green** —
    the heavy `firmware_core::protocol::` consumer resolves with zero consumer edits (no file outside `protocol.rs`→`protocol/`
    was touched). No trailing whitespace, no missing final newlines, no double-blank runs; the only >120-char lines are
    pre-existing verbatim content (em-dash/table byte-length), carried unchanged.
