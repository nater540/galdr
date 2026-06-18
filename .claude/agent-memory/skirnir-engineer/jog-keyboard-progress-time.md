---
name: jog-keyboard-progress-time
description: skirnir UI features — continuous (press-and-hold) jog, keyboard hotkey jogging, and stream elapsed/ETA clock
metadata:
  type: project
---

Three operator-facing UI features layered on the mature engine (all logic pure + unit-tested, views stay thin).
See [[gui-architecture]] for the layer split and [[engine-architecture]] for the realtime path.

**Continuous (press-and-hold) jog.** `UiState.jog_continuous: bool` (default false) flips the jog pad between
fixed-step and continuous. The `cont` chip (design §03's `0.01/.../10.0/cont` selector) toggles it; picking a
numeric step clears it. Intents: `Intent::JogStart{axis,dir,feed}` (shell sends `continuous_jog_line` — a long
`$J=` toward `CONTINUOUS_JOG_DISTANCE_MM = 10_000.0` mm, the host-side "jog forever" idiom since grbl has no
such command) and `Intent::JogStop` (shell injects jog-cancel `0x85`). In `views.rs`: `jog_sense(state)` returns
`Sense::click_and_drag()` in continuous mode else `Sense::click()`; `emit_jog(response,...)` brackets
`drag_started`→JogStart / `drag_stopped`→JogStop, and a bare `clicked()` in continuous mode emits start+stop so
a quick tap still nudges. Fixed-step still emits one bounded `Intent::Jog`. The shared wire builders
`jog_line`/`continuous_jog_line` live in `intent.rs` (the shell's old inline `format!` was replaced by `jog_line`).

**Keyboard hotkeys.** Pure `intent::key_to_intent(Hotkey, BadgeState, jog_step, jog_feed) -> Option<Intent>`
over an egui-free `Hotkey` enum (Arrow*/PageUp/PageDown/Escape/FeedHold/CycleResume). Bed convention: ↑=Y+ ↓=Y−
→=X+ ←=X− PageUp=Z+ PageDown=Z−. Arrows fire a STEP jog (key-repeat gives the hold cadence; never leave an axis
coasting) only in Idle|Jog (same gate as the pad). Escape = JogCancel while jogging else SoftReset (panic stop);
Hold/Resume = `!`/`~`. `shell.rs::pump_hotkeys(ctx, sink)` does the thin `egui::Key`→`Hotkey` translation, called
first thing each frame, and GUARDS on `ctx.memory(|m| m.focused().is_some())` so typing in the console field never
jogs. `FeedHold`/`CycleResume` variants exist + tested but are NOT bound to physical keys yet (would need Shift,
conflict-prone; toolbar already has the buttons). `intent.rs` now imports `crate::app::badge::BadgeState` — fine,
both modules are NOT gui-gated so the headless `--no-default-features` build still passes.

**Stream elapsed/ETA clock.** New pure module `app/progress.rs` (NOT gui-gated; only `std::time`):
`estimate(elapsed: Duration, acked, total) -> TimeEstimate{elapsed, remaining, total}` projects remaining =
elapsed*(total-acked)/acked, returning `None` for remaining/total until projectable (acked==0 or elapsed==0),
clamping acked≤total. `format_mmss(Option<Duration>)` → `m:ss` (hours roll into minutes, None→`--:--`). The shell
owns the wall clock: `stream_started: Option<Instant>`, maintained by `track_stream_clock()` (edges only) inside
`pump_events()` — stamped when `connection==Streaming`, cleared otherwise. The reducer stays free of
`Instant::now()`. `stream_time()` builds the estimate; passed as a new `time` arg into `views::dock(...)` →
`dock_progress(progress, time)`, which now renders the design's `m:ss / m:ss` pair (was previously omitted as
deferred). dock_progress right-closure is right-to-left so clock drawn first = leftmost reads correctly.

Style/build note: project uses 2-space indent (`.editorconfig`), NO `rustfmt.toml` — stock `cargo fmt` reformats
to 4-space and VIOLATES the convention, so do NOT run it; match the existing 2-space by hand. After this work:
172 tests pass (was 159), `--no-default-features` 148 pass, clippy clean, app boots without panic.
