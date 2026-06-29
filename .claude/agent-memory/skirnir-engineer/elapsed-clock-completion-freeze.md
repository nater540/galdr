---
name: elapsed-clock-completion-freeze
description: The dock elapsed/ETA clock FREEZES on genuine job completion via a stream_finished_at latch in shell.rs; elapsed = finished−started when latched. Don't clear stream_started on leaving Streaming (that blanks, not freezes).
metadata:
  type: project
---

The dock's elapsed/ETA clock is split: `shell.rs` owns the wall clock (`stream_started: Option<Instant>` +
`stream_finished_at: Option<Instant>`), the pure projection lives in `app/progress.rs` (`estimate`/`physics_estimate`,
no `Instant::now()`). `stream_time()` computes `elapsed = finished−started` when the finish is LATCHED (frozen), else
`started.elapsed()` (live), else `Duration::default()`.

**The bug (2026-06-29): the clock kept counting after the job finished + machine returned to Idle.** Root cause:
elapsed was `stream_started.elapsed()` (live `now−start`) every frame with NO terminal latch. The original
`track_stream_clock` *cleared* `stream_started` when leaving `Streaming` — that would BLANK the clock to 0:00, not
freeze it, and it only fired via the reducer's `complete_if_drained` (final `ok`); if that path didn't fire (e.g. a
lost final ack, the Galdr firmware lost-TX-wake class) the lifecycle stayed `Streaming` and the clock counted forever.

**Fix = a display-only finish latch (does NOT touch the streaming lifecycle, which stays host-driven in the reducer).**
`track_stream_clock` now: (1) on `Disconnected` clears both; (2) re-stamps `stream_started=now` + clears finish when
entering `Streaming` AND (`stream_started.is_none()` OR `stream_finished_at.is_some()`) — the `finished.is_some()`
disjunct is LOAD-BEARING: after a completed run `stream_started` is still `Some`, so a plain `is_none()` start-edge
would NOT re-arm a second run (it'd stay frozen on the old job); (3) while a run is timing and not yet frozen, latch
`stream_finished_at=now` once when the run is OVER. "Over" = pure `progress::stream_is_complete(total, acked,
run_idle)` (`total>0 && acked>=total && machine status==Idle`) OR a terminal lifecycle state (`Idle`/`Alarm`/`Error`,
covering graceful Stop/Abort mid-job). `Hold` is deliberately EXCLUDED so a feed-hold pause keeps the clock running.

**Why `run_idle` (the firmware status state, not just the host lifecycle):** a mid-stream `?` poll often reads
`<Idle>` for an instant as the planner drains between blocks — but `acked < total` then, so `stream_is_complete`'s
`acked>=total` term holds the latch off. That is the transient-vs-genuine-completion discriminator the latch needs.
The host deliberately does NOT adopt `<Idle>` into the `Streaming` lifecycle (core.rs ~L374), so the clock can't rely
on the lifecycle alone — it reads `view.status.machine_state.state` directly.

ETA stops naturally at completion: non-sim `estimate` with `acked>=total` → remaining 0; sim path
`eta.rs::remaining_seconds(completed>=lines.len())` sums an empty slice → 0.

**Tests:** pure `stream_is_complete` in progress.rs (genuine end, transient-idle hold-off, pre-start). Shell tests set
`app.stream_started`/`view.connection`/`view.progress`/`view.status` directly and call `track_stream_clock()` (the
same seam the existing `stream_time` tests use — `Instant::now()` isn't injectable, so assert the LATCH state + that
`stream_time().elapsed` doesn't advance across a real 20ms gap once frozen). Covers freeze-on-complete, transient-idle
no-freeze, new-run-clears, Hold-keeps-counting, graceful-stop-freezes. Related: [[jog-keyboard-progress-time]],
[[gui-architecture]], [[engine-architecture]].
