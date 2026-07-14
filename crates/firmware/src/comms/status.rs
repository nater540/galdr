//! `<...>` status reporting (DOC-08 §5): the two core-0 tasks that produce the machine status report — the
//! [`status_responder`], which formats one `<...>` line from the shared [`MachineSnapshot`] (overwriting the live
//! MPos + planner free-block count at report time) whenever `STATUS_REQUEST` fires, and the periodic
//! [`auto_report_task`] (Phase F), which raises `STATUS_REQUEST` on the `$481` interval (floored at
//! [`AUTO_REPORT_FLOOR_MS`], gated by `0x8C` suspend) so an auto-report is byte-identical to a `?` poll and never
//! adds a second USB writer. Extracted verbatim from `comms.rs` (architecture-refactor A1, step 12). Both tasks stay
//! `pub` (spawned from `main` via the re-export); the interval helpers (`effective_auto_report_interval`,
//! `AUTO_REPORT_FLOOR_MS`) are status-internal and stay private. `coordinate_report`/`parser_snapshot` were earmarked
//! here but already moved to `syscmd.rs` in Step 6 (their only callers are the `$#`/`$G` handlers), so they were left
//! there — relocating them again would be churn for no gain. `min_axis_max_rate` STAYS in `comms.rs` (it derives the
//! `StatusCfg` cache in `state.rs`, not the status task). `comms.rs` re-exports this module
//! (`pub(crate) use status::*;`); `use super::*` supplies the whole parent surface (the status signals/statics,
//! `read_live_position`, the snapshot/override accessors, `enqueue`, and the firmware_core types) — no explicit
//! imports needed.

use super::*;

/// The status reporter: format a `<...>` report whenever the [`STATUS_REQUEST`] Signal fires (set by
/// `usb_rx` on `?`/`0x80`/`0x87`). It starts from the shared [`MachineSnapshot`] (run-state / feed / spindle
/// / RX free — the fields the executor does not own) and OVERWRITES the two genuinely-live fields at report
/// time: the MPos from the [`LIVE_POSITION`] atomics (Finding #5 — no longer frozen for a whole block), and
/// the planner free-block count from the live queue depth (Finding #10 — accurate while idle or streaming).
/// Reading these live, rather than from a periodically-published copy, keeps `?` truthful between publishes.
#[embassy_executor::task]
pub async fn status_responder() -> ! {
  loop {
    crate::crash::record_comms_stage(crate::crash::CommsTask::Status, crate::crash::CommsStage::StatusWaitRequest);
    STATUS_REQUEST.wait().await;
    // From here on the task is BUILDING the report — it takes the `MACHINE`/`PLANNER` locks and enqueues the line,
    // all of which can park (cross-core lock contention or a full `RESPONSE` channel). Mark that span so a wedge
    // mid-report is distinguishable from the idle `?`-wait above.
    crate::crash::record_comms_stage(crate::crash::CommsTask::Status, crate::crash::CommsStage::StatusBuildReport);
    // Comms-progress heartbeat: serving a `?` is the most frequent host-facing work (skirnir polls ~5 Hz), so this
    // is the watchdog's primary "the comms path is alive" signal. A wedge that stops answering `?` (the real-board
    // failure: writes succeed, DRO frozen) freezes this counter, which — with the host still sending RX — is what
    // trips the comms-stall feed-withhold. NOTE: this bump is BEFORE the report is built, so it advances on the
    // INTENT to serve; the `usb_tx` bump covers actual output, so the two together bracket the response path.
    COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
    // Read the cached, pre-derived status config (steps/mm, the `$10` MPos/WPos choice, and the feed ceiling)
    // from a synchronous `Cell` — NO `SETTINGS` lock, no ~30-field `Settings` copy on the hot path (Finding
    // #14). The cache is refreshed at every settings-commit site (see `refresh_status_cfg`), so a `$100`/`$10`/
    // `$110` change is reflected in the very next report.
    let cfg = STATUS_CFG.lock(|c| c.get());
    let steps_per_mm = cfg.steps_per_mm;
    let mut snap = *MACHINE.lock().await;
    // Live MPos: read the executor's per-burst step atomics and convert via the host-tested `steps_to_mm`.
    let position = read_live_position();
    snap.mpos_mm = steps_to_mm(&position, &steps_per_mm);
    // Phase B work-position reporting: the live WCO from the coordinate model, the MPos-vs-WPos choice from the
    // `$10` mask bit 0 (cached above), and the `WCO:` include decision from the refresh cadence. Folding these in
    // here keeps the math host-tested (protocol) and the wiring thin.
    let wco = coordinates().wco();
    snap.wco_mm = wco;
    snap.position_report = cfg.position_report;
    snap.include_wco = WCO_REPORTER.lock(|c| {
      let mut reporter = c.get();
      let include = reporter.should_include(wco);
      c.set(reporter);
      include
    });
    // Live `Bf:` free-block count, read from the planner queue under its lock so it is accurate whether the
    // machine is idle, streaming, or back-pressured — consistent with the advertised `BLOCK_QUEUE_LEN`.
    let blocks_free = planner_blocks_free().await;
    snap.planner_blocks_free = blocks_free;
    // Phase E: the live overrides drive both the `Ov:` element (on its change/periodic cadence) and the realized
    // `FS:` feed/spindle. Read the override snapshot once and apply it to the executor-published programmed feed
    // (scaled by the FEED override for a feed/jog move or the RAPID override for a G0, and CLAMPED to the most-
    // restrictive axis max-rate so a feed boost never exceeds `$110-112`) and to the programmed spindle RPM.
    let ov = overrides();
    snap.overrides = ov;
    snap.include_ov = OV_REPORTER.lock(|c| {
      let mut reporter = c.get();
      let include = reporter.should_include(ov);
      c.set(reporter);
      include
    });
    let programmed_feed = f32::from_bits(LIVE_PROGRAMMED_FEED_MM_MIN.load(Ordering::Acquire));
    let is_rapid = LIVE_BLOCK_IS_RAPID.load(Ordering::Relaxed);
    snap.feed_mm_min = if is_rapid {
      // A rapid is already governed by the axis max-rate; the rapid override only ever scales it DOWN.
      ov.scaled_rapid(programmed_feed)
    } else {
      // A feed/jog move scales by the feed override, clamped to the most-restrictive axis max-rate so scaling up
      // cannot exceed the configured rate limit (grbl's rule). The cached `min_axis_max_rate` is the ceiling.
      ov.scaled_feed(programmed_feed, cfg.min_axis_max_rate)
    };
    let programmed_rpm = PROGRAMMED_SPINDLE_RPM.load(Ordering::Acquire).min(u16::MAX as u32) as u16;
    snap.spindle_rpm = ov.scaled_rpm(programmed_rpm);
    // `Pn:` input pins. The probe sources `Pn:P` from its last-sampled logical asserted state (after `$6` invert,
    // published by the probe cycle). X/Y/Z limits source from [`LIMIT_LEVELS`] — the logical-triggered mask (after
    // `$5` invert) the core-1 executor republishes from a level sample at idle (the `Ticker`), at every block
    // boundary, and post-homing, so it tracks both press AND release. Door / feed-hold / reset / cycle-start are
    // optional DOC-06 control-input GPIO that this board does not wire, so they stay `false`; the assembly handles
    // them when a future board sources them — the path is the same host-tested [`PinReport::write_letters`].
    snap.pins = PinReport {
      probe: PROBE_ASSERTED.load(Ordering::Acquire),
      limits: limit_levels(),
      ..PinReport::new_idle()
    };
    // Compose the reported State from the authoritative latched control mode plus whether a block is in
    // flight. "Running" is true if the executor is mid-block OR the planner still holds queued blocks, so the
    // report shows `Run` from the instant a move is queued until the queue drains and the last burst finishes,
    // and `Idle` only when truly quiescent. Every non-Normal mode (Hold/Alarm/Check/Sleep) ignores `running`.
    let queued = blocks_free < firmware_core::planner::BLOCK_QUEUE_LEN as u8;
    let running = EXECUTOR_RUNNING.load(Ordering::Acquire) || queued;
    // A `$H` cycle in progress overrides the wire State to `Home` regardless of the latched control mode (which
    // sits in the boot-lock alarm or Normal while homing runs) — grbl reports `Home` for the cycle's duration
    // (research finding #1). Otherwise the State is the latched control mode + live Run/Idle derivation.
    snap.state = if HOMING_ACTIVE.load(Ordering::Acquire) {
      MachineState::Home
    } else {
      control_state().machine_state(running)
    };
    let mut s = Response::new();
    if ResponseWriter::status_report(&mut s, &snap).is_ok() {
      enqueue(s).await;
    }
    // Replay the reset-reason line (which dog fired) then the crash report after the first status too (a host may
    // poll `?` before `$I`). Consumed, so each is emitted at most once more total across the `$I`/`?` paths.
    #[cfg(feature = "capture-reset")]
    if let Some(line) = take_pending_reset_report() {
      enqueue(line).await;
    }
    for line in take_pending_crash_report() {
      enqueue(line).await;
    }
  }
}

/// The firmware-side floor for the auto-report cadence, milliseconds (Phase F). The `$481` setter and the
/// settings sanitizer already enforce grblHAL's `[100, 1000]` range, but this is a defense-in-depth clamp at the
/// task boundary: even if the live [`AUTO_REPORT_INTERVAL_MS`] mirror were somehow set to a tiny value, the
/// report task can never fire faster than this, so it can never starve the single `usb_tx` writer (a report
/// every few ms would crowd out `ok`s and break the host's character-counting flow control). 50 ms is half the
/// grblHAL floor — comfortably below any real DRO cadence yet a hard upper bound on report frequency.
const AUTO_REPORT_FLOOR_MS: u32 = 50;

/// The periodic auto-status-report task (Phase F, DOC-08 §5): when `$481` is non-zero and auto-reporting is not
/// suspended by `0x8C`, it raises [`STATUS_REQUEST`] every interval-ms so [`status_responder`] builds and emits
/// the SAME `<...>` report a `?` poll produces — there is exactly one status-report formatter and one USB
/// writer, so an auto-report and a `?`-triggered report are byte-identical and never interleave a partial line.
///
/// It signals `STATUS_REQUEST` rather than formatting the report itself, so this task adds NO new writer to the
/// USB endpoint and cannot race `status_responder` (the single owner of report formatting). When disabled (the
/// interval is 0 or auto-reporting is suspended) it parks on [`AUTO_REPORT_WAKE`] instead of spinning, so a
/// disabled auto-report costs nothing; a `$481=` enable or a `0x8C` resume wakes it at once. The interval is
/// re-read every iteration, so a `$481=` change takes effect on the next tick WITHOUT a reboot, and is floored at
/// [`AUTO_REPORT_FLOOR_MS`] so it can never starve the writer.
#[embassy_executor::task]
pub async fn auto_report_task() -> ! {
  loop {
    let interval = effective_auto_report_interval();
    match interval {
      // Disabled (interval 0 or suspended by `0x8C`): park until the enable state changes, then re-evaluate.
      None => AUTO_REPORT_WAKE.wait().await,
      // Enabled: race the interval tick against a wake (an enable-state change). On the tick, request a report;
      // on a wake, just loop to re-read the (possibly changed) interval. Reusing `STATUS_REQUEST` means the
      // report goes out through the one formatter / one writer, never a partial or interleaved line.
      Some(period) => match select(Timer::after(period), AUTO_REPORT_WAKE.wait()).await {
        Either::First(()) => STATUS_REQUEST.signal(()),
        Either::Second(()) => {}
      },
    }
  }
}

/// The effective auto-report period (Phase F): `None` when auto-reporting is disabled (a zero `$481` interval or
/// suspended by the `0x8C` toggle), else the configured interval floored at [`AUTO_REPORT_FLOOR_MS`] so the
/// report task can never be driven fast enough to starve the USB writer. Reads the live mirrors, so a `$481=`
/// change or a `0x8C` toggle is observed on the next call.
fn effective_auto_report_interval() -> Option<Duration> {
  if AUTO_REPORT_SUSPENDED.load(Ordering::Relaxed) {
    return None;
  }
  let interval = AUTO_REPORT_INTERVAL_MS.load(Ordering::Relaxed);
  if interval == 0 {
    None
  } else {
    Some(Duration::from_millis(interval.max(AUTO_REPORT_FLOOR_MS) as u64))
  }
}
