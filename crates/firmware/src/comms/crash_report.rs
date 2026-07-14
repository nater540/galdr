//! Post-mortem crash-report formatters (Signature-A/B lockup capture): pure `crate::crash::*` breadcrumb →
//! grbl `[MSG:CRASH ...]` `Response` line renderers, extracted verbatim from `comms.rs` (architecture-refactor
//! A1, step 2). They consume esp-hal-bound `crate::crash` types, so per the nemesis finding they stay in the
//! firmware crate (a `comms/` submodule) and must NOT move to firmware-core. No I/O — each returns a formatted
//! `Response` (or `None` if the text could not be wrapped); `maybe_emit_crash_report` in `comms.rs` owns the
//! emit/stash and calls these via `comms.rs`'s `pub(crate) use crash_report::*;` re-export, so those call sites
//! keep resolving these names unqualified exactly as before the split.

use firmware_core::protocol::ResponseWriter;

use super::Response;

/// Format a captured PANIC into a `[MSG:CRASH panic <file>:<line> core=<N>]` line for the boot dump. `core=1` is the
/// core-1 (motion / APP_CPU) panic — the stack-overflow prime suspect; `core=0` is core-0 (comms / PRO_CPU). When
/// the source file string could not be recovered (no location, or a stale pointer from a different flashed image —
/// see [`crate::crash::BUILD_ID`]), it falls back to `?:<line>`. Pure formatting; no I/O. Kept under
/// [`RESPONSE_CAPACITY`] (a worst-case file path is bounded by the recovered length cap in the decoder).
pub(crate) fn format_panic_report(panic: &crate::crash::PanicReport) -> Option<Response> {
  use core::fmt::Write as _;
  let mut inner: heapless::String<128> = heapless::String::new();
  match panic.file {
    Some(file) => {
      let _ = write!(inner, "panic {}:{} core={}", file, panic.line, panic.core);
    }
    None => {
      let _ = write!(inner, "panic ?:{} core={}", panic.line, panic.core);
    }
  }
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// Build the `[MSG:CRASH ...]` line from a decoded breadcrumb. Renders, in order: the WATCHDOG WITHHOLD CLASS when
/// present (`core1-motion-wedge` / `core0-comms-wedge` — the most decisive datum, naming WHY the dog was forced to
/// fire), the last executor stage (with axis for the per-axis RMT stages, e.g. `axis1:wait_begin`), the snapshot
/// "which side froze first" verdict, and the newest comms/motion beats. Returns `None` only if the text could not be
/// wrapped (never in practice — the line is far under [`RESPONSE_CAPACITY`]). Pure formatting; no I/O.
pub(crate) fn format_crash_report(breadcrumb: &crate::crash::Breadcrumb) -> Option<Response> {
  use core::fmt::Write as _;
  // Build the inner text (without the `[MSG:...]` envelope), then wrap it. Sized well under RESPONSE_CAPACITY.
  let mut inner: heapless::String<128> = heapless::String::new();
  let _ = write!(inner, "CRASH");
  // Withhold class first when the watchdog deliberately forced the reset — the single most useful datum (an executor
  // death that just stopped feeding leaves no withhold marker, so this is absent then).
  if let Some(reason) = crate::crash::withhold_label(breadcrumb.withhold) {
    let _ = write!(inner, " {}", reason);
  }
  // Stage: name plus axis for the per-axis RMT stages. `write!` into a fixed string cannot panic; ignore the
  // `Result` (a full buffer just truncates, which still yields a usable, if clipped, report).
  let stage = breadcrumb.last_stage;
  if crate::crash::stage_has_axis(stage) {
    let _ = write!(inner, " stage=axis{}:{}", crate::crash::stage_axis(stage), crate::crash::stage_label(stage));
  } else {
    let _ = write!(inner, " stage={}", crate::crash::stage_label(stage));
  }
  // The CORE-0 comms park-point: prefer the first NON-IDLE task slot (the likely stuck await) for the summary; if
  // every slot is idle-class, show the consumer's slot (the in-order pipeline owner). The full per-task breakdown
  // is on the separate `comms:` line below. This is the field that localizes the core-0 wedge.
  if let Some(label) = stuck_comms_stage_label(&breadcrumb.comms_stages) {
    let _ = write!(inner, " comms-stage={}", label);
  }
  // Which side stopped advancing first, from the snapshot ring (comms = core-0 side, motion = core-1 side). OMITTED
  // for the `core0-executor-stall` class: there the WHOLE core-0 executor died at once (taking the snapshot pusher with
  // it), so the ring shows the last HEALTHY comms/motion progress and the comms-vs-motion "froze-first" verdict is not
  // meaningful — it would read a contradictory `no-stall` next to `core0-executor-stall`. The absolute `beats` below
  // still convey the last-known counters.
  if !crate::crash::withhold_was_executor_stall(breadcrumb.withhold) {
    let verdict = crate::crash::froze_first(&breadcrumb.snapshots);
    let _ = write!(inner, " {}", verdict);
  }
  // Newest beats (index 0 of the newest-first ring): `comms` is the core-0 host-facing progress counter, `motion`
  // the core-1 executor beat — so the operator sees the absolute counters too.
  let newest = breadcrumb.snapshots[0];
  let _ = write!(inner, " beats comms={} motion={}", newest.core0_beat, newest.core1_beat);
  // The free-running watchdog heartbeat (Signature-B discriminator): a large value means `watchdog_feed` ran through
  // the wedge (B-1, alive-but-fooled — the dead-zone backstop or another withhold is what finally forced this reset);
  // a small/frozen value means the feed task itself died (B-2). Read alongside the withhold class above.
  let _ = write!(inner, " wdog={}", breadcrumb.watchdog_heartbeat);
  // Remind that the breadcrumb is watchdog-survival only (so a power-cycle would have lost it — useful context).
  let _ = write!(inner, " (RWDT-reset; not power-cycle)");
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// Format the captured RMT channel-hang hardware state into a second `[MSG:CRASH rmt<axis>: ...]` line. The DECISIVE
/// field is `end`: `end=1` means TX-END WAS asserted (the transmission finished but our wait missed the completion
/// — a driver/usage bug, fix how we wait); `end=0` means TX-END never fired (the transmission genuinely never
/// completed — a memory/encoding/start issue, chase that). `fsm` is the channel TX FSM state (bits 22:24 of
/// `ch_tx_status`): non-zero ⇒ the channel is still mid-transmission. `thr` = a pending half-block threshold
/// (refill) event; `err` = a latched transmission error. The raw register words (`ir`/`is`/`st`/`cf`) are included
/// so any other bit can be re-derived off-board. `nsym` is the hung burst's symbol count; `burst#` is which
/// transmission since boot wedged. Kept as its own line so it stays under [`RESPONSE_CAPACITY`].
pub(crate) fn format_rmt_hang_report(hang: &crate::crash::RmtHang) -> Option<Response> {
  use core::fmt::Write as _;
  // The TX FSM state is bits 22:24 of the ch_tx_status word; surface it decoded so the report reads at a glance.
  let fsm = (hang.tx_status >> 22) & 0x7;
  let mut inner: heapless::String<128> = heapless::String::new();
  let _ = write!(
    inner,
    "CRASH rmt{}: end={} thr={} err={} fsm={} nsym={} burst#={} ir={:#010x} is={:#010x} st={:#010x} cf={:#010x}",
    hang.axis,
    hang.tx_end as u8,
    hang.tx_thr as u8,
    hang.tx_err as u8,
    fsm,
    hang.nsym,
    hang.burst_seq,
    hang.int_raw,
    hang.int_st,
    hang.tx_status,
    hang.tx_conf0,
  );
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// Format the captured USB-TX-stall discriminator into a `[MSG:CRASH usbtx: ...]` line — the §11 drumbeat capture.
/// The DECISIVE field is `verdict`, classified by the pure, host-tested [`firmware_core::diag::UsbTxStall::verdict`]:
/// `host-not-reading` (the EP1 IN FIFO is full because the host stopped draining — host/skirnir side, peripheral
/// healthy), `lost-tx-wake` (room in the FIFO + the TX-empty event asserted yet the write future never woke — an
/// esp-hal-side lost USB TX-done wake, §11.4 H-A), `core1-wedged` (core 1 froze mid-block — the USB stall is
/// downstream of a core-1 wedge, §11.4 H-B), or `ambiguous`. The raw signals back the verdict and split the H-A
/// sub-flavor: `free` (host drained), `empty` (int_raw TX-empty event), `iena` (int_ena still armed ⇒ the ISR never
/// ran; both `empty`/`iena` clear ⇒ the ISR ran but the embassy re-poll was lost), `mov`/`exec` (core-1 health),
/// `rdepth` (RESPONSE backlog). (The former `wstg` write-vs-flush-stage field is retired: §18/§19 collapsed `usb_tx`
/// to a poll path with no separate write/flush await stage, so it was always `0`.) `rmt_to` is the run's
/// RMT-wait-timeout count: `n>=K && rmt_to=0` POSITIVELY excludes
/// the RMT theory for the drumbeat (§11.1) by evidence, not inference. `wnd` is the WINDOWED stall count (§13.8): a
/// high `wnd` with a low consecutive `n` means the link was ALTERNATING-degraded (recoveries kept resetting the K
/// counter) rather than purely stuck — a distinct Signature class. Its own line so it stays under [`RESPONSE_CAPACITY`].
pub(crate) fn format_usb_tx_stall_report(
  stall: &firmware_core::diag::UsbTxStall,
  rmt_wait_count: u32,
  response_len: u16,
  stall_window: u32,
) -> Option<Response> {
  use core::fmt::Write as _;
  let verdict = firmware_core::diag::usb_tx_verdict_label(stall.verdict());
  let mut inner: heapless::String<128> = heapless::String::new();
  // `rlen` is the stalled response's BYTE length (the §13.1 single-chunk-widening discriminator): `rlen<=64` means
  // a future write-stage recovery could safely recover it (whole-or-nothing single FIFO chunk), `rlen>64` means it
  // could truncate (write_async parks between 64 B chunks). `rdepth` stays the (clamped-nibble) channel occupancy.
  let _ = write!(
    inner,
    "CRASH usbtx: {} free={} empty={} iena={} mov={} exec={} rdepth={} rlen={} n={} rmt_to={} wnd={}",
    verdict,
    stall.data_free as u8,
    stall.serial_in_empty as u8,
    stall.int_ena_armed as u8,
    stall.motion_advancing as u8,
    stall.executor_running as u8,
    stall.response_depth,
    response_len,
    stall.timeout_count,
    rmt_wait_count,
    stall_window,
  );
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// The label of the LIKELY-STUCK core-0 comms await: the first NON-idle-class task slot (a task parked
/// mid-operation), else the consumer's slot (the in-order pipeline owner) even if idle, else `None` when no comms
/// stage was ever recorded. Used for the summary line's `comms-stage=` field; the per-task breakdown is on the
/// `comms:` detail line.
fn stuck_comms_stage_label(stages: &[u32; crate::crash::COMMS_TASK_COUNT]) -> Option<&'static str> {
  // First a non-idle slot — that is the task stuck mid-operation, the most informative.
  for packed in stages {
    if !crate::crash::comms_stage_is_idle(*packed)
      && let Some(label) = crate::crash::comms_stage_label(*packed)
    {
      return Some(label);
    }
  }
  // Otherwise the consumer's slot (index = CommsTask::Consumer = 2), the pipeline owner, even if idle-class.
  crate::crash::comms_stage_label(stages[crate::crash::CommsTask::Consumer as usize])
}

/// Format the per-task core-0 comms-stage slots into a `[MSG:CRASH comms: rx=.. line=.. con=.. tx=.. sta=..]` line,
/// so the boot dump shows EVERY task's parked await (the stuck one has a non-idle stage; healthy ones sit at an
/// idle-class wait). Kept as its own line so it stays under [`RESPONSE_CAPACITY`]. Returns `None` only if no comms
/// stage was recorded at all (every slot untagged), in which case there is nothing useful to print.
pub(crate) fn format_comms_stage_report(stages: &[u32; crate::crash::COMMS_TASK_COUNT]) -> Option<Response> {
  use core::fmt::Write as _;
  // Nothing tagged at all ⇒ skip (the summary already shows whatever single field it could).
  if stages.iter().all(|p| crate::crash::comms_stage_label(*p).is_none()) {
    return None;
  }
  let lbl = |task: crate::crash::CommsTask| crate::crash::comms_stage_label(stages[task as usize]).unwrap_or("-");
  let mut inner: heapless::String<128> = heapless::String::new();
  let _ = write!(
    inner,
    "CRASH comms: rx={} line={} con={} tx={} sta={}",
    lbl(crate::crash::CommsTask::UsbRx),
    lbl(crate::crash::CommsTask::LineAssembler),
    lbl(crate::crash::CommsTask::Consumer),
    lbl(crate::crash::CommsTask::UsbTx),
    lbl(crate::crash::CommsTask::Status),
  );
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}
