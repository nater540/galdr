//! Pure, host-tested diagnostics logic for the streaming-lockup capture (the `usb_tx` 2 s-drumbeat
//! investigation — see `docs/streaming-lockup-investigation.md` §11). This module owns ONLY the decision
//! logic that benefits from off-target unit tests: the bounded "K consecutive `usb_tx` write timeouts"
//! escape counter, and the pack / decode / classify of the USB-TX-stall discriminator word. The RTC_FAST
//! storage, the live register reads (`ep1_conf.serial_in_ep_data_free`, `int_raw.serial_in_empty`), and the
//! `software_reset()` stay in the `firmware` crate's `crash.rs` / `comms.rs` wiring, which calls into here.
//!
//! ## Why this lives in `firmware-core`
//! `firmware` is an esp-hal binary with NO host test target (Xtensa only). The branching that decides the
//! experimental VERDICT — "is the drumbeat a lost USB TX-done wake (H-A), a wedged core 1 (H-B), or simply a
//! host that stopped reading?" — must be host-runnable so it is verified by `cargo test`, not by flashing.
//! The trivial bit-packing is kept here alongside it so the encode and decode are tested as a round-trip pair.

/// The number of CONSECUTIVE `usb_tx` write/flush timeouts that force the breadcrumb-capturing software reset.
///
/// Each timeout is the 2 s [`USB_TX_TIMEOUT`](crate) firing because the USB TX-done event never arrived, so `K`
/// timeouts span ≈`K * 2 s`. `K = 3` ⇒ ≈6 s: comfortably past any legitimate ≈1.5 s worst-case single write, and
/// clearly INSIDE the 8 s RWDT window so the escape (which carries the discriminator) is GUARANTEED to be the
/// resetter rather than a bare RWDT reset that captures nothing. It also recovers the wedge faster than the
/// ≈16 s drumbeat observed on the bench (§11), and fixes the §11.6 watchdog-mask defect (the per-loop
/// `COMMS_PROGRESS` bump at the 2 s cadence was evading the 3 s comms-stall detector).
pub const USB_TX_STALL_ESCAPE_K: u16 = 3;

/// The bounded consecutive-timeout counter for the `usb_tx` write path. A COMPLETED write resets it to zero; a
/// timeout increments it. When it reaches [`USB_TX_STALL_ESCAPE_K`] the caller captures the discriminator and
/// resets the board. Saturating so a pathological run can never wrap the count back below the threshold.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsbTxStallCounter {
  consecutive_timeouts: u16,
}

impl UsbTxStallCounter {
  /// A fresh counter (zero consecutive timeouts).
  pub const fn new() -> Self {
    UsbTxStallCounter { consecutive_timeouts: 0 }
  }

  /// Record the outcome of one `usb_tx` write attempt. `timed_out == false` (a completed write) RESETS the run to
  /// zero — a single good write means the host is draining again, so the drumbeat is broken. `timed_out == true`
  /// increments (saturating). Returns `true` exactly when the count has REACHED [`USB_TX_STALL_ESCAPE_K`], i.e.
  /// the caller should capture the discriminator + reset NOW. It returns `true` only on the transition reaching K
  /// (and would keep returning true above K, but the caller resets on the first), so callers need not special-case.
  pub fn record(&mut self, timed_out: bool) -> bool {
    if timed_out {
      self.consecutive_timeouts = self.consecutive_timeouts.saturating_add(1);
    } else {
      self.consecutive_timeouts = 0;
    }
    self.consecutive_timeouts >= USB_TX_STALL_ESCAPE_K
  }

  /// The current consecutive-timeout count (for the discriminator's `timeout_count` field).
  pub fn count(&self) -> u16 {
    self.consecutive_timeouts
  }
}

/// The terminal outcome of writing one `usb_tx` response over the §18/§19 POLL-BASED path. With polling there is no
/// waker to lose (the lost-wake CLASS is eliminated at the source), so only two outcomes remain: the response's bytes
/// were all handed to the FIFO ([`Completed`](WriteOutcome::Completed)), or the FIFO stayed full past the bounded
/// poll deadline because the host is genuinely not draining ([`Stalled`](WriteOutcome::Stalled), which still feeds the
/// K-escape / `handle_usb_tx_wedge`). The old `CompletedLostWakeRecovered` variant and the `classify*` helpers are
/// GONE — they existed only to disambiguate a lost wake from a real stall on the await path, which no longer exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriteOutcome {
  /// Every byte of the response was accepted into the IN FIFO (each packet committed with `wr_done`). The host
  /// receives them as it reads; a non-draining host is caught on the FOLLOWING response's first poll.
  Completed,
  /// The IN FIFO stayed full for the whole [`USB_TX_TIMEOUT`](crate) poll deadline — the host is not reading (or the
  /// link dropped). A genuine stall: drop-and-continue + count toward the [`USB_TX_STALL_ESCAPE_K`] bounded escape.
  Stalled,
}

impl WriteOutcome {
  /// Whether this outcome counts as a STALL toward the bounded escape. Only [`Stalled`](WriteOutcome::Stalled) does;
  /// a [`Completed`](WriteOutcome::Completed) made progress. This is what feeds [`UsbTxStallCounter::record`].
  pub fn is_stall(self) -> bool {
    matches!(self, WriteOutcome::Stalled)
  }
}

/// The action the poll-based `usb_tx` write loop takes after ONE non-blocking `write_byte_nb`/`flush_tx_nb` attempt
/// (the pure decision behind the §19 loop, host-tested here so the sacred-path policy has a single source of truth).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PollAction {
  /// The nb op made progress (the byte went into the FIFO / the packet committed) — advance to the next byte/chunk.
  Advance,
  /// The nb op returned `WouldBlock` (FIFO full = host not draining yet) but the deadline has NOT passed — the loop
  /// must `yield_now().await` and re-poll. Re-reads the hardware each poll, so there is no waker to lose.
  Yield,
  /// The nb op returned `WouldBlock` AND the bounded poll deadline has passed — a genuine host-not-reading stall.
  Stall,
}

/// The pure per-poll decision for the poll-based `usb_tx` write loop (§19). `made_progress` is `nb_result.is_ok()`
/// (the esp-hal error type is `Infallible`, so an `Err` is always `WouldBlock`); `deadline_exceeded` is
/// `Instant::now() >= deadline`. Progress always wins (a byte that went out is never a stall even at the deadline);
/// otherwise a passed deadline is a stall and a live deadline yields.
pub fn usb_tx_poll_action(made_progress: bool, deadline_exceeded: bool) -> PollAction {
  if made_progress {
    PollAction::Advance
  } else if deadline_exceeded {
    PollAction::Stall
  } else {
    PollAction::Yield
  }
}

/// A pure TEST-ONLY model of one response's poll-write, used solely by this module's unit tests to exercise the
/// stall-vs-complete POLICY end-to-end without hardware. It is NOT the live loop's source of truth (finding #12): the
/// firmware's `write_response_polled` drives its own byte/commit loop and, after the §18/§19 poll rewrite, chooses
/// `Advance` DIRECTLY on progress — routing only the non-progress case through [`usb_tx_poll_action`] (the genuinely
/// shared policy). This reducer feeds a scripted sequence of the same observations so the tests can assert the
/// terminating outcome; it is `#[cfg(test)]` because nothing ships against it. `total_ops` is the number of ops that
/// must each make progress for the response to complete. Each [`step`](Self::step) returns `Some` once it terminates.
#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PollWriteProgress {
  ops_remaining: u32,
}

#[cfg(test)]
impl PollWriteProgress {
  /// A reducer for a response requiring `total_ops` successful ops. `0` completes on the first `step` (nothing to
  /// send — never happens on the wire, but defined for totality).
  fn new(total_ops: u32) -> Self {
    PollWriteProgress { ops_remaining: total_ops }
  }

  /// Feed one poll observation. Returns `Some(Completed)` once all ops have progressed, `Some(Stalled)` on a
  /// deadline-exceeded `WouldBlock`, or `None` while more polls are needed (a progress that is not the last op, or a
  /// yield). A yield does NOT consume an op — the same op is retried next poll.
  fn step(&mut self, made_progress: bool, deadline_exceeded: bool) -> Option<WriteOutcome> {
    if self.ops_remaining == 0 {
      return Some(WriteOutcome::Completed);
    }
    match usb_tx_poll_action(made_progress, deadline_exceeded) {
      PollAction::Advance => {
        self.ops_remaining -= 1;
        if self.ops_remaining == 0 { Some(WriteOutcome::Completed) } else { None }
      }
      PollAction::Yield => None,
      PollAction::Stall => Some(WriteOutcome::Stalled),
    }
  }
}

/// The firmware-only signals sampled at the K-th `usb_tx` timeout, used to classify WHY the USB TX path stalled
/// (the H-A vs H-B vs host-not-reading discrimination of §11.4 / §11.5, done post-mortem over CDC rather than via
/// the transport-blocked live RTT). All fields are read at the timeout instant in `comms.rs` and packed via
/// [`pack_usb_tx_stall`] into one RTC_FAST word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UsbTxStall {
  /// `USB_DEVICE.ep1_conf().serial_in_ep_data_free()` — the hardware "the host has accepted the previous IN
  /// packet, the EP1 FIFO has room for another" bit. `false` ⇒ the FIFO is full because the host is NOT draining.
  pub data_free: bool,
  /// `USB_DEVICE.int_raw().serial_in_empty()` — the raw TX-FIFO-empty event the async write future waits on. SET
  /// while the future never woke is the lost-wake smoking gun (the event fired but the ISR/waker missed it).
  pub serial_in_empty: bool,
  /// `USB_DEVICE.int_ena().serial_in_empty()` — whether the TX-empty interrupt is still ARMED. The async write
  /// future arms it; the core-0 ISR CLEARS it on fire (then wakes `WAKER_TX`). So `serial_in_empty == true` (event
  /// asserted) with `int_ena_armed == false` (the ISR already ran and disarmed) yet the future never woke is the
  /// sharpest lost-WAKER signature (ISR fired, waker lost); `int_ena_armed == true` with the event asserted means
  /// the ISR itself never ran (the interrupt was masked / core 0 never serviced it) — a different lost-INTERRUPT
  /// flavor. Both are H-A sub-cases distinguished here.
  pub int_ena_armed: bool,
  /// Whether the core-1 [`MOTION_LIVENESS`] beat ADVANCED across the stall window (core 1 still scheduling). `true`
  /// ⇒ core 1 is alive (favors H-A, an isolated USB-event loss); `false` ⇒ core 1 also froze (favors H-B).
  pub motion_advancing: bool,
  /// `EXECUTOR_RUNNING` at the timeout instant — a motion block was in flight. With `motion_advancing == false`
  /// this means core 1 is wedged mid-block (the H-B core-1-wedge signature), not merely idle.
  pub executor_running: bool,
  /// The `RESPONSE` channel occupancy at the timeout — non-zero confirms the writer is the bottleneck (responses
  /// are queued behind it), the head-of-line-blocking story that starves the status reporter.
  pub response_depth: u8,
  /// The consecutive-timeout count when captured (≥ [`USB_TX_STALL_ESCAPE_K`]). Packed into 6 bits (saturates at
  /// 63) — ample for a "reached K and kept timing out" diagnostic; the exact magnitude past K is not load-bearing.
  pub timeout_count: u16,
}

/// The byte length of the response whose write stalled at the K-escape — the load-bearing field for the single-chunk
/// widening fix (sound ONLY for `len <= 64`: one `write_async` chunk → all bytes are pushed before the future parks,
/// so dropping it cannot truncate). A 4-byte `ok` (Signature A) is `len=4`; a full `<...>` status is ~90 B. Kept a
/// SIBLING of [`UsbTxStall`] (carried in its own RTC_FAST word by `crash.rs`, like `rmt_wait_count`/`recovered_count`),
/// NOT a packed-word field, so the [`pack_usb_tx_stall`]/[`decode_usb_tx_stall`] round-trip stays a clean bijection.
/// `0` means no stall captured this run (or a zero-length response, which never occurs on the wire).
pub type UsbTxStallLen = u16;

/// The post-mortem VERDICT the boot dump renders from a decoded [`UsbTxStall`], answering the §11.5 question.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbTxVerdict {
  /// `data_free == false`: the EP1 IN FIFO is full because the HOST stopped reading (or the USB-CDC link dropped).
  /// The peripheral is healthy; the stall is host-side (skirnir / the OS USB stack), NOT a firmware bug.
  HostNotReading,
  /// `data_free == true` (host drained the FIFO) AND core 1 alive AND responses backed up (`response_depth > 0`),
  /// yet the write future never completed — a LOST USB TX-done wake (§11.4 H-A), an esp-hal/embassy-side
  /// missed-interrupt / waker-loss bug. The host IS reading; only the device→host completion/wake was dropped.
  /// THREE sub-fingerprints, all the same H-A class (the boot dump's raw `int_ena`/`empty` fields tell them apart):
  /// `int_ena_armed == true` ⇒ the ISR NEVER ran (interrupt still armed); `serial_in_empty == true` with
  /// `int_ena_armed == false` ⇒ the ISR fired (cleared the enable) but the event is latched and the future never
  /// completed (waker race); BOTH clear (the 2026-06-25 captured case) ⇒ the ISR ran (it cleared both bits) and
  /// called `WAKER_TX.wake()`, but the embassy re-poll was lost — `UsbSerialJtagWriteFuture::poll` returns Ready iff
  /// int_ena is clear, so the future WOULD have completed if re-polled; it simply never was.
  LostTxWake,
  /// Core 1 froze (`motion_advancing == false`) while a block was in flight (`executor_running`): the USB stall is
  /// downstream of a core-1 / cross-core wedge (§11.4 H-B), core 0 starved or a shared-lock hazard, NOT an
  /// isolated USB-event loss. The usb_tx symptom is a victim, not the root.
  Core1Wedged,
  /// A non-reproducing / contradictory snapshot: host has room, core 1 alive, but `response_depth == 0` — the
  /// writer is NOT actually parked behind a backed-up RESPONSE channel, so there is no real stall to classify.
  /// (A genuine lost-waker stall always shows `response_depth > 0`, so it is never silently filed here.) If this
  /// ever appears on a real wedge, widen the instrumentation (the build-#2 cross-core lock breadcrumb).
  Ambiguous,
}

impl UsbTxStall {
  /// Classify the stall into the experimental [`UsbTxVerdict`]. The order is load-bearing: a full FIFO
  /// (`!data_free`) is host-not-reading regardless of anything else (the host literally isn't consuming, so no
  /// firmware signal downstream matters); only with room in the FIFO do the lost-wake vs core-1-wedge tests apply.
  pub fn verdict(&self) -> UsbTxVerdict {
    if !self.data_free {
      // The host has not drained the previous IN packet — the FIFO is full because nothing is reading it.
      return UsbTxVerdict::HostNotReading;
    }
    if !self.motion_advancing && self.executor_running {
      // Room in the FIFO, but core 1 is frozen mid-block — the USB stall is downstream of a core-1 wedge (H-B).
      return UsbTxVerdict::Core1Wedged;
    }
    // Room in the FIFO (host drained), core 1 healthy, AND responses backed up (`response_depth > 0` — we are
    // genuinely parked in the write await behind a non-empty RESPONSE channel): the device→host completion path
    // failed despite the host reading — a LOST USB TX-done wake (H-A). This holds across ALL three peripheral
    // sub-states, which the raw `int_ena`/`empty` dump fields then tell apart (the verdict is the same H-A class):
    //   - `int_ena_armed`            ⇒ the ISR NEVER ran (interrupt still armed);
    //   - `serial_in_empty`          ⇒ the event is latched but the future never completed (waker race);
    //   - both clear (the captured case) ⇒ the ISR ran (it cleared both) and called `WAKER_TX.wake()`, but the
    //     embassy re-poll was lost — `UsbSerialJtagWriteFuture::poll` returns Ready iff int_ena is clear, so it
    //     WOULD have completed if re-polled. All three are the same lost-wake fault, just at different points.
    if self.data_free && self.response_depth > 0 {
      return UsbTxVerdict::LostTxWake;
    }
    // Only Ambiguous when NOT actually backed up (`response_depth == 0`): a non-reproducing / contradictory snapshot
    // (the writer is not parked behind a real backlog), so no signal points anywhere — widen the instrumentation.
    UsbTxVerdict::Ambiguous
  }
}

/// Tag in the high half of the packed USB-TX-stall word, so a cold-boot zero / garbage word never decodes as a
/// real stall capture (mirrors the `crash.rs` tag convention for the panic / RMT / withhold words).
pub const USB_TX_STALL_TAG: u32 = 0x5554_0000; // "UT".

/// Bit positions within the low half of the packed word. The four boolean signals occupy single bits; the
/// timeout count is saturated into a byte; the response depth into a nibble (the channel is depth-8, fits in 4 bits).
mod bits {
  // All packed fields MUST stay within the low 16 bits — the high half (`0xFFFF_0000`) is the tag, and any field
  // bleeding into bit 16 would corrupt the tag check on decode (a bug TDD caught: a byte-wide count at shift 9
  // reached bit 16). Layout: five flag bits (0..5), a RESERVED bit (5), a depth nibble (6..10), a 6-bit count
  // (10..16). The depth/count shifts MUST stay fixed even though bit 5 is now unused — a surviving cross-version
  // breadcrumb is decoded on its face, so renumbering them would corrupt a prior run's depth/count on the next boot.
  pub const DATA_FREE: u32 = 1 << 0;
  pub const SERIAL_IN_EMPTY: u32 = 1 << 1;
  pub const MOTION_ADVANCING: u32 = 1 << 2;
  pub const EXECUTOR_RUNNING: u32 = 1 << 3;
  pub const INT_ENA_ARMED: u32 = 1 << 4;
  // Bit 5 is RESERVED — formerly `write_stage_stall`, retired once §18/§19 collapsed `usb_tx` to a poll path with no
  // write-vs-flush await stage (the flag was always `false`). The bit is left UNUSED (the packer never sets it and the
  // decoder never reads it) rather than reclaimed: renumbering the depth/count shifts below would misdecode a
  // cross-version breadcrumb from a prior image, so the numeric layout MUST stay fixed. No const is defined for it.
  /// Response depth in bits 6..10 (a nibble; the depth-8 channel → 0..=8 fits, clamp at 15).
  pub const RESPONSE_DEPTH_SHIFT: u32 = 6;
  pub const RESPONSE_DEPTH_MASK: u32 = 0xF << RESPONSE_DEPTH_SHIFT;
  /// Timeout count in bits 10..16 (6 bits, saturated to [`TIMEOUT_COUNT_CLAMP`]). 6 bits is ample for a ≥K
  /// diagnostic — the exact magnitude past the escape is not load-bearing, only "it reached K and kept timing out"
  /// (`K = 3`, and 63 ≫ 3). Bit 5 below it is the reserved former `write_stage_stall` slot; the shift stays at 10.
  pub const TIMEOUT_COUNT_SHIFT: u32 = 10;
  pub const TIMEOUT_COUNT_MASK: u32 = 0x3F << TIMEOUT_COUNT_SHIFT;
  /// The saturation ceiling for the 6-bit packed timeout count.
  pub const TIMEOUT_COUNT_CLAMP: u16 = 0x3F;
}

/// Pack a [`UsbTxStall`] capture into one tagged `u32` for the RTC_FAST breadcrumb. The four booleans become
/// single bits, the response depth a nibble (clamped to 15), the timeout count a byte (saturated to 255). The
/// high half is the [`USB_TX_STALL_TAG`] so the decode side can reject an untagged / garbage word.
pub fn pack_usb_tx_stall(stall: &UsbTxStall) -> u32 {
  let mut word = USB_TX_STALL_TAG;
  if stall.data_free {
    word |= bits::DATA_FREE;
  }
  if stall.serial_in_empty {
    word |= bits::SERIAL_IN_EMPTY;
  }
  if stall.motion_advancing {
    word |= bits::MOTION_ADVANCING;
  }
  if stall.executor_running {
    word |= bits::EXECUTOR_RUNNING;
  }
  if stall.int_ena_armed {
    word |= bits::INT_ENA_ARMED;
  }
  // Bit 5 is intentionally left clear — it is the retired `write_stage_stall` slot, reserved so the depth/count
  // shifts stay stable across versions (see the `bits` module).
  word |= (stall.response_depth.min(15) as u32) << bits::RESPONSE_DEPTH_SHIFT;
  word |= (stall.timeout_count.min(bits::TIMEOUT_COUNT_CLAMP) as u32) << bits::TIMEOUT_COUNT_SHIFT;
  word
}

/// Decode a packed USB-TX-stall word back into a [`UsbTxStall`], or `None` when the word is untagged (cold boot /
/// no stall captured this run). The response depth and timeout count come back clamped to the packed widths — the
/// depth and count are diagnostic, so the clamp is acceptable (a depth-8 channel and a ≥K count both fit).
pub fn decode_usb_tx_stall(word: u32) -> Option<UsbTxStall> {
  if word & 0xFFFF_0000 != USB_TX_STALL_TAG {
    return None;
  }
  Some(UsbTxStall {
    data_free: word & bits::DATA_FREE != 0,
    serial_in_empty: word & bits::SERIAL_IN_EMPTY != 0,
    motion_advancing: word & bits::MOTION_ADVANCING != 0,
    executor_running: word & bits::EXECUTOR_RUNNING != 0,
    int_ena_armed: word & bits::INT_ENA_ARMED != 0,
    // Bit 5 (retired `write_stage_stall`) is intentionally not read — see the `bits` module reserved-slot note.
    response_depth: ((word & bits::RESPONSE_DEPTH_MASK) >> bits::RESPONSE_DEPTH_SHIFT) as u8,
    timeout_count: ((word & bits::TIMEOUT_COUNT_MASK) >> bits::TIMEOUT_COUNT_SHIFT) as u16,
  })
}

/// A short, stable verdict label for the boot `[MSG:CRASH usbtx: ...]` line.
pub fn usb_tx_verdict_label(verdict: UsbTxVerdict) -> &'static str {
  match verdict {
    UsbTxVerdict::HostNotReading => "host-not-reading",
    UsbTxVerdict::LostTxWake => "lost-tx-wake",
    UsbTxVerdict::Core1Wedged => "core1-wedged",
    UsbTxVerdict::Ambiguous => "ambiguous",
  }
}

/// How many consecutive `watchdog_feed` intervals `usb_tx` may produce NO completed write WHILE responses are queued
/// before the dead-zone backstop withholds the RWDT feed (Signature-B instrumentation). At the 500 ms feed interval
/// this is `16 * 500 ms = 8 s` — the same order as the RWDT itself, so a board genuinely emitting nothing for ≥8 s
/// while lines are backed up is wedged by definition. Chosen well ABOVE the normal worst case (a single legitimate
/// usb_tx write completes in sub-ms, and even the K-escape's own ≈6 s drumbeat self-resets before this), so this can
/// only fire on a true silent lock — NOT on healthy streaming, back-pressure, or the normal recovery path.
pub const DEAD_ZONE_STALL_TICKS: u32 = 16;

/// The dead-zone backstop decision (Signature B): should `watchdog_feed` WITHHOLD the RWDT feed because the board is
/// silently locked? This closes the gap where the existing withholds cannot fire — the comms-stall withhold needs
/// `host_active` (recent RX) and the core-1 withhold needs `EXECUTOR_RUNNING`, so a board with the host gone quiet
/// AND the executor idle (`exec=0`) is fed forever. This backstop is INDEPENDENT of both: it fires purely on "there
/// are responses QUEUED to send (`response_depth > 0`) yet `usb_tx` has COMPLETED no write for
/// [`DEAD_ZONE_STALL_TICKS`] intervals" — a board sitting on a non-empty `RESPONSE` backlog emitting nothing for ~8 s
/// is wedged regardless of RX/executor state. Requiring `response_depth > 0` is the false-trip guard: a truly idle
/// board (nothing to send) legitimately completes no writes and must NOT be reset.
///
/// `tx_complete_frozen_ticks` is how many consecutive feed intervals the `usb_tx`-completed-write counter has not
/// advanced; the caller tracks it (resets to 0 whenever a write completes). Returns `true` to withhold (force a
/// reset so the silent lock leaves a breadcrumb), `false` to keep feeding.
pub fn dead_zone_withhold(response_depth: usize, tx_complete_frozen_ticks: u32) -> bool {
  response_depth > 0 && tx_complete_frozen_ticks >= DEAD_ZONE_STALL_TICKS
}

/// The number of consecutive feed intervals the core-1 motion beat may stay frozen WHILE a block is in flight before
/// [`watchdog_decision`] declares a core-1 wedge. A pure mirror of the firmware's `CORE1_STALL_TICKS` so the decision
/// is host-tested against the SAME threshold the survivable-watchdog ISR uses (the firmware passes its own constant
/// in [`WatchdogInputs::core1_stall_ticks`], but this is the canonical default and the value the tests pin). At the
/// ISR's 250 ms cadence `16 * 250 ms = 4 s` — the same ~4 s order as the async feeder's 500 ms × 8.
pub const CORE1_STALL_TICKS: u32 = 16;

/// The number of consecutive feed intervals [`COMMS_PROGRESS`] may stay frozen WHILE the host is active before
/// [`watchdog_decision`] declares a core-0 comms wedge. A pure mirror of the firmware's `COMMS_STALL_TICKS`; at the
/// ISR's 250 ms cadence `12 * 250 ms = 3 s` — the same ~3 s order as the async feeder's 500 ms × 6.
pub const COMMS_STALL_TICKS: u32 = 12;

/// The number of consecutive feed intervals the core-0 EXECUTOR-LIVENESS beat may stay frozen before
/// [`watchdog_decision`] declares a full core-0 async-executor stall. This is the §17.15 root-cause FIX detector: an
/// UNGATED beat that a dedicated core-0 async task bumps every interval, so it advances on a healthy board WHETHER OR
/// NOT there is host traffic, motion, or queued responses — unlike the three work-driven detectors, all of which are
/// gated off in exactly the executor-stall wedge (host aged out + motion idle + RESPONSE drained). At the ISR's 250 ms
/// cadence `16 * 250 ms = 4 s` — comfortably above any legitimate core-0 quiesce during streaming, well below the
/// point of no return, and matching [`CORE1_STALL_TICKS`].
pub const EXECUTOR_STALL_TICKS: u32 = 16;

/// Which wedge class made [`watchdog_decision`] withhold the watchdog feed. A PURE mirror of the firmware's
/// `crash::WithholdReason`, kept here so the survivable-watchdog ISR's decision is fully host-tested; `crash.rs` maps
/// this to its existing on-wire `WithholdReason`. The precedence when several conditions hold at once is
/// `Core1Motion > Core0ExecutorStall > Core0Comms > DeadZone` — the most-specific first (the core-1 stage marker pins
/// an exact RMT channel), then the definitive core-0 executor-dead signal, then its finer-grained sub-cases.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WithholdKind {
  /// The core-1 motion beat froze while a block was in flight — a core-1 / RMT wedge.
  Core1Motion,
  /// The core-0 async EXECUTOR itself stalled: the ungated executor-liveness beat froze for [`EXECUTOR_STALL_TICKS`]
  /// while the hardware ISR kept firing. The §17.15 root-cause catch — the full-executor-stall wedge the three
  /// work-driven detectors below all structurally miss (they are gated by host/motion/response state, all of which
  /// evaluate quiescent in exactly this wedge). Subsumes `Core0Comms` and `DeadZone` when the whole executor is dead.
  Core0ExecutorStall,
  /// The host was driving the board but the core-0 comms path stopped making forward progress.
  Core0Comms,
  /// The dead-zone backstop: responses queued yet `usb_tx` completed no write for [`DEAD_ZONE_STALL_TICKS`] —
  /// independent of host-active / executor state (the Signature-B silent lock the other two structurally miss).
  DeadZone,
}

/// The inputs the survivable-watchdog ISR samples each feed interval, fed to the pure [`watchdog_decision`]. Carries
/// the frozen-tick counters the ISR maintains (the same bookkeeping the async `watchdog_feed` does today), the live
/// `RESPONSE` backlog + executor / host state, and the stall thresholds — so the wedge conditions are decided HERE
/// (host-tested) and the ISR is a thin shell that feeds or withholds both dogs on the result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogInputs {
  /// Consecutive feed intervals the core-1 [`MOTION_LIVENESS`] beat has stayed frozen WHILE a block is in flight.
  /// The ISR resets this to 0 whenever the beat advances OR no block is in flight (an idle executor is not a wedge).
  pub core1_frozen_ticks: u32,
  /// Consecutive feed intervals [`COMMS_PROGRESS`] has stayed frozen WHILE the host is active. The ISR resets this to
  /// 0 whenever the counter advances OR the host is not active (a quiescent board legitimately makes no progress).
  pub comms_frozen_ticks: u32,
  /// Consecutive feed intervals `usb_tx` has completed NO write (the [`USB_TX_COMPLETED`] beat frozen). The ISR
  /// resets this to 0 on any completed/recovered write. Ungated by host/executor state — that is the dead zone.
  pub tx_complete_frozen_ticks: u32,
  /// Consecutive feed intervals the core-0 EXECUTOR-LIVENESS beat has stayed frozen. The ISR resets this to 0 whenever
  /// the beat advances; it is UNGATED (no host/motion/response condition) — that is the whole point, since the beat is
  /// bumped by a dedicated core-0 async task that runs on a healthy board regardless of work, so a freeze means the
  /// executor itself stalled. The ISR only STARTS counting once the beat has advanced at least once (a boot guard so
  /// the pre-first-bump zero does not accrue a false stall). `0` in builds without the detector wired.
  pub executor_alive_frozen_ticks: u32,
  /// The core-0 executor-stall threshold to apply (defaults to [`EXECUTOR_STALL_TICKS`]). `0` disables the detector
  /// (a threshold of 0 would false-trip immediately, so the firmware passes 0 only in builds that do not wire it).
  pub executor_stall_ticks: u32,
  /// The current `RESPONSE` channel occupancy. The dead-zone backstop fires ONLY when this is `> 0` (responses are
  /// queued to send), the false-trip guard against resetting a truly idle board that legitimately sends nothing.
  pub response_depth: usize,
  /// Whether a motion block is in flight (`EXECUTOR_RUNNING`). The caller uses it to gate the core-1 freeze count;
  /// it is carried for completeness / future use even though the gated `core1_frozen_ticks` already encodes it.
  pub block_in_flight: bool,
  /// Whether the host is actively driving the board (recent RX). The caller uses it to gate the comms freeze count;
  /// carried for completeness even though the gated `comms_frozen_ticks` already encodes it.
  pub host_active: bool,
  /// The core-1 stall threshold to apply (the firmware passes its own constant; defaults to [`CORE1_STALL_TICKS`]).
  pub core1_stall_ticks: u32,
  /// The core-0 comms stall threshold to apply (defaults to [`COMMS_STALL_TICKS`]).
  pub comms_stall_ticks: u32,
}

/// The survivable-watchdog ISR's decision for one feed interval: whether to feed each dog and, when withholding, why.
/// When `withhold_reason` is `Some`, BOTH `feed_rwdt` and `feed_swd` are `false` (a wedge withholds both dogs so the
/// next reset — RWDT or SuperWDT — leaves a breadcrumb); when `None`, both are `true` (healthy → feed both).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogDecision {
  /// Feed the RTC watchdog this interval. `false` ⇒ withhold (let it run out toward a reset).
  pub feed_rwdt: bool,
  /// Feed the SuperWDT this interval. `false` ⇒ withhold. Always equal to [`feed_rwdt`](Self::feed_rwdt) — the two
  /// dogs are fed/withheld together so a withhold cannot be masked by one dog still being fed.
  pub feed_swd: bool,
  /// The wedge class that forced a withhold, or `None` when feeding (healthy / idle / host-absent).
  pub withhold_reason: Option<WithholdKind>,
}

/// The pure survivable-watchdog decision for one feed interval (the core the ISR calls so the ISR is a thin shell).
/// Any of: a core-1 motion wedge (`core1_frozen_ticks >= core1_stall_ticks`), a full core-0 executor stall
/// (`executor_alive_frozen_ticks >= executor_stall_ticks`, the §17.15 root-cause catch, disabled when
/// `executor_stall_ticks == 0`), a core-0 comms wedge (`comms_frozen_ticks >= comms_stall_ticks`), or the dead-zone
/// backstop ([`dead_zone_withhold`] on `response_depth` + `tx_complete_frozen_ticks`) ⇒ WITHHOLD BOTH dogs, with
/// precedence `Core1Motion > Core0ExecutorStall > Core0Comms > DeadZone`; otherwise FEED BOTH. The frozen-tick counters
/// are assumed already gated by the caller (the ISR zeroes `core1_frozen_ticks` when no block is in flight,
/// `comms_frozen_ticks` when the host is inactive, and only accrues `executor_alive_frozen_ticks` once the beat has
/// advanced at least once) — so this function is a pure threshold comparison + precedence.
pub fn watchdog_decision(inputs: WatchdogInputs) -> WatchdogDecision {
  let core1_wedged = inputs.core1_frozen_ticks >= inputs.core1_stall_ticks;
  // The executor-stall detector is DISABLED when its threshold is 0 (a build that does not wire the beat), so a
  // permanently-zero frozen count can never trip it. Otherwise a frozen beat past the threshold is a full core-0 stall.
  let executor_stalled = inputs.executor_stall_ticks > 0 && inputs.executor_alive_frozen_ticks >= inputs.executor_stall_ticks;
  let comms_wedged = inputs.comms_frozen_ticks >= inputs.comms_stall_ticks;
  let dead_zone = dead_zone_withhold(inputs.response_depth, inputs.tx_complete_frozen_ticks);
  let withhold_reason = if core1_wedged {
    Some(WithholdKind::Core1Motion)
  } else if executor_stalled {
    Some(WithholdKind::Core0ExecutorStall)
  } else if comms_wedged {
    Some(WithholdKind::Core0Comms)
  } else if dead_zone {
    Some(WithholdKind::DeadZone)
  } else {
    None
  };
  // A withhold starves BOTH dogs (so whichever fires first leaves the breadcrumb); a clean interval feeds both.
  let feed = withhold_reason.is_none();
  WatchdogDecision { feed_rwdt: feed, feed_swd: feed, withhold_reason }
}

/// The result of one [`ExecutorFreezeTracker::observe`] sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExecutorFreezeSample {
  /// Whether THIS sample was frozen: the beat did not advance AND the boot guard has passed (`alive != 0`).
  pub frozen: bool,
  /// Consecutive frozen samples INCLUDING this one (`0` when this sample advanced). Feed this straight into
  /// [`WatchdogInputs::executor_alive_frozen_ticks`] — it is exactly the counter both ISRs maintain by hand today.
  pub frozen_ticks: u32,
  /// Whether `frozen_ticks` has reached the stall threshold this sample — the executor-stall declaration (the edge
  /// AND every sample thereafter while it stays frozen). Equals [`watchdog_decision`]'s `executor_stalled` test, so
  /// the caller applies its OWN record-once latch (record the breadcrumb, or withhold + reset + GPIO).
  pub crossed: bool,
}

/// The pure, host-testable core-0 EXECUTOR-LIVENESS freeze reducer shared (in intent) by the two TIMG1 ISRs — the
/// production detector-only `stall_detector` and the `capture-reset` `survivable_watchdog` — both of which hand-roll
/// the SAME seed → boot-guard → frozen-tick → threshold-cross state machine over the ungated `EXECUTOR_ALIVE` beat.
/// Extracting it here gives that machine host-test coverage and ONE authoritative definition; each ISR still supplies
/// its own ACTION on a crossing. It is NOT yet wired into the ISRs (bench-gated on the unverified capture-reset
/// survivable path, `docs/homing-bench-checklist.md` §10) — it is the verified drop-in target for that later dedup, so
/// it mirrors the ISRs' behavior EXACTLY (including the benign boot quirk noted below) rather than "improving" it.
///
/// State machine (identical to both ISRs today):
/// - **Seed:** the first `observe` seeds `last` from the sample, so the first delta reads as "no change" rather than a
///   spurious move from the `u32::MAX` sentinel. (Consequence: the very first sample with `alive != 0` counts as one
///   frozen tick — benign, since a stall needs [`EXECUTOR_STALL_TICKS`] = 16 consecutive and a healthy beat advances by
///   the next sample, resetting the count. This mirrors the ISRs' `seed_swap` + `alive == last` on the first fire.)
/// - **Boot guard:** a sample is frozen ONLY once the beat has advanced past its initial `0` (`alive != 0`), so the
///   pre-first-bump zero — before the core-0 heartbeat task is even scheduled — cannot accrue a false stall at boot.
/// - **Count:** consecutive frozen samples accumulate (saturating, so a long genuine freeze never wraps below the
///   threshold); ANY advance resets the count to `0`.
/// - **Cross:** `frozen_ticks >= stall_ticks` (with `stall_ticks > 0`) declares the stall — the caller latches it.
#[derive(Clone, Copy, Debug)]
pub struct ExecutorFreezeTracker {
  /// Last-sampled beat; `u32::MAX` = unseeded (mirrors the ISRs' `LAST_EXECUTOR_ALIVE` sentinel).
  last: u32,
  /// Consecutive frozen samples so far (mirrors the ISRs' `EXECUTOR_ALIVE_FROZEN`).
  frozen_ticks: u32,
}

impl ExecutorFreezeTracker {
  /// A fresh, unseeded tracker: `last == u32::MAX`, zero frozen ticks — matching the ISR statics' initial values.
  pub const fn new() -> Self {
    Self { last: u32::MAX, frozen_ticks: 0 }
  }

  /// Observe one `alive` sample against `stall_ticks` and advance the machine. `stall_ticks == 0` disables the
  /// crossing (a build that does not wire the beat), matching [`watchdog_decision`]'s `executor_stall_ticks == 0`
  /// guard — a permanently-frozen count then never reports `crossed`.
  pub fn observe(&mut self, alive: u32, stall_ticks: u32) -> ExecutorFreezeSample {
    // Seed on the first sample so the first delta is "no change" rather than a spurious move from the sentinel.
    let last = if self.last == u32::MAX { alive } else { self.last };
    self.last = alive;
    // Boot guard: only a beat that has advanced past 0 can be "frozen"; the pre-first-bump zero never accrues.
    let frozen = alive != 0 && alive == last;
    self.frozen_ticks = if frozen { self.frozen_ticks.saturating_add(1) } else { 0 };
    let crossed = stall_ticks > 0 && self.frozen_ticks >= stall_ticks;
    ExecutorFreezeSample { frozen, frozen_ticks: self.frozen_ticks, crossed }
  }
}

impl Default for ExecutorFreezeTracker {
  fn default() -> Self {
    Self::new()
  }
}

/// The length of the sliding window (in feed intervals) the [`WindowedStallCounter`] ages over. A `usb_tx` write
/// timeout seen now stays counted for this many subsequent intervals before it ages out, so the window measures
/// "how degraded was the link over the recent past" rather than the instantaneous state. 16 intervals at the ISR's
/// 250 ms cadence ≈ 4 s of history — long enough to span the §13.8 ALTERNATING recovered/stall pattern (which keeps
/// resetting the consecutive K counter yet still represents a locking link) but short enough to age back to 0 once
/// the link genuinely recovers.
pub const STALL_WINDOW_LEN: u16 = 16;

/// A WINDOWED `usb_tx`-stall counter (§13.8): the number of write timeouts within a true sliding window of the last
/// [`STALL_WINDOW_LEN`] feed intervals. It distinguishes a PURE consecutive stall run (Signature A — what the bounded
/// [`UsbTxStallCounter`] K-escape catches) from an ALTERNATING recovered/stall pattern that resets the consecutive K
/// counter on every recovery yet still represents a degraded / intermittently-locking link: an alternating pattern
/// keeps this windowed count near half-full even though the consecutive count never reaches K.
///
/// ## Model
/// The window is an exact ring of the last [`STALL_WINDOW_LEN`] samples, packed one-bit-per-interval into a `u16`
/// (bit set ⇒ that interval timed out). Each [`record`](Self::record) shifts the ring left by one and ORs in the new
/// sample's bit; [`count`](Self::count) is the population count of the live window. This is an EXACT trailing-N
/// window (not a lossy linear age-out), so a 1:1 alternating run reads ≈`N/2`, a pure stall run reads `N`, and a long
/// quiet run reads `0` — the three regimes are cleanly separable. The whole state is one `u16` (the ring) plus a
/// cached popcount; no per-interval history buffer, so it is cheap enough to call from the watchdog ISR.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WindowedStallCounter {
  /// The trailing-[`STALL_WINDOW_LEN`] sample ring, one bit per interval (bit 0 = most recent). Bits at or above
  /// `STALL_WINDOW_LEN` are masked off on every shift so they never contribute to the count.
  window: u16,
}

impl WindowedStallCounter {
  /// The mask of live window bits: the low [`STALL_WINDOW_LEN`] bits. Anything shifted into a higher bit has aged out
  /// of the trailing window and is cleared, so it cannot inflate the count.
  const WINDOW_MASK: u16 = if STALL_WINDOW_LEN >= 16 { u16::MAX } else { (1u16 << STALL_WINDOW_LEN) - 1 };

  /// A fresh counter (an empty window).
  pub const fn new() -> Self {
    WindowedStallCounter { window: 0 }
  }

  /// Advance the window by one feed interval: shift the sample ring left by one (the oldest sample ages out at the
  /// trailing edge), mask to the window width, and OR in this interval's bit (`1` ⇒ `timed_out`). Returns the updated
  /// [`count`](Self::count) for convenience.
  pub fn record(&mut self, timed_out: bool) -> u16 {
    self.window = ((self.window << 1) & Self::WINDOW_MASK) | (timed_out as u16);
    self.count()
  }

  /// The number of `usb_tx` timeouts currently within the sliding window (`0..=`[`STALL_WINDOW_LEN`]) — the popcount
  /// of the live ring. A high value means the link has been timing out frequently over the recent past, whether the
  /// timeouts were CONSECUTIVE (Signature A) or ALTERNATING with recoveries (§13.8); only a long QUIET run reads 0.
  pub fn count(&self) -> u16 {
    self.window.count_ones() as u16
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A neutral baseline capture the verdict/round-trip tests mutate one field at a time, so adding a future signal
  /// never forces touching every literal. Defaults to the "healthy host, room in FIFO, core 1 alive" shape.
  fn base() -> UsbTxStall {
    UsbTxStall {
      data_free: true,
      serial_in_empty: false,
      motion_advancing: true,
      executor_running: false,
      int_ena_armed: false,
      response_depth: 8,
      timeout_count: 3,
    }
  }

  #[test]
  fn counter_increments_on_timeout_and_fires_at_k() {
    let mut c = UsbTxStallCounter::new();
    assert_eq!(c.count(), 0);
    // K = 3: the first two timeouts must NOT fire, the third must.
    assert!(!c.record(true), "1st timeout should not reach K");
    assert_eq!(c.count(), 1);
    assert!(!c.record(true), "2nd timeout should not reach K");
    assert_eq!(c.count(), 2);
    assert!(c.record(true), "3rd consecutive timeout must reach K=3");
    assert_eq!(c.count(), 3);
  }

  #[test]
  fn completed_write_resets_the_run() {
    let mut c = UsbTxStallCounter::new();
    assert!(!c.record(true));
    assert!(!c.record(true));
    // A single completed write breaks the drumbeat — the run resets to zero.
    assert!(!c.record(false));
    assert_eq!(c.count(), 0);
    // ...and it now takes a fresh K timeouts to fire again, never carrying the earlier partial run.
    assert!(!c.record(true));
    assert!(!c.record(true));
    assert!(c.record(true));
  }

  #[test]
  fn dead_zone_withhold_fires_only_with_backlog_and_long_freeze() {
    // The false-trip guard: an IDLE board (nothing queued) never withholds, no matter how long usb_tx has been
    // quiet — it legitimately completes no writes.
    assert!(!dead_zone_withhold(0, DEAD_ZONE_STALL_TICKS), "idle board (depth 0) must never withhold");
    assert!(!dead_zone_withhold(0, DEAD_ZONE_STALL_TICKS + 100), "still no withhold with depth 0");
    // With responses QUEUED but the freeze not yet long enough, keep feeding (healthy/back-pressured streaming).
    assert!(!dead_zone_withhold(8, DEAD_ZONE_STALL_TICKS - 1), "short freeze with backlog still feeds");
    assert!(!dead_zone_withhold(1, 0), "a single queued response with a fresh write does not withhold");
    // The genuine silent lock: responses queued AND no completed write for the full window → withhold (force reset).
    assert!(dead_zone_withhold(1, DEAD_ZONE_STALL_TICKS), "backlog + full freeze must withhold");
    assert!(dead_zone_withhold(8, DEAD_ZONE_STALL_TICKS + 5), "and stays withholding past the threshold");
  }

  #[test]
  fn write_outcome_is_stall_only_for_stalled() {
    // The collapsed outcome: only `Stalled` counts toward the escape; `Completed` made progress.
    assert!(WriteOutcome::Stalled.is_stall());
    assert!(!WriteOutcome::Completed.is_stall());
  }

  #[test]
  fn poll_action_advances_on_progress_even_past_the_deadline() {
    // Progress ALWAYS wins: a byte that went into the FIFO is never a stall, even if the deadline has also passed.
    assert_eq!(usb_tx_poll_action(true, false), PollAction::Advance);
    assert_eq!(usb_tx_poll_action(true, true), PollAction::Advance);
  }

  #[test]
  fn poll_action_yields_before_the_deadline_and_stalls_after() {
    // WouldBlock (no progress): yield while the deadline is live, stall once it passes.
    assert_eq!(usb_tx_poll_action(false, false), PollAction::Yield);
    assert_eq!(usb_tx_poll_action(false, true), PollAction::Stall);
  }

  #[test]
  fn poll_write_progress_completes_when_every_op_progresses() {
    // A response needing 3 ops (bytes + commits): three progressing polls complete it, and not before.
    let mut p = PollWriteProgress::new(3);
    assert_eq!(p.step(true, false), None, "op 1 done, more to go");
    assert_eq!(p.step(true, false), None, "op 2 done, more to go");
    assert_eq!(p.step(true, false), Some(WriteOutcome::Completed), "op 3 completes the response");
  }

  #[test]
  fn poll_write_progress_yields_do_not_advance() {
    // A yield (WouldBlock before the deadline) does NOT consume an op — the same op is retried until it progresses.
    let mut p = PollWriteProgress::new(2);
    assert_eq!(p.step(false, false), None, "yield: no progress, deadline live");
    assert_eq!(p.step(false, false), None, "still yielding");
    assert_eq!(p.step(true, false), None, "op 1 finally progresses");
    assert_eq!(p.step(true, false), Some(WriteOutcome::Completed), "op 2 completes");
  }

  #[test]
  fn poll_write_progress_stalls_on_a_deadline_would_block() {
    // Some progress, then the FIFO stays full past the deadline → Stalled, regardless of ops remaining.
    let mut p = PollWriteProgress::new(5);
    assert_eq!(p.step(true, false), None);
    assert_eq!(p.step(false, true), Some(WriteOutcome::Stalled), "deadline WouldBlock is a genuine stall");
  }

  #[test]
  fn poll_write_progress_zero_ops_completes_immediately() {
    // Totality: an empty response (never on the wire) completes on the first step without any progress.
    let mut p = PollWriteProgress::new(0);
    assert_eq!(p.step(false, false), Some(WriteOutcome::Completed));
  }

  #[test]
  fn stalled_outcome_feeds_the_k_escape_and_completed_resets_it() {
    // Wiring contract: `Stalled.is_stall()` accrues toward the K-escape; a `Completed` resets the run — so only a run
    // of GENUINE (host-not-reading) stalls trips the escape, exactly K of them.
    let mut c = UsbTxStallCounter::new();
    assert!(!c.record(WriteOutcome::Stalled.is_stall())); // genuine stall #1
    assert!(!c.record(WriteOutcome::Stalled.is_stall())); // genuine stall #2
    assert!(!c.record(WriteOutcome::Completed.is_stall())); // a completion breaks the run
    assert_eq!(c.count(), 0, "a completed write resets the run");
    assert!(!c.record(WriteOutcome::Stalled.is_stall()));
    assert!(!c.record(WriteOutcome::Stalled.is_stall()));
    assert!(c.record(WriteOutcome::Stalled.is_stall()), "K consecutive genuine stalls trips the escape");
  }

  #[test]
  fn count_saturates_without_wrapping_below_k() {
    let mut c = UsbTxStallCounter::new();
    for _ in 0..70_000u32 {
      c.record(true);
    }
    assert_eq!(c.count(), u16::MAX, "count must saturate, not wrap");
    assert!(c.record(true), "a saturated counter still reports the escape");
  }

  #[test]
  fn pack_decode_round_trips_all_fields() {
    let stall = UsbTxStall { data_free: true, serial_in_empty: false, int_ena_armed: true, ..base() };
    let word = pack_usb_tx_stall(&stall);
    assert_eq!(word & 0xFFFF_0000, USB_TX_STALL_TAG, "must carry the tag");
    assert_eq!(decode_usb_tx_stall(word), Some(stall));
  }

  #[test]
  fn pack_decode_round_trips_the_other_polarity() {
    let stall = UsbTxStall {
      data_free: false,
      serial_in_empty: true,
      motion_advancing: false,
      executor_running: true,
      int_ena_armed: false,
      response_depth: 0,
      timeout_count: 50, // within the 6-bit packed width, so it round-trips exactly.
    };
    assert_eq!(decode_usb_tx_stall(pack_usb_tx_stall(&stall)), Some(stall));
  }

  #[test]
  fn int_ena_armed_round_trips_independently() {
    // The new bit must not collide with the depth/count fields it now sits below.
    let armed = UsbTxStall { int_ena_armed: true, response_depth: 7, timeout_count: 9, ..base() };
    let unarmed = UsbTxStall { int_ena_armed: false, response_depth: 7, timeout_count: 9, ..base() };
    assert_eq!(decode_usb_tx_stall(pack_usb_tx_stall(&armed)), Some(armed));
    assert_eq!(decode_usb_tx_stall(pack_usb_tx_stall(&unarmed)), Some(unarmed));
    assert_ne!(pack_usb_tx_stall(&armed), pack_usb_tx_stall(&unarmed), "the bit must change the word");
  }

  #[test]
  fn decode_rejects_untagged_word() {
    assert_eq!(decode_usb_tx_stall(0), None, "cold-boot zero is not a stall");
    assert_eq!(decode_usb_tx_stall(0xFFFF_FFFF), None, "erased-flash garbage is not a stall");
    assert_eq!(decode_usb_tx_stall(0x1234_0007), None, "a different tag is not a stall");
  }

  #[test]
  fn timeout_count_saturates_into_the_packed_width() {
    let stall = UsbTxStall { response_depth: 15, timeout_count: 5000, ..base() };
    let decoded = decode_usb_tx_stall(pack_usb_tx_stall(&stall)).unwrap();
    // The count packs into 6 bits, so it saturates at 63 — and crucially must NOT bleed into bit 16 (the tag).
    assert_eq!(decoded.timeout_count, 63, "count is clamped to the 6-bit packed width");
    assert_eq!(decoded.response_depth, 15, "depth fills the nibble");
  }

  #[test]
  fn max_field_values_never_corrupt_the_tag() {
    // The regression that motivated the 7-bit count: with every packed field maxed, the word must still decode
    // (the tag in the high half must be intact — no field may reach bit 16).
    let stall = UsbTxStall {
      data_free: true,
      serial_in_empty: true,
      motion_advancing: true,
      executor_running: true,
      int_ena_armed: true,
      response_depth: 15,
      timeout_count: 63,
    };
    let word = pack_usb_tx_stall(&stall);
    assert_eq!(word & 0xFFFF_0000, USB_TX_STALL_TAG, "no field may bleed into the tag's high half");
    assert_eq!(decode_usb_tx_stall(word), Some(stall));
  }

  #[test]
  fn verdict_host_not_reading_when_fifo_full() {
    // data_free == false dominates: the host is not draining, regardless of the other signals.
    let stall = UsbTxStall { data_free: false, serial_in_empty: true, executor_running: true, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::HostNotReading);
  }

  #[test]
  fn verdict_lost_tx_wake_is_h_a() {
    // Room in the FIFO, the empty event asserted, core 1 alive → the TX-done wake was lost (H-A, esp-hal-side).
    let stall = UsbTxStall { serial_in_empty: true, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::LostTxWake);
  }

  #[test]
  fn verdict_core1_wedged_is_h_b() {
    // Room in the FIFO but core 1 frozen mid-block → the USB stall is downstream of a core-1 wedge (H-B). This
    // wins even if serial_in_empty is set, because a frozen core 1 is the more fundamental fault.
    let stall = UsbTxStall { serial_in_empty: true, motion_advancing: false, executor_running: true, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::Core1Wedged);
  }

  #[test]
  fn verdict_lost_tx_wake_when_int_ena_still_armed() {
    // bughunter's SHARPEST H-A fingerprint: the host drained (data_free), core 1 healthy, the empty EVENT is NOT
    // currently asserted, but the TX-empty INTERRUPT is still ARMED — the ISR never ran for this write. That is a
    // lost wake (H-A), not ambiguous: the future parked with the interrupt armed and no empty-edge re-fired.
    let stall = UsbTxStall { serial_in_empty: false, int_ena_armed: true, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::LostTxWake);
  }

  #[test]
  fn verdict_lost_waker_when_isr_fully_serviced_but_parked() {
    // THE ACTUAL 2026-06-25 CAPTURE: free=1 empty=0 mov=1 exec=1 rdepth=8 int_ena=0. The ISR ran (it cleared BOTH
    // int_ena and int_raw) and called WAKER_TX.wake(), but the embassy re-poll was lost, so usb_tx stayed parked in
    // the write await with the FIFO drainable and responses backed up. With the host draining (data_free) and a
    // non-empty RESPONSE backlog (rdepth > 0), this is a lost waker (H-A) — NOT ambiguous. (Verified against
    // esp-hal 1.1.1 `UsbSerialJtagWriteFuture::poll`: it returns Ready iff int_ena is CLEAR, so int_ena==0 means the
    // future WOULD have completed if re-polled — it simply never was.)
    let stall = UsbTxStall {
      data_free: true,
      serial_in_empty: false,
      int_ena_armed: false,
      motion_advancing: true,
      executor_running: true,
      response_depth: 8,
      timeout_count: 3,
    };
    assert_eq!(stall.verdict(), UsbTxVerdict::LostTxWake);
  }

  #[test]
  fn verdict_ambiguous_only_when_not_actually_backed_up() {
    // Ambiguous now requires a state that is NOT genuinely stalled-with-backlog: host has room, no event, not armed,
    // core 1 alive, AND `rdepth == 0` (we are not actually parked behind a backed-up RESPONSE channel). That is a
    // contradictory / non-reproducing snapshot, not a lost waker — keep it Ambiguous so a real lost-waker (rdepth>0)
    // is never silently filed here.
    let stall = UsbTxStall { serial_in_empty: false, int_ena_armed: false, response_depth: 0, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::Ambiguous);
  }

  #[test]
  fn idle_core1_not_running_is_not_core1_wedged() {
    // motion_advancing == false but executor_running == false means core 1 is legitimately IDLE (no block in
    // flight), not wedged — so it must NOT classify as Core1Wedged. With serial_in_empty set it reads as a lost wake.
    let stall = UsbTxStall { serial_in_empty: true, motion_advancing: false, executor_running: false, ..base() };
    assert_eq!(stall.verdict(), UsbTxVerdict::LostTxWake);
  }

  /// A neutral, HEALTHY baseline for the watchdog-decision tests: no frozen ticks, no backlog, host driving, a block
  /// in flight, and the canonical thresholds. Tests mutate one field at a time to assert each withhold condition.
  fn healthy_inputs() -> WatchdogInputs {
    WatchdogInputs {
      core1_frozen_ticks: 0,
      comms_frozen_ticks: 0,
      tx_complete_frozen_ticks: 0,
      executor_alive_frozen_ticks: 0,
      executor_stall_ticks: EXECUTOR_STALL_TICKS,
      response_depth: 0,
      block_in_flight: true,
      host_active: true,
      core1_stall_ticks: CORE1_STALL_TICKS,
      comms_stall_ticks: COMMS_STALL_TICKS,
    }
  }

  #[test]
  fn watchdog_feeds_both_dogs_when_healthy() {
    // No wedge condition met → feed BOTH dogs, no withhold reason.
    let d = watchdog_decision(healthy_inputs());
    assert!(d.feed_rwdt, "healthy → feed RWDT");
    assert!(d.feed_swd, "healthy → feed SuperWDT");
    assert_eq!(d.withhold_reason, None);
    // Just-below-threshold freezes are still healthy (the boundary is `>=`, so one short of each feeds).
    let near = WatchdogInputs {
      core1_frozen_ticks: CORE1_STALL_TICKS - 1,
      comms_frozen_ticks: COMMS_STALL_TICKS - 1,
      tx_complete_frozen_ticks: DEAD_ZONE_STALL_TICKS - 1,
      response_depth: 8,
      ..healthy_inputs()
    };
    let d = watchdog_decision(near);
    assert_eq!(d.withhold_reason, None, "one short of every threshold still feeds");
    assert!(d.feed_rwdt && d.feed_swd);
  }

  #[test]
  fn watchdog_withholds_both_dogs_on_core1_wedge() {
    let inputs = WatchdogInputs { core1_frozen_ticks: CORE1_STALL_TICKS, ..healthy_inputs() };
    let d = watchdog_decision(inputs);
    assert_eq!(d.withhold_reason, Some(WithholdKind::Core1Motion));
    assert!(!d.feed_rwdt, "a wedge withholds the RWDT");
    assert!(!d.feed_swd, "a wedge withholds the SuperWDT too — both, so neither masks the withhold");
  }

  #[test]
  fn watchdog_withholds_both_dogs_on_comms_wedge() {
    let inputs = WatchdogInputs { comms_frozen_ticks: COMMS_STALL_TICKS, ..healthy_inputs() };
    let d = watchdog_decision(inputs);
    assert_eq!(d.withhold_reason, Some(WithholdKind::Core0Comms));
    assert!(!d.feed_rwdt && !d.feed_swd);
  }

  #[test]
  fn watchdog_withholds_both_dogs_on_dead_zone() {
    // Responses queued AND usb_tx idle for the full dead-zone window, with no core1/comms freeze → DeadZone.
    let inputs = WatchdogInputs {
      response_depth: 1,
      tx_complete_frozen_ticks: DEAD_ZONE_STALL_TICKS,
      ..healthy_inputs()
    };
    let d = watchdog_decision(inputs);
    assert_eq!(d.withhold_reason, Some(WithholdKind::DeadZone));
    assert!(!d.feed_rwdt && !d.feed_swd);
  }

  #[test]
  fn watchdog_dead_zone_matches_dead_zone_withhold() {
    // The decision's DeadZone arm must agree with the standalone `dead_zone_withhold` for every depth/freeze combo —
    // the same backstop semantics, just surfaced through the unified decision. (No core1/comms freeze, so DeadZone is
    // the only candidate and is reached iff `dead_zone_withhold` is true.)
    for &depth in &[0usize, 1, 8] {
      for &frozen in &[0u32, DEAD_ZONE_STALL_TICKS - 1, DEAD_ZONE_STALL_TICKS, DEAD_ZONE_STALL_TICKS + 5] {
        let inputs =
          WatchdogInputs { response_depth: depth, tx_complete_frozen_ticks: frozen, ..healthy_inputs() };
        let withholds_dead_zone =
          watchdog_decision(inputs).withhold_reason == Some(WithholdKind::DeadZone);
        assert_eq!(
          withholds_dead_zone,
          dead_zone_withhold(depth, frozen),
          "decision DeadZone must match dead_zone_withhold for depth={depth} frozen={frozen}"
        );
      }
    }
  }

  #[test]
  fn watchdog_withholds_on_executor_stall_when_all_work_detectors_are_quiescent() {
    // The §17.15 ROOT-CAUSE case: a full core-0 executor stall where the three work-driven detectors are ALL gated off
    // — no block in flight (core-1 idle), host inactive (aged out), and NO responses queued (usb_tx drained). The old
    // decision would FEED here (the wedge that never reset); the ungated executor-liveness detector now catches it.
    let inputs = WatchdogInputs {
      executor_alive_frozen_ticks: EXECUTOR_STALL_TICKS,
      block_in_flight: false,
      host_active: false,
      response_depth: 0,
      ..healthy_inputs()
    };
    let d = watchdog_decision(inputs);
    assert_eq!(d.withhold_reason, Some(WithholdKind::Core0ExecutorStall), "an executor stall must withhold");
    assert!(!d.feed_rwdt && !d.feed_swd, "an executor stall withholds BOTH dogs");
    // One short of the threshold still feeds (the boundary is `>=`).
    let near = WatchdogInputs { executor_alive_frozen_ticks: EXECUTOR_STALL_TICKS - 1, ..inputs };
    assert_eq!(watchdog_decision(near).withhold_reason, None, "one short of the executor threshold still feeds");
  }

  #[test]
  fn watchdog_executor_stall_detector_is_disabled_when_threshold_is_zero() {
    // A build that does not wire the executor-liveness beat passes `executor_stall_ticks = 0`; the permanently-zero
    // frozen count must NEVER trip (a threshold of 0 with `>=` would otherwise false-trip every interval).
    let inputs = WatchdogInputs {
      executor_stall_ticks: 0,
      executor_alive_frozen_ticks: 0,
      block_in_flight: false,
      host_active: false,
      ..healthy_inputs()
    };
    assert_eq!(watchdog_decision(inputs).withhold_reason, None, "threshold 0 disables the detector");
    // Even a large frozen count cannot trip a disabled detector.
    let big = WatchdogInputs { executor_alive_frozen_ticks: 10_000, ..inputs };
    assert_eq!(watchdog_decision(big).withhold_reason, None, "a disabled detector ignores any frozen count");
  }

  #[test]
  fn watchdog_precedence_is_core1_then_executor_then_comms_then_dead_zone() {
    // When ALL conditions hold at once, the most-specific (core-1, which carries the exact RMT stage marker) wins,
    // then the definitive core-0 executor-dead signal, then core-0 comms, then the dead-zone backstop.
    let all = WatchdogInputs {
      core1_frozen_ticks: CORE1_STALL_TICKS,
      executor_alive_frozen_ticks: EXECUTOR_STALL_TICKS,
      comms_frozen_ticks: COMMS_STALL_TICKS,
      response_depth: 8,
      tx_complete_frozen_ticks: DEAD_ZONE_STALL_TICKS,
      ..healthy_inputs()
    };
    assert_eq!(watchdog_decision(all).withhold_reason, Some(WithholdKind::Core1Motion));
    // Drop core-1 → the executor-stall detector wins over comms + dead-zone.
    let no_core1 = WatchdogInputs { core1_frozen_ticks: 0, ..all };
    assert_eq!(watchdog_decision(no_core1).withhold_reason, Some(WithholdKind::Core0ExecutorStall));
    // Drop executor too → comms wins over dead-zone.
    let comms_and_dz = WatchdogInputs { executor_alive_frozen_ticks: 0, ..no_core1 };
    assert_eq!(watchdog_decision(comms_and_dz).withhold_reason, Some(WithholdKind::Core0Comms));
    // Drop comms too → dead-zone is the residual.
    let dz_only = WatchdogInputs { comms_frozen_ticks: 0, ..comms_and_dz };
    assert_eq!(watchdog_decision(dz_only).withhold_reason, Some(WithholdKind::DeadZone));
  }

  #[test]
  fn watchdog_honors_caller_supplied_thresholds() {
    // The firmware passes its own stall thresholds; a tighter threshold trips sooner. With `core1_stall_ticks = 2`,
    // two frozen ticks is already a wedge even though it is far below the default CORE1_STALL_TICKS.
    let inputs = WatchdogInputs { core1_frozen_ticks: 2, core1_stall_ticks: 2, ..healthy_inputs() };
    assert_eq!(watchdog_decision(inputs).withhold_reason, Some(WithholdKind::Core1Motion));
  }

  #[test]
  fn windowed_counter_consecutive_run_pins_at_ceiling() {
    // A PURE consecutive stall run (Signature A territory): the windowed count climbs to and pins at the ceiling.
    let mut w = WindowedStallCounter::new();
    assert_eq!(w.count(), 0);
    for i in 1..=STALL_WINDOW_LEN {
      let c = w.record(true);
      assert_eq!(c, i, "consecutive timeouts climb one per interval");
    }
    // Past the window length it saturates — it can hold at most STALL_WINDOW_LEN timeouts.
    for _ in 0..10 {
      assert_eq!(w.record(true), STALL_WINDOW_LEN, "windowed count saturates at the window length");
    }
    assert_eq!(w.count(), STALL_WINDOW_LEN);
  }

  #[test]
  fn windowed_counter_alternating_pattern_still_accrues() {
    // The §13.8 ALTERNATING recovered/stall pattern: each recovery resets the CONSECUTIVE K counter, but the windowed
    // count still accrues toward a degraded reading. Over a long alternating run it holds around half the ceiling —
    // clearly non-zero, distinguishing a degraded link from a healthy one.
    let mut w = WindowedStallCounter::new();
    let mut timed_out = true;
    for _ in 0..200 {
      w.record(timed_out);
      timed_out = !timed_out;
    }
    // A true trailing-N ring over a 1:1 alternating run holds EXACTLY half the window full — clearly elevated, unlike
    // a healthy link's 0 and distinct from a pure run's full window. This is the §13.8 signal the consecutive K
    // counter misses entirely (every recovery resets it to 0).
    assert_eq!(w.count(), STALL_WINDOW_LEN / 2, "a 1:1 alternating link reads exactly half the window full");
    // And a burst-heavy alternating pattern (two stalls per recovery) accrues a clearly-elevated count.
    let mut w2 = WindowedStallCounter::new();
    for _ in 0..50 {
      w2.record(true);
      w2.record(true);
      w2.record(false);
    }
    assert!(w2.count() >= STALL_WINDOW_LEN / 2, "a stall-heavy alternating link reads a clearly elevated window");
  }

  #[test]
  fn windowed_counter_long_quiet_ages_back_to_zero() {
    // After a stall run, a long QUIET stretch ages every timeout out of the window → back to 0 (a recovered link).
    let mut w = WindowedStallCounter::new();
    for _ in 0..STALL_WINDOW_LEN {
      w.record(true);
    }
    assert_eq!(w.count(), STALL_WINDOW_LEN);
    // It takes at most STALL_WINDOW_LEN clean intervals to fully age out (one timeout falls off the trailing edge
    // per clean interval).
    for _ in 0..STALL_WINDOW_LEN {
      w.record(false);
    }
    assert_eq!(w.count(), 0, "a full window of clean intervals ages the count back to zero");
    // ...and it stays at 0 (saturating subtract, never underflows).
    assert_eq!(w.record(false), 0, "ageing a zero window stays at zero, never wraps");
  }

  #[test]
  fn freeze_tracker_boot_guard_holds_the_pre_first_bump_zero() {
    // Before the core-0 heartbeat task is scheduled the beat is a flat 0. That must NEVER accrue a stall, no matter
    // how many samples elapse — the `alive != 0` boot guard is the whole point (else a board boots straight to a
    // false executor-stall breadcrumb).
    let mut t = ExecutorFreezeTracker::new();
    for _ in 0..(EXECUTOR_STALL_TICKS + 5) {
      let s = t.observe(0, EXECUTOR_STALL_TICKS);
      assert!(!s.frozen, "a flat-zero beat is never frozen (boot guard)");
      assert_eq!(s.frozen_ticks, 0);
      assert!(!s.crossed);
    }
  }

  #[test]
  fn freeze_tracker_first_nonzero_sample_counts_one_frozen_tick() {
    // Faithful mirror of the ISRs' seed: the first sample seeds `last` from the value, so `alive == last` reads as
    // frozen when `alive != 0`. This is the documented benign quirk — one tick, far below the threshold, cleared by
    // the next advance. Locked here so a future ISR rewire against this reducer preserves it rather than "fixing" it.
    let mut t = ExecutorFreezeTracker::new();
    let s = t.observe(5, EXECUTOR_STALL_TICKS);
    assert!(s.frozen, "first nonzero sample reads frozen (seed = no-change)");
    assert_eq!(s.frozen_ticks, 1);
    assert!(!s.crossed, "one tick is far below the threshold");
  }

  #[test]
  fn freeze_tracker_advancing_beat_never_freezes() {
    // A healthy, monotonically advancing beat resets the frozen count to 0 every sample. Seed at the boot-zero the
    // real beat starts from (not frozen, boot guard) so the first-nonzero-sample seed quirk does not apply.
    let mut t = ExecutorFreezeTracker::new();
    assert!(!t.observe(0, EXECUTOR_STALL_TICKS).frozen); // boot-zero seed: boot guard holds, last := 0.
    for beat in 1..=(EXECUTOR_STALL_TICKS + 10) {
      let s = t.observe(beat, EXECUTOR_STALL_TICKS);
      assert!(!s.frozen, "an advancing beat is not frozen");
      assert_eq!(s.frozen_ticks, 0);
      assert!(!s.crossed);
    }
  }

  #[test]
  fn freeze_tracker_crosses_at_exactly_the_threshold_and_holds() {
    // Model the real boot sequence: seed at the boot-zero, let the beat advance once (0 -> 1, establishing last := 1
    // WITHOUT the first-sample quirk pre-loading a tick), THEN hold it frozen at 1.
    let mut t = ExecutorFreezeTracker::new();
    assert!(!t.observe(0, EXECUTOR_STALL_TICKS).frozen); // boot-zero seed.
    assert!(!t.observe(1, EXECUTOR_STALL_TICKS).frozen); // beat advances 0 -> 1: not frozen, last := 1.
    // The beat is now stuck at 1. It takes EXECUTOR_STALL_TICKS consecutive frozen samples to cross.
    for tick in 1..EXECUTOR_STALL_TICKS {
      let s = t.observe(1, EXECUTOR_STALL_TICKS);
      assert!(s.frozen);
      assert_eq!(s.frozen_ticks, tick);
      assert!(!s.crossed, "must not cross before the threshold ({tick} < {EXECUTOR_STALL_TICKS})");
    }
    let s = t.observe(1, EXECUTOR_STALL_TICKS);
    assert_eq!(s.frozen_ticks, EXECUTOR_STALL_TICKS);
    assert!(s.crossed, "crosses at exactly the threshold");
    // And it STAYS crossed while the beat remains frozen (the caller's own latch makes the ACTION once).
    let s = t.observe(1, EXECUTOR_STALL_TICKS);
    assert!(s.crossed && s.frozen_ticks == EXECUTOR_STALL_TICKS + 1, "stays crossed past the threshold");
  }

  #[test]
  fn freeze_tracker_advance_mid_run_resets_and_must_re_accrue() {
    // A recovery partway through a frozen run drops the count to 0; a subsequent freeze must climb from scratch.
    let mut t = ExecutorFreezeTracker::new();
    t.observe(1, EXECUTOR_STALL_TICKS); // seed.
    for _ in 0..(EXECUTOR_STALL_TICKS - 1) {
      t.observe(1, EXECUTOR_STALL_TICKS); // climb to threshold - 1.
    }
    let recovered = t.observe(2, EXECUTOR_STALL_TICKS); // the executor advanced.
    assert!(!recovered.frozen && recovered.frozen_ticks == 0 && !recovered.crossed, "recovery resets the count");
    let s = t.observe(2, EXECUTOR_STALL_TICKS); // frozen again at the NEW value.
    assert_eq!(s.frozen_ticks, 1, "the next freeze re-accrues from zero, not from the pre-recovery count");
  }

  #[test]
  fn freeze_tracker_zero_threshold_disables_the_crossing() {
    // A build that does not wire the beat passes stall_ticks == 0; a permanently-frozen count then never crosses —
    // matching `watchdog_decision`'s `executor_stall_ticks == 0` guard.
    let mut t = ExecutorFreezeTracker::new();
    t.observe(1, 0); // seed.
    for _ in 0..100 {
      let s = t.observe(1, 0);
      assert!(s.frozen, "the beat is genuinely frozen");
      assert!(!s.crossed, "but a zero threshold never declares a stall");
    }
  }

  #[test]
  fn freeze_tracker_crossed_matches_watchdog_decision() {
    // The reducer is a drop-in for what the ISR feeds `watchdog_decision`: routing `frozen_ticks` through the pure
    // decision must declare Core0ExecutorStall on exactly the sample the tracker reports `crossed` (with the other
    // detectors quiescent). This ties the extracted SM to the decision it will feed once the ISRs are rewired.
    let mut t = ExecutorFreezeTracker::new();
    t.observe(1, EXECUTOR_STALL_TICKS); // seed on an advance.
    let quiescent = WatchdogInputs {
      core1_frozen_ticks: 0,
      comms_frozen_ticks: 0,
      tx_complete_frozen_ticks: 0,
      executor_alive_frozen_ticks: 0,
      executor_stall_ticks: EXECUTOR_STALL_TICKS,
      response_depth: 0,
      block_in_flight: false,
      host_active: false,
      core1_stall_ticks: CORE1_STALL_TICKS,
      comms_stall_ticks: COMMS_STALL_TICKS,
    };
    for _ in 0..(EXECUTOR_STALL_TICKS * 2) {
      let s = t.observe(1, EXECUTOR_STALL_TICKS);
      let decision = watchdog_decision(WatchdogInputs { executor_alive_frozen_ticks: s.frozen_ticks, ..quiescent });
      let declared = decision.withhold_reason == Some(WithholdKind::Core0ExecutorStall);
      assert_eq!(s.crossed, declared, "tracker.crossed must agree with watchdog_decision's executor-stall verdict");
    }
  }
}
