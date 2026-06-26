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

/// The classified outcome of one `usb_tx` write attempt, AFTER applying the poll-after-arm completion recheck (the
/// fix for the §12 lost-wake root cause). The esp-hal async write future completes only when its `serial_in_empty`
/// waker is delivered; the captured root cause is that the wake is LOST (the ISR ran, cleared `int_ena`, woke
/// `WAKER_TX`, but the embassy executor never re-polled), so a write whose bytes ALREADY left for the host parks
/// for the full 2 s `USB_TX_TIMEOUT`. We cannot fix esp-hal's internal future from the task, but we CAN layer a
/// polling backstop over its event-driven wait: when the timeout fires, re-read the hardware "host drained the
/// FIFO" bit (`serial_in_ep_data_free`) and decide what actually happened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriteOutcome {
  /// The `with_timeout` future resolved normally — the write/flush completed within the bound (the healthy path,
  /// and the path once the lost-wake fix lands, since most writes will not even reach the timeout).
  Completed,
  /// The `with_timeout` TIMED OUT, but the post-timeout recheck found the host HAD drained the FIFO
  /// (`serial_in_ep_data_free == true`): the bytes are out, only the esp-hal completion wake was lost. The write
  /// effectively SUCCEEDED — treat it as completed (reset the stall counter, do not escalate), which breaks the
  /// 2 s lost-wake drumbeat. Distinguished from [`Completed`] only so the caller can COUNT recovered lost-wakes
  /// (a live "this bug is happening" signal) without changing control flow.
  CompletedLostWakeRecovered,
  /// The `with_timeout` TIMED OUT and the FIFO was still NOT drained (`serial_in_ep_data_free == false`): the host
  /// genuinely is not reading (or the peripheral is truly stuck). This is a REAL stall — it counts toward the
  /// [`USB_TX_STALL_ESCAPE_K`] bounded escape exactly as before.
  Stalled,
}

impl WriteOutcome {
  /// Classify a `usb_tx` write attempt from the `with_timeout` result and (on a timeout) the post-timeout
  /// host-drained state. `timed_out` is `with_timeout(...).is_err()`; `data_free_after_timeout` is
  /// `ep1_conf.serial_in_ep_data_free` re-read AFTER the timeout (only consulted when `timed_out`). This is the
  /// poll-after-arm recheck: a timeout with the FIFO drained is a recovered lost-wake, not a stall.
  pub fn classify(timed_out: bool, data_free_after_timeout: bool) -> WriteOutcome {
    if !timed_out {
      return WriteOutcome::Completed;
    }
    if data_free_after_timeout {
      // The timeout fired but the host HAS drained the FIFO — the write's bytes are gone, only the wake was lost.
      WriteOutcome::CompletedLostWakeRecovered
    } else {
      // The FIFO is still full at the timeout: a genuine stall (host not reading / peripheral stuck).
      WriteOutcome::Stalled
    }
  }

  /// Whether this outcome counts as a STALL toward the bounded escape. Only a genuine [`Stalled`](WriteOutcome::
  /// Stalled) does; both completed variants reset the run (the write made progress). This is what feeds
  /// [`UsbTxStallCounter::record`].
  pub fn is_stall(self) -> bool {
    matches!(self, WriteOutcome::Stalled)
  }

  /// Whether this outcome is a RECOVERED lost-wake (a timeout the recheck rescued). The caller bumps a diagnostic
  /// counter on this so a live build can report "the lost-wake bug fired N times but recovered" without wedging.
  pub fn is_recovered_lost_wake(self) -> bool {
    matches!(self, WriteOutcome::CompletedLostWakeRecovered)
  }

  /// Classify the WRITE stage of a `usb_tx` response, returning a terminal [`WriteOutcome`] when the write stage
  /// already decides it, or `None` when the write completed cleanly and the caller must proceed to the FLUSH stage.
  /// The two terminal write-stage dispositions:
  /// - `write_timed_out` ⇒ [`Stalled`](WriteOutcome::Stalled): a mid-`write_all` timeout with possibly-UNWRITTEN
  ///   bytes — never recoverable (recovering would truncate), so it counts toward the K-escape.
  /// - `write_errored` (the embedded-io `Ok(Err)` host-closed-port case) ⇒ [`Completed`](WriteOutcome::Completed):
  ///   a clean drop-and-continue that the reconnect / banner path re-syncs — NOT a stall, so it must not flow to
  ///   the flush or count toward the escape (matching the pre-split behavior, which discarded write errors).
  /// - otherwise (`None`) ⇒ all bytes reached the FIFO; defer to [`classify_split`](WriteOutcome::classify_split)
  ///   for the flush-stage recovery decision.
  ///
  /// A timeout takes precedence over an error (a timed-out write never produced an `Ok(Err)` in the first place).
  pub fn classify_write_stage(write_timed_out: bool, write_errored: bool) -> Option<WriteOutcome> {
    if write_timed_out {
      Some(WriteOutcome::Stalled)
    } else if write_errored {
      Some(WriteOutcome::Completed)
    } else {
      None
    }
  }

  /// Classify a `usb_tx` response when the write and flush are timed SEPARATELY (the truncation-safe two-stage
  /// sequencing). This is the SAFE variant of [`classify`](WriteOutcome::classify): the poll-after-arm recovery is
  /// only valid once ALL bytes are provably in the FIFO, which is true at the FLUSH stage but NOT mid-`write_all`.
  ///
  /// `write_async` (esp-hal) pushes the response in ≤64-byte chunks and awaits the TX-empty wake BETWEEN chunks, so a
  /// lost-wake can strand `write_all` AFTER the first chunk — at which point `serial_in_ep_data_free` is `true` (the
  /// host drained chunk 1) yet the REMAINING bytes were never written. Treating that as recovered would advance to
  /// the next response and emit a TRUNCATED line. So:
  /// - `write_timed_out == true` ⇒ a mid-write timeout, possibly with UNWRITTEN bytes ⇒ [`Stalled`](WriteOutcome::
  ///   Stalled), NEVER recovered (it counts toward the K-escape; a clean reset beats a silent truncation).
  /// - otherwise the write completed (all bytes are in the FIFO) and only the FLUSH could have stranded, so defer to
  ///   [`classify`](WriteOutcome::classify) with the flush-stage timeout + the post-flush `data_free` recheck — the
  ///   ONLY place the recovery is sound.
  ///
  /// `data_free_after_flush` is only consulted when `!write_timed_out && flush_timed_out`.
  pub fn classify_split(write_timed_out: bool, flush_timed_out: bool, data_free_after_flush: bool) -> WriteOutcome {
    if write_timed_out {
      // Bytes may be unwritten — recovery would truncate. A genuine stall, full stop.
      return WriteOutcome::Stalled;
    }
    // All bytes are in the FIFO; only a flush-stage lost-wake is recoverable (the captured case).
    WriteOutcome::classify(flush_timed_out, data_free_after_flush)
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
  /// The consecutive-timeout count when captured (≥ [`USB_TX_STALL_ESCAPE_K`]). Packed into 7 bits (saturates at
  /// 127) — ample for a "reached K and kept timing out" diagnostic; the exact magnitude past K is not load-bearing.
  pub timeout_count: u16,
}

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
  // reached bit 16). Layout: five flag bits (0..5), a depth nibble (5..9), a 7-bit count (9..16).
  pub const DATA_FREE: u32 = 1 << 0;
  pub const SERIAL_IN_EMPTY: u32 = 1 << 1;
  pub const MOTION_ADVANCING: u32 = 1 << 2;
  pub const EXECUTOR_RUNNING: u32 = 1 << 3;
  pub const INT_ENA_ARMED: u32 = 1 << 4;
  /// Response depth in bits 5..9 (a nibble; the depth-8 channel → 0..=8 fits, clamp at 15).
  pub const RESPONSE_DEPTH_SHIFT: u32 = 5;
  pub const RESPONSE_DEPTH_MASK: u32 = 0xF << RESPONSE_DEPTH_SHIFT;
  /// Timeout count in bits 9..16 (7 bits, saturated to [`TIMEOUT_COUNT_CLAMP`]). 7 bits is ample for a ≥K
  /// diagnostic — the exact magnitude past the escape is not load-bearing, only "it reached K and kept timing out".
  pub const TIMEOUT_COUNT_SHIFT: u32 = 9;
  pub const TIMEOUT_COUNT_MASK: u32 = 0x7F << TIMEOUT_COUNT_SHIFT;
  /// The saturation ceiling for the 7-bit packed timeout count.
  pub const TIMEOUT_COUNT_CLAMP: u16 = 0x7F;
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
  fn write_outcome_completed_when_not_timed_out() {
    // A normal completion — `data_free_after_timeout` is irrelevant (not consulted) when not timed out.
    assert_eq!(WriteOutcome::classify(false, false), WriteOutcome::Completed);
    assert_eq!(WriteOutcome::classify(false, true), WriteOutcome::Completed);
    assert!(!WriteOutcome::classify(false, false).is_stall());
    assert!(!WriteOutcome::classify(false, false).is_recovered_lost_wake());
  }

  #[test]
  fn write_outcome_recovers_lost_wake_when_timeout_but_fifo_drained() {
    // THE FIX: the 2 s timeout fired, but the host HAD drained the FIFO (data_free=true) → the bytes are out and
    // only the esp-hal wake was lost. Treat as a recovered completion, NOT a stall — this breaks the drumbeat.
    let o = WriteOutcome::classify(true, true);
    assert_eq!(o, WriteOutcome::CompletedLostWakeRecovered);
    assert!(!o.is_stall(), "a recovered lost-wake must NOT count toward the escape");
    assert!(o.is_recovered_lost_wake(), "and it IS countable as a recovered lost-wake for diagnostics");
  }

  #[test]
  fn write_outcome_is_stall_when_timeout_and_fifo_still_full() {
    // A genuine stall: timeout AND the FIFO is still full (host not reading / peripheral stuck) → counts toward K.
    let o = WriteOutcome::classify(true, false);
    assert_eq!(o, WriteOutcome::Stalled);
    assert!(o.is_stall());
    assert!(!o.is_recovered_lost_wake());
  }

  #[test]
  fn split_mid_write_timeout_is_always_a_stall_never_recovered() {
    // THE TRUNCATION GUARD: a write-stage timeout may leave unwritten bytes (write_async parks between 64B chunks),
    // so it must NEVER be classified as recovered — even if the host had drained an earlier chunk (data_free=true).
    // It is a genuine stall that counts toward the K-escape; a clean reset beats a silent truncation.
    let o = WriteOutcome::classify_split(true, false, true);
    assert_eq!(o, WriteOutcome::Stalled);
    assert!(o.is_stall());
    assert!(!o.is_recovered_lost_wake());
    // ...and the same regardless of the flush/data_free args, since a write timeout short-circuits.
    assert_eq!(WriteOutcome::classify_split(true, true, true), WriteOutcome::Stalled);
    assert_eq!(WriteOutcome::classify_split(true, false, false), WriteOutcome::Stalled);
  }

  #[test]
  fn write_stage_error_is_a_clean_drop_not_a_stall() {
    // A write ERROR (host closed the port, embedded-io `Ok(Err)`) is NOT a stall — it's a clean drop-and-continue
    // (Completed), matching the pre-split behavior that discarded write errors. It must NOT count toward the escape
    // and must NOT flow to the flush stage. (write_timed_out=false, write_errored=true → Some(Completed).)
    let o = WriteOutcome::classify_write_stage(false, true);
    assert_eq!(o, Some(WriteOutcome::Completed));
    assert!(!o.unwrap().is_stall(), "a host-closed write error is not a stall");
    assert!(!o.unwrap().is_recovered_lost_wake());
  }

  #[test]
  fn write_stage_timeout_is_a_stall_and_wins_over_error() {
    // A write TIMEOUT is a possibly-mid-write stall and takes precedence (a timed-out write never produced Ok(Err)).
    assert_eq!(WriteOutcome::classify_write_stage(true, false), Some(WriteOutcome::Stalled));
    assert_eq!(WriteOutcome::classify_write_stage(true, true), Some(WriteOutcome::Stalled));
  }

  #[test]
  fn write_stage_clean_defers_to_the_flush_stage() {
    // A clean write (no timeout, no error) returns None → the caller proceeds to time + classify the flush stage.
    assert_eq!(WriteOutcome::classify_write_stage(false, false), None);
  }

  #[test]
  fn split_recovers_only_at_the_flush_stage() {
    // The write completed (all bytes in the FIFO), and the FLUSH stranded with the host having drained → the safe,
    // recoverable lost-wake (the captured case). This is where recovery is sound.
    let o = WriteOutcome::classify_split(false, true, true);
    assert_eq!(o, WriteOutcome::CompletedLostWakeRecovered);
    assert!(o.is_recovered_lost_wake());
    assert!(!o.is_stall());
  }

  #[test]
  fn split_flush_timeout_with_fifo_full_is_a_genuine_stall() {
    // Write completed, flush timed out, but the FIFO is STILL full (host not reading) → a genuine stall toward K.
    assert_eq!(WriteOutcome::classify_split(false, true, false), WriteOutcome::Stalled);
  }

  #[test]
  fn split_clean_write_and_flush_is_completed() {
    // Neither stage timed out → a clean completion (the healthy path).
    assert_eq!(WriteOutcome::classify_split(false, false, true), WriteOutcome::Completed);
    // data_free arg is irrelevant when flush did not time out.
    assert_eq!(WriteOutcome::classify_split(false, false, false), WriteOutcome::Completed);
  }

  #[test]
  fn recovered_lost_wake_resets_the_stall_counter_like_a_completion() {
    // Wiring contract: feeding the counter `is_stall()` means a recovered lost-wake (is_stall == false) RESETS the
    // run exactly like a clean completion, so a stream of recovered lost-wakes never trips the K-escape — only a
    // run of GENUINE stalls does. This is what makes the fix break the drumbeat instead of just resetting on it.
    let mut c = UsbTxStallCounter::new();
    assert!(!c.record(WriteOutcome::classify(true, false).is_stall())); // genuine stall #1
    assert!(!c.record(WriteOutcome::classify(true, false).is_stall())); // genuine stall #2
    // A recovered lost-wake breaks the run even though it arrived via a timeout.
    assert!(!c.record(WriteOutcome::classify(true, true).is_stall()));
    assert_eq!(c.count(), 0, "a recovered lost-wake resets the run");
    // Only a fresh K genuine stalls fires the escape.
    assert!(!c.record(WriteOutcome::classify(true, false).is_stall()));
    assert!(!c.record(WriteOutcome::classify(true, false).is_stall()));
    assert!(c.record(WriteOutcome::classify(true, false).is_stall()));
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
      timeout_count: 100, // within the 7-bit packed width, so it round-trips exactly.
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
    // The count packs into 7 bits, so it saturates at 127 — and crucially must NOT bleed into bit 16 (the tag).
    assert_eq!(decoded.timeout_count, 127, "count is clamped to the 7-bit packed width");
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
      timeout_count: 127,
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
}
