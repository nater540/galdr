//! The coordinated step-output contract (DOC-02).
//!
//! These are the only hardware-facing traits the kinematics core needs: the segment generator in
//! [`crate::motion`] decides every per-tick step mask and period and pushes them into a [`StepSink`].
//! The firmware bin implements the sink over the three RMT TX channels; host tests implement it as a
//! recording buffer. Everything here is pure data + one trait, so the trapezoid/DDA/timing math stays
//! host-testable. `firmware-core` re-exports these from its `hal_traits` module, so the firmware sees
//! them at the familiar `firmware_core::hal_traits::*` paths.

use crate::planner::AXES;

/// The maximum number of [`StepEvent`]s (RMT PulseCode symbols) a single burst may carry. The ESP32-S3 RMT
/// memory block holds 48 symbols (`SOC_RMT_MEM_WORDS_PER_CHANNEL`), but the firmware bin appends one
/// mandatory `end_marker` symbol after the events, so an N-event burst encodes to N+1 symbols. Capping at
/// **46** events makes a full burst 46 + 1 = 47 symbols — one symbol SHORT of the 48-slot block, so slot 47
/// is ALWAYS left free. This is deliberate and load-bearing (see below); it keeps every burst inside a single
/// block so the driver never borrows the adjacent channel's memory and the interrupt-priority streaming-refill
/// (ping-pong) path is never relied upon (DOC-02). A burst larger than this is rejected with
/// [`StepError::BurstTooLong`].
///
/// ## Why 46, not 47 — the never-completely-full-block rule (RMT TX-completion hang fix)
/// A confirmed real-board lockup (`stage=axis0:wait_begin` with no `wait_done` in the crash breadcrumb) was the
/// core-1 RMT TX-completion `wait()` spinning forever on channel 0 (X) — TX-END never asserted. Reading the
/// esp-hal 1.1.1 RMT writer (`rmt/writer.rs`): on the blocking one-shot path the whole buffer is written into the
/// 48-slot block and `mem_tx_wrap_en` is set. If the buffer EXACTLY fills the block (48 symbols) and the writer's
/// last-written code is NOT a length-zero end marker — which can arise from any off-by-one or truncation
/// (`count = data.len().min(48)` silently drops a 49th marker) — the writer stays in `WriterState::Active`, there
/// is no free slot for esp-hal to inject its own terminating marker, the hardware read pointer WRAPS slot-47 → 0
/// and re-transmits, and `wait()` busy-polls `Event::End` forever. Capping at 46 events guarantees the encoded
/// buffer is at most 47 symbols, so `data.len() < 48` is ALWAYS true: esp-hal then either reaches `Done` (our
/// marker in slot ≤46) or, on any malformed buffer, injects a marker into the free slot and returns a CLEAN
/// `Error` from `transmit` BEFORE starting TX — the `Active`/wrap/hang path becomes structurally unreachable.
/// The cost is negligible: a 47-event dominant-axis move now spans two bursts instead of one (one extra
/// transmit), which on the dedicated core-1 executor is a sub-microsecond overhead. Do NOT raise this back to 47.
pub const MAX_SYMBOLS_PER_BURST: usize = 46;

/// The direction state latched onto the three axis DIR outputs before a burst. One `bool` per axis,
/// indexed `[X, Y, Z]`: `true` is the positive (increasing-step) direction, `false` is negative. The
/// segment generator derives this from the signs of `Block::steps`; it is constant for a whole planner
/// block (a straight-line move never reverses an axis mid-block), so it is set once per block, never
/// per tick. On target the implementation drives the DIR GPIOs and then honors the `$29` direction
/// setup delay before the first step pulse of the following burst.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DirState {
  /// Per-axis direction, `[X, Y, Z]`; `true` = positive, `false` = negative.
  pub dir: [bool; AXES],
}

/// A single DDA tick: the synchronized step decision for all three axes on one step-generator tick,
/// plus the full step period that tick occupies. This is the unit the RMT layer turns into exactly one
/// PulseCode per channel: an axis that steps emits a HIGH-for-`$0`-ticks / LOW-for-remainder pulse, and
/// an axis that does not step emits a full-period-LOW (silent) symbol so all three channel bursts stay
/// the same length and remain sample-aligned under the motion executor's `join3` await (DOC-02).
///
/// The dominant axis (largest step count in the block) steps on every tick; subordinate axes step only
/// when their Bresenham error accumulator crosses, so `step` is a per-tick mask, not a constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StepEvent {
  /// Per-axis step mask for this tick, `[X, Y, Z]`; `true` means the axis emits a step pulse this tick.
  pub step: [bool; AXES],
  /// The full step period of this tick in [`MotionConfig`](crate::motion::MotionConfig) timer ticks
  /// (one HIGH+LOW pulse). The HIGH width is the `$0` step-pulse time; the LOW width is the remainder.
  /// A larger period is a slower instantaneous velocity; varying it across ticks realizes the ramp.
  pub period_ticks: u32,
}

/// Errors a [`StepSink`] may return. All are recoverable by the caller (the motion executor): the
/// segment generator surfaces them rather than panicking, per the no-`unwrap` library rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepError {
  /// A burst exceeded [`MAX_SYMBOLS_PER_BURST`] events. The segment generator never emits such a burst
  /// (it flushes at the cap); this guards a sink against a mis-sized burst from any other producer.
  BurstTooLong,
  /// The underlying step transport failed (on target: an RMT transmit error). Host recorders never
  /// return this.
  Transport,
}

/// Sink for coordinated, axis-synchronized step bursts. Implemented over the three RMT TX channels on
/// target (one burst becomes one PulseCode array per channel, transmitted concurrently and awaited
/// together) and as a recording buffer in host tests.
///
/// ## Contract
/// - [`set_direction`](StepSink::set_direction) is called once before the burst(s) of a block whenever
///   the direction changes; the implementation latches the DIR outputs and observes the `$29` setup
///   delay before the next emitted pulse. Direction never changes within a planner block.
/// - [`emit_burst`](StepSink::emit_burst) submits one constant-rate-region slice of up to
///   [`MAX_SYMBOLS_PER_BURST`] [`StepEvent`]s. Every event is one synchronized tick across all axes;
///   the slice length is the symbol count for each channel, so all channels stay aligned. A slice
///   longer than the cap must be rejected with [`StepError::BurstTooLong`].
/// - The sink is purely a consumer of decided ticks; all trapezoid/DDA/timing math lives in the
///   segment generator so it stays host-testable.
pub trait StepSink {
  /// Latch the per-axis direction outputs ahead of the next burst, honoring the direction setup delay.
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError>;

  /// Emit one burst of up to [`MAX_SYMBOLS_PER_BURST`] synchronized step ticks. Returns
  /// [`StepError::BurstTooLong`] if the slice exceeds the cap, or [`StepError::Transport`] on a
  /// hardware transmit failure.
  fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError>;
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The ESP32-S3 RMT memory block holds 48 symbols. The firmware bin appends one `end_marker` after the events
  /// of a burst, so a full burst of [`MAX_SYMBOLS_PER_BURST`] events encodes to `MAX_SYMBOLS_PER_BURST + 1`
  /// symbols. That total must be STRICTLY LESS than one block, so slot 47 is always free: a completely-full block
  /// can trip the esp-hal 1.1.1 RMT writer into `WriterState::Active` + wrap-around re-transmit when the last code
  /// is not a marker, hanging `wait()` forever (the confirmed `axis0:wait_begin` lockup — see
  /// [`MAX_SYMBOLS_PER_BURST`]). Leaving a free slot keeps the burst in one block AND makes that wrap-trap
  /// unreachable (esp-hal injects a terminating marker into the spare slot and errors cleanly instead).
  #[test]
  fn full_burst_plus_end_marker_leaves_a_free_slot_in_the_rmt_block() {
    const RMT_BLOCK_SYMBOLS: usize = 48;
    assert!(
      MAX_SYMBOLS_PER_BURST + 1 < RMT_BLOCK_SYMBOLS,
      "events + end marker must be strictly less than one RMT block so slot 47 stays free (never a full block)"
    );
  }
}
