//! Core-1 real-time step generation (DOC-02 / DOC-01): the esp-hal wiring that turns planner blocks into
//! RMT pulse trains on the X/Y/Z step channels.
//!
//! This is the hardware adapter for [`firmware_core::motion`]: the pure, host-tested
//! [`SegmentGenerator`](firmware_core::motion::SegmentGenerator) decides every per-tick step mask and step
//! period, and this module realizes those decisions as RMT [`PulseCode`] bursts and live machine position.
//! All trapezoid / DDA / timing math stays in firmware-core; here we only drive GPIO and the RMT peripheral.
//!
//! ## Why core 1, and why blocking RMT
//! The `motion_executor` task runs ALONE on core 1 (APP_CPU) on an [`InterruptExecutor`] at
//! [`Priority::Priority3`], so it preempts nothing and gets uncontested CPU for real-time stepping (DOC-01).
//! [`SegmentGenerator::run_block`](firmware_core::motion::SegmentGenerator::run_block) is *synchronous* and
//! calls [`StepSink::emit_burst`](firmware_core::hal_traits::StepSink::emit_burst) synchronously, so the sink
//! cannot `.await`. We therefore use esp-hal's **blocking** RMT transmit/wait path: each burst starts all
//! three channel transmits (the hardware then runs them concurrently and sample-aligned), then blocking-waits
//! all three before returning. Blocking core 1 for a sub-millisecond burst is the intended design, not a
//! stall — nothing else runs there. The only `.await` in the executor sits *between* blocks (awaiting
//! [`BLOCK_AVAILABLE`](crate::comms::BLOCK_AVAILABLE) and, on a hold, the [`HOLD_WAKE`](crate::comms::HOLD_WAKE)
//! level-change wake that releases the park).
//!
//! ## What is compile-verified vs host-tested
//! The RMT/PulseCode encoding and the core-1 executor glue touch esp-hal and are verified by the Xtensa
//! build only (no hardware in CI). The genuinely testable logic — the live step→mm position tracking — lives
//! as [`StepCounter`](firmware_core::motion::StepCounter) in firmware-core, where it runs under `cargo test`.
//!
//! ## Still stubbed (out of scope for this phase, deliberate TODOs)
//! - STEP_EN (GPIO8) is driven enabled (active-low → low) once at init so the steppers hold; the full
//!   enable/disable-on-idle policy is a later refinement (DOC-03/DOC-05).
//! - `$0`/`$29` and `steps_per_mm` come from the in-memory placeholder settings; esp-storage persistence is
//!   DOC-00. Feed-hold deceleration is per-block (Stage 1); smooth ramp-down is a later refinement.
//!
//! ## Diagnosing a core-1 stall (defmt tracing)
//! The executor runs ALONE on core 1, so a wedge shows on the wire as a permanent `Run` with `FS:0`. Build a
//! logging image (`just build --features defmt`, flash it, then `just monitor`), send `G0 X5`, and read the LAST
//! [`mtrace`] line over RTT. The trace points form a linear chain; the last one seen localizes the fault:
//!
//! - (nothing) — the core-1 InterruptExecutor never started the task; investigate `main`'s `start_second_core`.
//! - `executor loop entered` then silence on `G0 X5` — the wake/enqueue handshake never reached the executor;
//!   the block sat in the planner queue (so `?` shows `Run` from the `queued` term and `FS:0`). Suspect
//!   `BLOCK_AVAILABLE` (enqueued without signaling, or the signal consumed elsewhere).
//! - `hold requested -> parking` — `HOLD_REQUESTED` is wedged set; the block is never popped. NOT the RMT path.
//! - `popping block (taking PLANNER lock)` with no `PLANNER lock acquired` — the `PLANNER` mutex is held across
//!   an await on core 0 (cross-core lock contention). NOT the RMT path.
//! - `block popped` / `feed published` but no `run_block returned` — the stall is inside the generator/RMT emit.
//!   Then the `emit_burst` / per-axis `transmit` / `wait begin` / `wait ok` lines pin it to the exact RMT axis:
//!   a `wait begin` with no matching `wait ok`/`wait err` is a never-completing blocking `wait()` (TX-END never
//!   fired) on that channel — the genuine RMT hardware path.

use core::sync::atomic::Ordering;

use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rmt::{Channel, PulseCode, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::Blocking;
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_time::{Duration, Ticker, Timer};

use firmware_core::hal_traits::{
  probe_triggered, DigitalIn, DirState, ProbeConfig, ProbeInput, StepError, StepEvent, StepSink,
  MAX_SYMBOLS_PER_BURST,
};
use firmware_core::homing::{HomingConfig, HomingError, HOMING_GROUPS};
use firmware_core::motion::{silent_symbol_halves, MotionConfig, ProbeStepper, SegmentGenerator, StepCounter};
use firmware_core::planner::{Block, Planner, A_AXIS, AXES};

use crate::comms::{
  overrides, ProbeRequest, ProbeResult, BLOCK_AVAILABLE, EXECUTOR_RUNNING, HARD_LIMITS_ENABLED,
  HARD_LIMIT_TRIPPED, HOLD_REQUESTED, HOLD_WAKE, HOMING_ACTIVE, HOME_REQUEST, HOME_RESULT, LIMIT_LEVELS,
  LIMIT_TRIGGERED, LIVE_BLOCK_IS_RAPID, LIVE_POSITION, LIVE_PROGRAMMED_FEED_MM_MIN, MOTION_LIVENESS, MOTION_PARKED,
  MOTION_RESET, MOTION_RESET_PENDING, PLANNER, PROBE_ASSERTED, PROBE_REQUEST, PROBE_RESULT, SLOT_FREED,
};

/// Core-1 motion-executor trace point. Expands to a `defmt::trace!` only under the `defmt` feature and to
/// NOTHING otherwise, so the default non-logging build pays zero cost and stays `#![deny(warnings)]`-clean.
/// These traces localize a core-1 stall: the executor runs alone on core 1, so the LAST trace line seen over
/// RTT pinpoints exactly where it wedged (see the module-level "Diagnosing a core-1 stall" notes). Routed over
/// esp-println's defmt/RTT sink, separate from the grbl USB CDC stream, so tracing never perturbs the protocol
/// or the host's character-counting flow control.
///
/// Every call site traces only values that ALREADY exist on the executing path (never a value computed solely
/// for the trace), so the no-`defmt` empty expansion can never leave an unused binding — keeping the hot path
/// allocation-free and warning-clean either way.
#[cfg(feature = "defmt")]
macro_rules! mtrace {
  ($($arg:tt)*) => { defmt::trace!($($arg)*) };
}

/// No-op counterpart of [`mtrace`] for builds without the `defmt` feature: expands to nothing.
#[cfg(not(feature = "defmt"))]
macro_rules! mtrace {
  ($($arg:tt)*) => {{}};
}

/// One RMT TX channel per axis, indexed `[X, Y, Z]`. The blocking transmit API consumes the channel and
/// hands it back from the transaction's `wait()`, so each channel is held as an `Option` and taken /
/// restored across every burst (a `None` only ever occurs transiently inside [`RmtStepSink::emit_burst`]).
type AxisChannel = Channel<'static, Blocking, Tx>;

/// The RMT-backed [`StepSink`]: drives the three DIR GPIOs and the three RMT step channels so one
/// [`StepEvent`] burst becomes one equal-length [`PulseCode`] array per channel, transmitted concurrently
/// and awaited together. It is the on-target counterpart of firmware-core's recording test sink.
pub struct RmtStepSink {
  /// The three step-pulse channels, `[X, Y, Z]`. `Some` between bursts; momentarily `None` while a
  /// transaction owns the channel inside [`emit_burst`](RmtStepSink::emit_burst).
  channels: [Option<AxisChannel>; AXES],
  /// The three DIR outputs, `[X, Y, Z]` (GPIO5/6/7). Latched once per block by
  /// [`set_direction`](RmtStepSink::set_direction).
  dir: [Output<'static>; AXES],
  /// The `$0` step-pulse HIGH width in RMT ticks (one tick = 1 µs at the configured clock divider). Held
  /// here so the per-burst PulseCode builder needs only the timing config, not the whole [`MotionConfig`].
  step_pulse_ticks: u32,
  /// The `$29` direction-setup delay in microseconds: after the DIR outputs change, the first following
  /// step must not rise until this delay elapses. Honored as a short blocking delay in `set_direction`.
  dir_setup_us: u32,
  /// A blocking delay source for the `$29` setup hold. Cheap to hold (zero-sized) and avoids reaching for
  /// the lower-level ROM delay; the busy-wait runs on the dedicated core where blocking is intended.
  delay: Delay,
  /// The direction last latched onto the DIR outputs, or `None` before the first latch. The `$29` setup
  /// delay is applied only when [`set_direction`](RmtStepSink::set_direction) actually CHANGES the latched
  /// direction (Finding #9): a run of same-direction blocks re-drives the identical GPIO levels with no
  /// real setup transition, so paying the delay every block would needlessly stall motion.
  last_dir: Option<DirState>,
  /// Scratch per-channel PulseCode buffer reused across bursts to keep the sink allocation-free. Sized to
  /// the burst cap plus one for the mandatory `end_marker`. Indexed `[channel][symbol]`.
  scratch: [[PulseCode; MAX_SYMBOLS_PER_BURST + 1]; AXES],
  /// Monotonic count of bursts emitted since boot — captured into the crash breadcrumb on an RMT `wait()` timeout
  /// so the boot dump reports WHICH transmission (since boot) wedged. Wraps harmlessly (diagnostic only).
  burst_seq: u32,
  /// OBSERVE-ONLY (task #22 §15): the SOURCE + axis of the most recent `emit_burst` error, so the silent-truncation
  /// counter at the `run_block` swallow site can attribute the abandoned block to the wait-err / transmit-start /
  /// burst-too-long arm and the RMT channel. Set on each Err-return path of [`emit_burst`](RmtStepSink::emit_burst),
  /// read (and cleared) by [`take_last_error`](RmtStepSink::take_last_error). `None` between errors.
  last_error: Option<TruncationSource>,
}

/// OBSERVE-ONLY (task #22 §15): which `emit_burst` arm abandoned a block, plus the RMT channel/axis it happened on.
/// Recorded on the sink so the `run_block` truncation counter can split [`crate::comms::RUN_BLOCK_TRUNCATED`] by
/// source — the wait-error arm (channel survives → recurring, the prime suspect), the transmit-start arm (channel
/// lost), or a burst-too-long (an encoder/planner bug, a different root).
#[derive(Clone, Copy)]
pub enum TruncationSource {
  /// The RMT `transmit()` START failed for `axis` (`emit_burst`'s transmit arm) — the channel is lost.
  TransmitStart { axis: u8 },
  /// The bounded RMT `wait()` returned an ERROR for `axis` (`emit_burst`'s wait arm) — the channel survives, so this
  /// recurs without a reset; the cleanest fit for a recurring non-deterministic truncation.
  WaitError { axis: u8 },
  /// A burst exceeded `MAX_SYMBOLS_PER_BURST` — not a transient RMT error; an encoder/planner bug. `axis` is the
  /// (nominal) channel the over-long burst was being built for; the cap is per-burst, not per-axis.
  BurstTooLong { axis: u8 },
}

impl RmtStepSink {
  /// Build the sink from the three already-configured RMT TX channels and the three DIR outputs, plus the
  /// step-pulse / direction-setup timing taken from the [`MotionConfig`]. The channels must be configured
  /// at the same tick rate the generator's `MotionConfig.tick_hz` assumes (see [`init`]).
  pub fn new(channels: [AxisChannel; AXES], dir: [Output<'static>; AXES], config: &MotionConfig, dir_setup_us: u32) -> Self {
    RmtStepSink {
      channels: channels.map(Some),
      dir,
      step_pulse_ticks: config.step_pulse_ticks,
      dir_setup_us,
      delay: Delay::new(),
      last_dir: None,
      scratch: [[PulseCode::end_marker(); MAX_SYMBOLS_PER_BURST + 1]; AXES],
      burst_seq: 0,
      last_error: None,
    }
  }

  /// OBSERVE-ONLY (task #22 §15): take + clear the most recent `emit_burst` error source, for the `run_block`
  /// truncation counter to attribute an abandoned block. Returns `None` if no error was recorded since the last take.
  fn take_last_error(&mut self) -> Option<TruncationSource> {
    self.last_error.take()
  }

  /// Encode one channel's burst into its scratch buffer: one PulseCode per [`StepEvent`], then the mandatory
  /// `end_marker`. A stepping axis gets a HIGH(`$0`)/LOW(`period − $0`) pulse; a silent axis gets the period
  /// split into TWO non-zero LOW sub-intervals so all three channels stay the same length and sample-aligned.
  /// Returns the number of symbols written (events + 1 for the end marker).
  ///
  /// ## Why the silent symbol is two LOW halves, not `LOW(period) / LOW(0)` (Finding #1, the showstopper)
  /// An RMT pulse code with EITHER length field zero is an end marker (`is_end_marker() == length1()==0 ||
  /// length2()==0`), and the hardware STOPS transmission at the first end marker. A silent axis is idle on
  /// tick 0 of any coordinated (Bresenham) move, so a `LOW(period) / LOW(0)` symbol would terminate that
  /// channel immediately and drop every remaining step on the subordinate axis. We therefore split the period
  /// into two non-zero halves via [`silent_symbol_halves`], guaranteeing neither field is zero. The split is
  /// host-tested in firmware-core. The stepping symbol's halves are both non-zero too: `$0 ≥ 1` and the
  /// generator clamps the period to `[min_period, RMT_MAX_FIELD_LEN + $0]` so `period − $0 ≥ min_low ≥ 1`
  /// and `period − $0 ≤ RMT_MAX_FIELD_LEN`; `new_clamped` is a final defensive guard on the field width.
  fn encode_channel(&mut self, axis: usize, ticks: &[StepEvent]) -> usize {
    let pulse = self.step_pulse_ticks;
    for (slot, event) in self.scratch[axis].iter_mut().zip(ticks.iter()) {
      let period = event.period_ticks.max(pulse + 1);
      *slot = if event.step[axis] {
        // A real step: HIGH for `$0` ticks, then LOW for the remainder of the period. Both halves are
        // non-zero (see the method doc), so this is never an accidental end marker.
        let high = clamp_len(pulse);
        let low = clamp_len(period - pulse);
        PulseCode::new_clamped(Level::High, high, Level::Low, low)
      } else {
        // A silent tick: stay LOW for the whole period, but split across TWO non-zero halves so the symbol
        // is never an end marker (Finding #1). The channel advances in lock-step with the stepping channels
        // without emitting an edge.
        let (a, b) = silent_symbol_halves(period);
        PulseCode::new_clamped(Level::Low, clamp_len(a), Level::Low, clamp_len(b))
      };
    }
    // Terminate the buffer with the explicit end marker; this is the ONLY symbol that may carry a zero
    // length, and it must be last so transmission stops exactly at the end of the encoded ticks.
    self.scratch[axis][ticks.len()] = PulseCode::end_marker();
    ticks.len() + 1
  }
}

/// The RMT-adjacent probe digital input (DOC-09, Phase C): a single GPIO (PROBE, GPIO21 — see [`init_probe`])
/// read as the raw electrical level. The `$6` invert is applied by the host-tested
/// [`probe_triggered`](firmware_core::hal_traits::probe_triggered), so this carries no settings knowledge — it
/// just reports the pin level. `$19` (pull-up disable) is honored at pin config in [`init_probe`].
pub struct RmtProbeInput {
  /// The PROBE input pin, configured with (or without, per `$19`) the internal pull-up at bring-up.
  pin: Input<'static>,
}

impl RmtProbeInput {
  /// Wrap a configured probe input pin.
  pub fn new(pin: Input<'static>) -> Self {
    RmtProbeInput { pin }
  }
}

impl ProbeInput for RmtProbeInput {
  /// The raw electrical level of the probe pin: `true` = high. The trigger decision (with `$6` invert) is made by
  /// [`probe_triggered`](firmware_core::hal_traits::probe_triggered) in the probe cycle, not here.
  fn is_high(&self) -> bool {
    self.pin.is_high()
  }
}

/// A limit-switch digital input (DOC-06): one GPIO (X/Y/Z limit = GPIO10/11/12, DOC-00 manifest) read as the
/// raw electrical level. Mirrors [`RmtProbeInput`] exactly — the `$5` invert is applied by the host-tested
/// [`limit_triggered`](firmware_core::hal_traits::limit_triggered), so this carries no settings knowledge. The
/// internal pull-up is always enabled (the NC fail-safe depends on it): an intact closed switch grounds the pin
/// LOW, and an open switch or a broken wire lets the pull-up raise it HIGH = triggered. The homing seek/locate
/// walker samples this between single-tick bursts; the bin's limit ISR turns its rising edge into the
/// [`LIMIT_TRIGGERED`](crate::comms::LIMIT_TRIGGERED) signal for the hard-limit path.
pub struct RmtLimitInput {
  /// The limit input pin, configured with the internal pull-up at bring-up (see [`init_limits`]).
  pin: Input<'static>,
}

impl RmtLimitInput {
  /// Wrap a configured limit input pin.
  pub fn new(pin: Input<'static>) -> Self {
    RmtLimitInput { pin }
  }

  /// Await a RISING edge on this limit pin (DOC-06 / research finding #14). esp-hal's `wait_for_rising_edge`
  /// arms the GPIO rising-edge interrupt for this pad and registers an embassy waker, so this is the genuine
  /// interrupt-driven edge wait (no hand-written ISR, no polling) — the executor parks until the hardware edge
  /// fires. Under the NC fail-safe wiring a closed switch holds the pin LOW; opening it (a trip) or a broken
  /// wire lets the pull-up raise it HIGH, which is exactly the rising edge awaited here.
  async fn wait_for_rising_edge(&mut self) {
    self.pin.wait_for_rising_edge().await;
  }
}

impl DigitalIn for RmtLimitInput {
  /// The raw electrical level of the limit pin: `true` = high. The trigger decision (with the `$5` invert and
  /// the NC fail-safe sense) is made by [`limit_triggered`](firmware_core::hal_traits::limit_triggered) in the
  /// homing/hard-limit path, not here — this reader is deliberately invert-agnostic.
  fn is_high(&self) -> bool {
    self.pin.is_high()
  }
}

/// Clamp an RMT pulse length (in ticks) into the 15-bit field. The segment generator already keeps periods
/// well under the limit, so this only ever acts defensively on a degenerate config.
fn clamp_len(ticks: u32) -> u16 {
  if ticks > PulseCode::MAX_LEN as u32 {
    PulseCode::MAX_LEN
  } else {
    ticks as u16
  }
}

impl StepSink for RmtStepSink {
  /// Latch the per-axis DIR outputs, then honor the `$29` direction-setup delay before the first following
  /// step may rise — but ONLY when the direction actually changed (Finding #9). Because the sink method is
  /// synchronous (the generator calls it inline), the delay is a short blocking busy-spin rather than an
  /// await — a few microseconds on the dedicated core, negligible, and now skipped entirely across a run of
  /// same-direction blocks where re-driving the identical GPIO levels needs no setup transition.
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
    let changed = self.last_dir != Some(dir);
    for axis in 0..AXES {
      // `true` is the positive (increasing-step) direction → drive the DIR GPIO high; `false` → low. The
      // physical polarity is set by wiring; the planner/generator agree on this sign convention (DOC-02).
      self.dir[axis].set_level(if dir.dir[axis] { Level::High } else { Level::Low });
    }
    self.last_dir = Some(dir);
    if changed && self.dir_setup_us > 0 {
      // Hold off the first step for the `$29` setup delay, only on a real direction change. A blocking
      // busy-delay is correct here: the sink method is synchronous (the generator calls it inline) so it
      // cannot await, and core 1 is dedicated to motion, so a few-microsecond spin (now only when DIR
      // genuinely flips, not every block) is negligible.
      self.delay.delay_micros(self.dir_setup_us);
    }
    Ok(())
  }

  /// Emit one synchronized burst across the three channels: encode each channel's PulseCode array, start
  /// all three transmits FIRST so the hardware runs them concurrently and sample-aligned, then blocking-wait
  /// all three before returning. A burst longer than the cap is rejected (the generator never emits one).
  fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
    if ticks.len() > MAX_SYMBOLS_PER_BURST {
      // OBSERVE-ONLY (§15): record the source so the run_block truncation counter can split it out (encoder/planner
      // bug, not a transient RMT error). No axis is meaningful yet (encoding hasn't begun); record 0.
      self.last_error = Some(TruncationSource::BurstTooLong { axis: 0 });
      return Err(StepError::BurstTooLong);
    }
    if ticks.is_empty() {
      return Ok(());
    }
    // Liveness beat (diagnostic): advance the core-1 progress counter per BURST as well as per loop turn, so a long
    // single block (seconds of bursts without returning to the drain loop) still shows core 1 as ADVANCING to the
    // core-0 watchdog sampler — otherwise a legitimate long move would read as a false "stall". `Relaxed` single
    // store; see [`MOTION_LIVENESS`](crate::comms::MOTION_LIVENESS). A frozen counter mid-burst now unambiguously
    // means core 1 wedged inside the RMT transmit/wait below — exactly the suspected lockup site.
    MOTION_LIVENESS.store(MOTION_LIVENESS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
    // Count this transmission so an RMT `wait()` timeout can record WHICH burst since boot wedged.
    self.burst_seq = self.burst_seq.wrapping_add(1);
    // Encode all three channels into their scratch buffers up front; `len` is the same for every channel
    // (events + end marker), keeping the three transmits identical in length.
    let len = self.encode_channel(0, ticks);
    let _ = self.encode_channel(1, ticks);
    let _ = self.encode_channel(2, ticks);
    // Burst boundary: `events` is the per-tick step count, `symbols` = events + 1 end marker (= the slice length
    // transmitted to each RMT channel). If "emit_burst" prints but a following "wait ok" for some axis never
    // does, THAT axis's blocking `wait()` is spinning forever (the RMT TX-END never fired) — the RMT path stall.
    mtrace!("motion: emit_burst (events={=usize}, symbols={=usize})", ticks.len(), len);
    crate::crash::record_stage(crate::crash::Stage::EmitBurst, 0);

    // Start all three transmits before waiting any, so the three channels fire together. `transmit`
    // consumes the channel; we take it out of its slot and restore it from the transaction's `wait()`.
    let mut txns: [Option<_>; AXES] = [None, None, None, None];
    for (axis, slot) in txns.iter_mut().enumerate() {
      // The channel is always `Some` here (only this method takes it, and it restores it before returning).
      // A missing channel would be an internal invariant break; treat it as a transport failure rather than
      // unwrapping, per the no-`unwrap` rule.
      let Some(channel) = self.channels[axis].take() else {
        // OBSERVE-ONLY (§15): an internal-invariant missing channel — attribute to the transmit-start arm for `axis`.
        self.last_error = Some(TruncationSource::TransmitStart { axis: axis as u8 });
        return Err(StepError::Transport);
      };
      // Breadcrumb: about to start this axis's RMT transmit. A reboot frozen here pins the wedge to the transmit
      // START of this channel (rarer than a wait wedge, but distinguishable). `axis` is small; the cast is exact.
      crate::crash::record_stage(crate::crash::Stage::AxisTransmit, axis as u8);
      match channel.transmit(&self.scratch[axis][..len]) {
        Ok(txn) => {
          mtrace!("motion: axis {=usize} transmit Ok", axis);
          *slot = Some(txn);
        }
        Err(_) => {
          mtrace!("motion: axis {=usize} transmit Err -> abandoning block", axis);
          // `Channel::transmit` takes the channel BY VALUE and, on the start error path (esp-hal 1.0.0),
          // returns only the `Error` — the channel is moved in and not handed back, so this axis's channel
          // is genuinely lost and its `Option` slot stays `None`; that axis cannot transmit again. This is a
          // hardware fault (a mis-sized buffer / missing end marker is excluded by construction here), so it
          // should escalate to an ALARM that re-inits the RMT peripheral. The executor has no alarm state
          // yet (Stage 1), so for now we surface a transport error (which abandons the current block) and the
          // channel remains down until reboot. TODO(DOC-06 alarm path): on this error raise an alarm and
          // re-run `motion::init`'s RMT bring-up to reclaim the channel. The previous comment claiming "the
          // next reset re-inits RMT" was FALSE — nothing currently re-inits RMT — and is corrected here.
          // OBSERVE-ONLY (§15): attribute this abandoned block to the transmit-start arm for `axis` (channel lost).
          self.last_error = Some(TruncationSource::TransmitStart { axis: axis as u8 });
          return Err(StepError::Transport);
        }
      }
    }

    // Now complete every channel with a BOUNDED poll-loop (was an unbounded blocking `wait()`). The hardware ran
    // the channels concurrently; polling them in sequence only spins until the longest finishes. `TxTransaction::
    // poll()` is non-blocking and returns `true` on completion (End or Error), after which `wait()` returns
    // immediately and hands the channel back — so the HAPPY PATH is byte-for-byte the same busy-poll-then-recover
    // the old `wait()` did, with no extra awaits and no perturbation to step timing. The ONLY new behavior is the
    // TIMEOUT branch: if a channel's TX-END never fires (the confirmed `axis0:wait_begin` hang), the poll-loop
    // gives up after `RMT_WAIT_TIMEOUT_CYCLES`, snapshots the channel's RMT hardware registers into the breadcrumb
    // (so the next boot's `[MSG:CRASH ...]` shows whether TX-END was actually set), drops the transaction (which on
    // the S3 does an immediate `stop_tx` — `rmt_has_tx_immediate_stop` — with no drop-hang), and forces a software
    // reset so the breadcrumb is deterministically read on the next boot.
    let mut result = Ok(());
    for (axis, slot) in txns.iter_mut().enumerate() {
      if let Some(mut txn) = slot.take() {
        mtrace!("motion: axis {=usize} wait begin", axis);
        // Breadcrumb: about to wait on this axis's RMT TX-END — the prime core-1-wedge suspect. A reboot frozen at
        // `axisN:wait_begin` means channel N's TX-END never fired; the RMT-hang capture below records WHY.
        crate::crash::record_stage(crate::crash::Stage::AxisWaitBegin, axis as u8);
        // Bound the wait by the Xtensa CPU CYCLE COUNTER, NOT embassy `Instant`. This loop is a non-yielding
        // busy-spin on the high-priority core-1 InterruptExecutor; spinning here masks the timer interrupt that
        // advances esp-rtos/embassy time, so `Instant::now()` FREEZES mid-spin and an Instant deadline never trips
        // (the first cut at this bug — the RWDT caught the hang instead). `get_cycle_count()` is a per-core CCOUNT
        // read that increments every CPU cycle regardless of interrupts, so it always advances here.
        let start = esp_hal::xtensa_lx::timer::get_cycle_count();
        // Poll until done or the cycle budget elapses. `poll()` is the same volatile status read the old `wait()`
        // spun on, so a completing burst exits here in the same number of reads — no slower on the happy path.
        let timed_out = loop {
          if txn.poll() {
            break false;
          }
          if esp_hal::xtensa_lx::timer::get_cycle_count().wrapping_sub(start) >= RMT_WAIT_TIMEOUT_CYCLES {
            break true;
          }
        };
        if timed_out {
          // THE HANG. Capture channel `axis`'s RMT hardware state into the breadcrumb FIRST (while the channel is
          // still in its hung state — before stop_tx perturbs it), then DETERMINISTICALLY reset. `len` is this
          // burst's symbol count (events + end marker).
          mtrace!("motion: axis {=usize} wait TIMEOUT -> capturing RMT state + resetting", axis);
          // Bump the monotonic RMT-wait-timeout count (the RMT path STILL resets on this first timeout — unchanged).
          // Carrying the count lets the boot dump POSITIVELY exclude the RMT theory for the §11 drumbeat: a captured
          // `usbtx:` discriminator with `n>=K` alongside `rmt_to=0` proves the drumbeat was the USB-TX path, not
          // this RMT path firing repeatedly (which it cannot — one timeout here = one reset).
          crate::crash::bump_rmt_wait_timeout();
          capture_rmt_hang(axis as u8, len as u16, self.burst_seq);
          // Drop the transaction so the S3's immediate `stop_tx` halts the runaway channel (no drop-hang, since
          // `rmt_has_tx_immediate_stop`), then force a full software reset. We reset DIRECTLY rather than abandoning
          // the channel and falling through to the watchdog because the post-abort state is ambiguous — the
          // executor would resume erroring fast (re-advancing `MOTION_LIVENESS`), so the core-1-stall watchdog might
          // NOT fire and the captured breadcrumb might never be read. A software reset (`RTC_CNTL_SW_SYS_RST` =
          // `CoreSw`) preserves the RTC_FAST breadcrumb (it does not reset the RTC domain) and is classified as a
          // fault reset by `main`'s `reset_was_watchdog_or_fault`, so the next boot reads and emits the `[MSG:CRASH
          // rmt0: ...]` line. This is the FRONT HALF of the eventual timeout-backstop; for now it is purely the
          // diagnostic reset (a real backstop would feed-hold + ALARM + require re-home — DOC-06, deferred).
          drop(txn);
          esp_hal::system::software_reset();
        }
        // Completed: `wait()` returns immediately now that `poll()` reported done, handing the channel back.
        match txn.wait() {
          Ok(channel) => {
            mtrace!("motion: axis {=usize} wait ok", axis);
            crate::crash::record_stage(crate::crash::Stage::AxisWaitDone, axis as u8);
            self.channels[axis] = Some(channel);
          }
          Err((_, channel)) => {
            mtrace!("motion: axis {=usize} wait err", axis);
            crate::crash::record_stage(crate::crash::Stage::AxisWaitDone, axis as u8);
            self.channels[axis] = Some(channel);
            // OBSERVE-ONLY (§15): the RMT wait() ERROR arm — the prime recurring-truncation suspect (channel SURVIVES,
            // so it recurs without a reset). Attribute to this `axis`. If several axes err in one burst the last wins
            // — acceptable for the diagnostic (the run_block counter still counts ONE truncation for the burst).
            self.last_error = Some(TruncationSource::WaitError { axis: axis as u8 });
            result = Err(StepError::Transport);
          }
        }
      }
    }
    result
  }
}

/// How long the bounded RMT `wait()` poll-loop ([`RmtStepSink::emit_burst`]) spins for a channel's TX-END before
/// declaring a hang, capturing the RMT hardware state, and abandoning the burst — expressed in CPU CYCLES, because
/// the loop must time itself off the cycle counter (embassy `Instant` freezes in this busy-spin; see the loop).
/// 480M cycles = 2 s at the S3's 240 MHz `CpuClock::max()` (set in `main`). Well above the ~1.5 s worst-case
/// LEGITIMATE single burst, below the 8 s RWDT, and within one u32 CCOUNT wrap (~17.9 s) so `wrapping_sub` is
/// exact. The exact wall-clock is non-critical — anything between the ~tens-of-µs legit burst and the 8 s RWDT
/// works — so even a clock-frequency mismatch stays safely in range.
const RMT_WAIT_TIMEOUT_CYCLES: u32 = 480_000_000;

/// Snapshot RMT channel `axis`'s hardware status registers into the crash breadcrumb at a `wait()` timeout (the
/// hang), so the next boot's `[MSG:CRASH ...]` reports the BIFURCATING fact: was TX-END actually asserted (the
/// transmission finished but our wait missed it) or not (it genuinely never completed)? All reads are plain,
/// side-effect-free volatile loads of the memory-mapped RMT register block (`int_raw`/`int_st` are raw,
/// non-clearing status; `ch_tx_status`/`ch_tx_conf0` are plain config/status) — verified against the installed
/// esp-hal 1.1.1 / esp32s3 PAC. No `unsafe` at the call site (`RMT::regs()` wraps it); safe from the core-1
/// InterruptExecutor (a peripheral register read needs no lock). The interrupt-field channel index is `u8`; the
/// `ch_tx_*(usize)` register index is `usize` — matched here exactly as esp-hal does.
fn capture_rmt_hang(axis: u8, nsym: u16, burst_seq: u32) {
  let rmt = esp_hal::peripherals::RMT::regs();
  let int_raw_r = rmt.int_raw().read();
  let int_st_r = rmt.int_st().read();
  let hang = crate::crash::RmtHang {
    axis,
    tx_end: int_raw_r.ch_tx_end(axis).bit(),
    tx_thr: int_raw_r.ch_tx_thr_event(axis).bit(),
    tx_err: int_raw_r.ch_tx_err(axis).bit(),
    nsym,
    int_raw: int_raw_r.bits(),
    int_st: int_st_r.bits(),
    tx_status: rmt.ch_tx_status(axis as usize).read().bits(),
    tx_conf0: rmt.ch_tx_conf0(axis as usize).read().bits(),
    burst_seq,
  };
  crate::crash::record_rmt_hang(&hang);
}

/// The core-1 motion executor (DOC-01 / DOC-02): the single task on the high-priority interrupt executor.
/// It drains the shared [`PLANNER`] queue and realizes each block as RMT step pulses through `sink`, tracks
/// the live machine position, and honors feed-hold / soft-reset at block boundaries. It replaces the
/// Stage-1 `block_drain_stub`: where the stub only paced time and published the planner's *planned*
/// position, this publishes the true *live* (interpolated) MPos straight from the executed step counter.
///
/// ## Loop shape
/// 1. Service a pending soft reset (dedicated [`MOTION_RESET`] / [`MOTION_RESET_PENDING`], NOT the shared
///    `SOFT_RESET` — an embassy `Signal` wakes one waiter, so the executor needs its own — Finding #3):
///    RETAIN the live position (Change A) — a `0x18` abort keeps MPos at the (suspect, mid-move) stop point so
///    `$X` unlocks there, matching grbl; the consumer's `reset_pipeline` syncs the rebuilt planner to it.
/// 2. At each block boundary (never mid-burst), honor the hold LEVEL: while [`HOLD_REQUESTED`](crate::comms::
///    HOLD_REQUESTED) is set, PARK — acknowledge the park via [`MOTION_PARKED`](crate::comms::MOTION_PARKED) so
///    the consumer's quiesce primitive observes a real "parked" fact (Finding #11/#3), then wait on
///    [`HOLD_WAKE`](crate::comms::HOLD_WAKE) and re-read the level (a `~` resume / `$SLP`-then-reset clears it).
///    The level is authoritative and re-read every wake, so a hold can never be missed or drained-as-stale, and
///    a legitimate cycle-start can never be discarded (Finding #11). A reset also breaks the park.
/// 3. Pop the next block under the planner lock, peeking the following block's `entry_speed_sq` as this
///    block's `exit_speed_sq` (0.0 when the queue holds only this block, so it stops at rest). Release the
///    lock BEFORE any RMT transmit — the planner mutex is never held across step emission.
/// 4. If the queue was empty, honor the hold level FIRST (so a hold latched while idle still parks and is
///    acknowledged — Finding #2), then await [`BLOCK_AVAILABLE`](crate::comms::BLOCK_AVAILABLE) rather than
///    polling, racing [`MOTION_RESET`] so a `0x18` wakes the executor promptly, [`HOLD_WAKE`] so a hold latched
///    while waiting parks promptly, AND [`PROBE_REQUEST`](crate::comms::PROBE_REQUEST) so a `G38.x` probe runs
///    once the queue has fully drained. Servicing the probe only in the empty-queue branch keeps it strictly
///    AFTER any blocks queued before it (the consumer flushes look-ahead when issuing a probe, so no NEW blocks
///    follow it) — the probe is a synchronized boundary, never reordered.
/// 5. Run the block synchronously through the [`SegmentGenerator`], advancing the live [`StepCounter`] and
///    publishing the live step position into the [`LIVE_POSITION`] atomics after EACH burst (so MPos is live
///    within a long block, not frozen until the block ends — Finding #5). A reset pending between bursts
///    aborts the block early via a sink error; the live position is then RETAINED (Change A) at the last
///    published step position, not zeroed — the next loop iteration's reset service re-publishes the retained value.
/// 6. A probe ([`run_probe`]) walks a probe block one tick per burst, sampling the probe input between every
///    step and stopping on the expected edge, then publishes the latched stop position + outcome via
///    [`PROBE_RESULT`](crate::comms::PROBE_RESULT) for the consumer to turn into `[PRB:]` / position sync / alarm.
pub async fn run(
  sink: &mut RmtStepSink,
  probe: &mut RmtProbeInput,
  limits: &mut [RmtLimitInput; AXES],
  config: MotionConfig,
  max_rate_mm_min: [f32; AXES],
) -> ! {
  let generator = SegmentGenerator::new(config);
  let prober = ProbeStepper::new(config);
  let mut counter = StepCounter::new();
  // The most-restrictive axis max-rate in mm/s — the conservative ceiling the Phase-E feed-override scale-up is
  // clamped to (squared, since the generator reasons in v²), so a boosted feed never exceeds `$110-112`.
  let max_rate_mm_s = min_max_rate_mm_s(&max_rate_mm_min);
  // A steady idle-cadence ticker for the `Pn:` limit-level publish (DOC-06 / DOC-08). It races the empty-queue
  // `select` below so a switch RELEASE while the machine sits idle is reflected within one tick: the idle limit
  // detector only ever wakes on a rising edge (a press), never on a release, so without this periodic level
  // sample a released switch would leave a stale `Pn:X/Y/Z` latched forever. The ticker is created ONCE here (so
  // its period stays steady across loop turns) and only ever advanced from the empty-queue idle arm — it adds NO
  // work to the real-time burst path, which never touches it. ~50 ms keeps the reported state well inside the
  // ~100 ms freshness budget while costing only three GPIO reads + one atomic store per tick at idle.
  let mut limit_ticker = Ticker::every(Duration::from_millis(50));
  // The per-axis hard-limit ARMING state (DOC-06): "was this switch triggered at the previous `check_hard_limits`
  // sample". The alarm is EDGE-armed off this — `hard_limit_alarm_armed` fires only on a not-triggered ->
  // triggered transition — so a switch the machine is merely PARKED on (e.g. left engaged after an aborted `$H`
  // seek, which performs no pull-off) never re-fires `ALARM:1`. Seeded to the SETTLED levels at the arming reset
  // points — a soft reset (below) and after every homing cycle ([`run_homing`]) — so a held switch is "already
  // known, not a new trip", while a genuine new over-travel during later motion still alarms. The executor is the
  // single owner of this state; the consumer owns the control state it feeds.
  let mut limit_armed: [bool; AXES] = sample_limit_triggered(limits);
  // The executor task is alive and entering its drain loop on core 1. If THIS line never appears over RTT, the
  // core-1 InterruptExecutor / second-core bring-up never reached the task (look at main's start_second_core).
  mtrace!("motion: executor loop entered");
  crate::crash::record_stage(crate::crash::Stage::LoopEntered, 0);
  loop {
    // Liveness beat (diagnostic), per drain-loop turn: bump the cross-core progress counter so the core-0
    // watchdog-feed task can tell whether core 1 is still scheduling at the block level (this site covers the
    // empty-queue idle wait + block-pop cadence; `emit_burst` bumps it again per burst so a long single block also
    // reads as advancing). `Relaxed` + `wrapping_add` is a single native store with no synchronization cost; a wrap
    // is harmless (the sampler tests for inequality, not magnitude). It NEVER gates the watchdog feed, so it cannot
    // starve the dog if it misbehaves.
    MOTION_LIVENESS.store(MOTION_LIVENESS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);

    // Service a pending soft reset at the top of the loop: RETAIN the live position (Change A) rather than zero
    // it, matching grbl — a `0x18` abort keeps MPos so `$X` unlocks at the same coordinates. The consumer's
    // `reset_pipeline` rebuilds the planner and SYNCS it to this retained position, so the two stay consistent.
    // `MOTION_RESET_PENDING` is the poll-able flag the sink also tests mid-block; clearing it AND draining the
    // `MOTION_RESET` signal here keeps the two in sync so a reset is serviced exactly once.
    if MOTION_RESET_PENDING.swap(false, Ordering::AcqRel) {
      MOTION_RESET.try_take();
      retain_live_position(&counter);
      // Re-seed the hard-limit arming to the settled levels: a `0x18` after an aborted `$H` leaves a switch
      // parked-engaged, and the warm reset returns to Idle / boot-lock, so that held level is "already known" —
      // re-seeding here means the first post-reset block boundary sees no FRESH edge from it (the principled fix
      // for the `error:9` re-lock). A genuinely new over-travel during later motion still alarms on its own edge.
      limit_armed = sample_limit_triggered(limits);
    }

    // Honor the hold LEVEL at the block boundary (never mid-block — a burst in flight is never split, matching
    // DOC-02): while `HOLD_REQUESTED` is set, PARK until it clears. `park_on_hold` pulses `MOTION_PARKED` so the
    // consumer's quiesce primitive observes a real "parked" acknowledgment (Finding #11/#3), then waits on
    // `HOLD_WAKE` and re-reads the LEVEL — a `~` resume (or a `$SLP`-then-reset) clears it, and the LEVEL being
    // authoritative is what makes a hold impossible to miss and a legitimate resume impossible to discard. A
    // pending soft reset breaks the park; loop back to service it at the top.
    if HOLD_REQUESTED.load(Ordering::Acquire) {
      // A stuck `HOLD_REQUESTED` would park here forever, leaving any queued block unpopped (so `?` shows `Run`
      // from the `queued` term and `FS:0` because the feed is never published). If this is the last trace line,
      // the hold level is wedged set, not the RMT path.
      mtrace!("motion: hold requested -> parking");
      park_on_hold().await;
      continue;
    }

    // Pop the next block and peek the one after it for the exit speed, all under one short lock, releasing
    // it before any transmit so the consumer can keep enqueuing while this block executes. If "lock acquired"
    // never follows "popping block", the `PLANNER` mutex is held across an await on core 0 (cross-core lock
    // contention), NOT the RMT path.
    mtrace!("motion: popping block (taking PLANNER lock)");
    let popped = {
      let mut guard = PLANNER.lock().await;
      mtrace!("motion: PLANNER lock acquired");
      crate::crash::record_stage(crate::crash::Stage::LockAcquired, 0);
      match guard.as_mut() {
        Some(planner) => take_block(planner),
        None => None,
      }
    };

    match popped {
      Some((block, exit_speed_sq)) => {
        // A queue slot just freed: wake the consumer's arc-drive loop so an in-progress over-subdivided arc
        // refills PROACTIVELY (Bug 4) — the instant a slot opens, while THIS block executes — instead of only
        // after the consumer's poll interval. Keeping the buffer topped up stops the executor draining to the
        // look-ahead's forced-stop chunk tail, so a large arc stays continuous across chunk boundaries. Raised
        // unconditionally on every pop; it is consumed only while an arc is pending and is otherwise a no-op.
        SLOT_FREED.signal(());
        // A block was popped: trace its dominant-axis event count and rapid flag so the log shows the block
        // actually reached the executor. If this prints but "run_block returned" never does, the stall is
        // INSIDE run_block (the generator + RMT emit path).
        mtrace!(
          "motion: block popped (events={=u32}, rapid={=bool}, exit_sq={=f32})",
          block.step_event_count,
          block.rapid,
          exit_speed_sq
        );
        crate::crash::record_stage(crate::crash::Stage::BlockPopped, 0);
        // Publish "a block is in flight" so `status_responder` reports `Run` for the whole duration of this
        // block — including the tail after the queue drained but the last burst is still emitting. Cleared
        // when the block finishes (or aborts). `AcqRel`/`Acquire` publishes the flag to the core-0 reporter.
        EXECUTOR_RUNNING.store(true, Ordering::Release);
        mtrace!("motion: EXECUTOR_RUNNING set -> entering run_block");
        run_block(&generator, &block, exit_speed_sq, max_rate_mm_s, sink, &mut counter);
        mtrace!("motion: run_block returned");
        // OBSERVE-ONLY probe (task #22, gcode-chunk-skip): a motion block was popped and run to return — the "EXEC"
        // leg of the lines/acks/execs cross-check. Counted here, after `run_block`, so it tracks blocks the executor
        // actually processed. (A soft-reset/hold can abort a block mid-run and still return here — rare on a clean
        // stream and a reset discards those acks host-side anyway — so `BLOCKS_EXECUTED` is a per-run DELTA signal,
        // cross-checked against `ACKS_EMITTED`, not an exact equality. `ACKS_EMITTED > BLOCKS_EXECUTED` growing
        // during a skipping run would point at a motion-side block drop; equal growth exonerates the motion path.)
        crate::comms::BLOCKS_EXECUTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        // Motion stopped (block finished or aborted): zero the published programmed feed so `FS:` reads 0 while
        // idle. The next block republishes it. The override-scaled REALIZED feed is computed by the reporter.
        LIVE_PROGRAMMED_FEED_MM_MIN.store(0, Ordering::Release);
        EXECUTOR_RUNNING.store(false, Ordering::Release);
        // DOC-06 hard-limit check at the block BOUNDARY: sample the limit switches now that a block finished and
        // raise the alarm (halting the queue) if `$21` is on and a switch tripped during normal motion. Sampling
        // at the boundary (not mid-burst — a burst is never split) bounds detection to one block, which is
        // adequate for an over-travel safety abort on short PCB-milling blocks. The shared-pin rule (no alarm
        // while homing) is enforced inside `check_hard_limits`. The arming state is threaded so a switch the
        // machine was already parked on (held level) never re-fires — only a FRESH over-travel edge alarms.
        check_hard_limits(limits, &mut limit_armed);
      }
      // Queue empty: await a freshly enqueued block instead of polling, racing the dedicated motion reset (so a
      // reset while idle is observed promptly), the hold-level wake (so a hold latched while idle is honored —
      // the executor loops back to `park_on_hold` rather than running the next block — Finding #2), AND a probe
      // request (so a `G38.x` issued while the executor is idle — the common case, since the consumer flushes
      // look-ahead before a probe — wakes it immediately). The probe request `wait()` CONSUMES the request, so
      // it is run inline here rather than looped back to the top-of-loop `try_take` (which would find it gone).
      None => {
        // Queue empty: about to await a fresh block. If "block popped" never follows a BLOCK_AVAILABLE wake but
        // `?` shows a queued block, the wake/enqueue handshake is racing (block enqueued without signaling, or
        // the signal consumed elsewhere) — not the RMT path.
        mtrace!("motion: queue empty -> awaiting block/reset/hold/probe/home");
        crate::crash::record_stage(crate::crash::Stage::IdleWaiting, 0);
        // Race the existing four idle wakes against a `$H` homing request (DOC-06). Homing, like a probe, is a
        // synchronized boundary serviced only from the empty-queue branch — the consumer flushes look-ahead and
        // blocks on the result, so no block ever follows it out of order. The outer `select` consumes the homing
        // request inline (running the whole cycle) rather than looping back, exactly as the probe arm does.
        let idle = select4(BLOCK_AVAILABLE.wait(), MOTION_RESET.wait(), HOLD_WAKE.wait(), PROBE_REQUEST.wait());
        // Race the four idle wakes against a `$H` homing request, a debounced limit rising-edge trip (so a switch
        // PRESSED while the machine sits idle still raises the hard-limit alarm, DOC-06 / finding #1), AND the
        // limit-level ticker (so a switch RELEASE while idle is reflected in `Pn:` within one tick — the edge wait
        // never fires on release). `wait_for_limit_trip` is the genuine interrupt-driven edge wait + `$26` resample
        // and SIGNALS `LIMIT_TRIGGERED` on a confirmed trip; the ticker only ever PUBLISHES the live level mask.
        // Nested `select`s keep each arm typed.
        let edge_or_home = select(select(idle, HOME_REQUEST.wait()), wait_for_limit_trip(limits));
        match select(edge_or_home, limit_ticker.next()).await {
          Either::First(Either::First(Either::First(Either4::First(())))) => mtrace!("motion: woke on BLOCK_AVAILABLE"),
          Either::First(Either::First(Either::First(Either4::Second(())))) => {
            mtrace!("motion: woke on MOTION_RESET (idle)");
            MOTION_RESET_PENDING.store(false, Ordering::Release);
            // RETAIN the live position on the idle reset path too (Change A): a reset from Idle keeps MPos at the
            // resting position rather than zeroing it, matching the top-of-loop reset service and grbl.
            retain_live_position(&counter);
            // Re-seed the hard-limit arming on the idle reset path too (same invariant as the top-of-loop reset).
            limit_armed = sample_limit_triggered(limits);
          }
          // A hold-level change while idle: loop back so the top-of-loop hold check re-reads the LEVEL and parks
          // if it is set (or simply proceeds if a spurious wake found it clear). Re-reading the level — never
          // acting on the edge — is the Finding #11 invariant.
          Either::First(Either::First(Either::First(Either4::Third(())))) => mtrace!("motion: woke on HOLD_WAKE (idle)"),
          Either::First(Either::First(Either::First(Either4::Fourth(request)))) => {
            run_probe(&prober, &request, probe, sink, &mut counter)
          }
          Either::First(Either::First(Either::Second(config))) => {
            run_homing(&config, limits, sink, &mut counter);
            // SEED the hard-limit arming to the settled post-cycle levels. A homing seek drives INTO the switch and
            // an abort/FAIL leaves the axis parked-engaged with no pull-off, so the level can still be asserted when
            // `HOMING_ACTIVE` clears. Seeding here marks that held level "already known" — the first normal block
            // boundary after the cycle then sees NO fresh edge from it, so the parked switch never latches a stale
            // `ALARM:1` (the principled fix). A real new over-travel during a later move still alarms on its edge.
            limit_armed = sample_limit_triggered(limits);
          }
          // A debounced limit rising-edge trip while idle: sample the switches and raise the hard-limit alarm if
          // due (DOC-06). `wait_for_limit_trip` already applied the `$26` debounce and signalled `LIMIT_TRIGGERED`.
          // `check_hard_limits` also refreshes the published `Pn:` mask from the freshly-sampled levels.
          Either::First(Either::Second(())) => check_hard_limits(limits, &mut limit_armed),
          // The idle limit-level tick: republish the live `Pn:` mask from a level sample so a RELEASE (or a press
          // too subtle to confirm at the debounce, e.g. a switch held without over-travel) is reflected within
          // one tick. Pure publish — no alarm decision here; that stays on the debounced edge path above.
          Either::Second(()) => publish_limit_levels(limits),
        }
      }
    }
  }
}

/// Park the executor on the hold LEVEL until it clears (or a soft reset breaks the park). This is the executor
/// side of the level-based hold/resume protocol (Finding #11): it first PUBLISHES the park acknowledgment via
/// [`MOTION_PARKED`] so the consumer's [`quiesce_executor`](crate::comms::quiesce_executor) primitive observes a
/// real "the executor has parked" fact — closing the race jog-cancel/probe-abort previously had with the
/// `EXECUTOR_RUNNING` clear (Finding #3) — then loops waiting on [`HOLD_WAKE`] and RE-READING the authoritative
/// [`HOLD_REQUESTED`] level each wake. Re-reading the level (never acting on an edge) is what makes a hold
/// impossible to miss and a legitimate cycle-start resume impossible to drain-as-stale.
///
/// `EXECUTOR_RUNNING` is already false here (the boundary check runs only after a block finished or from idle),
/// so the reporter sees the latched control state (`Hold`/`Sleep`) drive the wire state, never a phantom `Run`.
/// A pending soft reset breaks the park immediately: the caller loops back to the top to service the reset
/// (which clears the level and zeroes the live position), so the executor never stays parked across a warm
/// reset (Finding #2).
async fn park_on_hold() {
  // Acknowledge the park exactly once per entry, AFTER the level has been observed set, so a waiter in
  // `quiesce_executor` that raised the level then awaited the ack is released the moment the executor rests.
  MOTION_PARKED.signal(());
  loop {
    // A soft reset latched while parked must win immediately: zero the position and let the caller service it.
    if MOTION_RESET_PENDING.load(Ordering::Acquire) {
      return;
    }
    // The level cleared (a `~` resume or the reset-path clear): leave the park and run the next block.
    if !HOLD_REQUESTED.load(Ordering::Acquire) {
      return;
    }
    // Wait for a level change (or a reset wake) rather than spinning. The level is re-read at the top of the
    // loop after every wake, so a coalesced or spurious `HOLD_WAKE` is harmless.
    match select(HOLD_WAKE.wait(), MOTION_RESET.wait()).await {
      Either::First(()) => {}
      Either::Second(()) => {
        // A reset broke the park: leave `MOTION_RESET_PENDING` set so the top-of-loop reset service runs (it
        // drains `MOTION_RESET` and zeroes the position). Returning here lets the caller `continue` to it.
        return;
      }
    }
  }
}

/// Pop the next block and compute its exit speed from the *following* block's planned entry speed (squared),
/// or `0.0` when this is the only queued block (it must stop at rest). Peeking after the pop reads the new
/// head, which is the block that will run next — exactly the exit-speed semantics the generator expects, and
/// the block whose entry the planner now FREEZES (busy-block protection, Finding #4) so this exit stays valid.
fn take_block(planner: &mut Planner) -> Option<(Block, f32)> {
  let block = planner.pop_block()?;
  let exit_speed_sq = planner.peek_block().map(|next| next.entry_speed_sq).unwrap_or(0.0);
  Some((block, exit_speed_sq))
}

/// Realize one block through the generator while advancing the live step counter, applying the LIVE feed/rapid
/// override (Phase E). The counter is fed in lock-step with the sink: `set_direction` latches the per-axis sign
/// from the block, and each emitted [`StepEvent`] advances the counter and publishes the live position into
/// [`LIVE_POSITION`] per burst — so the live MPos mirrors exactly what the RMT channels emit, updated within the
/// block rather than only at its end. A generator/sink error (including a soft-reset abort tested between bursts)
/// abandons the block; the next status report still reflects the steps published so far.
///
/// The override is read from the shared [`overrides`] HERE, per block, so a `0x91`/`0x9B` arriving while the
/// queue runs takes effect on the very next block without re-planning (grbl applies overrides in the stepper).
/// A G0 rapid scales by the rapid override; a feed/jog move scales by the feed override and is clamped to the
/// axis max-rate ceiling (`max_rate_mm_s`, squared) so a boost never exceeds `$110-112`. The PROGRAMMED nominal
/// feed (rapid max-rate for a G0, feed nominal otherwise) is published into [`LIVE_PROGRAMMED_FEED_MM_MIN`] +
/// [`LIVE_BLOCK_IS_RAPID`] so the reporter renders the override-scaled REALIZED `FS:` feed.
fn run_block(
  generator: &SegmentGenerator,
  block: &Block,
  exit_speed_sq: f32,
  max_rate_mm_s: f32,
  sink: &mut RmtStepSink,
  counter: &mut StepCounter,
) {
  // Read the live overrides once for this block and pick the scale: a rapid (G0) uses the rapid override, a
  // feed/jog move uses the feed override. Both are percentages; the generator takes a fraction.
  let ov = overrides();
  let override_pct = if block.rapid { ov.rapid } else { ov.feed };
  let override_scale = override_pct as f32 / 100.0;
  // The squared max-rate ceiling the scale-up clamps to. A rapid is already at the max-rate and the rapid
  // override only scales DOWN, so it needs no extra ceiling (INFINITY); a feed move clamps to the axis max-rate.
  let max_speed_sq = if block.rapid { f32::INFINITY } else { max_rate_mm_s * max_rate_mm_s };

  // Publish the PROGRAMMED nominal feed (mm/min) and the rapid flag so the reporter can render the realized feed.
  // The block stores nominal speed as mm/s; convert to mm/min for the `FS:` units.
  LIVE_PROGRAMMED_FEED_MM_MIN.store((block.nominal_speed() * 60.0).to_bits(), Ordering::Release);
  LIVE_BLOCK_IS_RAPID.store(block.rapid, Ordering::Relaxed);
  // The programmed feed is now published, so a `?` from here on should show a non-zero `FS:`. If the log shows
  // this line but the wire still reports `FS:0`, the stall is BEFORE this point (the feed was never published) —
  // which means the executor never reached run_block, contradicting an "emit_burst hang" and pointing upstream.
  mtrace!("motion: feed published ({=f32} mm/min) -> running generator", block.nominal_speed() * 60.0);
  crate::crash::record_stage(crate::crash::Stage::FeedPublished, 0);

  // Latch the live counter's direction from the same step signs the generator latches onto the sink, so the
  // counter advances each axis the correct way. A zero-length block never steps, so this is harmless then.
  counter.set_direction(DirState {
    dir: core::array::from_fn(|axis| block.steps[axis] >= 0),
  });
  // The generator returns the tick count or a recoverable error; on error we simply stop emitting this
  // block. The error is not surfaced upward because Stage 1 has no alarm state machine yet (DOC-06/Stage 2);
  // the abandoned block leaves the machine where the last published burst put it, which the live MPos shows.
  //
  // The §15 silent-skip mechanism is precisely THIS swallow — a mid-block `emit_burst` Err abandons the REST of the
  // block's step bursts (a truncated cut). The program would otherwise continue and the host already acked the line at
  // plan time, so the skip is invisible. We CAPTURE the result, COUNT the truncation (split by source + axis), and —
  // task #22 / §15.6 — route a genuine mid-block step-output Transport fault into the ALARM path: a broken step sync
  // loses position certainty on open-loop steppers, so per the grbl lost-step-sync contract (§14.3) the correct
  // response is feed-hold + `ALARM:17` (MotorFault) + require re-home, NEVER silent abandonment or a silent reset. The
  // `tracking` borrow of `sink` ends before we read `sink.take_last_error()`.
  let outcome = {
    let mut tracking = CountingSink::live(sink, counter);
    generator.run_block_scaled(block, exit_speed_sq, override_scale, max_speed_sq, &mut tracking)
  };
  if outcome.is_err() {
    let source = sink.take_last_error();
    record_block_truncation(source);
    if source.is_some() {
      // A REAL mid-block step-output fault (any `emit_burst` Transport arm: a bounded-`wait()` error, a failed
      // `transmit()` start, or a burst-too-long encoder bug) broke the step sync mid-cut. Raise the motion-fault
      // alarm so the consumer halts the program, locks into `ALARM:17`, and forces a re-home. A `None` source is the
      // generator's all-or-nothing `InvalidConfig` (a degenerate config that would reject EVERY block, not a mid-cut
      // step-sync break) — it is counted above but does not raise the per-block motion fault. The alarm is
      // source-agnostic; the faulting arm/axis is already recorded by `record_block_truncation` for the breadcrumb.
      mtrace!("motion: run_block truncated -> MOTION_FAULT (raising ALARM:17)");
      crate::comms::MOTION_FAULT.signal(());
    }
  }
}

/// OBSERVE-ONLY (task #22 §15): record a silently-swallowed `run_block` truncation into the FREE-RUNNING RTC_FAST
/// probe counters (via `crash::bump_run_block_truncated`), split by the `emit_burst` error SOURCE + axis recorded on
/// the sink. Pure counter writes — no behavior change. RTC_FAST (not `.bss`) so the count SURVIVES the K-escape
/// `software_reset` that fires on a `usb_tx` wedge — otherwise a run that wedges+resets would zero the count mid-run
/// (the run-1 confound). A `None` source (the generator erred for a reason other than an `emit_burst` arm — e.g.
/// `InvalidConfig`) still bumps the total so no truncation is lost.
fn record_block_truncation(source: Option<TruncationSource>) {
  let (crash_source, axis) = match source {
    Some(TruncationSource::WaitError { axis }) => (Some(crate::crash::TruncSource::WaitErr), axis),
    Some(TruncationSource::TransmitStart { axis }) => (Some(crate::crash::TruncSource::TxStart), axis),
    Some(TruncationSource::BurstTooLong { axis }) => (Some(crate::crash::TruncSource::BurstTooLong), axis),
    None => (None, 0),
  };
  crate::crash::bump_run_block_truncated(crash_source, axis);
}

/// The most-restrictive (smallest) per-axis max-rate in mm/SECOND, the conservative ceiling the Phase-E feed
/// override scale-up is clamped to. A non-positive / empty set yields `f32::INFINITY` (no clamp) so a degenerate
/// setting cannot pin motion to zero. Computed once in [`run`] from the `$110-112` mm/min rates.
fn min_max_rate_mm_s(max_rate_mm_min: &[f32; AXES]) -> f32 {
  let min_mm_min = max_rate_mm_min
    .iter()
    .copied()
    .filter(|r| r.is_finite() && *r > 0.0)
    .fold(f32::INFINITY, f32::min);
  if min_mm_min.is_finite() {
    min_mm_min / 60.0
  } else {
    f32::INFINITY
  }
}

/// Run one `G38.x` probe cycle (Phase C): build a probe block from the live position to the request `target`,
/// walk it one tick at a time through the [`ProbeStepper`] while WATCHING the probe input between every step, and
/// publish the latched stop position + outcome back to the consumer via [`PROBE_RESULT`].
///
/// The probe is sampled by reading the raw pin level and applying the request's `$6` invert through the
/// host-tested [`probe_triggered`]; the `toward`/away sense picks the stop edge (trigger for a toward probe,
/// release for an away probe). The live [`StepCounter`] advances in lock-step (through the same [`CountingSink`]
/// that publishes the live MPos per burst), so the stop position is published live as the probe moves and the
/// final latched steps are exactly where the probe stopped.
///
/// ## ALARM:4 already-at-edge detection
/// For a TOWARD probe that is ALREADY triggered before the first step, the [`ProbeStepper`] returns a trigger
/// with `steps_emitted == 0`; this surfaces as `already_at_edge` so the consumer maps it to `ALARM:4` (wrong
/// initial state) rather than `ALARM:5`.
///
/// ## Sampling-resolution limit (hardware boundary, DOC-02)
/// The probe is sampled at SINGLE-STEP granularity (between one-tick bursts) — the finest the RMT burst
/// architecture allows, since a burst in flight cannot be preempted. Over-travel past the trigger is bounded by
/// one step plus the in-flight burst's deceleration; the probe feed should be kept low (25-100 mm/min) to bound
/// it, exactly as `docs/tlo-offsets.md` Finding #10 prescribes.
fn run_probe(prober: &ProbeStepper, request: &ProbeRequest, probe: &RmtProbeInput, sink: &mut RmtStepSink, counter: &mut StepCounter) {
  // Publish "running" so `?` reports `Run` (grbl shows `Run`/`Run:2` during a probe) for the cycle's duration.
  EXECUTOR_RUNNING.store(true, Ordering::Release);

  let start = counter.position_steps();
  let block = probe_block_to(start, request.target);
  let config = ProbeConfig { invert: request.invert, pullup_disable: false };

  // The stop predicate: for a TOWARD probe stop when the probe TRIGGERS, for an AWAY probe when it RELEASES.
  // Each sample also publishes the LOGICAL probe-asserted state (after `$6` invert) into [`PROBE_ASSERTED`] so
  // the `Pn:P` status letter reflects the probe during the cycle (Phase E) — this is the one input continuously
  // sampled today; continuous idle sampling of all inputs is a DOC-06 follow-up.
  let toward = request.toward;
  let mut at_stop_edge = || {
    let triggered = probe_triggered(probe.is_high(), &config);
    PROBE_ASSERTED.store(triggered, Ordering::Release);
    if toward { triggered } else { !triggered }
  };

  // Track whether the very first sample was already at the stop edge, so the consumer can distinguish ALARM:4
  // (already-at-edge) from ALARM:5 (no contact). `ProbeStepper` returns `steps_emitted == 0` with a trigger in
  // that case, but we also need the raw position; the counter holds it.
  let mut tracking = CountingSink::live(sink, counter);
  let outcome = prober.run_probe(&block, request.step_period_ticks, &mut at_stop_edge, &mut tracking);

  // The latched stop position is exactly what the counter shows (the CountingSink advanced it per emitted step).
  let stop_steps = counter.position_steps();
  EXECUTOR_RUNNING.store(false, Ordering::Release);

  // Map the stepper outcome to the consumer's success/alarm distinction. A probe that stops at the edge with ZERO
  // steps was ALREADY at its expected stop edge before any motion — grbl's "probe not in the expected initial
  // state" (ALARM:4): it is NOT a valid probe success, because the probe never moved onto the workpiece. We
  // report it as `triggered = false` with `already_at_edge = true` so the consumer raises ALARM:4 (for an
  // alarming mode) instead of acking a degenerate zero-travel "success".
  let (triggered, already_at_edge) = match outcome {
    Ok(o) if o.triggered && o.steps_emitted == 0 => (false, true),
    Ok(o) => (o.triggered, false),
    // A sink/config error abandons the probe; report a no-trigger so the consumer alarms (for an alarming mode)
    // rather than fabricating a success. This mirrors `run_block`'s error tolerance.
    Err(_) => (false, false),
  };
  PROBE_RESULT.signal(ProbeResult { triggered, stop_steps, already_at_edge });
}

/// Run one `$H` homing cycle (DOC-06) on core 1, where the RMT step channels and the limit inputs live. Walks
/// [`HOMING_GROUPS`](firmware_core::homing::HOMING_GROUPS) in order — Z first, then X and Y — running the pure,
/// host-tested [`home_axis`](firmware_core::homing::home_axis) primitive for each axis through a [`CountingSink`]
/// so the live MPos tracks the seek/locate/pull-off motion. On success it SYNCS the live step counter to each
/// axis's post-homing machine-zero position (the value the consumer also pushes into the planner/parser) and
/// publishes the result via [`HOME_RESULT`](crate::comms::HOME_RESULT); a no-contact / sink failure publishes the
/// error so the consumer raises the homing-fail alarm + reset.
///
/// ## X+Y run SEQUENTIALLY here, not concurrently
/// Research finding #10 notes X and Y CAN home concurrently on Galdr's independent RMT channels (each stopping as
/// its own switch latches). This executor is a single synchronous task, and [`home_axis`] is a blocking per-axis
/// walk, so the axes in a group are homed one after another rather than interleaved. That is mechanically valid
/// (grbl supports single-axis homing too) and keeps the cycle simple and verifiable; concurrent X+Y co-motion
/// (for speed / gantry squaring) is a later refinement that would interleave the two channels' single-tick bursts.
///
/// ## Hard limits are suppressed during the cycle (shared-pin rule)
/// The hard-limit alarm path is gated OFF while `MachineState::Home` is active (the limit switches are EXPECTED
/// to trip during homing), and re-armed after — the consumer publishes the `Home` control state for the cycle's
/// duration, so the limit-monitor never raises `ALARM:1` mid-home (research finding #17). This function only
/// emits the seek/locate motion; the control-state gating lives in the consumer + the limit monitor.
fn run_homing(config: &HomingConfig, limits: &mut [RmtLimitInput; AXES], sink: &mut RmtStepSink, counter: &mut StepCounter) {
  // Publish "running" so `?` reports motion for the cycle's duration (the consumer reports `Home` via the control
  // state; this keeps EXECUTOR_RUNNING truthful so a stale `Idle` is never reported mid-cycle).
  EXECUTOR_RUNNING.store(true, Ordering::Release);

  let mut zero_steps = counter.position_steps();
  let mut result: Result<[i32; AXES], HomingError> = Ok(zero_steps);

  // TODO(DOC-06): concurrent intra-group homing. The per-axis independent-RMT-channel design (research finding
  // #10) supports X+Y seeking together, each channel stopping as its OWN switch latches. Doing it would require
  // a new firmware-core primitive that interleaves several axes' single-tick bursts under one loop with a
  // per-axis stop predicate + per-axis phase state — `home_axis` today owns one sink and runs all four phases
  // linearly for ONE axis, so it cannot be driven concurrently without restructuring its borrow model. That is
  // not a contained change, so the within-group axes are homed SEQUENTIALLY here. It is mechanically valid
  // (grbl supports single-axis homing) and keeps the cycle verifiable; concurrency is a speed/squaring refinement.
  'cycle: for group in HOMING_GROUPS {
    for &axis in *group {
      // Split the borrow: `home_axis` needs `&mut sink` (through the CountingSink) and `&limits[axis]`. The
      // CountingSink wraps the step sink + the live counter so the seek/locate/pull-off motion tracks the live
      // position. It is the QUIET variant (finding #5): a homing seek emits thousands of single-step bursts, so
      // publishing per burst would do thousands of 3× atomic stores per seek; we publish at the phase boundary
      // (after each axis completes) below instead, which is ample DRO resolution for a homing cycle.
      let outcome = {
        let mut tracking = CountingSink::quiet(sink, counter);
        firmware_core::homing::home_axis(config, axis, &mut tracking, &limits[axis])
      };
      match outcome {
        Ok(o) => {
          zero_steps[axis] = o.zero_steps;
          // Phase-boundary publish (finding #5): push the live MPos now that this axis has finished all four
          // homing phases, so `?` reflects the homing progress without the per-burst atomic flood.
          publish_live_position(counter);
        }
        Err(e) => {
          // A no-contact / sink failure aborts the whole cycle: position is suspect, so publish the error and
          // stop. The consumer raises the homing-fail alarm + forces a reset; this does not fabricate a homed state.
          result = Err(e);
          break 'cycle;
        }
      }
    }
  }

  // On success, SYNC the live counter to the post-homing machine-zero position so the published MPos snaps to
  // the established zero (the consumer mirrors this into the planner/parser commanded position). On failure the
  // live position is left where the aborted seek stopped; the consumer's reset zeroes it.
  if result.is_ok() {
    counter.sync_to(zero_steps);
    publish_live_position(counter);
    result = Ok(zero_steps);
  }

  EXECUTOR_RUNNING.store(false, Ordering::Release);
  // Refresh the published `Pn:` limit mask from the settled post-pull-off levels: a cycle that finishes with the
  // switches released (the normal case) must clear any `Pn:X/Y/Z` the in-cycle trips would otherwise have left.
  publish_limit_levels(limits);
  HOME_RESULT.signal(result);
}

/// Await a DEBOUNCED limit-switch trip while the executor is idle (DOC-06 / research findings #1 & #14). Races a
/// rising edge across all three X/Y/Z limit pins (the genuine interrupt-driven [`Input::wait_for_rising_edge`],
/// no hand-written ISR, no polling — the executor parks until the hardware edge fires), then runs the `$26`
/// debounce RESAMPLE: wait `$26` ms, then re-read the live levels and confirm at least one axis still reads
/// triggered through the host-tested [`limit_triggered`](firmware_core::hal_traits::limit_triggered) (honoring
/// the live `$5` sense + the NC fail-safe). A confirmed trip SIGNALS [`LIMIT_TRIGGERED`](crate::comms::
/// LIMIT_TRIGGERED) — giving that documented seam a real producer — and returns so the idle arm samples + alarms.
/// A glitch that does NOT persist past the debounce (EMI, NC-pull-up settling) is rejected: the function loops
/// and re-arms the edge wait rather than returning a false trip.
///
/// This is the idle-path detector ONLY; an in-MOTION over-travel is caught by the block-boundary
/// [`check_hard_limits`] call (a burst in flight is never split). The shared-pin rule (no alarm while homing) is
/// applied downstream in [`check_hard_limits`] / [`hard_limit_alarm_armed`], not here.
async fn wait_for_limit_trip(limits: &mut [RmtLimitInput; AXES]) {
  loop {
    // Park on a rising edge from ANY linear limit pin. `select` over the three borrows races them concurrently;
    // the first edge to fire resolves. The borrows are split per element so all three can be awaited at once. The
    // rotary A axis (index 3) has NO physical limit switch (DOC-10.6), so its placeholder input is never waited
    // on — TODO(DOC-00/Phase-5): finalize A's exclusion from all limit sampling once the carrier wiring is set.
    {
      let [x, y, z, _a] = limits;
      let edge = select(
        select(x.wait_for_rising_edge(), y.wait_for_rising_edge()),
        z.wait_for_rising_edge(),
      );
      edge.await;
    }
    mtrace!("motion: limit rising edge -> debounce resample");
    // Debounce: wait `$26` ms, then confirm the level persists. A zero `$26` still resamples immediately (one
    // `Timer::after(0)` yields), so a glitch already cleared by the read is rejected even with debounce disabled.
    let debounce_ms = crate::comms::limit_debounce_ms();
    Timer::after(Duration::from_millis(debounce_ms as u64)).await;
    let config = crate::comms::limit_config();
    // The rotary A axis has no limit switch (DOC-10.6), so exclude it — its placeholder pin must never count as
    // a trip regardless of `$5`.
    let still_tripped = (0..AXES)
      .any(|i| i != A_AXIS && firmware_core::hal_traits::limit_triggered(limits[i].is_high(), &config));
    if still_tripped {
      // A confirmed trip: publish the documented `LIMIT_TRIGGERED` seam and return so the idle arm samples the
      // switches and raises `ALARM:1` (gated by `$21` + the not-homing shared-pin rule) via `check_hard_limits`.
      mtrace!("motion: limit trip confirmed after debounce");
      LIMIT_TRIGGERED.signal(());
      return;
    }
    // The edge did not persist past the debounce — a glitch. Drain any stale `LIMIT_TRIGGERED` we are about to
    // re-arm against, then loop to wait for the next genuine edge rather than returning a false trip.
    mtrace!("motion: limit edge rejected by debounce (glitch)");
  }
}

/// Sample the X/Y/Z limit switches and raise the hard-limit alarm if one FRESHLY tripped during normal motion
/// (DOC-06). Reads the raw pin levels, applies the live `$5` invert + `$21` enable + the shared-pin (not-homing)
/// rule via the host-tested [`hard_limit_alarm_armed`](firmware_core::homing::hard_limit_alarm_armed), and on a
/// FRESH over-travel edge signals [`HARD_LIMIT_TRIPPED`] so the consumer enters `ALARM:1` and resets the pipeline.
///
/// The alarm is EDGE-armed: `armed` carries the per-axis "was triggered at the previous sample" state across
/// calls, so the alarm fires only on a not-triggered -> triggered transition. A switch the machine is merely
/// PARKED on (a level already asserted at the previous sample — e.g. left engaged after an aborted `$H` seek, which
/// performs no pull-off) therefore never re-fires a stale `ALARM:1`. This is the principled fix for the post-homing
/// `error:9` re-lock: the held level used to latch a spurious trip the instant `HOMING_ACTIVE` cleared, which
/// survived the soft reset and re-locked the machine after `$X`. Genuine over-travel during a normal move is a
/// fresh edge and still alarms. `armed` is re-seeded to the settled levels at the reset / post-homing boundaries
/// in the executor loop, so a held switch is "already known". The published `Pn:` mask stays purely level-based.
///
/// Called at every block boundary and on the idle limit-ISR wake; a no-op when `$21` is off, while homing, or when
/// no axis makes a new triggered transition.
fn check_hard_limits(limits: &[RmtLimitInput; AXES], armed: &mut [bool; AXES]) {
  // Build the live limit config from the `$5` mirror and read the enable + homing-active flags (all `Relaxed`
  // atomics — a real-time-safe read, no async settings lock).
  let config = crate::comms::limit_config();
  let enabled = HARD_LIMITS_ENABLED.load(Ordering::Relaxed);
  let homing = HOMING_ACTIVE.load(Ordering::Acquire);
  // Sample the raw pin levels ONCE for this call (finding #6): `from_fn` over `AXES` cannot silently desync from
  // the axis count/order the way a hardcoded `[0, 1, 2]` index list could. Both the alarm decision AND the
  // published `Pn:` mask are derived from this single sample, so they are one coherent read of the same instant.
  // The rotary A axis (index 3) has no physical limit switch (DOC-10.6); its placeholder pin reads as NOT high so
  // it can never raise a hard-limit alarm or appear in `Pn:`, regardless of `$5`. TODO(Phase-5): finalize wiring.
  let raw_high: [bool; AXES] = core::array::from_fn(|i| i != A_AXIS && limits[i].is_high());
  // `hard_limit_alarm_armed` applies the `$5` invert internally, returning the post-`$5` logical-triggered array
  // and an EDGE-armed alarm keyed on `*armed` (the previous-sample triggered state). Feed the RAW levels here (NOT
  // pre-inverted); reuse `decision.triggered` for the published mask so the `Pn:` view is the logical (post-`$5`)
  // state, no double-invert. Carry `decision.next_armed` back so the next call sees this sample as its baseline.
  let decision = firmware_core::homing::hard_limit_alarm_armed(raw_high, &config, enabled, homing, *armed);
  *armed = decision.next_armed;
  // Refresh the published `Pn:` limit mask from this same sample, so the host's endstop view is current the
  // instant motion stops — not only after the next idle tick — and it agrees exactly with the alarm decision.
  LIMIT_LEVELS.store(firmware_core::homing::pack_limit_mask(decision.triggered), Ordering::Release);
  if decision.alarm {
    // A limit FRESHLY tripped during normal motion with `$21` on: halt and raise `ALARM:1`. The consumer locks the
    // alarm (position is likely lost from the abrupt stop) and resets the pipeline; the executor's reset path
    // zeroes the live position. Signal once — the alarm latches, so a re-trip before service is harmless.
    HARD_LIMIT_TRIPPED.signal(());
  }
}

/// Sample the X/Y/Z limit pins and return their per-axis LOGICAL-triggered state (post-`$5` invert + NC fail-safe).
/// This is the seed for the hard-limit ARMING state: at the reset / post-homing boundaries the executor calls this
/// to mark the currently-asserted switches "already known", so a held level is not mistaken for a fresh over-travel
/// edge on the next [`check_hard_limits`]. It reads LEVELS, not edges, and raises no alarm and publishes nothing —
/// it only computes the triggered array, sharing the `$5`/NC logic with [`check_hard_limits`] and the `Pn:` publish.
fn sample_limit_triggered(limits: &[RmtLimitInput; AXES]) -> [bool; AXES] {
  let config = crate::comms::limit_config();
  // A (index 3) has no limit switch (DOC-10.6) → always not-triggered, so it is never armed against a held level.
  core::array::from_fn(|axis| {
    axis != A_AXIS && firmware_core::hal_traits::limit_triggered(limits[axis].is_high(), &config)
  })
}

/// Build a probe [`Block`] from the current machine position `start` (steps) to the probe `target` (steps). A
/// probe runs at a constant feed (no trapezoid), so the block's speed fields are placeholders the [`ProbeStepper`]
/// ignores — only the step deltas, dominant-axis count, and (for direction latching) the signs matter. The mm
/// length is left at 1.0 as a non-zero placeholder (the prober uses the caller's fixed period, not the mm length).
fn probe_block_to(start: [i32; AXES], target: [i32; AXES]) -> Block {
  let mut steps = [0i32; AXES];
  let mut step_event_count = 0u32;
  for axis in 0..AXES {
    steps[axis] = target[axis] - start[axis];
    step_event_count = step_event_count.max(steps[axis].unsigned_abs());
  }
  Block {
    steps,
    step_event_count,
    // The unit vector / mm length / speeds are unused by the probe stepper (it walks at a fixed period and exact
    // integer Bresenham); set benign non-degenerate placeholders.
    unit_vec: [0.0; AXES],
    millimeters: 1.0,
    acceleration: 1.0,
    nominal_speed_sq: 1.0,
    max_entry_speed_sq: 1.0,
    entry_speed_sq: 0.0,
    rapid: false,
    jog: false,
  }
}

/// A [`StepSink`] decorator that advances a [`StepCounter`] and publishes the live position as bursts pass
/// through to the real RMT sink, so MPos is derived from exactly the events the hardware emits (not
/// re-derived from the block) and stays live WITHIN a block. It forwards `set_direction`/`emit_burst` to the
/// inner sink, tallies steps, pushes the running step position into [`LIVE_POSITION`], and — between bursts —
/// aborts the block early on a pending soft reset (Finding #3) so a reset during a multi-second block stops the
/// motion within one burst rather than after the whole block. The live position is then RETAINED (Change A) at the
/// last published step position, not zeroed — matching grbl's "reset keeps MPos" behavior.
struct CountingSink<'a> {
  inner: &'a mut RmtStepSink,
  counter: &'a mut StepCounter,
  /// Publish the live [`LIVE_POSITION`] atomics after EVERY burst when `true` (normal block / probe motion, so
  /// MPos stays live within a long block). Set `false` for the `$H` homing cycle (finding #5): a homing seek
  /// emits thousands of single-step bursts, so a per-burst publish would do thousands of 3× `Release` stores per
  /// seek; homing instead publishes at PHASE boundaries (after each axis + the final zero sync) in [`run_homing`].
  publish_per_burst: bool,
}

impl<'a> CountingSink<'a> {
  /// A [`CountingSink`] that publishes the live position after every burst — the default for normal block and
  /// probe motion, where bursts are large and MPos must track within a block.
  fn live(inner: &'a mut RmtStepSink, counter: &'a mut StepCounter) -> Self {
    CountingSink { inner, counter, publish_per_burst: true }
  }

  /// A [`CountingSink`] that does NOT publish per burst — for the homing cycle, whose single-step bursts would
  /// otherwise flood the position atomics (finding #5). The caller publishes at phase boundaries instead.
  fn quiet(inner: &'a mut RmtStepSink, counter: &'a mut StepCounter) -> Self {
    CountingSink { inner, counter, publish_per_burst: false }
  }
}

impl StepSink for CountingSink<'_> {
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
    // Latch the counter's direction from the SAME `DirState` the hardware gets, so the live position advances
    // the right way. For `run_block`/`run_probe` this is identical to the direction they already latched on the
    // counter (same block step signs), so it is a harmless re-latch; for the multi-phase HOMING cycle — whose
    // pull-off phases reverse direction mid-cycle — it is what keeps the live MPos correct across the reversals.
    self.counter.set_direction(dir);
    self.inner.set_direction(dir)
  }

  fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
    // Abort BETWEEN bursts on a pending soft reset (never mid-burst — a burst in flight is never split):
    // returning a sink error stops `run_block` early, and the executor's next iteration RETAINS the live
    // position (Change A) at the last published step position rather than zeroing it. This bounds reset latency
    // to one burst even inside a long block (Finding #3).
    if MOTION_RESET_PENDING.load(Ordering::Acquire) {
      return Err(StepError::Transport);
    }
    // Emit first, then count: only count the steps that actually reached the hardware, so a transport
    // failure mid-burst does not advance the live position past what was physically emitted.
    self.inner.emit_burst(ticks)?;
    for event in ticks {
      self.counter.advance(event);
    }
    // Publish the running step position after the burst (normal motion / probe), decoupled from the executor's
    // block-level `.await` so `?` reflects motion as it happens. `Release` stores pair with the reader's
    // `Acquire` loads. Suppressed for homing (finding #5) — its single-step bursts would flood these atomics;
    // `run_homing` publishes at phase boundaries instead.
    if self.publish_per_burst {
      publish_live_position(self.counter);
    }
    Ok(())
  }
}

/// RETAIN the live step position across a soft reset and re-publish it, matching grbl: a `0x18` abort RETAINS
/// MPos (it does NOT zero it) so `$X` then unlocks at the SAME coordinates and `$H` re-establishes certainty.
/// This is the Change A fix — the executor used to zero the counter + atomics here, which lost the operator's
/// zero on every Stop on a no-homing machine (`$22=0`), diverging from grbl. After an abort DURING motion the
/// retained value is the last per-burst-published live step position — "suspect" because the steps were
/// interrupted mid-move (exactly as grbl documents: `$X` unlocks at it, `$H` re-homes to recover) — and after a
/// reset from Idle it is simply the resting position. The re-publish is a coherent no-op (the atomics already
/// hold the live value) kept for symmetry with the previous reset-service shape and to pair `Release` stores with
/// the reader's `Acquire` loads. The executor remains the SINGLE owner of the live position (Finding #3); the
/// consumer's `reset_pipeline` rebuilds the planner and SYNCS it to this retained position so a subsequent
/// absolute move resolves relative to the retained MPos, not the origin.
fn retain_live_position(counter: &StepCounter) {
  publish_live_position(counter);
}

/// Publish the step counter's current position into the cross-core [`LIVE_POSITION`] atomics. `Release`
/// stores so a [`status_responder`](crate::comms::status_responder) `Acquire` read on core 0 sees a coherent
/// per-axis value. Steps→mm conversion is deferred to the reader's host-tested `steps_to_mm` (Finding #5).
fn publish_live_position(counter: &StepCounter) {
  let position = counter.position_steps();
  for axis in 0..AXES {
    LIVE_POSITION[axis].store(position[axis], Ordering::Release);
  }
}

/// Sample the three X/Y/Z limit pins and publish their LOGICAL-triggered state into the cross-core
/// [`LIMIT_LEVELS`](crate::comms::LIMIT_LEVELS) bitmask for the core-0 `Pn:` reader (DOC-06 / DOC-08). Reads the
/// raw levels, applies the live `$5` invert + NC fail-safe via the host-tested
/// [`limit_triggered`](firmware_core::hal_traits::limit_triggered), packs them as `bit0 = X`, `bit1 = Y`,
/// `bit2 = Z`, and `Release`-stores the mask (pairing with the reader's `Acquire`). It reads LEVELS, not edges, so
/// calling it tracks both press and release — this is the SINGLE writer of the published limit state. Called from
/// every point that already samples the pins ([`check_hard_limits`] at block boundaries / on the idle trip,
/// post-homing) and from the idle `Ticker` so a release at idle is reflected within one tick. Synchronous and
/// allocation-free: the idle-tick call adds no work to the real-time burst path, which never invokes it.
fn publish_limit_levels(limits: &[RmtLimitInput; AXES]) {
  let config = crate::comms::limit_config();
  // Sample each pin once and apply the `$5` invert + NC fail-safe to get the logical-triggered state, then pack
  // it through the host-tested [`pack_limit_mask`](firmware_core::homing::pack_limit_mask) so the bit layout has
  // a single definition shared with [`check_hard_limits`] and the `comms::limit_levels()` decode.
  let triggered: [bool; AXES] =
    core::array::from_fn(|axis| firmware_core::hal_traits::limit_triggered(limits[axis].is_high(), &config));
  LIMIT_LEVELS.store(firmware_core::homing::pack_limit_mask(triggered), Ordering::Release);
}

/// Configure the three RMT TX step channels (ch0/1/2 on GPIO1/2/4) and the three DIR outputs (GPIO5/6/7),
/// drive STEP_EN (GPIO8) enabled (active-low → low) so the steppers hold, and assemble the [`RmtStepSink`].
///
/// The RMT source clock is 80 MHz; with `clk_divider = 80` one tick is exactly 1 µs, matching the default
/// `MotionConfig.tick_hz = 1_000_000` the generator assumes (so a `period_ticks` value is a microsecond
/// count). The default `memsize` is one block (48 symbols); a full burst is `MAX_SYMBOLS_PER_BURST` (47)
/// events plus one `end_marker` = exactly 48 symbols, so it fits one block with no adjacent-channel
/// borrowing and no reliance on the interrupt-priority streaming refill (Finding #7). `STEP_EN` is returned
/// so its lifetime is held for the program duration (dropping it would
/// release the pin); the full enable/disable policy is a later refinement.
///
/// # Panics
/// `expect` is used here because this runs once in `main`'s init path, where a failure to bring up the RMT
/// peripheral or claim a step pin is an unrecoverable wiring/config fault, not a runtime condition
/// (CLAUDE.md permits `expect` in init). It never executes after boot.
#[allow(clippy::type_complexity)]
pub fn init(
  rmt: esp_hal::peripherals::RMT<'static>,
  step_pins: (
    esp_hal::peripherals::GPIO1<'static>,
    esp_hal::peripherals::GPIO2<'static>,
    esp_hal::peripherals::GPIO4<'static>,
    // A-STEP on the documented spare RMT ch3 (GPIO18, DOC-00). PROVISIONAL / bench-unverified (DOC-10 Phase 5).
    esp_hal::peripherals::GPIO18<'static>,
  ),
  dir_pins: (
    esp_hal::peripherals::GPIO5<'static>,
    esp_hal::peripherals::GPIO6<'static>,
    esp_hal::peripherals::GPIO7<'static>,
    // A-DIR — PROVISIONAL GPIO38 (a free S3 pin; DOC-00 manifest addition pending bench, DOC-10 Phase 5).
    esp_hal::peripherals::GPIO38<'static>,
  ),
  step_enable_pin: esp_hal::peripherals::GPIO8<'static>,
  config: &MotionConfig,
) -> (RmtStepSink, Output<'static>) {
  use esp_hal::rmt::Rmt;
  use esp_hal::time::Rate;

  // 80 MHz source / clk_divider 80 = 1 MHz = 1 tick per microsecond, matching `MotionConfig.tick_hz` (1 MHz,
  // a fixed firmware constant — only the `$0` step-pulse width inside `config` varies with settings).
  let rmt = Rmt::new(rmt, Rate::from_mhz(80)).expect("RMT peripheral init");
  let tx_config = TxChannelConfig::default()
    .with_clk_divider(80)
    .with_idle_output_level(Level::Low)
    .with_idle_output(true);

  // Each axis owns one TX channel on its step GPIO; the channels are independent (no shared memory block).
  // As of esp-hal 1.1 (#4302) `configure_tx` takes the config by reference and no longer binds the pin —
  // the step GPIO is attached afterwards with `Channel::with_pin`.
  let ch_x = rmt.channel0.configure_tx(&tx_config).expect("RMT ch0 (X step)").with_pin(step_pins.0);
  let ch_y = rmt.channel1.configure_tx(&tx_config).expect("RMT ch1 (Y step)").with_pin(step_pins.1);
  let ch_z = rmt.channel2.configure_tx(&tx_config).expect("RMT ch2 (Z step)").with_pin(step_pins.2);
  // A-STEP on the previously-spare ch3 (DOC-10). Compile-verified only; not driven on the bench yet (Phase 5).
  let ch_a = rmt.channel3.configure_tx(&tx_config).expect("RMT ch3 (A step)").with_pin(step_pins.3);

  // DIR outputs start low (positive direction); the first block latches the real direction before stepping.
  let out_cfg = OutputConfig::default();
  let dir_x = Output::new(dir_pins.0, Level::Low, out_cfg);
  let dir_y = Output::new(dir_pins.1, Level::Low, out_cfg);
  let dir_z = Output::new(dir_pins.2, Level::Low, out_cfg);
  let dir_a = Output::new(dir_pins.3, Level::Low, out_cfg);

  // STEP_EN (TMC ENN) is active-low: drive it low to ENABLE the drivers so the steppers hold at boot. The
  // full enable-on-motion / disable-on-idle policy is deferred; for now the drivers stay enabled.
  let step_enable = Output::new(step_enable_pin, Level::Low, out_cfg);

  // `$29` direction-setup delay in ticks (= microseconds at this divider). Placeholder default until
  // esp-storage settings are loaded; DOC-02 cites a 2 µs practical minimum (5–15 µs for opto drivers).
  let sink = RmtStepSink::new([ch_x, ch_y, ch_z, ch_a], [dir_x, dir_y, dir_z, dir_a], config, DIR_SETUP_US);
  (sink, step_enable)
}

/// Placeholder `$29` direction-setup delay in microseconds. TODO(DOC-00): load from esp-storage alongside
/// `$0`. DOC-02 cites a 2 µs practical minimum; 5 µs gives comfortable margin for the TMC2209 DIR-to-STEP
/// setup without measurably slowing motion (incurred once per block, not per step).
const DIR_SETUP_US: u32 = 5;

/// Configure the PROBE digital input on GPIO21 (DOC-09, Phase C) and wrap it as an [`RmtProbeInput`]. The DOC-00
/// GPIO manifest assigns no probe pin (it predates probing); GPIO21 is the first genuinely-free input pin
/// (GPIO16/17 are the optional feed-hold/cycle-start inputs, GPIO18 is the spare RMT ch3), so it is chosen here
/// for the dedicated probe input — kept separate from the limit switches as `docs/tlo-offsets.md` Finding #7
/// requires. The internal pull-up is ENABLED unless `$19` (`pullup_disable`) is set: a passive touch plate needs
/// the pull-up, so the default (`$19=0`) is pulled up. The `$6` invert is applied per-sample by the probe cycle,
/// not at pin config.
pub fn init_probe(probe_pin: esp_hal::peripherals::GPIO21<'static>, config: &ProbeConfig) -> RmtProbeInput {
  // `$19=0` (the passive-plate default) keeps the internal pull-up; `$19=1` disables it (an externally-biased
  // probe input). grbl's `$19` is exactly this pull-up-disable bit.
  let pull = if config.pullup_disable { Pull::None } else { Pull::Up };
  let input = Input::new(probe_pin, InputConfig::default().with_pull(pull));
  RmtProbeInput::new(input)
}

/// Configure the X/Y/Z limit inputs on GPIO10/11/12 (DOC-00 manifest) and wrap each as an [`RmtLimitInput`],
/// returned `[X, Y, Z]` (DOC-06). Every limit pin gets the internal PULL-UP unconditionally: Galdr wires
/// Normally-Closed switches to GND, so an intact switch holds the pin LOW and opening it (or a broken wire)
/// lets the pull-up raise it HIGH = triggered — the documented broken-wire fail-safe. The `$5` invert is a
/// LOGICAL trigger-sense flip applied per-sample by [`limit_triggered`](firmware_core::hal_traits::limit_triggered),
/// not a pin-config concern, so it does not change the pull here.
///
/// The rising-edge wait that feeds the hard-limit [`LIMIT_TRIGGERED`](crate::comms::LIMIT_TRIGGERED) seam is
/// driven by the core-1 executor itself, which owns these inputs: its idle loop awaits a rising edge on each pin
/// via [`RmtLimitInput::wait_for_rising_edge`] (the interrupt-driven esp-hal async edge wait), debounces, and
/// signals the trip. The homing seek/locate path samples these same inputs between bursts and needs no interrupt.
pub fn init_limits(
  x_lim: esp_hal::peripherals::GPIO10<'static>,
  y_lim: esp_hal::peripherals::GPIO11<'static>,
  z_lim: esp_hal::peripherals::GPIO12<'static>,
  // A-LIMIT placeholder. The rotary A axis has NO physical limit switch (DOC-10.6) and is never homed; this pin
  // exists only to fill the `[_; AXES]` array. PROVISIONAL GPIO39 (DOC-00 addition pending bench, DOC-10 Phase 5).
  a_lim: esp_hal::peripherals::GPIO39<'static>,
) -> [RmtLimitInput; AXES] {
  // The NC fail-safe requires the pull-up on every REAL limit pin regardless of `$5` (which is a logical sense
  // flip, applied in `limit_triggered`, not a pin-pull setting).
  let config = InputConfig::default().with_pull(Pull::Up);
  // The A placeholder is pulled DOWN so it reads LOW (not asserted) by default; combined with the explicit
  // `A_AXIS` exclusion in the sampling functions, A can never raise a hard-limit alarm or appear in `Pn:`.
  let a_config = InputConfig::default().with_pull(Pull::Down);
  [
    RmtLimitInput::new(Input::new(x_lim, config)),
    RmtLimitInput::new(Input::new(y_lim, config)),
    RmtLimitInput::new(Input::new(z_lim, config)),
    RmtLimitInput::new(Input::new(a_lim, a_config)),
  ]
}
