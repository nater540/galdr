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
//! [`BLOCK_AVAILABLE`](crate::comms::BLOCK_AVAILABLE) and, on a hold, [`CYCLE_START`](crate::comms::CYCLE_START)).
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

use core::sync::atomic::Ordering;

use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::rmt::{Channel, PulseCode, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::Blocking;
use embassy_futures::select::{select, Either};

use firmware_core::hal_traits::{DirState, StepError, StepEvent, StepSink, MAX_SYMBOLS_PER_BURST};
use firmware_core::motion::{silent_symbol_halves, MotionConfig, SegmentGenerator, StepCounter};
use firmware_core::planner::{Block, Planner, AXES};

use crate::comms::{
  BLOCK_AVAILABLE, CYCLE_START, FEED_HOLD, LIVE_POSITION, MOTION_RESET, MOTION_RESET_PENDING, PLANNER,
};

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
    }
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
      return Err(StepError::BurstTooLong);
    }
    if ticks.is_empty() {
      return Ok(());
    }
    // Encode all three channels into their scratch buffers up front; `len` is the same for every channel
    // (events + end marker), keeping the three transmits identical in length.
    let len = self.encode_channel(0, ticks);
    let _ = self.encode_channel(1, ticks);
    let _ = self.encode_channel(2, ticks);

    // Start all three transmits before waiting any, so the three channels fire together. `transmit`
    // consumes the channel; we take it out of its slot and restore it from the transaction's `wait()`.
    let mut txns: [Option<_>; AXES] = [None, None, None];
    for (axis, slot) in txns.iter_mut().enumerate() {
      // The channel is always `Some` here (only this method takes it, and it restores it before returning).
      // A missing channel would be an internal invariant break; treat it as a transport failure rather than
      // unwrapping, per the no-`unwrap` rule.
      let Some(channel) = self.channels[axis].take() else {
        return Err(StepError::Transport);
      };
      match channel.transmit(&self.scratch[axis][..len]) {
        Ok(txn) => *slot = Some(txn),
        Err(_) => {
          // `Channel::transmit` takes the channel BY VALUE and, on the start error path (esp-hal 1.0.0),
          // returns only the `Error` — the channel is moved in and not handed back, so this axis's channel
          // is genuinely lost and its `Option` slot stays `None`; that axis cannot transmit again. This is a
          // hardware fault (a mis-sized buffer / missing end marker is excluded by construction here), so it
          // should escalate to an ALARM that re-inits the RMT peripheral. The executor has no alarm state
          // yet (Stage 1), so for now we surface a transport error (which abandons the current block) and the
          // channel remains down until reboot. TODO(DOC-06 alarm path): on this error raise an alarm and
          // re-run `motion::init`'s RMT bring-up to reclaim the channel. The previous comment claiming "the
          // next reset re-inits RMT" was FALSE — nothing currently re-inits RMT — and is corrected here.
          return Err(StepError::Transport);
        }
      }
    }

    // Now block-wait every channel. The hardware ran them concurrently; waiting them in sequence only blocks
    // until the longest finishes. `wait` returns the channel back on success (and inside the error tuple on
    // failure), so we restore each one for the next burst either way.
    let mut result = Ok(());
    for (axis, slot) in txns.iter_mut().enumerate() {
      if let Some(txn) = slot.take() {
        match txn.wait() {
          Ok(channel) => self.channels[axis] = Some(channel),
          Err((_, channel)) => {
            self.channels[axis] = Some(channel);
            result = Err(StepError::Transport);
          }
        }
      }
    }
    result
  }
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
///    zero the live position so MPos returns to the origin in step with the consumer's pipeline reset.
/// 2. At each block boundary (never mid-burst), honor a feed-hold: if [`FEED_HOLD`](crate::comms::FEED_HOLD)
///    is set, drain any stale latched [`CYCLE_START`](crate::comms::CYCLE_START) (Finding #8) then await a
///    fresh one before running the block. A reset releases the hold.
/// 3. Pop the next block under the planner lock, peeking the following block's `entry_speed_sq` as this
///    block's `exit_speed_sq` (0.0 when the queue holds only this block, so it stops at rest). Release the
///    lock BEFORE any RMT transmit — the planner mutex is never held across step emission.
/// 4. If the queue was empty, await [`BLOCK_AVAILABLE`](crate::comms::BLOCK_AVAILABLE) rather than polling,
///    racing [`MOTION_RESET`] so a `0x18` wakes the executor promptly.
/// 5. Run the block synchronously through the [`SegmentGenerator`], advancing the live [`StepCounter`] and
///    publishing the live step position into the [`LIVE_POSITION`] atomics after EACH burst (so MPos is live
///    within a long block, not frozen until the block ends — Finding #5). A reset pending between bursts
///    aborts the block early via a sink error, then the next loop iteration zeroes the position.
pub async fn run(sink: &mut RmtStepSink, config: MotionConfig) -> ! {
  let generator = SegmentGenerator::new(config);
  let mut counter = StepCounter::new();
  loop {
    // Service a pending soft reset at the top of the loop: drop the live position so MPos returns to the
    // origin in step with the consumer's pipeline reset. `MOTION_RESET_PENDING` is the poll-able flag the
    // sink also tests mid-block; clearing it AND draining the `MOTION_RESET` signal here keeps the two in
    // sync so a reset is serviced exactly once.
    if MOTION_RESET_PENDING.swap(false, Ordering::AcqRel) {
      MOTION_RESET.try_take();
      reset_live_position(&mut counter);
    }

    // Honor a feed-hold at the block boundary: pause until cycle-start. Checking here (never mid-block)
    // matches DOC-02 — a burst in flight is never split. Drain any STALE latched cycle-start first (Finding
    // #8): a `~` that arrived while no hold was pending must not auto-release this fresh hold, so we clear it
    // before awaiting a genuinely new one. A soft reset also releases the hold.
    if FEED_HOLD.try_take().is_some() {
      CYCLE_START.try_take();
      match select(CYCLE_START.wait(), MOTION_RESET.wait()).await {
        Either::First(()) => {}
        Either::Second(()) => {
          MOTION_RESET_PENDING.store(false, Ordering::Release);
          reset_live_position(&mut counter);
          continue;
        }
      }
    }

    // Pop the next block and peek the one after it for the exit speed, all under one short lock, releasing
    // it before any transmit so the consumer can keep enqueuing while this block executes.
    let popped = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => take_block(planner),
        None => None,
      }
    };

    match popped {
      Some((block, exit_speed_sq)) => run_block(&generator, &block, exit_speed_sq, sink, &mut counter),
      // Queue empty: await a freshly enqueued block instead of polling, racing the dedicated motion reset so
      // a reset arriving while idle is observed promptly (it zeroes the position on the next iteration).
      None => match select(BLOCK_AVAILABLE.wait(), MOTION_RESET.wait()).await {
        Either::First(()) => {}
        Either::Second(()) => {
          MOTION_RESET_PENDING.store(false, Ordering::Release);
          reset_live_position(&mut counter);
        }
      },
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

/// Realize one block through the generator while advancing the live step counter. The counter is fed in
/// lock-step with the sink: `set_direction` latches the per-axis sign from the block, and each emitted
/// [`StepEvent`] advances the counter and publishes the live position into [`LIVE_POSITION`] per burst — so
/// the live MPos mirrors exactly what the RMT channels emit, updated within the block rather than only at its
/// end. A generator/sink error (including a soft-reset abort tested between bursts) abandons the block; the
/// next status report still reflects the steps published so far.
fn run_block(generator: &SegmentGenerator, block: &Block, exit_speed_sq: f32, sink: &mut RmtStepSink, counter: &mut StepCounter) {
  // Latch the live counter's direction from the same step signs the generator latches onto the sink, so the
  // counter advances each axis the correct way. A zero-length block never steps, so this is harmless then.
  counter.set_direction(DirState {
    dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0],
  });
  let mut tracking = CountingSink { inner: sink, counter };
  // The generator returns the tick count or a recoverable error; on error we simply stop emitting this
  // block. The error is not surfaced upward because Stage 1 has no alarm state machine yet (DOC-06/Stage 2);
  // the abandoned block leaves the machine where the last published burst put it, which the live MPos shows.
  let _ = generator.run_block(block, exit_speed_sq, &mut tracking);
}

/// A [`StepSink`] decorator that advances a [`StepCounter`] and publishes the live position as bursts pass
/// through to the real RMT sink, so MPos is derived from exactly the events the hardware emits (not
/// re-derived from the block) and stays live WITHIN a block. It forwards `set_direction`/`emit_burst` to the
/// inner sink, tallies steps, pushes the running step position into [`LIVE_POSITION`], and — between bursts —
/// aborts the block early on a pending soft reset (Finding #3) so a reset during a multi-second block zeroes
/// the position within one burst rather than after the whole block.
struct CountingSink<'a> {
  inner: &'a mut RmtStepSink,
  counter: &'a mut StepCounter,
}

impl StepSink for CountingSink<'_> {
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
    // The counter's direction is latched in `run_block` from the block's step signs (identical to what the
    // generator passes here), so forwarding to the hardware is all that is needed.
    self.inner.set_direction(dir)
  }

  fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
    // Abort BETWEEN bursts on a pending soft reset (never mid-burst — a burst in flight is never split):
    // returning a sink error stops `run_block` early, and the executor's next iteration zeroes the position.
    // This bounds reset latency to one burst even inside a long block (Finding #3).
    if MOTION_RESET_PENDING.load(Ordering::Acquire) {
      return Err(StepError::Transport);
    }
    // Emit first, then count: only count the steps that actually reached the hardware, so a transport
    // failure mid-burst does not advance the live position past what was physically emitted.
    self.inner.emit_burst(ticks)?;
    for event in ticks {
      self.counter.advance(event);
    }
    // Publish the running step position after the burst, decoupled from the executor's block-level `.await`
    // so `?` reflects motion as it happens. `Release` stores pair with the reader's `Acquire` loads.
    publish_live_position(self.counter);
    Ok(())
  }
}

/// Zero the live step counter and the published [`LIVE_POSITION`] atomics together, so a soft reset returns
/// MPos to the origin atomically from the reader's view. The executor is the SINGLE owner of the live
/// position (Finding #3): the consumer's `reset_pipeline` never writes it, so there is no cross-core race.
fn reset_live_position(counter: &mut StepCounter) {
  counter.reset();
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
  ),
  dir_pins: (
    esp_hal::peripherals::GPIO5<'static>,
    esp_hal::peripherals::GPIO6<'static>,
    esp_hal::peripherals::GPIO7<'static>,
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
  let ch_x = rmt.channel0.configure_tx(step_pins.0, tx_config).expect("RMT ch0 (X step)");
  let ch_y = rmt.channel1.configure_tx(step_pins.1, tx_config).expect("RMT ch1 (Y step)");
  let ch_z = rmt.channel2.configure_tx(step_pins.2, tx_config).expect("RMT ch2 (Z step)");

  // DIR outputs start low (positive direction); the first block latches the real direction before stepping.
  let out_cfg = OutputConfig::default();
  let dir_x = Output::new(dir_pins.0, Level::Low, out_cfg);
  let dir_y = Output::new(dir_pins.1, Level::Low, out_cfg);
  let dir_z = Output::new(dir_pins.2, Level::Low, out_cfg);

  // STEP_EN (TMC ENN) is active-low: drive it low to ENABLE the drivers so the steppers hold at boot. The
  // full enable-on-motion / disable-on-idle policy is deferred; for now the drivers stay enabled.
  let step_enable = Output::new(step_enable_pin, Level::Low, out_cfg);

  // `$29` direction-setup delay in ticks (= microseconds at this divider). Placeholder default until
  // esp-storage settings are loaded; DOC-02 cites a 2 µs practical minimum (5–15 µs for opto drivers).
  let sink = RmtStepSink::new([ch_x, ch_y, ch_z], [dir_x, dir_y, dir_z], config, DIR_SETUP_US);
  (sink, step_enable)
}

/// Placeholder `$29` direction-setup delay in microseconds. TODO(DOC-00): load from esp-storage alongside
/// `$0`. DOC-02 cites a 2 µs practical minimum; 5 µs gives comfortable margin for the TMC2209 DIR-to-STEP
/// setup without measurably slowing motion (incurred once per block, not per step).
const DIR_SETUP_US: u32 = 5;
