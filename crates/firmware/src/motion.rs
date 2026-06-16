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

use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::rmt::{Channel, PulseCode, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::Blocking;
use embassy_futures::select::{select, Either};

use firmware_core::hal_traits::{DirState, StepError, StepEvent, StepSink, MAX_SYMBOLS_PER_BURST};
use firmware_core::motion::{MotionConfig, SegmentGenerator, StepCounter};
use firmware_core::planner::{Block, Planner, AXES};

use crate::comms::{
  placeholder_motion_config, placeholder_steps_per_mm, BLOCK_AVAILABLE, CYCLE_START, FEED_HOLD, MACHINE,
  PLANNER, SOFT_RESET,
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
      scratch: [[PulseCode::end_marker(); MAX_SYMBOLS_PER_BURST + 1]; AXES],
    }
  }

  /// Encode one channel's burst into its scratch buffer: one PulseCode per [`StepEvent`], then the
  /// mandatory `end_marker`. A stepping axis gets a HIGH(`$0`)/LOW(`period − $0`) pulse; a silent axis gets
  /// a full-period LOW (`PulseCode::new(Low, period, Low, 0)`) so all three channels stay the same length
  /// and sample-aligned. Returns the number of symbols written (events + 1 for the end marker). Periods and
  /// the `$0` width are already bounded by the 15-bit RMT field: the generator floors the period at
  /// `min_period_ticks` (≥ `$0` + min-low) and `$0` is a small constant, so `period − $0` cannot underflow
  /// and neither half exceeds [`PulseCode::MAX_LEN`]; `new_clamped` guards the residual edge defensively.
  fn encode_channel(&mut self, axis: usize, ticks: &[StepEvent]) -> usize {
    let pulse = self.step_pulse_ticks;
    for (slot, event) in self.scratch[axis].iter_mut().zip(ticks.iter()) {
      let period = event.period_ticks.max(pulse + 1);
      *slot = if event.step[axis] {
        // A real step: HIGH for `$0` ticks, then LOW for the remainder of the period.
        let high = clamp_len(pulse);
        let low = clamp_len(period - pulse);
        PulseCode::new_clamped(Level::High, high, Level::Low, low)
      } else {
        // A silent tick: stay LOW for the whole period so this channel advances in lock-step with the
        // stepping channels without emitting an edge.
        PulseCode::new_clamped(Level::Low, clamp_len(period), Level::Low, 0)
      };
    }
    // Terminate the buffer; transmission hangs without an end marker (both lengths zero).
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
  /// step may rise. Because the sink method is synchronous (the generator calls it inline), the delay is a
  /// short blocking busy-spin rather than an await — a few microseconds on the dedicated core, negligible.
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
    for axis in 0..AXES {
      // `true` is the positive (increasing-step) direction → drive the DIR GPIO high; `false` → low. The
      // physical polarity is set by wiring; the planner/generator agree on this sign convention (DOC-02).
      self.dir[axis].set_level(if dir.dir[axis] { Level::High } else { Level::Low });
    }
    if self.dir_setup_us > 0 {
      // Hold off the first step for the `$29` setup delay. A blocking busy-delay is correct here: the sink
      // method is synchronous (the generator calls it inline) so it cannot await, and core 1 is dedicated to
      // motion, so a few-microsecond spin per block (not per step) is negligible.
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
          // The transmit failed to start; the channel is lost from this transaction, so we cannot restore
          // it. Surface a transport error — the executor abandons the block and the next reset re-inits RMT.
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
/// position, this publishes the true *live* (interpolated) MPos from the executed step counter.
///
/// ## Loop shape
/// 1. Pop the next block under the planner lock, peeking the following block's `entry_speed_sq` as this
///    block's `exit_speed_sq` (0.0 when the queue holds only this block, so it stops at rest). Release the
///    lock BEFORE any RMT transmit — the planner mutex is never held across step emission.
/// 2. If the queue was empty, await [`BLOCK_AVAILABLE`](crate::comms::BLOCK_AVAILABLE) (set by the consumer
///    after it enqueues a motion block) rather than polling — racing [`SOFT_RESET`](crate::comms::SOFT_RESET)
///    so a `0x18` wakes the executor promptly.
/// 3. At each block boundary (never mid-burst), honor a feed-hold: if [`FEED_HOLD`](crate::comms::FEED_HOLD)
///    is set, pause and await [`CYCLE_START`](crate::comms::CYCLE_START) before running the block (Stage-1
///    granularity is per-block; smooth deceleration is a later refinement).
/// 4. Run the block synchronously through the [`SegmentGenerator`], advancing the live [`StepCounter`] from
///    each emitted [`StepEvent`], then publish the live MPos and free-block count into [`MACHINE`].
///
/// A soft reset abandons the in-flight loop, zeroes the live position, and waits for the next block; the
/// consumer's `reset_pipeline` reconstructs the planner, so the executor simply observes an emptied queue.
pub async fn run(sink: &mut RmtStepSink) -> ! {
  let generator = SegmentGenerator::new(placeholder_motion_config());
  let steps_per_mm = placeholder_steps_per_mm();
  let mut counter = StepCounter::new();
  loop {
    // A soft reset at the top of the loop drops the live position so MPos returns to the origin in step with
    // the consumer's pipeline reset. The signal may also have been consumed by the consumer; observing it
    // here (non-blocking) and zeroing the counter keeps the published position consistent either way.
    if SOFT_RESET.try_take().is_some() {
      counter.reset();
      publish_position(&counter, &steps_per_mm).await;
    }

    // Honor a feed-hold at the block boundary: pause until cycle-start. Checking here (never mid-block)
    // matches DOC-02 step 4 — a burst in flight is never split. A soft reset also releases the hold.
    if FEED_HOLD.try_take().is_some() {
      match select(CYCLE_START.wait(), SOFT_RESET.wait()).await {
        Either::First(()) => {}
        Either::Second(()) => {
          counter.reset();
          publish_position(&counter, &steps_per_mm).await;
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
      Some((block, exit_speed_sq)) => {
        run_block(&generator, &block, exit_speed_sq, sink, &mut counter);
        publish_position(&counter, &steps_per_mm).await;
      }
      // Queue empty: await a freshly enqueued block instead of polling, racing a soft reset so a reset
      // arriving while idle is still observed promptly (it zeroes the position on the next iteration).
      None => match select(BLOCK_AVAILABLE.wait(), SOFT_RESET.wait()).await {
        Either::First(()) => {}
        Either::Second(()) => {
          counter.reset();
          publish_position(&counter, &steps_per_mm).await;
        }
      },
    }
  }
}

/// Pop the next block and compute its exit speed from the *following* block's planned entry speed (squared),
/// or `0.0` when this is the only queued block (it must stop at rest). Peeking after the pop reads the new
/// head, which is the block that will run next — exactly the exit-speed semantics the generator expects.
fn take_block(planner: &mut Planner) -> Option<(Block, f32)> {
  let block = planner.pop_block()?;
  let exit_speed_sq = planner.peek_block().map(|next| next.entry_speed_sq).unwrap_or(0.0);
  Some((block, exit_speed_sq))
}

/// Realize one block through the generator while advancing the live step counter. The counter is fed in
/// lock-step with the sink: `set_direction` latches the per-axis sign from the block, and each emitted
/// [`StepEvent`] advances the counter — so the live MPos mirrors exactly what the RMT channels emit. A
/// generator/sink error abandons the block (the next status report still reflects the steps emitted so far).
fn run_block(generator: &SegmentGenerator, block: &Block, exit_speed_sq: f32, sink: &mut RmtStepSink, counter: &mut StepCounter) {
  // Latch the live counter's direction from the same step signs the generator latches onto the sink, so the
  // counter advances each axis the correct way. A zero-length block never steps, so this is harmless then.
  counter.set_direction(DirState {
    dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0],
  });
  let mut tracking = CountingSink { inner: sink, counter };
  // The generator returns the tick count or a recoverable error; on error we simply stop emitting this
  // block. The error is not surfaced upward because Stage 1 has no alarm state machine yet (DOC-06/Stage 2);
  // the abandoned block leaves the machine where the last successful burst put it, which the live MPos shows.
  let _ = generator.run_block(block, exit_speed_sq, &mut tracking);
}

/// A [`StepSink`] decorator that advances a [`StepCounter`] as bursts pass through to the real RMT sink, so
/// the live position is derived from exactly the events the hardware emits (not re-derived from the block).
/// It forwards `set_direction`/`emit_burst` to the inner sink and tallies steps on the way through.
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
    // Emit first, then count: only count the steps that actually reached the hardware, so a transport
    // failure mid-burst does not advance the live position past what was physically emitted.
    self.inner.emit_burst(ticks)?;
    for event in ticks {
      self.counter.advance(event);
    }
    Ok(())
  }
}

/// Publish the live machine position (steps → mm) and the planner free-block count into the shared
/// [`MACHINE`] snapshot, so the status reporter answers `?` with the *live* MPos and a truthful `Bf:`. This
/// replaces the stub's planned-position publish (review finding #7: live vs planned position).
async fn publish_position(counter: &StepCounter, steps_per_mm: &[f32; AXES]) {
  let mpos = counter.position_mm(steps_per_mm);
  let free = {
    let guard = PLANNER.lock().await;
    match guard.as_ref() {
      Some(planner) => {
        let queued = planner.queued_len();
        firmware_core::planner::BLOCK_QUEUE_LEN.saturating_sub(queued) as u8
      }
      // No planner installed (init wiring bug, unreachable in a wired build): report the full queue free.
      None => firmware_core::planner::BLOCK_QUEUE_LEN as u8,
    }
  };
  let mut snap = MACHINE.lock().await;
  snap.mpos_mm = mpos;
  snap.planner_blocks_free = free;
}

/// Configure the three RMT TX step channels (ch0/1/2 on GPIO1/2/4) and the three DIR outputs (GPIO5/6/7),
/// drive STEP_EN (GPIO8) enabled (active-low → low) so the steppers hold, and assemble the [`RmtStepSink`].
///
/// The RMT source clock is 80 MHz; with `clk_divider = 80` one tick is exactly 1 µs, matching the default
/// `MotionConfig.tick_hz = 1_000_000` the generator assumes (so a `period_ticks` value is a microsecond
/// count). The default `memsize` is one block (48 symbols), exactly the burst cap — no adjacent-channel
/// borrowing. `STEP_EN` is returned so its lifetime is held for the program duration (dropping it would
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
) -> (RmtStepSink, Output<'static>) {
  use esp_hal::rmt::Rmt;
  use esp_hal::time::Rate;

  let config = placeholder_motion_config();
  // 80 MHz source / clk_divider 80 = 1 MHz = 1 tick per microsecond, matching `MotionConfig.tick_hz`.
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
  let sink = RmtStepSink::new([ch_x, ch_y, ch_z], [dir_x, dir_y, dir_z], &config, DIR_SETUP_US);
  (sink, step_enable)
}

/// Placeholder `$29` direction-setup delay in microseconds. TODO(DOC-00): load from esp-storage alongside
/// `$0`. DOC-02 cites a 2 µs practical minimum; 5 µs gives comfortable margin for the TMC2209 DIR-to-STEP
/// setup without measurably slowing motion (incurred once per block, not per step).
const DIR_SETUP_US: u32 = 5;
