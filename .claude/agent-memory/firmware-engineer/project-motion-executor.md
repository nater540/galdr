---
name: project-motion-executor
description: Galdr firmware-bin DOC-02 motion executor reality — verified esp-rtos second-core/InterruptExecutor API, esp-hal 1.0 blocking RMT API, the sync StepSink bridge, and the core-1 wiring decisions.
metadata:
  type: project
---

The DOC-02 core-1 motion executor + RMT StepSink was implemented 2026-06-16 (branch
`firmware/core-pipeline-and-streaming`), replacing `block_drain_stub`. Lives in
`crates/firmware/src/motion.rs`. The DOC-01/DOC-02 esp-hal-embassy sketch is WRONG; the verified reality:

**Second core (esp-rtos 0.2, xtensa): `esp_rtos::start_second_core::<STACK>(cpu_ctrl, int0, int1, &mut Stack, func)`.**
NOT `CpuControl::start_app_core`. Signature (xtensa): `(CPU_CTRL, SoftwareInterrupt<'static,0>,
SoftwareInterrupt<'static,1>, &'static mut Stack<N>, impl FnOnce()+Send+'static)`. The esp-rtos SMP
SCHEDULER consumes SWI 0 AND SWI 1 (context-switch IPIs on both cores). `start()` (core 0) takes only the
timer on xtensa (no int0 arg — that's `#[cfg(riscv)]`). So the Embassy `InterruptExecutor<SWI>` for motion
must use a FREE swi → **SWI=2 at Priority3**. SoftwareInterruptControl::new(peripherals.SW_INTERRUPT) gives
software_interrupt0..3. `func` runs on core 1 (pinned); create+start the InterruptExecutor INSIDE func so its
SWI-2 handler registers on core 1, then `func` returns (esp-rtos idles core 1's main thread; the interrupt
executor keeps running). Need `static_cell::StaticCell<Stack<N>>` (8 KiB) and `StaticCell<InterruptExecutor<2>>`.
`InterruptExecutor::start(Priority::Priority3) -> SendSpawner`; spawn the task via the SendSpawner.

**Blocking RMT EXISTS (esp-hal 1.0.0) — the sync StepSink works, no recording-sink fallback needed.**
`Rmt::new(peripherals.RMT, Rate::from_mhz(80)) -> Result<Rmt<'_, Blocking>>` — do NOT call `.into_async()`
(that switches to the async/future transmit path). `rmt.channel0/1/2` are `ChannelCreator`;
`.configure_tx(pin, TxChannelConfig) -> Result<Channel<'ch, Blocking, Tx>>`. Blocking TX is move-ownership:
`Channel::transmit(self, &[impl Into<PulseCode>+Copy]) -> Result<SingleShotTxTransaction>` (CONSUMES the
channel; data MUST end with `PulseCode::end_marker()` or returns `Error::EndMarkerMissing`), then
`txn.wait() -> Result<Channel, (Error, Channel)>` (blocks polling; RETURNS the channel back, or in the err
tuple). So the StepSink holds `Option<Channel<Blocking,Tx>>` per axis and take()/restore()s each burst. To keep
the three channels SAMPLE-ALIGNED: start all three `.transmit()` FIRST (hardware fires concurrently), THEN
`.wait()` each — never transmit→wait→transmit serially.

**TxChannelConfig is `procmacros::BuilderLite`** → `with_<field>`. DOC's `with_mem_block_symbols(48)` does NOT
exist; the field is `memsize: u8` (BLOCKS, default 1 = 48 symbols/one block) → use the default (or
`.with_memsize(1)`). Other builders: `with_clk_divider(u8)`, `with_idle_output_level(Level)`,
`with_idle_output(bool)`. `PulseCode::new(Level, len1:u16, Level, len2:u16)` panics if len>0x7FFF (15-bit);
`PulseCode::try_new(Level, impl TryInto<u16>, ...)->Option` for u32 inputs; `PulseCode::end_marker()`.
`Level`/`Output`/`OutputConfig` from `esp_hal::gpio`; `Output::new(pin, Level, OutputConfig)` +
`.set_level(Level)`/`set_high`/`set_low`. `Rate` from `esp_hal::time`.

**Sync/async bridge:** `SegmentGenerator::run_block(&block, exit_speed_sq, &mut sink)` is SYNC and calls
`sink.emit_burst` synchronously. The RMT sink uses the BLOCKING transmit+wait (above). Blocking core 1 during a
sub-ms burst is INTENDED ("uncontested CPU, preempts nothing") — core 1 runs only motion_executor. The only
`.await` in the executor is between blocks (await BLOCK_AVAILABLE) + feed-hold pause. `$29` dir-setup delay is a
blocking busy-spin inside `set_direction` (can't await in the sync trait); a few µs is negligible. NOTE: the
RMT/PulseCode encoding + core-1 glue are COMPILE-VERIFIED only (no hardware); host-tested logic = step→mm
conversion + direction→sign mapping, extracted to pure fns with tests.

**BLOCK_AVAILABLE / feed-hold / soft-reset wiring (all in the bin, NOT the pure planner lib):**
`BLOCK_AVAILABLE: Signal` added in comms.rs; `plan_command` signals it after a Queued (motion) outcome. Executor
awaits it when the queue is empty. Feed-hold: checked at the burst BOUNDARY between blocks; if FEED_HOLD set,
await CYCLE_START. UPDATED 2026-06-16 (see [[project-motion-executor-review-fixes]]): the executor uses a
DEDICATED `MOTION_RESET` Signal + `MOTION_RESET_PENDING` AtomicBool (NOT shared SOFT_RESET — Signal wakes one
waiter); the CountingSink aborts mid-block between bursts on the pending flag. Live MPos now flows through
`LIVE_POSITION: [AtomicI32; AXES]` published per-burst (not block-boundary into MACHINE); `status_responder`
reads the atomics + `steps_to_mm` and the live planner depth. The executor is the SINGLE owner of the reset MPos
(reset_pipeline no longer writes it). Burst cap is 47 events (47+marker = one 48-symbol RMT block).

**steps_per_mm source:** shared accessor `comms::placeholder_motion_config()` /
`placeholder_planner_config().steps_per_mm` (TODO load both from esp-storage). MotionConfig default = 1 MHz tick
so RMT clk_divider MUST be 80 (80 MHz/80 = 1 MHz, 1 tick = 1 µs) to match the generator's tick_hz.

See [[project-motion-contracts]], [[project-consumer-pipeline]], [[project-firmware-bringup]], [[project-hardware-map]].
</content>
</invoke>
