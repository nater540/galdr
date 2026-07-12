//! Post-mortem crash breadcrumb in RTC_FAST memory (the lockup-capture path, DOC-00 follow-up).
//!
//! ## Why a breadcrumb, not live logging
//! On the ESP32-S3 the grbl GCode comms and any esp-println/defmt output share the ONE built-in
//! USB-Serial-JTAG peripheral (see `firmware/Cargo.toml`'s `defmt` feature comment), so you cannot stream a
//! program and watch a live trace at the same time — one host owns the port. And the lockup under investigation
//! is a BOTH-CORES-DEAD wedge: at the moment of death the core-0 log sink emits nothing live. The capture is
//! therefore a POST-MORTEM: the executor leaves a compact breadcrumb in reset-surviving memory as it runs, the
//! RTC watchdog ([`crate::comms::watchdog_feed`]) resets the wedged board, and the next boot reads the breadcrumb
//! back and emits it as a plain grbl `[MSG:...]` line over the NORMAL TX path.
//!
//! ## Retention contract — survives a WATCHDOG reset, NOT a power-cycle
//! The breadcrumb lives in RTC_FAST RAM via `#[ram(unstable(rtc_fast, persistent))]`. RTC_FAST is in the RTC
//! power domain, which a CPU / system reset (the RWDT stage-0 "reset main system" action this firmware arms in
//! `main`) does NOT clear — esp-hal's `persistent` attribute explicitly targets "watchdog timeouts" as a
//! survival case. It IS cleared by a power-on reset or a brown-out (the RTC domain loses power). So the operator
//! MUST let the watchdog bite (wait ~8 s for the auto-reset) and must NOT yank power: a power-cycle wipes the
//! breadcrumb. The `persistent` attribute zero-inits the region only on the FIRST (cold) boot; thereafter the
//! startup init is skipped, so a value written before a warm reset is readable after it. The [`MAGIC`] validity
//! word distinguishes a real breadcrumb from cold-boot zero/garbage.
//!
//! ## Cost on the hot path
//! The executor updates only [`record_stage`] — a SINGLE relaxed atomic store per stage transition (the same
//! points the `mtrace!` chain already fires). The heavier snapshot-ring write runs on the core-0 watchdog-feed
//! task (every 500 ms), NOT on the real-time burst path, so it adds nothing to step generation.
//!
//! ## Types and `Persistable`
//! esp-hal 1.1's `#[ram(unstable(rtc_fast, persistent))]` requires the static's type to implement the unsafe
//! marker `esp_hal::Persistable`. That trait is implemented for `portable_atomic::AtomicU32` (NOT
//! `core::sync::atomic::AtomicU32`) and for `[T; N] where T: Persistable`, so the breadcrumb is a
//! `[portable_atomic::AtomicU32; LEN]` — a plain `static` (no `static mut`, no `unsafe` access), which is `Sync`
//! and valid for any bit pattern a mid-write reset could leave (the `Persistable` safety contract).

use portable_atomic::{AtomicU32, Ordering};

/// Validity magic for the breadcrumb. A non-zero, non-`0xFFFF_FFFF` pattern distinct from cold-boot zero fill and
/// from erased-flash `0xFF`, so the boot path can tell a real, written-this-run breadcrumb from uninitialized
/// RTC_FAST garbage. Stamped by [`init_magic`] once at boot and required by [`Breadcrumb::is_valid`].
pub const MAGIC: u32 = 0x6A1D_C2A5;

/// Parse a decimal `&str` to a `u32` in const context (`u32::from_str_radix` is not const-stable here), wrapping on
/// overflow. Used only to turn the `build.rs`-emitted [`BUILD_ID`] string into a constant; a non-digit yields the
/// value parsed so far (the build script only ever emits digits).
const fn parse_u32_decimal(s: &str) -> u32 {
  let bytes = s.as_bytes();
  let mut acc: u32 = 0;
  let mut i = 0;
  while i < bytes.len() {
    let b = bytes[i];
    if b < b'0' || b > b'9' {
      break;
    }
    acc = acc.wrapping_mul(10).wrapping_add((b - b'0') as u32);
    i += 1;
  }
  acc
}

/// Per-build identity word (emitted by `build.rs` as `GALDR_BUILD_ID`). The panic handler stores a `.rodata`
/// file-string pointer into RTC_FAST; that pointer is only meaningful against the SAME flashed image. The boot
/// decoder dereferences it ONLY when the breadcrumb's stored build id matches this constant, so a panic breadcrumb
/// left by a DIFFERENT image is reported without its (now-stale) file string rather than reading garbage.
pub const BUILD_ID: u32 = parse_u32_decimal(env!("GALDR_BUILD_ID"));

/// Number of snapshots in the liveness ring. Four 500-ms snapshots cover the last ~2 s before a reset — enough to
/// show which core stopped advancing FIRST (e.g. core 1's beat frozen across several snapshots while core 0's kept
/// climbing) rather than only the final instant. Small so the whole breadcrumb fits comfortably in RTC_FAST.
pub const RING_LEN: usize = 4;

/// Word layout of the RTC_FAST breadcrumb array. Fixed indices keep the read/write sides in lockstep.
mod idx {
  /// Validity magic ([`super::MAGIC`]) — non-magic means cold boot / garbage, ignore the breadcrumb.
  pub const MAGIC: usize = 0;
  /// The most-recent executor stage marker (packed by [`super::pack_stage`]). Updated on the hot path by
  /// [`super::record_stage`] — a single store per stage transition — so on a wedge it holds the LAST stage the
  /// core-1 executor reached (e.g. `axis 1 wait_begin` pins an RMT TX-END wedge to channel 1).
  pub const LAST_STAGE: usize = 1;
  /// Monotonic snapshot sequence counter (so the boot dump can order the ring and report how many snapshots the
  /// core-1 beat stayed frozen before the reset).
  pub const SEQ: usize = 2;
  /// Ring head: index (mod [`super::RING_LEN`]) of the NEXT snapshot slot to write. The newest snapshot is at
  /// `(head + RING_LEN - 1) % RING_LEN`.
  pub const HEAD: usize = 3;
  /// Why the watchdog WITHHELD the feed to force this reset (a [`super::WithholdReason`], tagged). `0`/untagged
  /// means the dog fired some other way (the feed task itself stopped, or a non-watchdog reset). Written by the
  /// core-0 watchdog task just before it stops feeding, so the boot dump can name the wedge CLASS (core-1 RMT axis
  /// vs core-0 comms stall) independently of the core-1 executor's [`LAST_STAGE`] marker.
  pub const WITHHOLD: usize = 4;
  /// RMT channel-0 hang snapshot — packed flags ([`super::RMT_SNAPSHOT_TAG`] in the high half, then `end`/`thr`/
  /// `err`/`axis`/`nsym`). Written by [`super::record_rmt_hang`] when the core-1 motion executor's bounded RMT
  /// `wait()` poll-loop TIMES OUT (the hang), capturing the hardware state BEFORE the reset. Untagged when no RMT
  /// hang was captured this run. This is THE bifurcating datum: `end=1` means the transmission finished but our
  /// wait missed it (driver/usage bug); `end=0` means it genuinely never completed (memory/encoding/start issue).
  pub const RMT_FLAGS: usize = 5;
  /// The whole RMT `int_raw` register word at the hang (raw, non-clearing per-channel interrupt status).
  pub const RMT_INT_RAW: usize = 6;
  /// The whole RMT `int_st` register word at the hang (masked interrupt status).
  pub const RMT_INT_ST: usize = 7;
  /// The whole RMT `ch_tx_status(0)` register word at the hang — its `state` field (bits 22:24) is the channel
  /// FSM status (transmitting vs idle), the authoritative "is ch0 still running" signal.
  pub const RMT_TX_STATUS: usize = 8;
  /// The whole RMT `ch_tx_conf0(0)` register word at the hang (mem-size, wrap-en, continuous-mode, idle level...).
  pub const RMT_TX_CONF0: usize = 9;
  /// The monotonic burst (transmission) counter at the hang — which RMT transmission number since boot wedged.
  pub const RMT_BURST_SEQ: usize = 10;
  /// First word of the per-CORE-0-task comms-stage slots ([`super::COMMS_TASK_COUNT`] of them, one per
  /// instrumented core-0 task). Each holds a tagged [`super::CommsStage`] written by the task IMMEDIATELY before
  /// every `.await` it can park on, so on a wedge each slot names exactly the await that task is parked on (an
  /// idle-class stage = parked waiting for work; a specific stage = stuck mid-operation). Because the slots are
  /// PER TASK, concurrent tasks never clobber each other's marker — the stuck task is unambiguous. Distinct from
  /// the core-1 [`LAST_STAGE`] (motion); the two cores are reported independently.
  pub const COMMS_STAGE_BASE: usize = 11;
  /// PANIC breadcrumb flags: [`super::PANIC_TAG`] in the high half, the panicking core in bits 0..1, and a
  /// "location present" bit. Written by the custom `#[panic_handler]` (see `main`) with MINIMAL stack — a handful
  /// of raw stores, no formatting — then `software_reset()`. A panic is otherwise invisible (esp-backtrace 0.19's
  /// default handler halts forever with no breadcrumb), so this turns ANY panic (incl. a core-1 stack overflow)
  /// into a visible, recoverable, located report. Untagged when no panic was captured this run.
  pub const PANIC_FLAGS: usize = COMMS_STAGE_BASE + super::COMMS_TASK_COUNT;
  /// The panic location's file `&str` DATA POINTER (`Location::file().as_ptr() as u32`). The file literal lives in
  /// `.rodata`/flash, so the pointer is stable across a software reset OF THE SAME IMAGE — re-read it at BOOT (full
  /// stack) to format the report. Guarded by [`PANIC_BUILD_ID`] so a pointer from a DIFFERENT flashed image is not
  /// dereferenced as garbage.
  pub const PANIC_FILE_PTR: usize = PANIC_FLAGS + 1;
  /// The panic location file string LENGTH (`Location::file().len() as u32`).
  pub const PANIC_FILE_LEN: usize = PANIC_FLAGS + 2;
  /// The panic location LINE number (`Location::line()`).
  pub const PANIC_LINE: usize = PANIC_FLAGS + 3;
  /// A build identity word, written by the panic handler from a per-build constant ([`super::BUILD_ID`]). The boot
  /// decoder dereferences [`PANIC_FILE_PTR`] ONLY when this matches the running image's `BUILD_ID` — so a breadcrumb
  /// left by a DIFFERENT flashed image (where the same flash address holds different bytes) is reported without its
  /// (now-meaningless) file string rather than reading garbage.
  pub const PANIC_BUILD_ID: usize = PANIC_FLAGS + 4;
  /// The USB-TX-stall discriminator (packed by `firmware_core::diag::pack_usb_tx_stall`, tagged). Written by the
  /// `usb_tx` bounded-escape (after [`USB_TX_STALL_ESCAPE_K`](firmware_core::diag::USB_TX_STALL_ESCAPE_K)
  /// consecutive write timeouts) just before its `software_reset()`, capturing the firmware-only signals that
  /// distinguish a host-not-reading stall from a lost USB TX-done wake (H-A) or a downstream core-1 wedge (H-B) —
  /// the §11 streaming-lockup discriminator, post-mortem over CDC. Untagged when no USB-TX stall fired this run.
  pub const USB_TX_STALL: usize = PANIC_BUILD_ID + 1;
  /// Monotonic count of RMT-wait-timeout firings this run (the `motion.rs` `emit_burst` bounded-wait timeout
  /// branch). The RMT path STILL resets on its FIRST timeout (unchanged), so this is normally 0 or 1 — but
  /// carrying it lets the boot dump POSITIVELY exclude the RMT theory: `usb_tx_timeouts >= K && rmt_wait_timeouts
  /// == 0` nails the drumbeat as USB-TX, not RMT, by evidence rather than inference (§11.1). No tag — a plain
  /// saturating count, valid only when the breadcrumb [`MAGIC`] is set.
  pub const RMT_WAIT_COUNT: usize = PANIC_BUILD_ID + 2;
  /// A FREE-RUNNING heartbeat bumped by `watchdog_feed` every loop iteration (Signature-B instrumentation). Unlike
  /// the gated COMMS/MOTION beats, this advances UNCONDITIONALLY whenever the feed task runs, so its value at boot
  /// says whether `watchdog_feed` itself was ALIVE through the wedge (climbed → B-1, the dog was fed but fooled) or
  /// DIED (froze → B-2, the core-0 executor/feed task itself stopped). Survives a watchdog/software reset (RTC_FAST),
  /// NOT a power-cycle — so on a dead-zone hang that the new backstop converts into a reset, this distinguishes B-1 vs
  /// B-2. (The former `RECOVERED_COUNT` slot at `+3` was retired with the §18/§19 poll-based `usb_tx`, which eliminated
  /// the lost-wake CLASS — there are no recovered lost-wakes to count; the following slots renumbered down by one.)
  pub const WATCHDOG_HEARTBEAT: usize = PANIC_BUILD_ID + 3;
  /// The byte length of the response whose write stalled at the K-escape (the §13.1 single-chunk-widening
  /// discriminator). Carried in its OWN word because the packed [`super::USB_TX_STALL`] bit-word is full; the boot
  /// dump emits it as `len=N`. `len <= 64` ⇒ the stalled response was a single `write_async` chunk (all bytes pushed
  /// before the future parked) ⇒ the widening fix can recover it without truncation; `len > 64` stays a genuine
  /// stall. A plain saturating count, valid only alongside a captured [`super::USB_TX_STALL`].
  pub const USB_TX_STALL_LEN: usize = PANIC_BUILD_ID + 4;
  /// FREE-RUNNING total count of silently-swallowed `run_block` truncations (the §15 silent-skip probe), bumped by
  /// the core-1 executor at the `let _ = run_block_scaled` swallow site. Lives in RTC_FAST (NOT a plain `.bss`
  /// atomic) so it SURVIVES the K-escape `software_reset` that fires on a `usb_tx` wedge — otherwise a run that
  /// wedges+resets zeroes the count mid-run (the run-1 confound). NOT consumed/cleared by [`take_breadcrumb`]: it
  /// free-runs across resets for the whole power-on session, so a single end-of-run `$I` poll reads the cumulative
  /// total even through intervening resets. Saturating.
  pub const RUN_BLOCK_TRUNCATED: usize = PANIC_BUILD_ID + 5;
  /// FREE-RUNNING packed per-SOURCE split of [`RUN_BLOCK_TRUNCATED`] (§15): `twait` (RMT wait-err arm) in bits 0..10,
  /// `ttx` (transmit-start arm) in bits 10..20, `tlong` (burst-too-long) in bits 20..26, and the last-truncation
  /// axis+1 in bits 26..29. Each sub-count saturates at its field width (ample for a diagnostic — the split only needs
  /// to show WHICH arm dominates, not an exact magnitude). Same RTC_FAST survive-the-reset + free-run rationale.
  pub const RUN_BLOCK_TRUNC_SPLIT: usize = PANIC_BUILD_ID + 6;
  /// The [`super::BUILD_ID`] of the image that last wrote the free-running §15 truncation words. RTC_FAST survives a
  /// software reset AND an `espflash` flash, so the trunc words would otherwise carry a PRIOR image's bytes into a
  /// fresh build (read as a bogus huge `trunc`). Stamped at boot by [`super::reset_truncation_on_new_build`]; when it
  /// does NOT match this image's `BUILD_ID`, the trunc words are ZEROED first (a clean per-build baseline) while still
  /// surviving same-image software_resets (the actual requirement).
  pub const TRUNC_BUILD_ID: usize = PANIC_BUILD_ID + 7;
  /// The last-seen WINDOWED `usb_tx`-stall count (`firmware_core::diag::WindowedStallCounter::count`, §13.8), mirrored
  /// here on each capturing reset so the boot dump can tell a PURE consecutive stall run (Signature A) from an
  /// ALTERNATING recovered/stall pattern that resets the consecutive K counter yet still represents a degraded /
  /// intermittently-locking link. Emitted as `wnd=N` on the `[MSG:CRASH usbtx: ...]` line. Written DIAGNOSTIC-only
  /// (`capture-reset`) by [`super::record_usb_tx_stall_window`]; the DECODE side is unconditional so a production board
  /// still replays a prior diagnostic run's window. A plain count, meaningful only alongside a captured USB-TX stall.
  pub const USB_TX_STALL_WINDOW: usize = PANIC_BUILD_ID + 8;
  /// First word of the snapshot ring (after the comms-stage + panic slots). Each snapshot is [`super::SNAP_WORDS`]
  /// words.
  pub const RING_BASE: usize = PANIC_BUILD_ID + 9;
}

/// Number of instrumented core-0 tasks, each with its own comms-stage breadcrumb slot. One per [`CommsTask`].
pub const COMMS_TASK_COUNT: usize = 5;

/// The instrumented core-0 comms tasks. The discriminant is the slot index into the comms-stage breadcrumb words
/// ([`idx::COMMS_STAGE_BASE`] + this). APPEND only — the values are decoded after a reset.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CommsTask {
  /// The USB reader half ([`usb_rx`](crate::comms::usb_rx)).
  UsbRx = 0,
  /// The line-assembly half ([`line_assembler`](crate::comms::line_assembler)).
  LineAssembler = 1,
  /// The parser → planner consumer ([`comms_consumer`](crate::comms::comms_consumer)).
  Consumer = 2,
  /// The single USB writer ([`usb_tx`](crate::comms::usb_tx)).
  UsbTx = 3,
  /// The status reporter ([`status_responder`](crate::comms::status_responder)).
  Status = 4,
}

/// The specific `.await` park-points instrumented across the core-0 comms tasks. Each task writes the stage it is
/// ABOUT TO await on into its [`CommsTask`] slot, so a wedge pins the exact stuck await. Idle-class stages (a task
/// parked waiting for work) are NORMAL; a non-idle stage persisting on a wedge is the culprit. Stable on-wire
/// values — APPEND, never renumber.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CommsStage {
  /// `usb_rx` awaiting the next USB read (idle-class: returns as soon as the host sends — with RX live it should
  /// NOT sit here, so a wedge here means the USB read future itself never completes).
  RxRead = 0,
  /// `line_assembler` awaiting a byte from `RX_PIPE` / a line reset (idle-class).
  LineWaitByte = 1,
  /// `line_assembler` blocked sending a framed line into `LINE_QUEUE` (back-pressure: the consumer is behind).
  LineSendQueue = 2,
  /// `comms_consumer` at its main `select` awaiting the next line / reset / safety tick (idle-class).
  ConsumerWaitLine = 3,
  /// `comms_consumer` inside `handle_line` flushing settings to flash (NVS write — `multicore_auto_park`).
  ConsumerFlashSettings = 4,
  /// `comms_consumer` inside `handle_line` flushing coordinates to flash (NVS write — `multicore_auto_park`).
  ConsumerFlashCoords = 5,
  /// `comms_consumer` blocked enqueuing a response (`RESPONSE.send` — full channel ⇒ `usb_tx` is behind/stuck).
  ConsumerEnqueue = 6,
  /// `comms_consumer` waiting in `plan_command` back-pressure for the executor to free a planner slot.
  ConsumerPlanBackpressure = 7,
  /// `comms_consumer` awaiting a `G38.x` probe result from core 1.
  ConsumerProbeResult = 8,
  /// `comms_consumer` awaiting a `$H` homing result from core 1.
  ConsumerHomeResult = 9,
  /// `comms_consumer` running a `G4` dwell / an M0/M1/M6 pause / a quiesce wait (synchronized boundaries).
  ConsumerSyncWait = 10,
  // (slot 11 reserved — was a cross-core-lock marker; the PLANNER/SETTINGS/MACHINE locks are brief and a
  // lock-held-across-await wedge is captured on the MOTION side, so it is not separately instrumented here.)
  /// `usb_tx` awaiting the next response on the `RESPONSE` channel (idle-class).
  TxWaitResponse = 12,
  /// `usb_tx` blocked WRITING/flushing a response over USB (`write_all`/`flush` — the prime "host-facing output
  /// never completes" suspect: a stuck write here backs up `RESPONSE` and blocks every producer).
  TxWrite = 13,
  /// `status_responder` awaiting a `?` request (idle-class).
  StatusWaitRequest = 14,
  /// `status_responder` taking the `MACHINE`/`PLANNER` lock or enqueuing the report (`RESPONSE.send`).
  StatusBuildReport = 15,
}

/// Words per snapshot in the ring: `[seq, core0_beat, core1_beat]`. The per-snapshot stage is omitted — the
/// always-current [`idx::LAST_STAGE`] word holds the final stage, which is what the report needs.
pub const SNAP_WORDS: usize = 3;

/// Total length of the breadcrumb array: the fixed header words plus the snapshot ring.
pub const LEN: usize = idx::RING_BASE + RING_LEN * SNAP_WORDS;

/// The crash breadcrumb, resident in RTC_FAST so it survives the RWDT system reset (see the module docs for the
/// retention contract — survives a watchdog reset, NOT a power-cycle). `#[ram(unstable(rtc_fast, persistent))]`
/// places it in `.rtc_fast.persistent` and skips the startup re-init after the first cold boot, so a value
/// written before a warm reset is readable after it. `[AtomicU32; LEN]` is `Persistable` + `Sync`, so this is a
/// plain `static` with lock-free, `unsafe`-free access.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static BREADCRUMB: [AtomicU32; LEN] = {
  // `AtomicU32` is not `Copy`, so the array cannot be built with `[AtomicU32::new(0); LEN]`; `from_fn` over a
  // const-evaluable closure constructs each element. The initializer only runs on the FIRST cold boot (the
  // `persistent` attribute discards it on warm resets), so it is the power-on default, not a per-boot clear.
  [const { AtomicU32::new(0) }; LEN]
};

/// Executor stage markers, mirroring the `mtrace!` chain in `motion.rs`. Encoded into the breadcrumb's
/// [`idx::LAST_STAGE`] word (with an axis index for the RMT sub-stages) so the boot dump can name exactly where
/// the core-1 executor was when it wedged. The discriminants are stable on-wire values (a reboot decodes them),
/// so APPEND new stages — never renumber existing ones.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Stage {
  /// The executor task entered its drain loop (the very first stage; a reboot showing this means it wedged
  /// before popping any block — suspect the second-core bring-up).
  LoopEntered = 0,
  /// Took the `PLANNER` lock to pop the next block.
  LockAcquired = 1,
  /// Popped a block and is about to run it.
  BlockPopped = 2,
  /// Published the programmed feed and entered the segment generator.
  FeedPublished = 3,
  /// Entered `emit_burst` (encoding done, about to transmit the burst on all axes).
  EmitBurst = 4,
  /// Started the RMT `transmit` for the axis in the marker's axis field.
  AxisTransmit = 5,
  /// Began the blocking `wait()` for the axis in the marker's axis field. A reboot frozen HERE pins the wedge to
  /// that RMT channel's TX-END never firing — the prime core-1-stall suspect.
  AxisWaitBegin = 6,
  /// The blocking `wait()` for the axis returned (Ok or Err). Reaching this for the last axis means the burst
  /// completed — a wedge is NOT in the RMT wait for that burst.
  AxisWaitDone = 7,
  /// Awaiting a fresh block on the empty-queue idle `select` (a legitimately idle executor sits here — NOT a
  /// wedge by itself; the snapshot ring's frozen-beat-while-should-be-moving check disambiguates).
  IdleWaiting = 8,
}

/// Pack a [`Stage`] plus an axis index into the single `u32` stored at [`idx::LAST_STAGE`]. Layout: stage code in
/// the low byte, axis index in the next byte, a fixed tag in the high half so a partially-written or garbage word
/// is unlikely to decode as a plausible stage. The axis is meaningful only for the per-axis RMT stages; other
/// stages pass `0`.
pub fn pack_stage(stage: Stage, axis: u8) -> u32 {
  const TAG: u32 = 0x5347_0000; // "SG" tag in the high half — a cheap plausibility guard for the decode side.
  TAG | ((axis as u32) << 8) | (stage as u8 as u32)
}

/// Record the executor's current stage into the breadcrumb (hot path). A SINGLE relaxed atomic store — cheap
/// enough to call at every `mtrace!` site, including per-axis inside `emit_burst`. `Relaxed` is correct: this is a
/// best-effort post-mortem marker, not a synchronization point, and the read side runs only after a reset (no
/// concurrent reader to order against). The axis is `0` for non-per-axis stages.
pub fn record_stage(stage: Stage, axis: u8) {
  BREADCRUMB[idx::LAST_STAGE].store(pack_stage(stage, axis), Ordering::Relaxed);
}

/// Tag in the high half of a comms-stage slot, so a cold-boot zero / garbage word never decodes as a stage.
const COMMS_STAGE_TAG: u32 = 0x4353_0000; // "CS".

/// Record the await park-point a core-0 comms task is ABOUT to enter, into that task's dedicated breadcrumb slot.
/// A SINGLE relaxed store — called immediately before every `.await` a task can park on, so on a wedge the slot
/// names exactly the stuck await. Per-task slots mean concurrent tasks never clobber each other's marker. `Relaxed`
/// is correct: a best-effort post-mortem marker read only after a reset, no concurrent reader to order against.
pub fn record_comms_stage(task: CommsTask, stage: CommsStage) {
  BREADCRUMB[idx::COMMS_STAGE_BASE + task as u8 as usize].store(COMMS_STAGE_TAG | (stage as u8 as u32), Ordering::Relaxed);
}

/// Decode a packed comms-stage slot word into a short, stable label, or `None` if untagged (cold boot / a task
/// that never recorded a stage). Idle-class stages are suffixed implicitly by their names (`*-wait*`/`*-read`).
pub fn comms_stage_label(packed: u32) -> Option<&'static str> {
  if packed & 0xFFFF_0000 != COMMS_STAGE_TAG {
    return None;
  }
  let label = match (packed & 0xFF) as u8 {
    0 => "rx-read",
    1 => "line-wait-byte",
    2 => "line-send-queue",
    3 => "consumer-wait-line",
    4 => "consumer-flash-settings",
    5 => "consumer-flash-coords",
    6 => "consumer-enqueue",
    7 => "consumer-plan-backpressure",
    8 => "consumer-probe-result",
    9 => "consumer-home-result",
    10 => "consumer-sync-wait",
    11 => "consumer-reserved",
    12 => "tx-wait-response",
    13 => "tx-write",
    14 => "status-wait-request",
    15 => "status-build-report",
    _ => return None,
  };
  Some(label)
}

/// True when a decoded comms-stage is an IDLE-CLASS park (a task legitimately waiting for work), so the boot dump
/// can flag a non-idle stuck stage as the likely culprit. The idle-class stages are the routine top-of-loop waits.
pub fn comms_stage_is_idle(packed: u32) -> bool {
  if packed & 0xFFFF_0000 != COMMS_STAGE_TAG {
    return false;
  }
  // RxRead(0), LineWaitByte(1), ConsumerWaitLine(3), TxWaitResponse(12), StatusWaitRequest(14) are idle-class.
  matches!((packed & 0xFF) as u8, 0 | 1 | 3 | 12 | 14)
}

/// Tag in the high half of the [`idx::PANIC_FLAGS`] word, marking a real panic capture vs cold-boot garbage.
const PANIC_TAG: u32 = 0x5041_0000; // "PA".

/// Bit in [`idx::PANIC_FLAGS`] set when the panic carried a source [`core::panic::Location`] (file + line).
const PANIC_HAS_LOCATION: u32 = 1 << 8;

/// Record a PANIC into the breadcrumb from the custom `#[panic_handler]`, then the caller `software_reset()`s. This
/// MUST be minimal-stack and allocation/lock/format-free: the panic may be a STACK OVERFLOW, so the handler runs on
/// a nearly-exhausted stack. It does only a handful of raw [`Ordering::Relaxed`] stores into the fixed RTC_FAST
/// array — NO formatting (the file string is stored as a raw `.rodata` POINTER + length and re-read at BOOT where
/// there is full stack), NO locks, NO `match` beyond the `Option`. `core` is `Cpu::current() as u8`. When
/// `location` is `None`, the location bit stays clear and the pointer/len/line are zeroed. [`BUILD_ID`] is stamped
/// so the boot decoder only dereferences the pointer against the same image.
///
/// Marked `#[inline(never)]` so it is one flat frame (minimizing the Xtensa windowed-ABI window-spill depth) and so
/// the panic handler's frame stays small regardless of how `core::fmt`-heavy the surrounding panic machinery is.
#[inline(never)]
pub fn record_panic(core: u8, file_ptr: u32, file_len: u32, line: u32, has_location: bool) {
  let flags = PANIC_TAG | (if has_location { PANIC_HAS_LOCATION } else { 0 }) | ((core & 0x3) as u32);
  // Store fields individually (no struct on the stack). The order is irrelevant — the reset happens after all of
  // them in the caller, and there is no concurrent reader (the read side runs only after the reset).
  BREADCRUMB[idx::PANIC_FILE_PTR].store(file_ptr, Ordering::Relaxed);
  BREADCRUMB[idx::PANIC_FILE_LEN].store(file_len, Ordering::Relaxed);
  BREADCRUMB[idx::PANIC_LINE].store(line, Ordering::Relaxed);
  BREADCRUMB[idx::PANIC_BUILD_ID].store(BUILD_ID, Ordering::Relaxed);
  // Stamp the validity MAGIC too: normally it is already set from this boot's `init_magic`, but a panic in EARLY
  // init (before `init_magic` ran) would otherwise leave the breadcrumb looking invalid. One more cheap store.
  BREADCRUMB[idx::MAGIC].store(MAGIC, Ordering::Relaxed);
  // Flags LAST so a reader that somehow saw a torn write still requires the tag (written here) to decode anything.
  BREADCRUMB[idx::PANIC_FLAGS].store(flags, Ordering::Relaxed);
}

/// Why the core-0 watchdog task WITHHELD the feed to deliberately force a reset. Recorded in the breadcrumb so the
/// boot dump can name the wedge CLASS, distinct from the core-1 executor's last [`Stage`]. Stable on-wire values
/// (a reboot decodes them) — APPEND, never renumber.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WithholdReason {
  /// The core-1 motion executor's beat froze WHILE a block was in flight — a core-1 / RMT wedge (the [`Stage`]
  /// marker pins the exact axis/channel).
  Core1Motion = 1,
  /// The host was actively driving the firmware (RX bytes flowing) but the core-0 comms path stopped making
  /// forward progress (no `?` served / no response emitted) — a core-0 stuck-await wedge, with the Embassy
  /// executor still alive. This is the case the original unconditional feed could not catch.
  Core0Comms = 2,
  /// The DEAD-ZONE backstop (Signature B): responses were QUEUED (`RESPONSE` depth > 0) yet `usb_tx` completed no
  /// write for ~8 s — INDEPENDENT of host-active / executor-running. This catches the silent total lock the other
  /// two withholds structurally miss (host gone quiet + executor idle → neither fires → fed forever). The boot
  /// `wdog=` heartbeat then says whether the feed task was alive-but-fooled (B-1) or had itself died (B-2).
  DeadZone = 3,
  /// The core-0 async EXECUTOR itself stalled (§17.15 root-cause fix): the UNGATED executor-liveness beat froze while
  /// the survivable hardware ISR kept firing. Unlike [`Core0Comms`] / [`DeadZone`] (gated by host / response state,
  /// both quiescent in a full stall) this fires purely on "the core-0 executor stopped running its tasks" — the wedge
  /// that previously fed the dogs forever. Subsumes the other two core-0 withholds when the whole executor is dead.
  // Constructed only by the `capture-reset` survivable ISR; the production async feeder never reaches this class.
  #[cfg_attr(not(feature = "capture-reset"), allow(dead_code))]
  Core0ExecutorStall = 4,
}

/// Tag in the high half of the [`idx::WITHHOLD`] word, distinct from the [`pack_stage`] tag, so a garbage/zeroed
/// word never decodes as a plausible reason.
const WITHHOLD_TAG: u32 = 0x5748_0000; // "WH".

/// Record why the watchdog is about to withhold the feed (force a reset). A single relaxed store from the core-0
/// watchdog task; read only after the reset, so no ordering is needed.
pub fn record_withhold(reason: WithholdReason) {
  BREADCRUMB[idx::WITHHOLD].store(WITHHOLD_TAG | (reason as u8 as u32), Ordering::Relaxed);
}

/// Decode the withhold word into a short label for the boot report, or `None` if the dog fired some other way
/// (untagged / zero — e.g. the feed task itself stopped, which leaves no withhold marker).
pub fn withhold_label(packed: u32) -> Option<&'static str> {
  if packed & 0xFFFF_0000 != WITHHOLD_TAG {
    return None;
  }
  match (packed & 0xFF) as u8 {
    1 => Some("core1-motion-wedge"),
    2 => Some("core0-comms-wedge"),
    3 => Some("dead-zone-silent-lock"),
    4 => Some("core0-executor-stall"),
    _ => None,
  }
}

/// Tag in the high half of the [`idx::RMT_FLAGS`] word, marking a real RMT-hang capture vs cold-boot garbage.
const RMT_SNAPSHOT_TAG: u32 = 0x524D_0000; // "RM".

/// The raw RMT channel-0 hardware state captured at an RMT `wait()` timeout, plus the hung burst's identity. Passed
/// to [`record_rmt_hang`] by the core-1 executor when its bounded poll-loop times out, and decoded by the boot dump.
/// All the register words are whole-word reads (side-effect-free) so the boot report can re-derive any bit.
#[derive(Clone, Copy)]
pub struct RmtHang {
  /// The hung channel index (0 = X). Carried so the report is unambiguous even though ch0 is the suspect.
  pub axis: u8,
  /// `int_raw.ch_tx_end(axis)` — TRUE means the transmission FINISHED (TX-END asserted) but our wait missed it.
  pub tx_end: bool,
  /// `int_raw.ch_tx_thr_event(axis)` — the half-block threshold event (refill request) is pending.
  pub tx_thr: bool,
  /// `int_raw.ch_tx_err(axis)` — a transmission error is latched.
  pub tx_err: bool,
  /// The symbol count of the hung burst (events + 1 end marker).
  pub nsym: u16,
  /// The whole `int_raw` register word.
  pub int_raw: u32,
  /// The whole `int_st` register word.
  pub int_st: u32,
  /// The whole `ch_tx_status(axis)` word (its `state` field, bits 22:24, is the channel FSM status).
  pub tx_status: u32,
  /// The whole `ch_tx_conf0(axis)` word.
  pub tx_conf0: u32,
  /// The monotonic transmission counter at the hang (which RMT transmission since boot wedged).
  pub burst_seq: u32,
}

/// Record an RMT channel-0 hang snapshot into the breadcrumb (called from the core-1 executor on a `wait()`
/// timeout, BEFORE the board resets). Packs the end/thr/err bits + axis + symbol count into [`idx::RMT_FLAGS`]
/// (tagged), and stores the four whole register words + the burst counter in their dedicated slots. Relaxed
/// stores — read only after the reset, no concurrent reader. This is the diagnostic half of the timeout backstop.
pub fn record_rmt_hang(hang: &RmtHang) {
  let flags = RMT_SNAPSHOT_TAG
    | ((hang.tx_end as u32) << 15)
    | ((hang.tx_thr as u32) << 14)
    | ((hang.tx_err as u32) << 13)
    | (((hang.axis & 0x7) as u32) << 8)
    | (hang.nsym.min(0xFF) as u32);
  BREADCRUMB[idx::RMT_FLAGS].store(flags, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_INT_RAW].store(hang.int_raw, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_INT_ST].store(hang.int_st, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_TX_STATUS].store(hang.tx_status, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_TX_CONF0].store(hang.tx_conf0, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_BURST_SEQ].store(hang.burst_seq, Ordering::Relaxed);
}

/// Record the USB-TX-stall discriminator into the breadcrumb (called from `usb_tx`'s bounded escape, BEFORE the
/// board resets). The `word` is already packed by [`firmware_core::diag::pack_usb_tx_stall`] at the call site (the
/// pure, host-tested encoder), so the discriminator is a SINGLE relaxed store of an opaque tagged word — keeping the
/// bit layout owned by the tested module and crash.rs purely the RTC_FAST storage. `response_len` is the byte length
/// of the stalled response (the §13.1 single-chunk-widening discriminator), stored in its own word because the packed
/// bit-word is full. Both are read only after the reset, no ordering.
///
/// DIAGNOSTIC-only (`capture-reset` build, §17): only the diagnostic K-escape WRITES this word (production raises
/// `ALARM:17` without resetting, so there is no breadcrumb to write). The DECODE/boot-dump side stays unconditional so
/// a production board still REPLAYS a breadcrumb left by a prior diagnostic run.
#[cfg(feature = "capture-reset")]
pub fn record_usb_tx_stall(word: u32, response_len: u16) {
  BREADCRUMB[idx::USB_TX_STALL].store(word, Ordering::Relaxed);
  BREADCRUMB[idx::USB_TX_STALL_LEN].store(response_len as u32, Ordering::Relaxed);
}

/// Mirror the WINDOWED `usb_tx`-stall count (the §13.8 alternating-vs-pure discriminator from
/// [`firmware_core::diag::WindowedStallCounter`]) into the breadcrumb, alongside a captured USB-TX stall. A single
/// relaxed store of the latest window popcount, read only after the reset. DIAGNOSTIC-only (`capture-reset` build,
/// §17): only a capturing path writes it (production raises `ALARM:17` without resetting). Boot dump emits it as
/// `wnd=N`: a high `wnd` with a low consecutive `n` says the link is ALTERNATING-degraded (recoveries kept resetting
/// the K counter) rather than purely stuck — a distinct Signature class. The DECODE side stays unconditional so a
/// production board still replays a prior diagnostic run's window.
#[cfg(feature = "capture-reset")]
pub fn record_usb_tx_stall_window(count: u16) {
  BREADCRUMB[idx::USB_TX_STALL_WINDOW].store(count as u32, Ordering::Relaxed);
}

/// Bump the monotonic RMT-wait-timeout count (called from `motion.rs` `emit_burst`'s timeout branch, BEFORE its
/// existing reset). The RMT path is UNCHANGED — it still resets on the first timeout — so this is normally 0 or 1;
/// it exists so the boot dump can POSITIVELY show the RMT path did not fire while the USB-TX escape did (the §11.1
/// clean exclusion). Saturating so a (single, pre-reset) bump can never wrap. A read-modify-write relaxed store is
/// safe: the only writer is the single core-1 executor, read only after the reset.
pub fn bump_rmt_wait_timeout() {
  let n = BREADCRUMB[idx::RMT_WAIT_COUNT].load(Ordering::Relaxed).saturating_add(1);
  BREADCRUMB[idx::RMT_WAIT_COUNT].store(n, Ordering::Relaxed);
  // Stamp MAGIC so a timeout captured before this boot's `init_magic` (it runs early, but be safe) is still valid.
  BREADCRUMB[idx::MAGIC].store(MAGIC, Ordering::Relaxed);
}

/// Which `emit_burst` arm abandoned a block, for the §15 truncation split. Mirrors `motion::TruncationSource` but
/// kept here as a tiny discriminant so `crash.rs` owns the RTC_FAST bit layout (the motion enum carries the axis).
#[derive(Clone, Copy)]
pub enum TruncSource {
  /// The RMT `wait()` ERROR arm — the prime recurring suspect (channel survives).
  WaitErr,
  /// The RMT `transmit()` START arm — channel lost.
  TxStart,
  /// A `BurstTooLong` — an encoder/planner bug, a different root.
  BurstTooLong,
}

mod trunc_split {
  // Packed layout of [`super::idx::RUN_BLOCK_TRUNC_SPLIT`]: twait 0..10 (sat 1023), ttx 10..20 (sat 1023),
  // tlong 20..26 (sat 63), axis+1 26..29 (0..=7). All within 32 bits, no tag (validity comes from the breadcrumb
  // MAGIC alongside the free-running total).
  pub const TWAIT_SHIFT: u32 = 0;
  pub const TWAIT_MASK: u32 = 0x3FF << TWAIT_SHIFT;
  pub const TTX_SHIFT: u32 = 10;
  pub const TTX_MASK: u32 = 0x3FF << TTX_SHIFT;
  pub const TLONG_SHIFT: u32 = 20;
  pub const TLONG_MASK: u32 = 0x3F << TLONG_SHIFT;
  pub const AXIS_SHIFT: u32 = 26;
  pub const AXIS_MASK: u32 = 0x7 << AXIS_SHIFT;
}

/// Bump the FREE-RUNNING §15 truncation counters in RTC_FAST (called by the core-1 executor at the `run_block`
/// swallow site). Increments the total [`idx::RUN_BLOCK_TRUNCATED`] plus the per-source sub-count in the packed
/// [`idx::RUN_BLOCK_TRUNC_SPLIT`] word, and stores the truncation axis (+1) there. RTC_FAST + free-running (NOT
/// cleared on consume) so the count SURVIVES the K-escape `software_reset` and a single end-of-run `$I` poll reads
/// the cumulative-across-resets total. Saturating per field. `axis` is the RMT channel index (0=X..3=A). The only
/// writer is the single core-1 executor; read only after the run / a reset, so `Relaxed` is correct.
pub fn bump_run_block_truncated(source: Option<TruncSource>, axis: u8) {
  let total = BREADCRUMB[idx::RUN_BLOCK_TRUNCATED].load(Ordering::Relaxed).saturating_add(1);
  BREADCRUMB[idx::RUN_BLOCK_TRUNCATED].store(total, Ordering::Relaxed);
  // Stamp MAGIC so a truncation captured before this boot's `init_magic` is still a valid breadcrumb at the next boot.
  BREADCRUMB[idx::MAGIC].store(MAGIC, Ordering::Relaxed);
  let Some(src) = source else {
    return;
  };
  let word = BREADCRUMB[idx::RUN_BLOCK_TRUNC_SPLIT].load(Ordering::Relaxed);
  let mut twait = (word & trunc_split::TWAIT_MASK) >> trunc_split::TWAIT_SHIFT;
  let mut ttx = (word & trunc_split::TTX_MASK) >> trunc_split::TTX_SHIFT;
  let mut tlong = (word & trunc_split::TLONG_MASK) >> trunc_split::TLONG_SHIFT;
  match src {
    TruncSource::WaitErr => twait = (twait + 1).min(0x3FF),
    TruncSource::TxStart => ttx = (ttx + 1).min(0x3FF),
    TruncSource::BurstTooLong => tlong = (tlong + 1).min(0x3F),
  }
  let axis_plus1 = (axis as u32 + 1).min(0x7);
  let packed = (twait << trunc_split::TWAIT_SHIFT)
    | (ttx << trunc_split::TTX_SHIFT)
    | (tlong << trunc_split::TLONG_SHIFT)
    | (axis_plus1 << trunc_split::AXIS_SHIFT);
  BREADCRUMB[idx::RUN_BLOCK_TRUNC_SPLIT].store(packed, Ordering::Relaxed);
}

/// Read the FREE-RUNNING §15 truncation counters out of RTC_FAST for the live `$I` `[MSG:SKIP]` line: returns
/// `(total, twait, ttx, tlong, axis_plus1)`. These are NOT consumed — they free-run for the whole power-on session,
/// so the `$I` line shows the cumulative count even after an intervening K-escape reset. A cold power-on starts them
/// at zero (the `persistent` attribute zero-inits on the first boot only).
pub fn read_run_block_truncated() -> (u32, u32, u32, u32, u32) {
  let total = BREADCRUMB[idx::RUN_BLOCK_TRUNCATED].load(Ordering::Relaxed);
  let word = BREADCRUMB[idx::RUN_BLOCK_TRUNC_SPLIT].load(Ordering::Relaxed);
  let twait = (word & trunc_split::TWAIT_MASK) >> trunc_split::TWAIT_SHIFT;
  let ttx = (word & trunc_split::TTX_MASK) >> trunc_split::TTX_SHIFT;
  let tlong = (word & trunc_split::TLONG_MASK) >> trunc_split::TLONG_SHIFT;
  let axis_plus1 = (word & trunc_split::AXIS_MASK) >> trunc_split::AXIS_SHIFT;
  (total, twait, ttx, tlong, axis_plus1)
}

/// Bump the free-running watchdog heartbeat (called by `watchdog_feed` every loop iteration, Signature-B
/// instrumentation). A single relaxed read-modify-write — the feed task is the sole writer; read only after a reset.
/// Its boot value says whether `watchdog_feed` kept running through a wedge (climbed → B-1 dog-fooled) or died
/// (froze → B-2). Saturating so a long uptime never wraps to a misleadingly-small value. MAGIC is already set from
/// this boot's `init_magic` by the time the feed task first runs, so no extra stamp here.
pub fn bump_watchdog_heartbeat() {
  let n = BREADCRUMB[idx::WATCHDOG_HEARTBEAT].load(Ordering::Relaxed).saturating_add(1);
  BREADCRUMB[idx::WATCHDOG_HEARTBEAT].store(n, Ordering::Relaxed);
}

/// Give the FREE-RUNNING §15 truncation counters a clean per-BUILD baseline. RTC_FAST survives a software reset AND
/// an `espflash` flash, so without this a fresh image would inherit the PRIOR image's bytes at the trunc word
/// addresses and read a bogus huge `trunc` (the run-1 false alarm). Called ONCE at boot, BEFORE the executor can
/// bump anything: if the stored [`idx::TRUNC_BUILD_ID`] does NOT match this image's [`BUILD_ID`], ZERO the trunc
/// words and stamp the current build id. On a SAME-image software reset the ids match, so the counters are PRESERVED
/// (the survive-the-reset requirement); only a genuinely new flashed build resets them. Self-correcting even if the
/// build-id word itself held prior-image garbage (garbage != BUILD_ID ⇒ mismatch ⇒ zero + stamp).
pub fn reset_truncation_on_new_build() {
  if BREADCRUMB[idx::TRUNC_BUILD_ID].load(Ordering::Relaxed) != BUILD_ID {
    BREADCRUMB[idx::RUN_BLOCK_TRUNCATED].store(0, Ordering::Relaxed);
    BREADCRUMB[idx::RUN_BLOCK_TRUNC_SPLIT].store(0, Ordering::Relaxed);
    BREADCRUMB[idx::TRUNC_BUILD_ID].store(BUILD_ID, Ordering::Relaxed);
  }
}

/// Stamp the validity [`MAGIC`] into the breadcrumb. Called once at boot AFTER the previous run's breadcrumb has
/// been read back, so this run's markers/snapshots are recognized as valid on the NEXT boot. Idempotent.
pub fn init_magic() {
  BREADCRUMB[idx::MAGIC].store(MAGIC, Ordering::Relaxed);
}

/// Append a liveness snapshot to the ring (called by the core-0 watchdog-feed task every ~500 ms, OFF the
/// real-time path). Records the monotonic sequence number and the two side-of-the-board liveness beats
/// (`core0_beat` = the core-0 host-facing [`COMMS_PROGRESS`](crate::comms::COMMS_PROGRESS), `core1_beat` = the core-1
/// [`MOTION_LIVENESS`](crate::comms::MOTION_LIVENESS)), then advances the ring head — so the ring holds the last
/// [`RING_LEN`] snapshots before a reset and the boot dump can see which side's beat stopped advancing first. All
/// relaxed stores (best-effort post-mortem).
pub fn push_snapshot(core0_beat: u32, core1_beat: u32) {
  let seq = BREADCRUMB[idx::SEQ].load(Ordering::Relaxed).wrapping_add(1);
  BREADCRUMB[idx::SEQ].store(seq, Ordering::Relaxed);
  let head = (BREADCRUMB[idx::HEAD].load(Ordering::Relaxed) as usize) % RING_LEN;
  let base = idx::RING_BASE + head * SNAP_WORDS;
  BREADCRUMB[base].store(seq, Ordering::Relaxed);
  BREADCRUMB[base + 1].store(core0_beat, Ordering::Relaxed);
  BREADCRUMB[base + 2].store(core1_beat, Ordering::Relaxed);
  BREADCRUMB[idx::HEAD].store(((head + 1) % RING_LEN) as u32, Ordering::Relaxed);
}

/// One decoded liveness snapshot from the ring.
#[derive(Clone, Copy)]
pub struct Snapshot {
  /// Monotonic sequence number when this snapshot was taken (0 = an empty/never-written ring slot).
  pub seq: u32,
  /// The core-0 side beat at that snapshot — the host-facing [`COMMS_PROGRESS`](crate::comms::COMMS_PROGRESS).
  pub core0_beat: u32,
  /// The core-1 side beat at that snapshot — the [`MOTION_LIVENESS`](crate::comms::MOTION_LIVENESS) executor beat.
  pub core1_beat: u32,
}

/// A decoded PANIC capture from the breadcrumb. The file string is recovered at BOOT (full stack) from the stored
/// `.rodata` pointer+len — but ONLY when the stored build id matched the running image (else `file` is `None`, the
/// pointer being meaningless against a different image). `core` is the panicking CPU (0 = ProCpu, 1 = AppCpu).
#[derive(Clone, Copy)]
pub struct PanicReport {
  /// The panicking core: 0 = ProCpu (core 0), 1 = AppCpu (core 1). A core-1 panic is the stack-overflow prime
  /// suspect.
  pub core: u8,
  /// The source line number, or 0 when the panic carried no location.
  pub line: u32,
  /// The recovered source-file string, or `None` when the panic had no location OR the breadcrumb came from a
  /// different flashed image (stale pointer — see [`BUILD_ID`]).
  pub file: Option<&'static str>,
}

/// A decoded copy of the breadcrumb, read once at boot. Decoupled from the live RTC_FAST static so the boot path
/// can consume + clear the magic immediately (avoiding a re-emit of a stale crumb on a later, unrelated boot) yet
/// still hold the data to format the `[MSG:...]` line and re-emit it on the first `$I`/status after connect.
#[derive(Clone, Copy)]
pub struct Breadcrumb {
  valid: bool,
  /// The most-recent stage marker (packed) — the LAST place the executor was before the reset.
  pub last_stage: u32,
  /// The watchdog withhold reason (packed) — why the dog was deliberately starved to force this reset, if it was
  /// (a [`WithholdReason`], or untagged when the feed task simply stopped). Decoded via [`withhold_label`].
  pub withhold: u32,
  /// The captured RMT channel-0 hardware state, IF the core-1 executor's bounded RMT `wait()` poll-loop timed out
  /// this run (the hang). `None` when no RMT hang was captured. This is the decisive diagnostic for the ch0 wedge.
  pub rmt_hang: Option<RmtHang>,
  /// The per-core-0-task comms-stage slots (packed), indexed by [`CommsTask`]. Each names the await its task was
  /// parked on at the reset (decode via [`comms_stage_label`]; idle-class via [`comms_stage_is_idle`]). The slot
  /// holding a NON-idle stage on a comms wedge is the stuck task.
  pub comms_stages: [u32; COMMS_TASK_COUNT],
  /// The decoded PANIC capture, IF the custom panic handler ran this run (a panic / CPU fault / core-1 stack
  /// overflow). `None` when no panic was captured. This is an INDEPENDENT class from the watchdog/RMT/comms dumps.
  pub panic: Option<PanicReport>,
  /// The decoded USB-TX-stall discriminator, IF `usb_tx`'s bounded escape fired this run (the §11 drumbeat
  /// capture). `None` when no USB-TX stall was captured. Its [`UsbTxVerdict`](firmware_core::diag::UsbTxVerdict)
  /// names host-not-reading vs lost-TX-wake (H-A) vs core-1-wedged (H-B).
  pub usb_tx_stall: Option<firmware_core::diag::UsbTxStall>,
  /// The byte length of the response whose write stalled at the K-escape (carried in its own RTC_FAST word). The
  /// §13.1 single-chunk-widening discriminator: meaningful only alongside [`usb_tx_stall`](Self::usb_tx_stall), where
  /// `len <= 64` (one `write_async` chunk) means the stalled response was fully pushed before the future parked and
  /// the widening fix can recover it without truncation. Emitted as `len=N` on the boot line.
  pub usb_tx_stall_len: u16,
  /// The last-seen WINDOWED `usb_tx`-stall count (§13.8 alternating-vs-pure discriminator). Mirrored from
  /// [`firmware_core::diag::WindowedStallCounter`] on a capturing reset; emitted as `wnd=N`. A high `wnd` with a low
  /// consecutive `n` says the link was ALTERNATING-degraded (recoveries kept resetting the K counter) rather than
  /// purely stuck. `0` when no window was recorded (or cold boot). Meaningful only alongside [`usb_tx_stall`](Self::
  /// usb_tx_stall).
  pub usb_tx_stall_window: u32,
  /// The monotonic RMT-wait-timeout count this run (normally 0; 1 if the RMT path reset on its first timeout). With
  /// a captured [`usb_tx_stall`](Self::usb_tx_stall) whose `timeout_count >= K` and this `== 0`, the boot dump
  /// POSITIVELY excludes the RMT theory for the drumbeat (§11.1).
  pub rmt_wait_count: u32,
  /// The PRIOR run's free-running watchdog heartbeat (bumped by `watchdog_feed` each loop). Compared against this
  /// run's count (always small at boot) it shows whether the feed task ran through the wedge: a large value means
  /// `watchdog_feed` was ALIVE but FOOLED (B-1, the dog kept being fed); a small/frozen value means the feed task
  /// itself died (B-2). Emitted in the boot dump as `wdog=N`.
  pub watchdog_heartbeat: u32,
  /// The snapshots, NEWEST first (index 0 is the most recent). Empty-seq entries are filtered by the formatter.
  pub snapshots: [Snapshot; RING_LEN],
}

impl Breadcrumb {
  /// True when the breadcrumb carries this firmware's [`MAGIC`] — i.e. it was written by a previous run, not
  /// cold-boot zero/garbage. The boot dump is emitted only when this holds AND the reset was a watchdog/fault
  /// reset (see [`crate::comms::maybe_emit_crash_report`]).
  pub fn is_valid(&self) -> bool {
    self.valid
  }
}

/// Read the breadcrumb out of RTC_FAST into an owned [`Breadcrumb`], ordering the ring NEWEST-first, and CONSUME
/// it by clearing the magic. Clearing means a later boot that did NOT crash (or a second reboot) does not re-emit
/// this same stale crumb. Called once early in boot, before [`init_magic`] re-stamps the magic for THIS run.
pub fn take_breadcrumb() -> Breadcrumb {
  let valid = BREADCRUMB[idx::MAGIC].load(Ordering::Relaxed) == MAGIC;
  let last_stage = BREADCRUMB[idx::LAST_STAGE].load(Ordering::Relaxed);
  let withhold = BREADCRUMB[idx::WITHHOLD].load(Ordering::Relaxed);
  // Decode the RMT-hang snapshot iff its tag is present (an RMT `wait()` timeout was captured this run).
  let rmt_flags = BREADCRUMB[idx::RMT_FLAGS].load(Ordering::Relaxed);
  let rmt_hang = if rmt_flags & 0xFFFF_0000 == RMT_SNAPSHOT_TAG {
    Some(RmtHang {
      axis: ((rmt_flags >> 8) & 0x7) as u8,
      tx_end: rmt_flags & (1 << 15) != 0,
      tx_thr: rmt_flags & (1 << 14) != 0,
      tx_err: rmt_flags & (1 << 13) != 0,
      nsym: (rmt_flags & 0xFF) as u16,
      int_raw: BREADCRUMB[idx::RMT_INT_RAW].load(Ordering::Relaxed),
      int_st: BREADCRUMB[idx::RMT_INT_ST].load(Ordering::Relaxed),
      tx_status: BREADCRUMB[idx::RMT_TX_STATUS].load(Ordering::Relaxed),
      tx_conf0: BREADCRUMB[idx::RMT_TX_CONF0].load(Ordering::Relaxed),
      burst_seq: BREADCRUMB[idx::RMT_BURST_SEQ].load(Ordering::Relaxed),
    })
  } else {
    None
  };
  // The per-task comms-stage slots, in `CommsTask` order.
  let comms_stages: [u32; COMMS_TASK_COUNT] =
    core::array::from_fn(|i| BREADCRUMB[idx::COMMS_STAGE_BASE + i].load(Ordering::Relaxed));
  // Decode the PANIC capture iff its tag is present. The file string is recovered from the stored `.rodata`
  // pointer+len, but ONLY when the stored build id matches THIS image's `BUILD_ID` (else the pointer is a stale
  // address from a different flashed image — report the panic without the file rather than reading garbage).
  let panic_flags = BREADCRUMB[idx::PANIC_FLAGS].load(Ordering::Relaxed);
  let panic = if panic_flags & 0xFFFF_0000 == PANIC_TAG {
    let core = (panic_flags & 0x3) as u8;
    let line = BREADCRUMB[idx::PANIC_LINE].load(Ordering::Relaxed);
    let same_image = BREADCRUMB[idx::PANIC_BUILD_ID].load(Ordering::Relaxed) == BUILD_ID;
    let file = if panic_flags & PANIC_HAS_LOCATION != 0 && same_image {
      let ptr = BREADCRUMB[idx::PANIC_FILE_PTR].load(Ordering::Relaxed) as *const u8;
      let len = BREADCRUMB[idx::PANIC_FILE_LEN].load(Ordering::Relaxed) as usize;
      // SAFETY: `ptr`/`len` came from `Location::file()` of the SAME image (build id matched), which is a
      // `&'static str` literal in `.rodata`/flash. Flash is not reloaded across a software reset, so the address
      // and length are still valid and point at the same UTF-8 bytes. We read at BOOT, after flash cache is up.
      // A sanity bound on `len` guards against a torn write yielding an absurd length AND keeps the recovered file
      // string within the `[MSG:CRASH panic ...]` line's RESPONSE_CAPACITY budget (a real source path is far under).
      if !ptr.is_null() && len > 0 && len <= 120 {
        let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
        core::str::from_utf8(bytes).ok()
      } else {
        None
      }
    } else {
      None
    };
    Some(PanicReport { core, line, file })
  } else {
    None
  };
  // Decode the USB-TX-stall discriminator via the pure, host-tested decoder (returns `None` if untagged). The
  // monotonic RMT-wait-timeout count is a plain word, meaningful only alongside a valid breadcrumb.
  let usb_tx_stall = firmware_core::diag::decode_usb_tx_stall(BREADCRUMB[idx::USB_TX_STALL].load(Ordering::Relaxed));
  // The stalled response's byte length is carried in its own word (the packed bit-word is full); meaningful only
  // alongside a captured `usb_tx_stall` — a `0` reads as "no length recorded" / no stall this run.
  let usb_tx_stall_len = BREADCRUMB[idx::USB_TX_STALL_LEN].load(Ordering::Relaxed) as u16;
  // The windowed `usb_tx`-stall count (§13.8) — a plain count, meaningful only alongside a captured `usb_tx_stall`.
  // Decoded unconditionally so a production build replays a prior diagnostic run's window.
  let usb_tx_stall_window = BREADCRUMB[idx::USB_TX_STALL_WINDOW].load(Ordering::Relaxed);
  let rmt_wait_count = BREADCRUMB[idx::RMT_WAIT_COUNT].load(Ordering::Relaxed);
  let watchdog_heartbeat = BREADCRUMB[idx::WATCHDOG_HEARTBEAT].load(Ordering::Relaxed);
  // The newest snapshot is at `(head + RING_LEN - 1) % RING_LEN`; walk backwards so index 0 is the most recent.
  let head = (BREADCRUMB[idx::HEAD].load(Ordering::Relaxed) as usize) % RING_LEN;
  let snapshots = core::array::from_fn(|i| {
    let slot = (head + RING_LEN - 1 - i) % RING_LEN;
    let base = idx::RING_BASE + slot * SNAP_WORDS;
    Snapshot {
      seq: BREADCRUMB[base].load(Ordering::Relaxed),
      core0_beat: BREADCRUMB[base + 1].load(Ordering::Relaxed),
      core1_beat: BREADCRUMB[base + 2].load(Ordering::Relaxed),
    }
  });
  // Consume: clear the magic, the withhold word, the RMT-flags tag, the PANIC-flags tag, AND every comms-stage slot
  // so this crumb is reported exactly once and no stale marker bleeds into a later, unrelated reset. `init_magic`
  // re-stamps magic.
  BREADCRUMB[idx::MAGIC].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::WITHHOLD].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_FLAGS].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::PANIC_FLAGS].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::USB_TX_STALL].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::USB_TX_STALL_LEN].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::USB_TX_STALL_WINDOW].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::RMT_WAIT_COUNT].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::WATCHDOG_HEARTBEAT].store(0, Ordering::Relaxed);
  for i in 0..COMMS_TASK_COUNT {
    BREADCRUMB[idx::COMMS_STAGE_BASE + i].store(0, Ordering::Relaxed);
  }
  Breadcrumb {
    valid,
    last_stage,
    withhold,
    rmt_hang,
    comms_stages,
    panic,
    usb_tx_stall,
    usb_tx_stall_len,
    usb_tx_stall_window,
    rmt_wait_count,
    watchdog_heartbeat,
    snapshots,
  }
}

/// Decode a packed last-stage marker into a short, stable label (e.g. `"axis1:wait_begin"`). Returns `"?"` for a
/// word that does not carry the [`pack_stage`] tag or names an unknown stage (cold-boot garbage / a future
/// stage). The per-axis stages carry the axis index; others omit it.
pub fn stage_label(packed: u32) -> &'static str {
  // Guard on the high-half tag first: a word without it is garbage, not a stage.
  if packed & 0xFFFF_0000 != 0x5347_0000 {
    return "?";
  }
  let stage = (packed & 0xFF) as u8;
  // The per-axis stages render their axis via `stage_axis`; the label here is the stage name only. Kept as a flat
  // match so the on-wire discriminants and their names have one definition shared with `Stage`.
  match stage {
    0 => "loop_entered",
    1 => "lock_acquired",
    2 => "block_popped",
    3 => "feed_published",
    4 => "emit_burst",
    5 => "transmit",
    6 => "wait_begin",
    7 => "wait_done",
    8 => "idle_waiting",
    _ => "?",
  }
}

/// True when the packed stage is one of the per-axis RMT stages (`transmit`/`wait_begin`/`wait_done`), whose axis
/// index is meaningful and should be shown in the report.
pub fn stage_has_axis(packed: u32) -> bool {
  if packed & 0xFFFF_0000 != 0x5347_0000 {
    return false;
  }
  matches!((packed & 0xFF) as u8, 5..=7)
}

/// Extract the axis index from a packed stage marker (the second byte). Meaningful only when [`stage_has_axis`].
pub fn stage_axis(packed: u32) -> u8 {
  ((packed >> 8) & 0xFF) as u8
}

/// Determine which SIDE (if either) stopped advancing FIRST, by scanning the NEWEST-first snapshot ring for the
/// longest trailing run over which each side's beat did NOT change. `core0_beat` is the core-0 host-facing comms
/// progress, `core1_beat` the core-1 motion beat. Returns a short verdict label: `"motion-froze-first"`,
/// `"comms-froze-first"`, `"both-froze"`, `"no-stall"`, or `"insufficient-data"` (fewer than two valid snapshots).
/// "Froze first" = its beat was unchanged across MORE of the most-recent snapshots than the other side's — i.e. it
/// stopped advancing earlier in the run-up to the reset.
pub fn froze_first(snapshots: &[Snapshot; RING_LEN]) -> &'static str {
  // Count valid (seq != 0) snapshots, newest-first. Fewer than two means we cannot compare a delta.
  let valid: heapless::Vec<&Snapshot, RING_LEN> = snapshots.iter().filter(|s| s.seq != 0).collect();
  if valid.len() < 2 {
    return "insufficient-data";
  }
  // Trailing run length (newest-first) over which the beat is unchanged: how many of the most-recent snapshots
  // share the newest snapshot's beat value. A longer run = stopped advancing earlier.
  let newest = valid[0];
  let mut core0_frozen_run = 1usize;
  let mut core1_frozen_run = 1usize;
  for s in valid.iter().skip(1) {
    if s.core0_beat == newest.core0_beat {
      core0_frozen_run += 1;
    } else {
      // Stop counting at the first change; only the TRAILING (most-recent) unchanged run matters.
      break;
    }
  }
  for s in valid.iter().skip(1) {
    if s.core1_beat == newest.core1_beat {
      core1_frozen_run += 1;
    } else {
      break;
    }
  }
  let core0_stalled = core0_frozen_run >= 2;
  let core1_stalled = core1_frozen_run >= 2;
  match (core0_stalled, core1_stalled) {
    (false, false) => "no-stall",
    (true, true) => {
      // Both stalled; the one with the LONGER frozen run stopped first. A tie = both froze together.
      if core1_frozen_run > core0_frozen_run {
        "motion-froze-first"
      } else if core0_frozen_run > core1_frozen_run {
        "comms-froze-first"
      } else {
        "both-froze"
      }
    }
    (false, true) => "motion-froze-first",
    (true, false) => "comms-froze-first",
  }
}
