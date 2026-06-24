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
  /// First word of the snapshot ring. Each snapshot is [`super::SNAP_WORDS`] words: `[seq, core0_beat,
  /// core1_beat]`. The current stage is carried by the always-updated [`LAST_STAGE`] word, not per snapshot.
  pub const RING_BASE: usize = 5;
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
    _ => None,
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
  // Consume: clear the magic AND the withhold word so this crumb is reported exactly once and a stale withhold
  // reason cannot bleed into a later, unrelated reset. `init_magic` re-stamps the magic for the new run.
  BREADCRUMB[idx::MAGIC].store(0, Ordering::Relaxed);
  BREADCRUMB[idx::WITHHOLD].store(0, Ordering::Relaxed);
  Breadcrumb { valid, last_stage, withhold, snapshots }
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
