//! Feed/rapid/spindle override state (`Ov:`) and the real-time adjust clamp.

/// The grbl override bounds (DOC-08 §3, `docs/gcode-streaming.md`). Feed and spindle overrides clamp to
/// `[10, 200]` percent; the rapid override is one of three discrete values `{25, 50, 100}`. The default for
/// all three is 100 % (no scaling). These are single-sourced here so the real-time decoder, the executor's
/// scaling, and the host tests agree on one set of limits.
pub const OVERRIDE_MIN_PCT: u8 = 10;

/// The maximum feed / spindle override percentage (grbl's `MAX_FEED_RATE_OVERRIDE` / spindle equivalent).
pub const OVERRIDE_MAX_PCT: u8 = 200;

/// The default (no-scaling) override percentage for all three overrides, latched on boot and on a `100 %`
/// reset byte (`0x90`/`0x95`/`0x99`).
pub const OVERRIDE_DEFAULT_PCT: u8 = 100;

/// The live feed / rapid / spindle override percentages plus the spindle-stop and coolant toggle state,
/// mutated by the real-time override bytes (`0x90`–`0xA4`) and rendered as the `Ov:` status element. It is a
/// pure, `Copy` state machine so the increment / clamp / reset matrix for every byte is host-tested, and so
/// the `firmware` bin can hold it behind a synchronous `Cell` (the [`ControlState`] pattern) and mutate it
/// from the non-blocking real-time reader half without awaiting.
///
/// grbl applies an override the instant the byte arrives, without re-planning the queued blocks: the executor
/// reads the live value per segment and scales the step timing. A change is never acked (override bytes are
/// real-time commands) and must be visible in the next `?` (`Ov:` and the realized `FS:`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Overrides {
  /// Feed override percentage, clamped to `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]`. Scales the programmed feed
  /// of G1/G2/G3 (and jog) moves.
  pub feed: u8,
  /// Rapid override percentage, one of `{25, 50, 100}`. Scales the rapid (G0) max-rate cruise.
  pub rapid: u8,
  /// Spindle override percentage, clamped to `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]`. Scales the commanded RPM.
  pub spindle: u8,
  /// Spindle-stop toggle (`0x9E`): when `true` the spindle is force-stopped (overriding the commanded RPM)
  /// until toggled off. Reported via the accessory state; the actual spindle output is a DOC-05 stub.
  pub spindle_stop: bool,
  /// Flood-coolant toggle (`0xA0`): the latched flood on/off state. The actual coolant output is a DOC-06 stub.
  pub flood: bool,
  /// Mist-coolant toggle (`0xA1`): the latched mist on/off state. The actual coolant output is a DOC-06 stub.
  pub mist: bool,
}

impl Default for Overrides {
  fn default() -> Self {
    Self::new()
  }
}

impl Overrides {
  /// The power-on / post-reset overrides: all three at 100 %, no spindle-stop, coolant off. grbl resets the
  /// overrides to their defaults on a soft reset, so the `firmware` bin re-seeds this on `0x18`.
  pub const fn new() -> Self {
    Overrides {
      feed: OVERRIDE_DEFAULT_PCT,
      rapid: OVERRIDE_DEFAULT_PCT,
      spindle: OVERRIDE_DEFAULT_PCT,
      spindle_stop: false,
      flood: false,
      mist: false,
    }
  }

  /// Apply one real-time override byte (`0x90`–`0xA4`), mutating the relevant override in place with grbl's
  /// clamping. Returns `true` if any field changed (so the caller can signal the executor / spindle and the
  /// `Ov:` cadence), `false` if the byte left the state unchanged (grbl ignores a no-op override). Unknown
  /// bytes in the range (e.g. the unused `0x98`, the `0xA2`/`0xA3`/`0xA4` tool/probe bytes Phase E does not
  /// model) are accepted as no-ops, never panicking.
  pub fn apply(&mut self, byte: u8) -> bool {
    let before = *self;
    match byte {
      // Feed override: reset-100, ±10, ±1, clamped to [MIN, MAX].
      0x90 => self.feed = OVERRIDE_DEFAULT_PCT,
      0x91 => self.feed = clamp_override(self.feed as i16 + 10),
      0x92 => self.feed = clamp_override(self.feed as i16 - 10),
      0x93 => self.feed = clamp_override(self.feed as i16 + 1),
      0x94 => self.feed = clamp_override(self.feed as i16 - 1),
      // Rapid override: one of the three discrete grbl values.
      0x95 => self.rapid = 100,
      0x96 => self.rapid = 50,
      0x97 => self.rapid = 25,
      // Spindle override: reset-100, ±10, ±1, clamped to [MIN, MAX].
      0x99 => self.spindle = OVERRIDE_DEFAULT_PCT,
      0x9A => self.spindle = clamp_override(self.spindle as i16 + 10),
      0x9B => self.spindle = clamp_override(self.spindle as i16 - 10),
      0x9C => self.spindle = clamp_override(self.spindle as i16 + 1),
      0x9D => self.spindle = clamp_override(self.spindle as i16 - 1),
      // Spindle-stop toggle and the coolant toggles are boolean flips.
      0x9E => self.spindle_stop = !self.spindle_stop,
      0xA0 => self.flood = !self.flood,
      0xA1 => self.mist = !self.mist,
      // Any other byte in the classified override range (0x98 unused, 0xA2/0xA3/0xA4 tool/probe) is a no-op.
      _ => {}
    }
    *self != before
  }

  /// The realized feed rate in mm/min for a programmed feed, after the feed override and a clamp to the most
  /// restrictive axis max-rate. Scaling UP must never exceed the configured `$110-112` rate (grbl clamps the
  /// override-scaled feed to the machine's rate limits), so `max_rate_mm_min` is the per-block dominant rate
  /// ceiling the caller supplies (`f32::INFINITY` for "no clamp", e.g. when the ceiling is unknown).
  pub fn scaled_feed(&self, programmed_mm_min: f32, max_rate_mm_min: f32) -> f32 {
    let scaled = programmed_mm_min * (self.feed as f32 / 100.0);
    scaled.clamp(0.0, max_rate_mm_min.max(0.0))
  }

  /// The realized rapid rate in mm/min for a rapid (G0) cruise rate, after the rapid override. A rapid is
  /// already governed by the axis max-rate, so the override only ever scales it DOWN (25/50/100 %); no extra
  /// max-rate clamp is needed beyond the planner's existing rate limiting, but the value is floored at 0.
  pub fn scaled_rapid(&self, rapid_mm_min: f32) -> f32 {
    (rapid_mm_min * (self.rapid as f32 / 100.0)).max(0.0)
  }

  /// The realized spindle speed in RPM after the spindle override and the spindle-stop toggle. A `spindle_stop`
  /// forces 0 regardless of the commanded RPM (grbl's `0x9E` halts the spindle); otherwise the commanded RPM is
  /// scaled by the override percentage and rounded to whole RPM (the `FS:` field is integer RPM).
  pub fn scaled_rpm(&self, programmed_rpm: u16) -> u16 {
    if self.spindle_stop {
      return 0;
    }
    let scaled = programmed_rpm as f32 * (self.spindle as f32 / 100.0);
    libm::roundf(scaled.max(0.0)) as u16
  }
}

/// Clamp an override candidate (after an increment that may under/overflow the byte range) into the grbl
/// `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]` band. The arithmetic is done in `i16` so a `-1`/`-10` from the
/// minimum cannot wrap a `u8`.
fn clamp_override(candidate: i16) -> u8 {
  candidate.clamp(OVERRIDE_MIN_PCT as i16, OVERRIDE_MAX_PCT as i16) as u8
}
