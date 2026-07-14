//! Input-pin (`Pn:`) status DTO and the signal-capability advertisement.

use super::*;
use heapless::String;

/// The asserted machine input pins, rendered as the `Pn:` status element (DOC-08 §4). Each `bool` is the
/// LOGICAL asserted state (after any invert handling, which the `firmware` bin applies when it samples the
/// raw GPIO — e.g. the probe via [`crate::hal_traits::probe_triggered`]), so this type carries no electrical
/// or settings knowledge and the `Pn:` letter assembly stays a pure, host-tested function. grbl OMITS the
/// `Pn:` element entirely when no pin is asserted, which [`ResponseWriter::status_report`] honors.
///
/// The fields cover the grbl signal letters the Galdr hardware can source: `P` probe (wired today, Phase C),
/// `X`/`Y`/`Z` limits, `D` door, `H` feed-hold input, `R` reset/e-stop, `S` cycle-start input. The limit /
/// door / control inputs are DOC-06 hardware that is not wired yet, so the bin reports them `false` (a clean
/// stub) until the GPIO backend lands — but the assembly logic is complete and tested so wiring a pin is a
/// one-line change in the bin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinReport {
  /// `P` — probe asserted (touch made). Wired today via the Phase-C probe input.
  pub probe: bool,
  /// `X`/`Y`/`Z` — limit switches asserted, `[X, Y, Z]` (DOC-06, not wired yet).
  pub limits: [bool; AXIS_COUNT],
  /// `D` — safety-door input asserted (DOC-06, not wired yet).
  pub door: bool,
  /// `H` — feed-hold input asserted (the optional FHOLD GPIO, DOC-06, not wired yet).
  pub hold: bool,
  /// `R` — reset / e-stop input asserted (DOC-06, not wired yet).
  pub reset: bool,
  /// `S` — cycle-start input asserted (the optional CYCSTART GPIO, DOC-06, not wired yet).
  pub cycle_start: bool,
}

impl PinReport {
  /// All pins unasserted — the power-on / quiescent view. `const` so it can seed the `const`
  /// [`MachineSnapshot::idle`] without a lazy default.
  pub const fn new_idle() -> Self {
    PinReport {
      probe: false,
      limits: [false; AXIS_COUNT],
      door: false,
      hold: false,
      reset: false,
      cycle_start: false,
    }
  }

  /// Whether any input pin is asserted. When `false`, [`ResponseWriter::status_report`] omits the `Pn:`
  /// element entirely (grbl's rule), so a quiescent machine's report never carries an empty `Pn:`.
  pub fn any(&self) -> bool {
    self.probe || self.limits.iter().any(|&l| l) || self.door || self.hold || self.reset || self.cycle_start
  }

  /// Append the set pin letters to `out` in grbl's documented order (`P` probe, `X`/`Y`/`Z` limits, `D`
  /// door, `H` hold, `R` reset, `S` cycle-start), writing nothing for an unset pin. For the `Pn:` status
  /// element each `true` means "asserted now"; for the `$I+` `[SIGNALS:]` capability line (via
  /// [`SIGNAL_CAPABILITIES`]) each `true` means "this input exists in the build" — the letter assembly is the
  /// same either way, which is exactly why both reuse this one function. The caller wraps it with the relevant
  /// tag. Returns [`FmtError`] only on a (never, with a correctly sized buffer) capacity failure.
  pub fn write_letters<const N: usize>(&self, out: &mut String<N>) -> Result<(), FmtError> {
    if self.probe {
      out.push('P').map_err(|_| FmtError)?;
    }
    // Limit letters in axis order, derived from [`AXIS_LETTERS`] (not a private `['X','Y','Z']` literal) so the
    // `Pn:`/`[SIGNALS:]` vocabulary can never drift from the `[AXS:]`/`[DRIVER:]` one. `limits` is sized to
    // [`AXIS_COUNT`], so the two arrays line up index-for-index; the A axis has no limit switch (its `limits`
    // slot stays `false`, DOC-10), so no `A` letter is ever emitted here.
    for (&letter, &triggered) in AXIS_LETTERS.iter().zip(self.limits.iter()) {
      if triggered {
        out.push(letter).map_err(|_| FmtError)?;
      }
    }
    if self.door {
      out.push('D').map_err(|_| FmtError)?;
    }
    if self.hold {
      out.push('H').map_err(|_| FmtError)?;
    }
    if self.reset {
      out.push('R').map_err(|_| FmtError)?;
    }
    if self.cycle_start {
      out.push('S').map_err(|_| FmtError)?;
    }
    Ok(())
  }
}

/// The input signals this firmware build can read, rendered as the `$I+` `[SIGNALS:]` capability line. Galdr
/// sources the `P` probe input and the `X`/`Y`/`Z` limit switches; the door/hold/reset/cycle-start control
/// inputs have no GPIO budgeted, so they stay `false` and are omitted. Because [`build_info`] renders this with
/// the very same [`PinReport::write_letters`] the `Pn:` status element uses, the advertised capability set and
/// the runtime status letters can never use a different letter vocabulary. The A axis has no limit switch
/// (DOC-10 rotary), so only `X`/`Y`/`Z` limits are advertised.
pub const SIGNAL_CAPABILITIES: PinReport = PinReport {
  probe: true,
  limits: [true, true, true, false],
  door: false,
  hold: false,
  reset: false,
  cycle_start: false,
};
