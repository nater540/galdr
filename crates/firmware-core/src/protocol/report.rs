//! Position / machine-snapshot / coordinate report DTOs and the refresh reporter.

use super::*;

/// Which position element a `<...>` status report carries, selected by the `$10` mask bit 0 (DOC-08 §4):
/// grbl/grblHAL report EITHER machine position (`MPos:`) OR work position (`WPos:`), never both, and a host
/// reconstructs the other from the `WCO:` element. Held in the [`MachineSnapshot`] so the formatter is a pure
/// function of the snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PositionReport {
  /// Report `MPos:` (machine position). The `$10` default — bit 0 set.
  Machine,
  /// Report `WPos:` (work position = `MPos − WCO`). Selected when `$10` bit 0 is clear.
  Work,
}

impl PositionReport {
  /// Derive the position-report mode from the `$10` status-report mask: bit 0 set ⇒ machine position,
  /// clear ⇒ work position. This is the one place the mask bit becomes a reporting choice, so the firmware
  /// bin and the formatter agree on the `$10` semantics.
  pub fn from_status_mask(mask: u8) -> Self {
    if mask & STATUS_MASK_MACHINE_POSITION != 0 {
      PositionReport::Machine
    } else {
      PositionReport::Work
    }
  }
}

/// An immutable, `Copy` snapshot of the live machine state the status formatter renders. The `firmware`
/// bin fills this from shared atomics/cells (live MPos from the motion executor, the active WCO from the
/// coordinate model, buffer free-counts from the planner queue and RX path) and hands it to
/// [`ResponseWriter::status_report`]; keeping the formatter pure over a snapshot is what makes status
/// reporting host-testable.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MachineSnapshot {
  /// Current run-state (report's first field).
  pub state: MachineState,
  /// Machine position in mm, `[X, Y, Z]`. Rendered as `MPos:` directly, or used (with `wco_mm`) to derive
  /// `WPos:` when [`position_report`](MachineSnapshot::position_report) is [`PositionReport::Work`].
  pub mpos_mm: [f32; AXIS_COUNT],
  /// The active Work Coordinate Offset in mm, `[X, Y, Z]` (= `G54..59[active] + G92 + TLO`). Used to derive
  /// `WPos = MPos − WCO` and rendered as the `WCO:` element on the refresh cadence.
  pub wco_mm: [f32; AXIS_COUNT],
  /// Which position element to report (`MPos:` vs `WPos:`), from the `$10` mask.
  pub position_report: PositionReport,
  /// Whether to include the `WCO:` element in THIS report (the change/periodic-refresh cadence — see
  /// [`RefreshReporter`]). grbl emits `WCO:` on change and at least every ~10-30 reports, not every report.
  pub include_wco: bool,
  /// The REALIZED feed rate in mm/min (`FS:` first field): the programmed feed scaled by the active feed
  /// override and clamped to the axis max-rate (or the rapid rate scaled by the rapid override for a G0). The
  /// `firmware` bin computes this from the executing block and the live [`Overrides`] (Phase E) so `FS:`
  /// reflects what the machine is actually doing, not the raw programmed word.
  pub feed_mm_min: f32,
  /// The REALIZED spindle speed in RPM (`FS:` second field): the commanded RPM scaled by the active spindle
  /// override (zero when the spindle-stop toggle is set). Computed by the bin from the live [`Overrides`].
  pub spindle_rpm: u16,
  /// Planner blocks free (`Bf:` first field).
  pub planner_blocks_free: u8,
  /// RX buffer bytes free (`Bf:` second field).
  pub rx_bytes_free: u16,
  /// The asserted input pins for the `Pn:` element (Phase E). Omitted from the report when none is asserted.
  pub pins: PinReport,
  /// The live feed / rapid / spindle override percentages for the `Ov:` element (Phase E).
  pub overrides: Overrides,
  /// Whether to include the `Ov:` element in THIS report (the change/periodic-refresh cadence — see
  /// [`RefreshReporter`]). grbl emits `Ov:` on change and at least every ~10-30 reports, not every report.
  pub include_ov: bool,
}

impl MachineSnapshot {
  /// A power-on default: idle at the origin with empty buffers fully free, machine-position reporting, and the
  /// WCO included (grbl emits `WCO:` in the first report after reset). Used as the initial shared state before
  /// the motion executor publishes a live position.
  pub const fn idle() -> Self {
    Self {
      state: MachineState::Idle,
      mpos_mm: [0.0; AXIS_COUNT],
      wco_mm: [0.0; AXIS_COUNT],
      position_report: PositionReport::Machine,
      include_wco: true,
      feed_mm_min: 0.0,
      spindle_rpm: 0,
      planner_blocks_free: BLOCK_BUFFER_SIZE as u8,
      rx_bytes_free: RX_BUFFER_SIZE as u16,
      pins: PinReport::new_idle(),
      overrides: Overrides::new(),
      // The first report after construction/reset includes `Ov:` (grbl's "first report" rule); the bin's
      // `Ov:` `RefreshReporter` re-affirms this, but seeding `true` keeps a hand-built idle snapshot consistent.
      include_ov: true,
    }
  }
}

/// A generic grbl change-only-element refresh cadence (DOC-08 §4): grbl reports a change-only element (`WCO:`,
/// `Ov:`) "in every 10 or 30 status reports (configurable), immediately in the next report after the value
/// changes, and in the first report after a reset". This small state machine decides, per report, whether the
/// tracked element should be included, keeping the change-detection + periodic-refresh rule pure and host-tested
/// rather than scattered through the firmware bin's status task. One generic type serves every such element: `T`
/// is the tracked value (`[f32; AXIS_COUNT]` for `WCO:`, [`Overrides`] for `Ov:`), and change detection is the
/// type's own `PartialEq` — element-wise for the `[f32; AXIS_COUNT]` array, so a non-finite component still reads
/// as "changed" against the baseline. It is `Copy` (when `T: Copy`) so the bin can hold it in a `Cell` beside the
/// other shared state.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RefreshReporter<T: Copy + PartialEq> {
  /// The value reported in the last report that included the element (so a change is detected against it).
  last_reported: T,
  /// Reports emitted since the element was last included; forces a periodic refresh at [`REFRESH_PERIOD`].
  since_refresh: u16,
  /// Set until the first report is emitted, so the FIRST report after construction/reset always includes it.
  force_first: bool,
}

impl<T: Copy + PartialEq> RefreshReporter<T> {
  /// A fresh reporter: the first report always includes the element (grbl's "first report after reset" rule),
  /// with `baseline` as the recorded value so any initial value DIFFERENT from it also counts as a change. The
  /// firmware seeds the baseline with the element's identity (origin WCO / default overrides). `const` so it can
  /// seed a `static` cell without a lazy `Option`.
  pub const fn new(baseline: T) -> Self {
    Self { last_reported: baseline, since_refresh: 0, force_first: true }
  }

  /// Reset the cadence so the next report includes the element (used on a soft reset, matching grbl's "first
  /// report after a reset includes the change-only elements"). Re-seeds the baseline with `baseline`.
  pub fn reset(&mut self, baseline: T) {
    *self = Self::new(baseline);
  }

  /// Decide whether the report being formatted NOW should include the tracked element, given its `current`
  /// value. Returns `true` when the value changed since it was last reported, when the periodic-refresh period
  /// has elapsed, or on the first report after construction/reset. Call exactly once per emitted report: it
  /// advances the internal counters and records the value when it answers `true`, so the change baseline and the
  /// period stay correct.
  pub fn should_include(&mut self, current: T) -> bool {
    let changed = current != self.last_reported;
    // Include on the Nth report of each window of `REFRESH_PERIOD`: with `since_refresh` counting the suppressed
    // reports since the last include, the (period − 1)th suppression makes the next report the periodic one, so
    // exactly one report in every `REFRESH_PERIOD` carries the element.
    let periodic = self.since_refresh >= REFRESH_PERIOD - 1;
    if self.force_first || changed || periodic {
      self.last_reported = current;
      self.since_refresh = 0;
      self.force_first = false;
      true
    } else {
      self.since_refresh = self.since_refresh.saturating_add(1);
      false
    }
  }
}

/// A `Copy` snapshot of the coordinate model the `$#` NGC-parameters report renders: the six G54-G59 work
/// offsets, the G28/G30 predefined positions, the G92 offset, the TLO scalar, and the last-probe result. The
/// firmware bin fills this from the shared [`crate::coords::CoordinateSystems`] (plus the last probe, Phase C)
/// and hands it to [`ResponseWriter::ngc_parameter_line`]; keeping the formatter pure over a snapshot makes the
/// `$#` wire output host-testable byte-for-byte.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoordinateReport {
  /// G54-G59 work-coordinate-system offsets in machine mm (index 0 = G54 … 5 = G59).
  pub wcs: [[f32; AXIS_COUNT]; 6],
  /// G28/G30 predefined positions in machine mm (index 0 = G28, 1 = G30).
  pub predefined: [[f32; AXIS_COUNT]; 2],
  /// The G92 offset in mm.
  pub g92: [f32; AXIS_COUNT],
  /// The dynamic tool-length offset scalar in mm (the Z axis), reported as `[TLO:z]`.
  pub tlo: f32,
  /// The last probe result in machine mm (`[PRB:x,y,z,a:flag]`). Filled by Phase C probing; zeros until then.
  pub probe: [f32; AXIS_COUNT],
  /// The last probe success flag (`1` = contact made, `0` = no contact / never probed).
  pub probe_success: bool,
}

impl Default for CoordinateReport {
  /// All offsets/positions zero, no probe — the power-on / first-boot `$#` view.
  fn default() -> Self {
    Self {
      wcs: [[0.0; AXIS_COUNT]; 6],
      predefined: [[0.0; AXIS_COUNT]; 2],
      g92: [0.0; AXIS_COUNT],
      tlo: 0.0,
      probe: [0.0; AXIS_COUNT],
      probe_success: false,
    }
  }
}

/// The last `G38.x` probe result (DOC-09, Phase C): the MACHINE position at the trigger instant in mm and the
/// contact flag. `Copy` so the firmware bin holds it behind a synchronous `Cell` (exactly the [`ControlState`] /
/// coordinate-model pattern) — the probe-cycle executor publishes the stop point and the `$#` / `[PRB:]` paths
/// read it. The power-on value is all-zero with `success = false`, matching grbl's "never probed" `[PRB:..:0]`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LastProbe {
  /// The MACHINE position at the probe trigger instant, in mm, `[X, Y, Z]`.
  pub position_mm: [f32; AXIS_COUNT],
  /// The contact flag: `true` if the expected probe edge was seen within travel, `false` otherwise (or never
  /// probed). Rendered as the trailing `:1`/`:0` of the `[PRB:]` line.
  pub success: bool,
}

impl LastProbe {
  /// The power-on / never-probed value: the origin with `success = false`, so `$#` shows `[PRB:0,0,0:0]` until a
  /// probe runs. `const` so it can seed a `static` cell without a lazy `Option`.
  pub const fn none() -> Self {
    LastProbe { position_mm: [0.0; AXIS_COUNT], success: false }
  }
}

impl Default for LastProbe {
  fn default() -> Self {
    Self::none()
  }
}

/// The per-line response a completed `G38.x` probe should produce, decided from the probe outcome and the mode's
/// alarm-on-fail flag (DOC-09, Phase C). Keeping this a pure host-tested decision means the firmware bin's probe
/// handler is a thin dispatch over a unit-tested truth table rather than ad-hoc branching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProbeResponse {
  /// The probe succeeded (saw its expected edge): emit a single `ok`, staying Idle. The `[PRB:..:1]` push already
  /// preceded it.
  Ok,
  /// The probe failed in an ALARMING mode (G38.2/.4): enter this alarm and emit `ALARM:N` (no `ok`). `ProbeFail-
  /// Initial` (4) when the probe was already at its expected edge before motion, else `ProbeFailContact` (5).
  Alarm(AlarmCode),
}

/// Decide a probe's per-line response (DOC-09): `triggered` is whether the expected edge was seen within travel;
/// `already_at_edge` is whether the probe was already at its expected stop edge before any motion (grbl's wrong
/// initial state); `alarm_on_fail` distinguishes the alarming modes (G38.2/.4) from the silent ones (G38.3/.5).
///
/// - A triggered probe → [`ProbeResponse::Ok`] (success, one `ok`).
/// - A non-triggered probe in a SILENT mode (G38.3/.5) → [`ProbeResponse::Ok`] (the sender checks `[PRB:..:0]`).
/// - A non-triggered probe in an ALARMING mode (G38.2/.4) → [`ProbeResponse::Alarm`]: `ALARM:4`
///   ([`AlarmCode::ProbeFailInitial`]) when `already_at_edge`, else `ALARM:5` ([`AlarmCode::ProbeFailContact`]).
pub fn probe_response(triggered: bool, already_at_edge: bool, alarm_on_fail: bool) -> ProbeResponse {
  if triggered || !alarm_on_fail {
    ProbeResponse::Ok
  } else if already_at_edge {
    ProbeResponse::Alarm(AlarmCode::ProbeFailInitial)
  } else {
    ProbeResponse::Alarm(AlarmCode::ProbeFailContact)
  }
}
