//! TMC2209 driver manager (DOC-03): the register-level orchestration above the datagram codec.
//!
//! The [`TmcManager`] drives the three stepper drivers on the shared single-wire bus through the
//! [`TmcBus`](crate::hal_traits::TmcBus) transport. It owns no I/O of its own — every byte goes through
//! the trait — so the whole init sequence, write verification, current scaling, and status polling are
//! host-tested against a mock bus. Only the firmware binary supplies the real UART1 transport.
//!
//! Responsibilities, mirroring DOC-03:
//! - **Presence check.** Read `IOIN` and confirm the `VERSION` field is [`EXPECTED_VERSION`]. A node that
//!   never answers (standalone VREF mode / unwired / broken bus) is flagged absent and skipped — motion can
//!   still proceed on a VREF-configured driver — rather than failing the whole bring-up.
//! - **Register init.** For each present node, program `GSTAT`/`SLAVECONF`/`GCONF`/`CHOPCONF`/`IHOLD_IRUN`/
//!   `TPOWERDOWN`/`TPWMTHRS`/`PWMCONF` (DOC-03 init steps 2–6 plus the send-delay and a status clear).
//! - **Write verification.** Read `IFCNT` before and after the writes; the counter must advance by exactly
//!   the number of writes issued, proving every datagram was accepted (DOC-03 init step 7).
//! - **Runtime current scaling.** Recompute `IRUN`/`IHOLD` (and the shared `CHOPCONF.vsense`) on demand.
//! - **Status polling.** Read and decode `DRV_STATUS` for over-temperature / short / open-load reporting.

use crate::drivers::tmc2209::registers::{
  chopconf, cs_for_current, gconf_uart_control, ifcnt, ihold_irun, ioin_version, pwmconf_stealthchop,
  rms_current_to_cs, slaveconf, CurrentScaling, DrvStatus, CHOPCONF, DRV_STATUS, EXPECTED_VERSION, GCONF,
  GSTAT, GSTAT_CLEAR_ALL, IFCNT, IHOLD_IRUN, IOIN, PWMCONF, R_SENSE_ADAFRUIT_6121_OHMS,
  R_SENSE_BTT_TMC2209_OHMS, SLAVECONF,
  TPOWERDOWN, TPWMTHRS,
};
use crate::drivers::tmc2209::TmcError;
use crate::hal_traits::TmcBus;

/// Number of stepper drivers the manager configures (X, Y, Z, A), aliasing `planner::AXES`. Node 3 is the
/// rotary A driver (DOC-10); its MS1/MS2 address strap and bus wiring are firmware-side and bench-deferred.
pub const AXIS_COUNT: usize = crate::planner::AXES;

/// Per-axis driver configuration: the bus node address and the motor current / microstepping to program.
/// These derive from the grblHAL `$`-settings the firmware loads (DOC-00); the manager holds a plain copy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisConfig {
  /// The 0..=3 UART node address strapped on this driver's MS1/MS2 pins (X=0, Y=1, Z=2 per DOC-03).
  pub node: u8,
  /// Target RMS run current in milliamps (used while the motor is moving).
  pub run_current_ma: u16,
  /// Target RMS hold current in milliamps (used at standstill); typically a fraction of the run current.
  pub hold_current_ma: u16,
  /// Microstep resolution in microsteps per full step (a power of two, 1..=256). Must match `$100–$102`
  /// steps/mm, which already fold in this multiplier.
  pub microsteps: u16,
}

/// Whole-bus TMC2209 configuration: the per-axis settings plus the parameters shared across all nodes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TmcConfig {
  /// Per-axis configuration, indexed `[X, Y, Z]`.
  pub axes: [AxisConfig; AXIS_COUNT],
  /// Sense-resistor value in ohms used by the current-scale math (Adafruit 6121 ⇒ 0.05 Ω — DOC-03).
  pub r_sense_ohms: f32,
  /// `IHOLDDELAY` programmed into `IHOLD_IRUN`: how gradually the run current ramps down to the hold current.
  pub ihold_delay: u8,
  /// `TPOWERDOWN`: standstill delay before the hold current is applied (≈ 20 ⇒ 0.3 s — DOC-03 init step 5).
  pub tpowerdown: u8,
  /// `TPWMTHRS`: StealthChop→SpreadCycle crossover velocity threshold (0 keeps StealthChop at all speeds).
  pub tpwmthrs: u32,
  /// `SLAVECONF.SENDDELAY` (units of 8 bit-times); ≥ 2 on this multi-node bus for clean turn-around (DOC-03).
  pub send_delay: u8,
}

impl Default for TmcConfig {
  /// Sensible bring-up defaults for the four NEMA-17 axes: nodes 0/1/2/3, 800 mA run / 400 mA hold, 1/16
  /// microstepping, `IHOLDDELAY`=7, `TPOWERDOWN`=20, StealthChop everywhere (`TPWMTHRS`=0), `SENDDELAY`=2.
  /// These stand in until esp-storage `$`-settings load. The sense-resistor value is selected by the
  /// `BREADBOARD_STEPSTICKS` toggle below — see `docs/breadboard-bringup.md`.
  fn default() -> Self {
    let axis = |node| AxisConfig { node, run_current_ma: 800, hold_current_ma: 400, microsteps: 16 };
    // Sense resistor — pick the constant matching your drivers (this is the only line to flip when moving
    // between the breadboard bring-up and the milled PCB; both values are kept present so the swap is trivial):
    //   Adafruit 6121 breakout (production / milled PCB) ......... R_SENSE_ADAFRUIT_6121_OHMS (0.05 Ω)
    //   BTT / Watterott / FYSETC stepstick (breadboard build) ... R_SENSE_BTT_TMC2209_OHMS  (0.11 Ω)
    // A wrong value mis-scales IRUN/IHOLD by ≈ 2.2× (the ratio of the two senses) and can overheat the motor.
    const BREADBOARD_STEPSTICKS: bool = false;
    let r_sense_ohms = if BREADBOARD_STEPSTICKS { R_SENSE_BTT_TMC2209_OHMS } else { R_SENSE_ADAFRUIT_6121_OHMS };
    TmcConfig {
      axes: [axis(0), axis(1), axis(2), axis(3)],
      r_sense_ohms,
      ihold_delay: 7,
      tpowerdown: 20,
      tpwmthrs: 0,
      send_delay: 2,
    }
  }
}

/// Errors the manager can surface above the raw transport. Bus/decode failures are wrapped via [`TmcError`];
/// the other two are configuration or verification failures the transport itself cannot express.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TmcManagerError {
  /// A transport or datagram-decode failure on the bus.
  Bus(TmcError),
  /// `IFCNT` did not advance by the number of writes issued, so at least one configuration datagram was not
  /// accepted by the driver (DOC-03 init step 7). Carries the node and the expected vs observed deltas.
  WriteVerification {
    /// The node whose write count did not match.
    node: u8,
    /// The number of writes the manager issued (the expected `IFCNT` delta).
    expected: u8,
    /// The actual `IFCNT` delta observed across the write sequence.
    actual: u8,
  },
  /// The configured microstep resolution is not a valid TMC2209 power-of-two value, so `CHOPCONF` could not
  /// be built. A configuration bug, surfaced loudly rather than silently mis-stepping.
  InvalidMicrosteps {
    /// The node whose microstep setting was invalid.
    node: u8,
    /// The offending microstep value.
    microsteps: u16,
  },
}

impl From<TmcError> for TmcManagerError {
  fn from(error: TmcError) -> Self {
    TmcManagerError::Bus(error)
  }
}

/// The outcome of initializing one axis: whether the driver answered and, if so, the current scale programmed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxisReport {
  /// The node address this report is for.
  pub node: u8,
  /// `true` if the driver answered `IOIN` with the expected `VERSION` and was therefore configured over UART;
  /// `false` if it was flagged absent (no reply or wrong version) and left in whatever standalone state it is in.
  pub present: bool,
  /// The `VERSION` byte read from `IOIN` (0 if the read timed out). [`EXPECTED_VERSION`] when present.
  pub version: u8,
  /// The run current scale computed for this axis. Programmed into the driver only when `present`; reported
  /// regardless so a caller can log the intended current even for an absent (VREF-mode) driver.
  pub current: CurrentScaling,
}

/// The TMC2209 manager: holds the bus configuration and drives all register access through a [`TmcBus`].
pub struct TmcManager {
  config: TmcConfig,
}

impl TmcManager {
  /// Build a manager for the given bus configuration.
  pub fn new(config: TmcConfig) -> Self {
    TmcManager { config }
  }

  /// The configuration this manager was built with.
  pub fn config(&self) -> &TmcConfig {
    &self.config
  }

  /// Initialize every axis in order, returning a per-axis result. A driver that is absent yields
  /// `Ok(AxisReport { present: false, .. })` (flag-and-skip, not an error), so the caller can configure the
  /// present drivers, log the absent ones, and decide whether to alarm — without one missing driver aborting
  /// the others. `async` because each [`init_axis`](Self::init_axis) awaits the bus.
  pub async fn init_all<B: TmcBus>(&self, bus: &mut B) -> [Result<AxisReport, TmcManagerError>; AXIS_COUNT] {
    // Async closures in `core::array::from_fn` are not available, so seed the array with placeholders and then
    // await each `init_axis` in order, overwriting each slot. The single bus is shared sequentially across the
    // three inits; nothing observes the placeholders since every slot is overwritten before the array returns.
    let mut reports: [Result<AxisReport, TmcManagerError>; AXIS_COUNT] =
      core::array::from_fn(|_| Err(TmcManagerError::Bus(TmcError::Timeout)));
    for (axis, slot) in reports.iter_mut().enumerate() {
      *slot = self.init_axis(bus, axis).await;
    }
    reports
  }

  /// Initialize one axis (DOC-03 init sequence). Probes presence via `IOIN.VERSION`; if present, runs the
  /// register writes and verifies them via `IFCNT`. Returns the per-axis report, or an error only on a real
  /// bus/verification/config failure (an absent driver is a successful flag-and-skip, not an error). `async`
  /// so the caller awaits the half-duplex bus exchanges instead of busy-spinning the executor.
  pub async fn init_axis<B: TmcBus>(&self, bus: &mut B, axis: usize) -> Result<AxisReport, TmcManagerError> {
    let config = &self.config.axes[axis];
    let node = config.node;
    let current = rms_current_to_cs(config.run_current_ma, self.config.r_sense_ohms);

    // Step 1: confirm a driver is actually on the bus. A `Timeout` means no reply — the documented
    // "standalone VREF mode / not present" case — so flag it absent and skip configuration. Any other bus
    // error is a genuine comms fault on a node that did respond, so it propagates.
    let version = match bus.read_reg(node, IOIN).await {
      Ok(raw) => ioin_version(raw),
      Err(TmcError::Timeout) => return Ok(AxisReport { node, present: false, version: 0, current }),
      Err(error) => return Err(error.into()),
    };
    if version != EXPECTED_VERSION {
      return Ok(AxisReport { node, present: false, version, current });
    }

    // Steps 2–7: program the registers and verify acceptance. `IFCNT` increments once per accepted write, so
    // the post/pre delta must equal the number of writes issued (the counter is 8-bit, so compare wrapping).
    let before = ifcnt(bus.read_reg(node, IFCNT).await?);
    let writes = self.write_config(bus, axis, &current).await?;
    let after = ifcnt(bus.read_reg(node, IFCNT).await?);
    let delta = after.wrapping_sub(before);
    if delta != writes {
      return Err(TmcManagerError::WriteVerification { node, expected: writes, actual: delta });
    }
    Ok(AxisReport { node, present: true, version, current })
  }

  /// Write the full register configuration for one axis and return the number of write datagrams issued (the
  /// expected `IFCNT` delta). The write set is held as an explicit table so the returned count is derived from
  /// it and can never drift out of step with the writes actually performed.
  async fn write_config<B: TmcBus>(&self, bus: &mut B, axis: usize, current: &CurrentScaling) -> Result<u8, TmcManagerError> {
    let config = &self.config.axes[axis];
    let node = config.node;
    // `CHOPCONF` carries both the microstep resolution and the `vsense` bit that the current scale assumes;
    // build it from the resolved `vsense` so `IRUN`/`IHOLD` and `CHOPCONF` agree. An invalid resolution is a
    // config bug surfaced loudly rather than silently mis-encoded.
    let chop =
      chopconf(config.microsteps, current.vsense).ok_or(TmcManagerError::InvalidMicrosteps { node, microsteps: config.microsteps })?;
    // The hold current shares the run current's `vsense` (one `CHOPCONF` per driver), so its CS is computed at
    // that same sensitivity rather than re-selecting a range.
    let hold_cs = cs_for_current(config.hold_current_ma, self.config.r_sense_ohms, current.vsense);

    // The ordered write set (DOC-03 init steps 2–6, plus the send-delay and a `GSTAT` clear). `GSTAT` is
    // cleared first so latched power-on flags do not mask a later fault; `SENDDELAY` is set before the bulk of
    // the writes so multi-node turn-around is clean. Every entry increments `IFCNT`.
    let writes: [(u8, u32); 8] = [
      (GSTAT, GSTAT_CLEAR_ALL),
      (SLAVECONF, slaveconf(self.config.send_delay)),
      (GCONF, gconf_uart_control()),
      (CHOPCONF, chop),
      (IHOLD_IRUN, ihold_irun(current.cs, hold_cs, self.config.ihold_delay)),
      (TPOWERDOWN, u32::from(self.config.tpowerdown)),
      (TPWMTHRS, self.config.tpwmthrs),
      (PWMCONF, pwmconf_stealthchop()),
    ];
    for &(reg, val) in &writes {
      bus.write_reg(node, reg, val).await?;
    }
    Ok(writes.len() as u8)
  }

  /// Recompute and program the run/hold current for one axis at runtime (DOC-03 current API). Re-selects the
  /// `vsense` range for the new run current and rewrites `CHOPCONF` (whose `vsense` may have flipped) before
  /// `IHOLD_IRUN`, so the two stay consistent. Returns the resolved run current scale.
  pub async fn set_current<B: TmcBus>(&self, bus: &mut B, axis: usize, run_ma: u16, hold_ma: u16) -> Result<CurrentScaling, TmcManagerError> {
    let config = &self.config.axes[axis];
    let node = config.node;
    let run = rms_current_to_cs(run_ma, self.config.r_sense_ohms);
    let hold_cs = cs_for_current(hold_ma, self.config.r_sense_ohms, run.vsense);
    let chop =
      chopconf(config.microsteps, run.vsense).ok_or(TmcManagerError::InvalidMicrosteps { node, microsteps: config.microsteps })?;
    // Rewrite `CHOPCONF` first: if `vsense` changed, the new `IRUN`/`IHOLD` CS values are only correct once the
    // matching sense range is in effect.
    bus.write_reg(node, CHOPCONF, chop).await?;
    bus.write_reg(node, IHOLD_IRUN, ihold_irun(run.cs, hold_cs, self.config.ihold_delay)).await?;
    Ok(run)
  }

  /// Read and decode the `DRV_STATUS` diagnostics for one axis (DOC-03 status polling). Returns the raw bus
  /// error directly — status polling has no verification step, so there is no manager-level error to add.
  pub async fn read_status<B: TmcBus>(&self, bus: &mut B, axis: usize) -> Result<DrvStatus, TmcError> {
    let raw = bus.read_reg(self.config.axes[axis].node, DRV_STATUS).await?;
    Ok(DrvStatus::from_raw(raw))
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::drivers::tmc2209::registers::CHOPCONF_VSENSE;

  /// A byte-buffer [`TmcBus`] mock: records every write, increments `IFCNT` like a real driver, and serves
  /// canned `IOIN`/`DRV_STATUS` replies. `ioin_responds = false` models an absent node (the read times out);
  /// `count_writes = false` models a driver that silently drops writes so `IFCNT` never advances.
  struct MockBus {
    ifcnt: u8,
    ioin: u32,
    drv_status: u32,
    ioin_responds: bool,
    count_writes: bool,
    writes: heapless::Vec<(u8, u8, u32), 64>,
  }

  impl MockBus {
    /// A present driver reporting the given `VERSION` byte in `IOIN`.
    fn present(version: u8) -> Self {
      MockBus {
        ifcnt: 0,
        ioin: u32::from(version) << 24,
        drv_status: 0,
        ioin_responds: true,
        count_writes: true,
        writes: heapless::Vec::new(),
      }
    }

    /// Find the value written to `reg` on `node`, if any.
    fn written(&self, node: u8, reg: u8) -> Option<u32> {
      self.writes.iter().find(|&&(n, r, _)| n == node && r == reg).map(|&(_, _, v)| v)
    }
  }

  impl TmcBus for MockBus {
    async fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError> {
      let _ = self.writes.push((node, reg, val));
      if self.count_writes {
        self.ifcnt = self.ifcnt.wrapping_add(1);
      }
      Ok(())
    }

    async fn read_reg(&mut self, _node: u8, reg: u8) -> Result<u32, TmcError> {
      match reg {
        IOIN => {
          if self.ioin_responds {
            Ok(self.ioin)
          } else {
            Err(TmcError::Timeout)
          }
        }
        IFCNT => Ok(u32::from(self.ifcnt)),
        DRV_STATUS => Ok(self.drv_status),
        _ => Ok(0),
      }
    }
  }

  /// Drive an immediately-ready future to completion with the no-op waker. The `MockBus` futures never pend
  /// (no real I/O), so a single poll resolves them — this keeps the manager tests off any async runtime /
  /// dev-dep, mirroring the `block_on` helper in the settings tests, and `Waker::noop` avoids hand-rolling a
  /// `RawWaker` (which `#![deny(unsafe_code)]` forbids).
  fn block_on<F: core::future::Future>(future: F) -> F::Output {
    use core::task::{Context, Poll, Waker};
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    match future.as_mut().poll(&mut context) {
      Poll::Ready(output) => output,
      Poll::Pending => panic!("bus future pended; the host MockBus must resolve in one poll"),
    }
  }

  fn manager() -> TmcManager {
    TmcManager::new(TmcConfig::default())
  }

  #[test]
  fn init_axis_present_configures_and_verifies() {
    let mut bus = MockBus::present(EXPECTED_VERSION);
    let report = block_on(manager().init_axis(&mut bus, 0)).expect("present driver configures");
    assert!(report.present);
    assert_eq!(report.version, EXPECTED_VERSION);
    assert_eq!(report.node, 0);
    // The canonical GCONF and a CHOPCONF / IHOLD_IRUN were written to node 0.
    assert_eq!(bus.written(0, GCONF), Some(gconf_uart_control()));
    assert!(bus.written(0, CHOPCONF).is_some());
    assert!(bus.written(0, IHOLD_IRUN).is_some());
    // Eight configuration writes were issued (GSTAT, SLAVECONF, GCONF, CHOPCONF, IHOLD_IRUN, TPOWERDOWN,
    // TPWMTHRS, PWMCONF), so IFCNT advanced by eight.
    assert_eq!(bus.writes.len(), 8);
    assert_eq!(bus.ifcnt, 8);
  }

  #[test]
  fn chopconf_vsense_matches_run_current_scale() {
    // The default 800 mA run current on 0.05 Ω selects the high-sensitivity range, so CHOPCONF.vsense is set
    // and the IHOLD_IRUN CS values are computed for that same range — the two must agree.
    let mut bus = MockBus::present(EXPECTED_VERSION);
    let report = block_on(manager().init_axis(&mut bus, 0)).expect("configures");
    let chop = bus.written(0, CHOPCONF).expect("CHOPCONF written");
    assert_eq!(report.current.vsense, chop & CHOPCONF_VSENSE != 0);
  }

  #[test]
  fn init_axis_absent_when_no_reply() {
    let mut bus = MockBus::present(EXPECTED_VERSION);
    bus.ioin_responds = false;
    let report = block_on(manager().init_axis(&mut bus, 1)).expect("absent driver is not an error");
    assert!(!report.present);
    assert_eq!(report.version, 0);
    // No configuration writes are issued to a driver that did not answer.
    assert!(bus.writes.is_empty());
  }

  #[test]
  fn init_axis_absent_on_version_mismatch() {
    let mut bus = MockBus::present(0x10);
    let report = block_on(manager().init_axis(&mut bus, 2)).expect("wrong version is flag-and-skip");
    assert!(!report.present);
    assert_eq!(report.version, 0x10);
    assert!(bus.writes.is_empty());
  }

  #[test]
  fn init_axis_detects_dropped_writes() {
    // A driver that ACKs nothing (IFCNT never advances) must fail verification, not silently report success.
    let mut bus = MockBus::present(EXPECTED_VERSION);
    bus.count_writes = false;
    match block_on(manager().init_axis(&mut bus, 0)) {
      Err(TmcManagerError::WriteVerification { node, expected, actual }) => {
        assert_eq!(node, 0);
        assert_eq!(expected, 8);
        assert_eq!(actual, 0);
      }
      other => panic!("expected WriteVerification, got {other:?}"),
    }
  }

  #[test]
  fn init_axis_rejects_invalid_microsteps() {
    let mut config = TmcConfig::default();
    config.axes[0].microsteps = 7;
    let mut bus = MockBus::present(EXPECTED_VERSION);
    match block_on(TmcManager::new(config).init_axis(&mut bus, 0)) {
      Err(TmcManagerError::InvalidMicrosteps { node, microsteps }) => {
        assert_eq!(node, 0);
        assert_eq!(microsteps, 7);
      }
      other => panic!("expected InvalidMicrosteps, got {other:?}"),
    }
  }

  #[test]
  fn init_all_reports_each_axis() {
    let mut bus = MockBus::present(EXPECTED_VERSION);
    let reports = block_on(manager().init_all(&mut bus));
    assert_eq!(reports.len(), AXIS_COUNT);
    for (axis, report) in reports.iter().enumerate() {
      let report = report.as_ref().expect("each present axis configures");
      assert!(report.present);
      assert_eq!(report.node, axis as u8);
    }
  }

  #[test]
  fn set_current_rewrites_chopconf_then_ihold_irun() {
    let mut bus = MockBus::present(EXPECTED_VERSION);
    let scale = block_on(manager().set_current(&mut bus, 0, 1500, 700)).expect("set current");
    let chop = bus.written(0, CHOPCONF).expect("CHOPCONF rewritten");
    assert_eq!(scale.vsense, chop & CHOPCONF_VSENSE != 0);
    // The IHOLD_IRUN write carries the resolved run CS in the IRUN field (bits 12:8).
    let ihold_irun_val = bus.written(0, IHOLD_IRUN).expect("IHOLD_IRUN written");
    assert_eq!(((ihold_irun_val >> 8) & 0x1F) as u8, scale.cs);
  }

  #[test]
  fn read_status_decodes_drv_status() {
    let mut bus = MockBus::present(EXPECTED_VERSION);
    bus.drv_status = (1 << 1) | (1 << 31); // over-temperature shutdown + standstill.
    let status = block_on(manager().read_status(&mut bus, 0)).expect("status read");
    assert!(status.overtemp_shutdown());
    assert!(status.standstill());
    assert!(status.has_fault());
  }
}
