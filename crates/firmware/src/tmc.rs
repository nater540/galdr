//! TMC2209 single-wire UART transport + manager task (DOC-03): the esp-hal wiring that carries the
//! host-tested [`firmware_core::drivers::tmc2209`] datagrams over UART1.
//!
//! This is the thin, non-host-testable adapter for the TMC2209 subsystem, mirroring how [`crate::motion`]
//! adapts the step generator and [`crate::comms`] adapts the streaming engine: ALL register logic, current
//! scaling, the init sequence, write verification, and status decoding live in firmware-core's
//! [`TmcManager`](firmware_core::drivers::tmc2209::manager::TmcManager) and are exercised by host tests; here
//! we only move datagram bytes between the UART1 peripheral and that logic through the
//! [`TmcBus`](firmware_core::hal_traits::TmcBus) trait, and run the bring-up / polling task.
//!
//! ## Single-wire half-duplex on GPIO9 (DOC-03 bus topology)
//! All three drivers share one PDN_UART node driven by UART1 on GPIO9. There is exactly one physical wire,
//! so the same GPIO carries both TX and RX: the pin is degraded to an [`AnyPin`](esp_hal::gpio::AnyPin) and a
//! second handle is cloned for the RX direction (an esp-hal boundary use of `unsafe`, permitted by CLAUDE.md
//! — only one pin is ever physically present, and the two handles only route the one line to the UART TX and
//! RX signals through the GPIO matrix). The MCU TX is push-pull; the required 1 kΩ series resistor on the
//! line (a hardware element, DOC-03) limits contention current when a driver also drives the bus. Because TX
//! and RX share the wire, every transmitted byte is echoed back and MUST be discarded before a reply is read
//! — [`Uart1TmcBus`] consumes exactly the echo length after each transmit.
//!
//! ## Async, non-stalling transport (the turn-around is awaited, not busy-spun)
//! The [`TmcBus`] trait is `async` (DOC-09), so [`Uart1TmcBus`] uses the non-blocking UART FIFO API and a
//! short poll loop that AWAITS an [`embassy_time::Timer`] between FIFO reads to bound how long it waits for a
//! reply. A driver that never answers (standalone VREF mode / unwired) yields [`TmcError::Timeout`] after the
//! 5 ms budget rather than hanging. Awaiting the inter-poll timer (instead of busy-spinning a `delay_micros`)
//! yields the single core-0 thread executor during the bus turn-around, so the comms / parser / planner tasks
//! keep running while a datagram round-trip is in flight — an absent driver no longer stalls the executor for
//! up to 5 ms. The exchanges are infrequent (a few round-trips at init and one `DRV_STATUS` read per axis per
//! poll interval), entirely off the real-time path; the core-1 step generation is wholly unaffected.
//!
//! ## Breadboard bring-up (BTT / stepstick drivers)
//! The default build targets the Adafruit 6121 breakout (0.05 Ω sense). When bringing the hardware up on a
//! breadboard with BTT / Watterott / FYSETC TMC2209 **stepstick** modules (0.11 Ω sense), flip the
//! `BREADBOARD_STEPSTICKS` toggle in [`TmcConfig::default`](firmware_core::drivers::tmc2209::manager::TmcConfig)
//! so the current-scale math uses the right sense resistor — a wrong value mis-scales `IRUN`/`IHOLD` by ≈ 2.2×
//! and can overheat the motor. The wiring differences (VIO → 3.3 V, MS1/MS2 address straps, the single 1 kΩ
//! UART series resistor, and keeping motor-coil current OFF the breadboard rails) are in
//! `docs/breadboard-bringup.md`. Note `tmc_r_sense_ohms` is a persisted setting: the compile-time default only
//! seeds a fresh/erased flash, so on a board with settings already stored, push the value over the `$PBX`
//! host-sync channel (or erase flash) rather than relying on the rebuild alone.

use embassy_time::{Duration, Timer};
use esp_hal::gpio::Pin;
use esp_hal::uart::{Config, Uart};
use esp_hal::Blocking;

use firmware_core::drivers::tmc2209::manager::{AxisReport, TmcConfig, TmcManager, AXIS_COUNT};
use firmware_core::drivers::tmc2209::{
  decode_read_reply, encode_read_request, encode_write, TmcError, READ_REPLY_LEN, READ_REQUEST_LEN,
  WRITE_DATAGRAM_LEN,
};
use firmware_core::hal_traits::TmcBus;
use firmware_core::protocol::DriverStatus;

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

/// Live per-axis TMC2209 bus health, published by [`tmc_manager`] after the init burst and REFRESHED every poll
/// round, so the comms layer can render the `$I+` `[DRIVER:]` report without re-touching the UART bus (a read
/// from the comms task would race the manager's polling and the half-duplex turn-around). Bit `i` set ⇒ axis
/// `i`'s driver is currently communicating AND not reporting a hard `DRV_STATUS` fault; the poll loop clears the
/// bit on an over-temp / short / open-load fault or a lost reply, and re-sets it on the next clean read, so the
/// report tracks live health rather than a frozen boot snapshot. A single `u8` covers all [`AXIS_COUNT`] axes
/// (≤ 4 today) — see the guard below. The mask is stored `Release` and loaded `Acquire`, paired with
/// [`TMC_INIT_DONE`] (stored last, loaded first) so a reader that sees init done also sees the populated bits.
static TMC_ONLINE_MASK: AtomicU8 = AtomicU8::new(0);

// The `u8` mask has one bit per axis, so it only covers AXIS_COUNT <= 8 axes. AXIS_LETTERS advertises axis
// changes as a "one-line change"; couple that promise to this second site with a compile-time guard so bumping
// AXIS_COUNT past 8 fails to build here (a loud error) instead of silently overflowing `1 << axis`.
const _: () = assert!(AXIS_COUNT <= 8, "TMC_ONLINE_MASK is a u8; widen it if AXIS_COUNT exceeds 8 axes");

/// `true` once [`tmc_manager`]'s init pass has populated [`TMC_ONLINE_MASK`]. Until then a `$I+` query reports
/// `init pending` instead of a misleading "all absent". Stored (`Release`) AFTER the mask so a reader that sees
/// `true` (via an `Acquire` load) is guaranteed to see the populated bits.
static TMC_INIT_DONE: AtomicBool = AtomicBool::new(false);

/// Snapshot the live per-axis TMC2209 bus health for the `$I+` `[DRIVER:]` report. Lock-free: reads the two
/// atomics the manager publishes, so the comms task never touches the UART bus directly. The `Acquire` load of
/// [`TMC_INIT_DONE`] pairs with the manager's `Release` store so seeing `initialized` implies seeing the mask.
pub fn driver_status() -> DriverStatus {
  let initialized = TMC_INIT_DONE.load(Ordering::Acquire);
  let mask = TMC_ONLINE_MASK.load(Ordering::Acquire);
  // Decode the per-axis bits with the same `from_fn` idiom as `comms::limit_levels`, so the bit layout
  // (`bit0 = X`, `bit1 = Y`, …) has one shared decoder shape across both reports.
  let online: [bool; AXIS_COUNT] = core::array::from_fn(|axis| mask & (1 << axis) != 0);
  DriverStatus { online, initialized }
}

/// Per-datagram timeout in microseconds: how long [`Uart1TmcBus`] busy-polls for an echo or reply before
/// declaring the node silent. A full 8-byte datagram at 115200 baud is ≈ 700 µs, and the driver inserts a
/// `SENDDELAY` turn-around on top, so 5 ms is comfortable headroom while still failing an absent node fast.
const BUS_TIMEOUT_US: u32 = 5_000;

/// Busy-poll granularity in microseconds between non-blocking FIFO reads while filling a datagram buffer.
/// Small enough that a reply is picked up promptly, large enough that the poll is not a tight spin.
const POLL_STEP_US: u32 = 50;

/// How often the manager task re-reads `DRV_STATUS` on every present driver to surface over-temperature /
/// short / open-load conditions. One second is responsive for thermal/fault reporting without loading the bus.
const STATUS_POLL_INTERVAL: Duration = Duration::from_secs(1);

/// The UART1-backed [`TmcBus`]: an async single-wire transport for the shared TMC2209 bus. It frames each
/// datagram through the host-tested codec, drives the bytes onto the line, discards the half-duplex echo, and
/// (for reads) collects and validates the reply, awaiting an [`embassy_time::Timer`] between FIFO polls so the
/// bus turn-around yields the executor rather than busy-spinning.
pub struct Uart1TmcBus {
  /// The blocking UART1 driver. Both its TX and RX signals route to the single GPIO9 line (see module docs).
  /// Its FIFO ops are non-blocking (`write`/`read_buffered` move whatever fits and return), so the reply wait
  /// is realized by awaiting a timer between polls — never by spinning a blocking delay.
  uart: Uart<'static, Blocking>,
}

impl Uart1TmcBus {
  /// Wrap a configured blocking UART1 instance as a TMC bus.
  pub fn new(uart: Uart<'static, Blocking>) -> Self {
    Uart1TmcBus { uart }
  }

  /// Discard any stale bytes left in the RX FIFO before starting a fresh exchange, so a leftover echo or a
  /// late reply from a previous datagram cannot be mistaken for this one's response. Best-effort and
  /// non-blocking: it reads whatever is buffered and stops as soon as the FIFO is empty (or errors).
  fn drain_rx(&mut self) {
    let mut scratch = [0u8; 16];
    while let Ok(read) = self.uart.read_buffered(&mut scratch) {
      if read == 0 {
        break;
      }
    }
  }

  /// Write every byte of `bytes`, looping because the FIFO may accept only part of the slice per call, then
  /// flush so the whole datagram is on the wire before the echo is read back.
  fn write_all(&mut self, mut bytes: &[u8]) -> Result<(), TmcError> {
    while !bytes.is_empty() {
      match self.uart.write(bytes) {
        // A non-empty slice should always make progress; a zero-write would otherwise spin forever, so treat
        // it as a transport fault rather than looping.
        Ok(0) => return Err(TmcError::Io),
        Ok(written) => bytes = &bytes[written..],
        Err(_) => return Err(TmcError::Io),
      }
    }
    self.uart.flush().map_err(|_| TmcError::Io)
  }

  /// Fill `buf` completely from the RX FIFO, polling with [`POLL_STEP_US`] granularity up to
  /// [`BUS_TIMEOUT_US`]. Uses the non-blocking `read_buffered` (not the blocking `read`) so an absent node
  /// times out cleanly instead of hanging the task forever. Between empty polls it AWAITS a [`Timer`] rather
  /// than busy-spinning a blocking delay, so the bus turn-around yields the core-0 executor to other tasks.
  async fn read_filling(&mut self, buf: &mut [u8]) -> Result<(), TmcError> {
    let mut filled = 0;
    let mut waited_us = 0;
    while filled < buf.len() {
      match self.uart.read_buffered(&mut buf[filled..]) {
        Ok(0) => {
          if waited_us >= BUS_TIMEOUT_US {
            return Err(TmcError::Timeout);
          }
          Timer::after(Duration::from_micros(POLL_STEP_US as u64)).await;
          waited_us += POLL_STEP_US;
        }
        Ok(read) => filled += read,
        Err(_) => return Err(TmcError::Io),
      }
    }
    Ok(())
  }

  /// Read and discard exactly `len` bytes of half-duplex echo (the bytes this MCU just transmitted, looped
  /// back on the shared wire). `len` is always ≤ [`WRITE_DATAGRAM_LEN`], the largest frame the bus sends.
  async fn consume_echo(&mut self, len: usize) -> Result<(), TmcError> {
    let mut scratch = [0u8; WRITE_DATAGRAM_LEN];
    self.read_filling(&mut scratch[..len]).await
  }
}

impl TmcBus for Uart1TmcBus {
  /// Frame and send a write-access datagram, then swallow its echo. The TMC2209 sends no reply to a write,
  /// so success here means "on the wire"; acceptance is confirmed separately by the manager via `IFCNT`.
  async fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError> {
    self.drain_rx();
    let frame = encode_write(node, reg, val);
    self.write_all(&frame)?;
    self.consume_echo(WRITE_DATAGRAM_LEN).await
  }

  /// Frame and send a read-request datagram, swallow its 4-byte echo, then collect and decode the 8-byte
  /// reply. A node that never answers surfaces as [`TmcError::Timeout`] from [`read_filling`]; a malformed
  /// reply surfaces as the matching decode error from [`decode_read_reply`].
  async fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError> {
    self.drain_rx();
    let request = encode_read_request(node, reg);
    self.write_all(&request)?;
    self.consume_echo(READ_REQUEST_LEN).await?;
    let mut reply = [0u8; READ_REPLY_LEN];
    self.read_filling(&mut reply).await?;
    decode_read_reply(&reply, reg)
  }
}

/// Configure UART1 as the single-wire TMC2209 bus on GPIO9 and wrap it as a [`Uart1TmcBus`].
///
/// The same GPIO9 line carries both TX and RX (single-wire half-duplex): the pin is degraded to an `AnyPin`
/// and a second handle is cloned for the RX signal. The clone is the documented esp-hal boundary use of
/// `unsafe` — there is exactly one physical pin, and both handles merely route that one line to UART1's TX
/// and RX through the GPIO matrix; nothing else touches GPIO9.
///
/// # Panics
/// `expect` is used here because this runs once in `main`'s init path, where a failure to bring up UART1 is
/// an unrecoverable wiring/config fault, not a runtime condition (CLAUDE.md permits `expect` in init).
pub fn init(uart1: esp_hal::peripherals::UART1<'static>, tmc_uart_pin: esp_hal::peripherals::GPIO9<'static>) -> Uart1TmcBus {
  // 115200 baud, 8N1 (the Config defaults), matching the TMC2209 UART (DOC-03).
  let config = Config::default().with_baudrate(115_200);
  // One physical line, two signal handles: degrade to AnyPin, clone for the RX direction (see fn docs).
  let tx_line = tmc_uart_pin.degrade();
  let rx_line = unsafe { tx_line.clone_unchecked() };
  let uart = Uart::new(uart1, config)
    .expect("UART1 TMC bus init")
    .with_tx(tx_line)
    .with_rx(rx_line);
  Uart1TmcBus::new(uart)
}

/// The `tmc_manager` task (DOC-03 / DOC-01): runs once on the core-0 thread executor to configure every
/// driver, then polls `DRV_STATUS` for faults on the present drivers forever.
///
/// On startup it runs the full per-axis init sequence (presence check via `IOIN.VERSION`, register
/// programming, `IFCNT` write verification). A driver that does not answer is flagged absent and skipped —
/// motion can still proceed on a VREF-configured driver (DOC-03) — so one missing driver never aborts the
/// others. After init it sleeps [`STATUS_POLL_INTERVAL`] between rounds and re-reads `DRV_STATUS` on each
/// present driver to detect over-temperature / short / open-load conditions.
///
/// ## Still stubbed (out of scope for this phase, deliberate TODOs)
/// - Diagnostics are surfaced via `defmt` only (no-op in the default build) so they never corrupt the grbl
///   USB stream; a present-but-misconfigured or faulting driver does not yet raise a machine ALARM. The
///   alarm-state machine (shared with the motion/limit fault path, DOC-06) is a later phase — when it lands,
///   an init/verify failure and a `DRV_STATUS` hard fault should force the spindle off and halt motion.
/// - The `TmcConfig` is now sourced from the persisted settings loaded at boot (DOC-04), but a runtime
///   `$x=val` / `$PBX` change to a current/microstep setting does not yet re-program the live drivers — it
///   takes effect on the next boot. Live re-application (a `Watch<TmcConfig>` this task selects on) is a
///   later refinement.
#[embassy_executor::task]
pub async fn tmc_manager(mut bus: Uart1TmcBus, config: TmcConfig) -> ! {
  let manager = TmcManager::new(config);

  // Configure every driver and record which ones actually answered, so the poll loop only queries present
  // drivers (querying an absent one would just time out every interval).
  let reports = manager.init_all(&mut bus).await;
  let mut present = [false; AXIS_COUNT];
  let mut online_mask: u8 = 0;
  for (axis, report) in reports.iter().enumerate() {
    present[axis] = report.as_ref().map(|report| report.present).unwrap_or(false);
    if present[axis] {
      online_mask |= 1 << axis;
    }
    log_init_result(axis, report);
  }
  // Publish the initial per-axis online state for the `$I+` `[DRIVER:]` report. `Release` the mask before
  // flagging init done (also `Release`) so a comms-task reader that observes `TMC_INIT_DONE == true` via an
  // `Acquire` load always sees the populated bits.
  TMC_ONLINE_MASK.store(online_mask, Ordering::Release);
  TMC_INIT_DONE.store(true, Ordering::Release);

  loop {
    // Sleep first so the bus is quiet immediately after the init burst, then poll. Awaiting here yields the
    // executor to the other core-0 tasks between rounds.
    Timer::after(STATUS_POLL_INTERVAL).await;
    for (axis, &is_present) in present.iter().enumerate() {
      if !is_present {
        continue;
      }
      let bit = 1u8 << axis;
      match manager.read_status(&mut bus, axis).await {
        // A hard `DRV_STATUS` fault (over-temp / short / open-load): the driver still answers but is unhealthy,
        // so drop it from the live health mask and log it. It re-joins the mask on the next clean read.
        Ok(status) if status.has_fault() => {
          online_mask &= !bit;
          log_fault(axis, status);
        }
        // A clean read: the driver is present and healthy, so (re-)mark it online.
        Ok(_) => online_mask |= bit,
        // No reply this round (the driver dropped off the half-duplex bus): drop it from the live mask so the
        // report stops claiming a silent driver is `ok`. A transient error self-heals on the next clean read.
        Err(_) => online_mask &= !bit,
      }
    }
    // Republish the refreshed health so `[DRIVER:]` reflects mid-job faults, not just the boot snapshot.
    TMC_ONLINE_MASK.store(online_mask, Ordering::Release);
  }
}

/// Surface one axis's init outcome over `defmt` (a no-op in the default, defmt-free build). Present drivers
/// log their version and resolved current scale; absent ones log that they were skipped; errors log loudly.
#[allow(unused_variables)]
fn log_init_result(axis: usize, report: &Result<AxisReport, firmware_core::drivers::tmc2209::manager::TmcManagerError>) {
  match report {
    #[cfg(feature = "defmt")]
    Ok(report) if report.present => defmt::info!(
      "TMC axis {} node {}: configured, version {=u8:#x}, CS {=u8} vsense {}",
      axis,
      report.node,
      report.version,
      report.current.cs,
      report.current.vsense,
    ),
    #[cfg(feature = "defmt")]
    Ok(report) => defmt::warn!("TMC axis {} node {}: absent (no reply / version {=u8:#x}), skipped", axis, report.node, report.version),
    #[cfg(feature = "defmt")]
    Err(error) => defmt::error!("TMC axis {} init failed: {:?}", axis, error),
    // Without defmt there is nowhere safe to log (the USB endpoint carries the grbl protocol), so the result
    // is computed for its configuration side effects and otherwise dropped until the alarm path exists.
    #[cfg(not(feature = "defmt"))]
    _ => {}
  }
}

/// Surface a `DRV_STATUS` hard fault over `defmt` (a no-op in the default build).
#[allow(unused_variables)]
fn log_fault(axis: usize, status: firmware_core::drivers::tmc2209::registers::DrvStatus) {
  #[cfg(feature = "defmt")]
  defmt::warn!(
    "TMC axis {} fault: overtemp {} short_gnd {} short_vs {} open_load {}",
    axis,
    status.overtemp_shutdown(),
    status.short_to_ground(),
    status.short_to_supply(),
    status.open_load(),
  );
}
