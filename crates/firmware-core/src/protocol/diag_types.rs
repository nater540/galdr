//! TMC2209 diagnostic wire DTOs (`[DRIVER:]`/`$I+`), co-located with their formatters.

use super::*;
use core::fmt::Write as _;
use heapless::String;

/// Live per-axis TMC2209 driver health for the `$I+` `[DRIVER:]` report. `online[axis]` is `true` when that
/// axis's driver answered on the shared UART bus at init (presence check + write verification) and is therefore
/// communicating; `false` means it was flagged absent (no reply / wrong version). Indexed in [`AXIS_LETTERS`]
/// order. `initialized` is `false` until the `firmware` bin's TMC init pass has populated `online`, so a `$I+`
/// query during the brief boot window reports `init pending` rather than a misleading "all absent". The
/// `firmware` bin owns the live values; this type keeps [`ResponseWriter::driver_info`] pure and host-testable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DriverStatus {
  /// Per-axis UART online flag, [`AXIS_LETTERS`] order; `true` = the driver is communicating.
  pub online: [bool; AXIS_COUNT],
  /// `false` until the TMC init pass has run and `online` reflects real bus results.
  pub initialized: bool,
}

/// The outcome of one node's TMC2209 presence read (`IOIN`) for the `$I+` diagnostic. The single-wire bus first
/// reads back the MCU's OWN transmitted bytes (the echo) before the driver's reply, and the reply is then
/// decoded and its `VERSION` byte checked — so a per-node outcome localizes a failure through every layer
/// without a scope: an echo timeout means the RX signal never saw the pin (firmware/pin-matrix fault); a reply
/// timeout with a good echo means routing is fine but the driver never answered (line/hardware fault); a decode
/// error means a reply arrived but failed sync/address/register/CRC (signal-integrity, echo-reply misalignment,
/// send-delay, or edge quality — inspect the raw bytes); and a version mismatch means the datagram decoded
/// cleanly but carried the wrong `VERSION` (wrong `EXPECTED_VERSION` const, a clone chip, or bit errors landing
/// in the version byte). `Responded` is a present, correctly-versioned driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TmcProbeStage {
  /// The presence read completed and the driver answered with the expected `VERSION`. The default, and the
  /// state for every present/healthy node.
  #[default]
  Responded,
  /// The half-duplex ECHO read timed out — the MCU's transmitted bytes never looped back on the shared line, so
  /// the RX signal is not seeing the pin. Points at the firmware/esp-hal pin-matrix, not the driver.
  EchoTimeout,
  /// The driver's REPLY read timed out. Carries a post-timeout RX-FIFO snapshot that distinguishes a genuinely
  /// silent driver from a reply that WAS on the wire but got gated away by the glitch detector: `fifo` is the
  /// number of bytes drained from the FIFO after the timeout (packed, saturates at 127), and `ready` is
  /// `read_ready()` sampled at the timeout — raw FIFO occupancy that, unlike a `read_buffered` drain, does not
  /// clear latched error flags. `fifo:0` + `!ready` ⇒ nothing arrived (driver silent / wiring / VIO); `fifo:N>0`
  /// with the drained bytes (dumped via [`IoinRawCapture`]) ⇒ the driver DID reply and the read strategy gated
  /// it; `fifo:0` + `ready` ⇒ occupancy reported but undrainable (a flag re-latched every poll).
  ReplyTimeout {
    /// Bytes drained from the RX FIFO after the timeout (saturating; the packed snapshot caps at 127).
    fifo: u8,
    /// `read_ready()` at the timeout — raw FIFO occupancy, independent of the latched glitch/framing flag.
    ready: bool,
  },
  /// A full reply arrived but failed to decode (bad sync / address / register / CRC). The physical layer works
  /// but the framing does not — signal integrity, echo-reply misalignment, send-delay, or edge quality. The
  /// raw reply bytes are surfaced alongside (see [`IoinRawCapture`]) so the framing can be eyeballed.
  DecodeError,
  /// The reply decoded cleanly but its `VERSION` byte did not match `EXPECTED_VERSION`. Carries the actual byte
  /// so a wrong version const, a clone chip, or a bit error in the version field is distinguishable at a glance.
  VersionMismatch(u8),
  /// The driver answered `IOIN` correctly but its config writes did not verify: `IFCNT` advanced by `actual`
  /// instead of the `expected` write count, so at least one register datagram was not accepted (reads work but
  /// writes are not landing — send-delay, write-frame corruption, or a write-protected driver). Both counts are
  /// nibble-packed in the snapshot, so each renders 0..=15 (the write set is 8; a larger value saturates at 15).
  WriteVerify {
    /// The number of config writes issued (the expected `IFCNT` delta).
    expected: u8,
    /// The observed `IFCNT` delta across the write sequence.
    actual: u8,
  },
  /// A non-timeout, non-decode init failure not otherwise categorized (e.g. an invalid microstep config, or a
  /// write-side bus error whose read stage is stale). Ensures an errored node is never silently `Responded`.
  InitError,
  /// The reply read tripped the RX glitch/framing detector but the datagram STILL decoded with a valid CRC — the
  /// driver is proven alive and we relied on RX tolerance. Carries the RX variant; renders `ok(<variant>)` so the
  /// reliance is visible, never silently swallowed. Still counts as a present/healthy driver in `[DRIVER:]`.
  RespondedDespiteGlitch(RxErrorKind),
  /// The reply read tripped the RX glitch/framing detector AND the datagram failed to decode — the glitch
  /// genuinely corrupted the bytes. Renders `crc(<variant>)`, distinguishing glitch-corrupted from a clean-line
  /// decode failure (bare `crc`) so a real signal-integrity problem is not mistaken for a framing coincidence.
  DecodeErrorGlitched(RxErrorKind),
}

impl TmcProbeStage {
  /// Pack this outcome into the 16-bit code the `firmware` bin stores per axis in its lock-free snapshot: the
  /// high byte is a discriminant tag and the low byte carries the payload — the version for
  /// [`VersionMismatch`](Self::VersionMismatch), or the two nibble-packed IFCNT counts for
  /// [`WriteVerify`](Self::WriteVerify) (each saturated at 15, ample for the 8-write set).
  pub fn to_bits(self) -> u16 {
    match self {
      TmcProbeStage::Responded => 0x0000,
      TmcProbeStage::EchoTimeout => 0x0100,
      TmcProbeStage::ReplyTimeout { fifo, ready } => {
        let payload = (fifo.min(0x7F)) | (u8::from(ready) << 7);
        0x0200 | u16::from(payload)
      }
      TmcProbeStage::DecodeError => 0x0300,
      TmcProbeStage::VersionMismatch(version) => 0x0400 | u16::from(version),
      TmcProbeStage::WriteVerify { expected, actual } => {
        let payload = (expected.min(0x0F) << 4) | actual.min(0x0F);
        0x0500 | u16::from(payload)
      }
      TmcProbeStage::InitError => 0x0600,
      TmcProbeStage::RespondedDespiteGlitch(variant) => 0x0700 | variant.to_code() as u16,
      TmcProbeStage::DecodeErrorGlitched(variant) => 0x0800 | variant.to_code() as u16,
    }
  }

  /// Recover an outcome from its 16-bit code. Any unknown tag decodes to `Responded` so a corrupt snapshot
  /// degrades to "healthy" rather than a false fault.
  pub fn from_bits(bits: u16) -> Self {
    let payload = (bits & 0xFF) as u8;
    match bits >> 8 {
      0x01 => TmcProbeStage::EchoTimeout,
      0x02 => TmcProbeStage::ReplyTimeout { fifo: payload & 0x7F, ready: payload & 0x80 != 0 },
      0x03 => TmcProbeStage::DecodeError,
      0x04 => TmcProbeStage::VersionMismatch(payload),
      0x05 => TmcProbeStage::WriteVerify { expected: payload >> 4, actual: payload & 0x0F },
      0x06 => TmcProbeStage::InitError,
      0x07 => TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::from_code(u32::from(payload))),
      0x08 => TmcProbeStage::DecodeErrorGlitched(RxErrorKind::from_code(u32::from(payload))),
      _ => TmcProbeStage::Responded,
    }
  }

  /// Write this outcome's compact `$I+` token into `out`: `ok`, `no-echo`, `no-reply(fifo:N)`, `crc`, `ver:0xNN` (the
  /// version as two lowercase hex digits), `wrver:<actual>/<expected>` (the IFCNT deltas), `err`, `ok(<variant>)`
  /// (decoded despite an RX glitch — proven alive on tolerance), or `crc(<variant>)` (glitch-corrupted decode).
  /// Kept a writer rather than a `&str` getter because several tokens are dynamic.
  pub(crate) fn write_token<const N: usize>(self, out: &mut String<N>) -> Result<(), FmtError> {
    match self {
      TmcProbeStage::Responded => out.push_str("ok").map_err(|_| FmtError),
      TmcProbeStage::EchoTimeout => out.push_str("no-echo").map_err(|_| FmtError),
      // `no-reply(fifo:N)` names the post-timeout FIFO occupancy: `fifo:0` = nothing arrived (driver silent);
      // `fifo:N>0` = the driver replied and the read gated it (see the dumped bytes); `fifo:0,rdy` = occupancy
      // reported but nothing drainable (a flag re-latched).
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: false } => out.push_str("no-reply(fifo:0)").map_err(|_| FmtError),
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: true } => out.push_str("no-reply(fifo:0,rdy)").map_err(|_| FmtError),
      TmcProbeStage::ReplyTimeout { fifo, .. } => write!(out, "no-reply(fifo:{fifo})").map_err(|_| FmtError),
      TmcProbeStage::DecodeError => out.push_str("crc").map_err(|_| FmtError),
      TmcProbeStage::VersionMismatch(version) => write!(out, "ver:{version:#04x}").map_err(|_| FmtError),
      TmcProbeStage::WriteVerify { expected, actual } => write!(out, "wrver:{actual}/{expected}").map_err(|_| FmtError),
      TmcProbeStage::InitError => out.push_str("err").map_err(|_| FmtError),
      TmcProbeStage::RespondedDespiteGlitch(variant) => write!(out, "ok({})", variant.token()).map_err(|_| FmtError),
      TmcProbeStage::DecodeErrorGlitched(variant) => write!(out, "crc({})", variant.token()).map_err(|_| FmtError),
    }
  }
}

/// Per-axis TMC2209 presence-probe read-stage outcomes for the `$I+` echo-vs-reply diagnostic (`[MSG:TMC-PROBE
/// ...]`). Captured by the `firmware` bin's TMC init pass — for each node it records whether the presence read
/// responded, timed out at the echo stage, or timed out at the reply stage — and rendered by
/// [`ResponseWriter::driver_probe`]. Indexed in [`AXIS_LETTERS`] order. Kept a pure snapshot so the formatter is
/// host-testable, mirroring [`DriverStatus`]; the `firmware` bin owns the live values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DriverProbe {
  /// Per-axis presence-probe stage outcome, [`AXIS_LETTERS`] order.
  pub stages: [TmcProbeStage; AXIS_COUNT],
  /// `false` until the TMC init pass has run and `stages` reflect real bus results.
  pub initialized: bool,
  /// The raw 8-byte `IOIN` reply of the first node whose presence read hit a [`TmcProbeStage::DecodeError`], so
  /// the framing/alignment can be eyeballed on `$I+`. `None` when no node had a decode error (the common case).
  pub raw_ioin: Option<IoinRawCapture>,
  /// The failing bus operation of the first node whose init hit a bare [`TmcProbeStage::InitError`], so `$I+`
  /// names exactly which register + operation + error kind failed (`err:wGSTAT:to`). `None` when no node was an
  /// `InitError` — every categorized outcome (timeout/decode/version/write-verify) is self-describing already.
  pub init_failure: Option<InitFailure>,
}

/// Per-axis TMC2209 bus-exchange statistics for the `$I+` `[MSG:TMC-BUS …]` margin meter: how many datagram
/// exchanges each axis has attempted since boot (presence re-probes while absent, `DRV_STATUS` health polls
/// while present) and how many of them FAILED (no decodable reply). On a marginal single-wire bus this turns a
/// bench A/B — pull-up value, bus pad, baud, wiring — into a numeric failure rate over a fixed interval instead
/// of an eyeballed `ok`/`--` flicker: a genuinely absent node reads `N/N` (100 %), a solid one `0/N`, a marginal
/// one somewhere between. Counters saturate at `u16::MAX` (≈ 18 h of 1 Hz rounds) rather than wrap, so a
/// long-running bench never shows a misleading small number. Kept a pure snapshot so the formatter is
/// host-testable; the firmware manager task owns the live counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusStats {
  /// Per-axis failed-exchange count, [`AXIS_LETTERS`] order. Saturating.
  pub fail: [u16; AXIS_COUNT],
  /// Per-axis attempted-exchange count, [`AXIS_LETTERS`] order. Saturating.
  pub total: [u16; AXIS_COUNT],
}

impl BusStats {
  /// `true` once any axis has attempted an exchange — the caller's gate for emitting the `[MSG:TMC-BUS …]`
  /// line, so a fresh boot (before the first poll round) adds no meaningless all-zero line.
  pub fn any_attempted(&self) -> bool {
    self.total.iter().any(|&total| total > 0)
  }
}

impl DriverProbe {
  /// True when the snapshot is populated AND at least one axis's presence read did not cleanly respond, so the
  /// diagnostic line is worth emitting. A fully-responding bus (every axis `Responded`) reports nothing, keeping
  /// a healthy `$I+` free of the extra line.
  pub fn should_report(&self) -> bool {
    self.initialized && self.stages.iter().any(|stage| *stage != TmcProbeStage::Responded)
  }
}

/// The kind of bus error a TMC2209 operation failed with, for the `$I+` init-failure diagnostic. A compact
/// projection of [`TmcError`](crate::drivers::tmc2209::TmcError): the decode family (bad sync / address /
/// register / CRC) collapses to [`Decode`](Self::Decode) since the raw-bytes dump already carries the detail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusErrorKind {
  /// The echo or reply did not arrive within the turn-around window (token `to`).
  Timeout,
  /// The UART peripheral reported a framing/overflow error (token `io`).
  Io,
  /// A reply arrived but failed to decode — bad sync / address / register / CRC (token `dec`).
  Decode,
}

impl BusErrorKind {
  /// Map a [`TmcError`](crate::drivers::tmc2209::TmcError) to its compact diagnostic kind.
  pub fn from_error(error: crate::drivers::tmc2209::TmcError) -> Self {
    use crate::drivers::tmc2209::TmcError;
    match error {
      TmcError::Timeout => BusErrorKind::Timeout,
      TmcError::Io => BusErrorKind::Io,
      _ => BusErrorKind::Decode,
    }
  }

  /// The compact `$I+` token for this kind: `to`, `io`, or `dec`.
  pub(crate) fn token(self) -> &'static str {
    match self {
      BusErrorKind::Timeout => "to",
      BusErrorKind::Io => "io",
      BusErrorKind::Decode => "dec",
    }
  }

  /// The 2-bit code used in the packed [`InitFailure`] snapshot.
  fn to_code(self) -> u32 {
    match self {
      BusErrorKind::Timeout => 0,
      BusErrorKind::Io => 1,
      BusErrorKind::Decode => 2,
    }
  }
}

/// The specific UART RX error underneath a [`BusErrorKind::Io`], for the `$I+` init-failure diagnostic. A pure
/// projection of esp-hal's `RxError` variants (the `firmware` bin maps them at the one seam where the variant is
/// still in scope, before it flattens to `TmcError::Io`), so this stays esp-hal-free and host-testable. The
/// distinction is load-bearing for the fix: `Glitch`/`Framing` point at edge quality (pull-up / baud / slew),
/// `Overflow` at read cadence, and `Parity` at config drift (impossible on this 8N1 bus, so a red flag).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxErrorKind {
  /// The RX FIFO overflowed (token `ovf`) — a read-cadence problem, not an edge-quality one.
  Overflow,
  /// A glitch was detected on the RX line (token `glt`) — edge quality: pull-up strength, slew, or noise.
  Glitch,
  /// A framing error: the received bits did not conform to the UART frame (token `frm`) — edge quality / baud.
  Framing,
  /// A parity error (token `par`) — impossible on this 8N1 bus, so a sign of config drift if it appears.
  Parity,
}

impl RxErrorKind {
  /// The compact `$I+` token for this RX error: `ovf`, `glt`, `frm`, or `par`.
  pub(crate) fn token(self) -> &'static str {
    match self {
      RxErrorKind::Overflow => "ovf",
      RxErrorKind::Glitch => "glt",
      RxErrorKind::Framing => "frm",
      RxErrorKind::Parity => "par",
    }
  }

  /// The 2-bit code used in the packed [`InitFailure`] snapshot.
  fn to_code(self) -> u32 {
    match self {
      RxErrorKind::Overflow => 0,
      RxErrorKind::Glitch => 1,
      RxErrorKind::Framing => 2,
      RxErrorKind::Parity => 3,
    }
  }

  /// Recover from the 2-bit snapshot code.
  fn from_code(code: u32) -> Self {
    match code & 0x3 {
      0 => RxErrorKind::Overflow,
      1 => RxErrorKind::Glitch,
      2 => RxErrorKind::Framing,
      _ => RxErrorKind::Parity,
    }
  }
}

/// Which half-duplex read produced an [`BusErrorKind::Io`], for the `$I+` init-failure diagnostic. A reply-side
/// glitch and an echo-side glitch call for different fixes (a "skip echo + reset FIFO" workaround helps only the
/// echo case), so the stage must travel with the RX error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IoStage {
  /// The half-duplex echo read (the MCU's own transmitted bytes looped back) — token `e`.
  Echo,
  /// The driver's reply read — token `r`.
  Reply,
}

impl IoStage {
  /// The compact `$I+` token for this stage: `e` (echo) or `r` (reply).
  pub(crate) fn token(self) -> &'static str {
    match self {
      IoStage::Echo => "e",
      IoStage::Reply => "r",
    }
  }
}

/// The failing bus operation captured for a bare [`TmcProbeStage::InitError`] node, so `$I+` can name exactly
/// which register access failed and how — the key datum being the register identity (a first-write `GSTAT`
/// failure is a systemic write-path fault; a later one is accumulating timing/contention). Kept a pure snapshot
/// with reversible bit-packing so both the packing and [`write_token`](Self::write_token) are host-testable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InitFailure {
  /// The axis (and thus [`AXIS_LETTERS`] letter) whose init failed.
  pub axis: usize,
  /// The register byte the failing operation targeted (rendered via [`register_name`], else hex).
  pub reg: u8,
  /// `true` if the failing operation was a register write, `false` if a read (its request transmit).
  pub was_write: bool,
  /// The bus error kind, or `None` when the init failed with no failed bus op (a config/logic error, token
  /// `cfg`) — the fallback that keeps a mis-eliminated cause from masquerading as a bus fault.
  pub kind: Option<BusErrorKind>,
  /// For a [`BusErrorKind::Io`] originating from a UART RX error, the read stage and specific RX error variant,
  /// rendered as an `@<stage><variant>` suffix (e.g. `@efrm`). `None` when the `Io` was a transmit-side error
  /// (no `RxError` in scope) or the failure was not an `Io` at all — so the suffix's absence is itself a signal
  /// (a TX-side `io` rather than an RX glitch/framing/overflow).
  pub io_detail: Option<(IoStage, RxErrorKind)>,
}

impl InitFailure {
  /// Pack into a `u32` for the firmware's lock-free snapshot. Bit 0 is a validity flag (a stored `0` means "no
  /// failure captured"); bits 1..=14 carry register, the read/write flag, a 2-bit kind (`3` ⇒ `None`), and axis.
  /// Bit 15 flags an [`io_detail`](Self::io_detail); when set, bit 16 is the [`IoStage`] and bits 17..=18 the
  /// [`RxErrorKind`]. All within the `u32`, so the whole failure round-trips through one atomic.
  pub fn to_bits(&self) -> u32 {
    let kind = self.kind.map(BusErrorKind::to_code).unwrap_or(3);
    let mut bits = 1
      | (u32::from(self.reg) << 1)
      | (u32::from(self.was_write) << 9)
      | (kind << 10)
      | ((self.axis as u32 & 0x7) << 12);
    if let Some((stage, variant)) = self.io_detail {
      let stage_bit = matches!(stage, IoStage::Reply) as u32;
      bits |= (1 << 15) | (stage_bit << 16) | (variant.to_code() << 17);
    }
    bits
  }

  /// Recover from the packed `u32`; returns `None` when the validity bit is clear (no failure captured).
  pub fn from_bits(bits: u32) -> Option<Self> {
    if bits & 1 == 0 {
      return None;
    }
    let reg = ((bits >> 1) & 0xFF) as u8;
    let was_write = (bits >> 9) & 1 != 0;
    let kind = match (bits >> 10) & 0x3 {
      0 => Some(BusErrorKind::Timeout),
      1 => Some(BusErrorKind::Io),
      2 => Some(BusErrorKind::Decode),
      _ => None,
    };
    let axis = ((bits >> 12) & 0x7) as usize;
    let io_detail = if (bits >> 15) & 1 != 0 {
      let stage = if (bits >> 16) & 1 != 0 { IoStage::Reply } else { IoStage::Echo };
      Some((stage, RxErrorKind::from_code(bits >> 17)))
    } else {
      None
    };
    Some(InitFailure { axis, reg, was_write, kind, io_detail })
  }

  /// Write the enriched `$I+` token for this failure: `err:<w|r><REG>:<kind>` (e.g. `err:wGSTAT:to`), plus an
  /// `@<stage><variant>` suffix when [`io_detail`](Self::io_detail) is present (e.g. `err:rIOIN:io@efrm` = an RX
  /// framing error on the echo read). The register is a mnemonic when known (via [`register_name`]) else `0xNN`;
  /// the kind is `to`/`io`/`dec`, or `cfg` when no bus op failed.
  pub(crate) fn write_token<const N: usize>(self, out: &mut String<N>) -> Result<(), FmtError> {
    let rw = if self.was_write { 'w' } else { 'r' };
    let kind = self.kind.map(BusErrorKind::token).unwrap_or("cfg");
    match crate::drivers::tmc2209::registers::register_name(self.reg) {
      Some(name) => write!(out, "err:{rw}{name}:{kind}").map_err(|_| FmtError)?,
      None => write!(out, "err:{rw}{:#04x}:{kind}", self.reg).map_err(|_| FmtError)?,
    }
    if let Some((stage, variant)) = self.io_detail {
      write!(out, "@{}{}", stage.token(), variant.token()).map_err(|_| FmtError)?;
    }
    Ok(())
  }
}

/// A captured raw `IOIN` reply for the `$I+` `[MSG:TMC-IOIN …]` framing dump. Surfaced only for a node whose
/// presence read hit a [`TmcProbeStage::DecodeError`] (a reply arrived but failed to decode), where the exact
/// bytes on the wire are the single most useful thing for diagnosing framing / echo-reply misalignment without
/// a scope. Kept a pure snapshot so [`ResponseWriter::driver_ioin_raw`] is host-testable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IoinRawCapture {
  /// The axis (and thus [`AXIS_LETTERS`] letter) the captured bytes belong to.
  pub axis: usize,
  /// The raw 8-byte reply datagram exactly as read from the bus, before decode.
  pub bytes: [u8; crate::drivers::tmc2209::READ_REPLY_LEN],
}

/// The fixed 8-byte pattern the boot loopback self-test transmits on GPIO9 to read back its own half-duplex
/// echo. It exercises every bit level and both alternating phases — all-low (`0x00`), all-high (`0xFF`), the
/// `0x55`/`0xAA` alternations, and the `0x0F`/`0xF0`/`0x33`/`0xCC` nibble splits — so a stuck bit, wrong level,
/// or marginal edge shows as a mismatch rather than a lucky pass. It is deliberately NOT a valid addressed read
/// request (its CRC will not match any node), so no driver replies to it — the test is safe to run with the
/// drivers attached (their high-Z receivers do not interfere).
pub const TMC_LOOPBACK_PATTERN: [u8; 8] = [0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x33, 0xCC];

/// The result of the boot loopback self-test (DOC-03 bring-up aid), rendered on `$I+` as
/// `[MSG:TMC-LOOPBACK sent:8 got:N match:M/8 err:<none|ovf|glt|frm|par>]`. Because RX and TX share GPIO9, the
/// MCU always half-duplex-echoes its own transmit; reading that echo back and comparing it to
/// [`TMC_LOOPBACK_PATTERN`] proves the MCU's TX + RX path and line levels WITHOUT a scope. A full `8/8` match
/// means TX, RX, and edges are healthy and any remaining fault is driver-side; `got:0` means the MCU half is
/// broken; a partial/mismatch means TX transmits but the levels/edges are marginal. Kept a pure snapshot with
/// reversible packing so both the packing and [`ResponseWriter::loopback`] are host-testable, mirroring
/// [`InitFailure`]; the `firmware` bin owns the live values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LoopbackReport {
  /// `false` until the boot self-test has run; suppresses the `$I+` line during the pre-test window.
  pub ran: bool,
  /// The number of bytes echoed back (0..=8).
  pub got: u8,
  /// The number of byte positions whose echo matched the transmitted pattern (0..=8).
  pub matched: u8,
  /// The first RX-error variant seen during the readback, or `None` if the echo came back clean.
  pub err: Option<RxErrorKind>,
}

impl LoopbackReport {
  /// Pack into a `u32` for the firmware's lock-free snapshot: bit 0 = `ran`, bits 1..=4 = `got`, bits 5..=8 =
  /// `matched`, bits 9..=11 = the RX-error code (`0` = none, else variant code + 1). `got`/`matched` are ≤ 8 so
  /// they fit a nibble.
  pub fn to_bits(&self) -> u32 {
    let err = self.err.map(|variant| variant.to_code() + 1).unwrap_or(0);
    u32::from(self.ran)
      | (u32::from(self.got.min(0x0F)) << 1)
      | (u32::from(self.matched.min(0x0F)) << 5)
      | (err << 9)
  }

  /// Recover from the packed `u32`.
  pub fn from_bits(bits: u32) -> Self {
    let err = match (bits >> 9) & 0x7 {
      0 => None,
      code => Some(RxErrorKind::from_code(code - 1)),
    };
    LoopbackReport {
      ran: bits & 1 != 0,
      got: ((bits >> 1) & 0x0F) as u8,
      matched: ((bits >> 5) & 0x0F) as u8,
      err,
    }
  }
}
