//! GCode parser modal-state DTOs and the `$G` [`ParserSnapshot`].

/// The active motion mode reported as the first word of a `$G` (`[GC:...]`) parser-state line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserMotion {
  /// G0 rapid positioning.
  Rapid,
  /// G1 linear feed move.
  Linear,
  /// G2 clockwise arc.
  ArcCw,
  /// G3 counter-clockwise arc.
  ArcCcw,
}

impl ParserMotion {
  /// The `G<n>` word for this motion mode.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserMotion::Rapid => "G0",
      ParserMotion::Linear => "G1",
      ParserMotion::ArcCw => "G2",
      ParserMotion::ArcCcw => "G3",
    }
  }
}

/// The active spindle state (modal group 7) reported in a `$G` line: M3/M4 when running, M5 when stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserSpindle {
  /// M3 — spindle on, clockwise.
  Clockwise,
  /// M4 — spindle on, counter-clockwise.
  CounterClockwise,
  /// M5 — spindle stop (the power-on default).
  Stop,
}

impl ParserSpindle {
  /// The `M<n>` word for this spindle state.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserSpindle::Clockwise => "M3",
      ParserSpindle::CounterClockwise => "M4",
      ParserSpindle::Stop => "M5",
    }
  }
}

/// The active plane (modal group 2) reported in a `$G` line: G17 XY / G18 ZX / G19 YZ.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserPlane {
  /// G17 — the XY plane (the power-on default).
  #[default]
  XY,
  /// G18 — the ZX plane.
  ZX,
  /// G19 — the YZ plane.
  YZ,
}

impl ParserPlane {
  /// The `G<n>` word for this plane.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserPlane::XY => "G17",
      ParserPlane::ZX => "G18",
      ParserPlane::YZ => "G19",
    }
  }
}

/// The active coolant state (modal group 8) reported in a `$G` line. Mist (M7) and flood (M8) are independent and
/// can both be active; M9 (all off) is the default. The formatter renders the active word(s): `M9` when both are
/// off, `M7` / `M8` for a single circuit, or `M7 M8` for both — matching grbl's `$G` group-8 output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserCoolant {
  /// Mist coolant (M7) active.
  pub mist: bool,
  /// Flood coolant (M8) active.
  pub flood: bool,
}

/// The active units mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserUnits {
  /// G20 inch units.
  Inch,
  /// G21 millimeter units.
  Millimeter,
}

impl ParserUnits {
  /// The `G<n>` word for this units mode.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserUnits::Inch => "G20",
      ParserUnits::Millimeter => "G21",
    }
  }
}

/// The active distance mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserDistance {
  /// G90 absolute distance.
  Absolute,
  /// G91 incremental distance.
  Incremental,
}

impl ParserDistance {
  /// The `G<n>` word for this distance mode.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserDistance::Absolute => "G90",
      ParserDistance::Incremental => "G91",
    }
  }
}

/// The active feed-rate mode reported in a `$G` line (modal group 5, DOC-10.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserFeedMode {
  /// G93 inverse-time feed.
  InverseTime,
  /// G94 units-per-minute feed (the grbl power-on default).
  UnitsPerMin,
}

impl ParserFeedMode {
  /// The `G<n>` word for this feed mode.
  pub(crate) fn word(self) -> &'static str {
    match self {
      ParserFeedMode::InverseTime => "G93",
      ParserFeedMode::UnitsPerMin => "G94",
    }
  }
}

/// A `Copy` snapshot of the live parser modal state the `$G` formatter renders. The `firmware` bin builds
/// this from the consumer's persistent `gcode::Parser` (via its `state()`) so the `[GC:...]` line reports
/// the real motion/units/distance/feed/spindle words rather than a hardcoded constant. Keeping the
/// formatter pure over a small snapshot — rather than importing the parser's modal type here — keeps
/// `protocol` free of any GCode-parsing coupling and the `$G` rendering host-testable in isolation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserSnapshot {
  /// Active motion mode (modal group 1) — the first `$G` word.
  pub motion: ParserMotion,
  /// Active units mode (modal group 6).
  pub units: ParserUnits,
  /// Active distance mode (modal group 3).
  pub distance: ParserDistance,
  /// Active feed-rate mode (modal group 5): reported as `G93`/`G94`.
  pub feed_mode: ParserFeedMode,
  /// Active work coordinate system (modal group 12): 0 = G54 … 5 = G59. Reported in `$G` as `G54`…`G59`.
  pub wcs: usize,
  /// Whether a dynamic tool-length offset (`G43.1`) is active (modal group 8): reported as `G43.1` when set,
  /// `G49` when clear.
  pub tlo_active: bool,
  /// Programmed feed rate (modal F), in the active units per minute.
  pub feed: f32,
  /// Active spindle state (modal group 7): reported as `M3`/`M4`/`M5`.
  pub spindle: ParserSpindle,
  /// Programmed spindle speed (modal S), in RPM.
  pub spindle_rpm: u16,
  /// Active plane (modal group 2): reported as `G17`/`G18`/`G19`.
  pub plane: ParserPlane,
  /// Active coolant state (modal group 8): reported as `M9` / `M7` / `M8` / `M7 M8`.
  pub coolant: ParserCoolant,
  /// The CURRENT (active) tool number, reported as `T<n>` (`T0` = no tool selected). Committed by `M6` from the
  /// pending `T` word; persists across program end / soft reset (grbl keeps the physically-loaded tool).
  pub tool: u16,
}

impl ParserSnapshot {
  /// The grbl power-on modal defaults (G0 rapid, mm, absolute, G54, G49, no feed, spindle off). Used before
  /// any motion word has been parsed, and as the basis for partial snapshots in tests.
  pub const fn power_on() -> Self {
    Self {
      motion: ParserMotion::Rapid,
      units: ParserUnits::Millimeter,
      distance: ParserDistance::Absolute,
      feed_mode: ParserFeedMode::UnitsPerMin,
      wcs: 0,
      tlo_active: false,
      feed: 0.0,
      spindle: ParserSpindle::Stop,
      spindle_rpm: 0,
      plane: ParserPlane::XY,
      coolant: ParserCoolant { mist: false, flood: false },
      tool: 0,
    }
  }
}
