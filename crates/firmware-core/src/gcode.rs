//! GCode tokenizer and parser (DOC-04).
//!
//! A streaming, allocation-free parser. Each CR/LF-terminated line is:
//! 1. **Lexed** into `(letter, f32)` word pairs, stripping whitespace and `(...)`/`;` comments,
//!    case-insensitive (see [`Lexer`]).
//! 2. **Validated** against the supported modal groups; at most one word per modal group per line,
//!    and unknown words produce a grblHAL error code (see [`GcodeError`]).
//! 3. **Applied** to a persistent [`ModalState`] (motion mode, units G20/G21, distance G90/G91,
//!    plane G17), then emitted as a [`PlannerCommand`].
//!
//! ## Scope boundary with the planner (DOC-05)
//! This parser performs no geometry. It records the active modal flags (units, distance mode, motion
//! mode) and the line's axis/parameter words verbatim, and hands them to the planner inside a
//! [`PlannerCommand`]. Unit conversion (G20/G21), coordinate-offset application, steps/mm scaling,
//! and arc subdivision are all DOC-05 concerns — the doc is explicit that targets are resolved "after
//! G20/G21 and coordinate offset application", which is the planner's job, not the lexer's. Keeping
//! the transform out of the parser makes the parser a pure, fully host-testable function of bytes in
//! to command out, and lets the planner own the single source of truth for kinematics.
//!
//! ## Allocation
//! The parser is `#![no_std]` and allocation-free. A line is folded into a fixed-size accumulator
//! struct as words stream past, so no growable container is ever needed: the meaningful words on any
//! one line are a bounded set (one motion mode, one feed, one spindle speed, the X/Y/Z/I/J/P/S
//! parameter words). Floats are parsed by a hand-written `no_std` lexer; `core::str`'s float parsing
//! is intentionally avoided so the numeric grammar matches what grbl senders actually emit.

/// grblHAL-compatible status codes returned when a line cannot be parsed or validated. The numeric
/// values match grblHAL's `error:N` responses so [`crate::protocol`] (DOC-08) can render them
/// directly without a translation table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GcodeError {
  /// `error:1` — a word had a command letter but no numeric value, or a stray value with no letter.
  ExpectedCommandLetterValue,
  /// `error:2` — a numeric value was malformed (e.g. `X1.2.3`, lone `-`, empty after the letter).
  BadNumberFormat,
  /// `error:21` — more than one word from the same modal group appeared on a single line (e.g. two
  /// motion words `G0 G1`). grbl's dedicated "Modal group violation" code (distinct from the `error:9`
  /// G-code-lock state, which the firmware reserves for rejecting GCode while in an alarm/jog state).
  ModalGroupViolation,
  /// `error:20` — an unsupported or unrecognized command word for the implemented GCode subset.
  UnsupportedCommand,
  /// `error:26` — a `G38.x` probe command carried no axis word, so there is no direction to probe. grbl's
  /// "No axis words in block" — a probe must name at least one axis to move toward/away.
  ProbeNoAxis,
  /// `error:22` — a `$J=` jog (or a feed move) carried no `F` word, so the feed rate is undefined. grbl's
  /// "Feed rate has not yet been set or is undefined" — a jog MUST name a feed; there is no modal feed for it.
  FeedRateUndefined,
  /// `error:26` — a `$J=` jog carried no axis word, so there is no direction to move. Same grbl class as
  /// [`ProbeNoAxis`](GcodeError::ProbeNoAxis) ("No axis words in block"); kept a distinct variant so a jog
  /// rejection reads clearly at the call site, while sharing the wire code 26.
  JogNoAxis,
  /// `error:22` — a `G38.x` probe was issued while `G93` inverse-time feed mode is active. A probe needs a
  /// well-defined units/min CONTACT speed, but inverse-time defines speed as distance ÷ duration — and a probe's
  /// distance is the arbitrary no-contact overshoot, so the seek speed would be meaningless. The probe is therefore
  /// rejected; the operator must switch to `G94` to probe. Distinct from [`FeedRateUndefined`](GcodeError::FeedRateUndefined)
  /// at the call site (so the rejection reads clearly) while sharing the feed-family wire code 22.
  ProbeInverseTimeUnsupported,
  /// `error:33` — a `G38.x` probe carried a rotary `A` axis word. Probing is LINEAR-ONLY: probing while a rotary
  /// axis moves is metrologically unsound (the surface normal rotates under a fixed probe vector → cosine error,
  /// invalid tip-radius compensation), so no mainstream controller probes through a rotary axis. Any `A` word in a
  /// probe is rejected — even a redundant `A` equal to the current position — so the contract is unambiguous. Maps
  /// to grbl's "Invalid target" code 33 (the probe-target-invalid family), distinct at the call site.
  ProbeRotaryAxisWord,
}

impl GcodeError {
  /// The numeric grblHAL status code for this error, suitable for an `error:N` response line.
  pub fn code(self) -> u8 {
    match self {
      GcodeError::ExpectedCommandLetterValue => 1,
      GcodeError::BadNumberFormat => 2,
      GcodeError::ModalGroupViolation => 21,
      GcodeError::UnsupportedCommand => 20,
      GcodeError::ProbeNoAxis => 26,
      GcodeError::FeedRateUndefined => 22,
      GcodeError::JogNoAxis => 26,
      GcodeError::ProbeInverseTimeUnsupported => 22,
      GcodeError::ProbeRotaryAxisWord => 33,
    }
  }
}

/// One lexed word: an uppercased command letter and its parsed `f32` value (e.g. `('X', -12.5)`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Word {
  /// The command letter, always uppercased by the lexer so matching is case-insensitive.
  pub letter: u8,
  /// The parsed numeric value following the letter.
  pub value: f32,
}

/// Streaming, allocation-free lexer over one GCode line. Yields [`Word`]s on demand, skipping
/// whitespace and stripping `(...)` block comments and `;` line comments. The lexer never allocates
/// and holds only a byte cursor into the borrowed line, so it works identically on host and target.
///
/// CR/LF line termination is the caller's responsibility: pass a single line (terminator already
/// removed). An embedded CR or LF inside the slice is treated as whitespace, which keeps a stray
/// terminator from corrupting a word but does not split the slice into multiple lines.
pub struct Lexer<'a> {
  bytes: &'a [u8],
  pos: usize,
}

impl<'a> Lexer<'a> {
  /// Create a lexer over `line`, which must be a single line with its CR/LF terminator removed.
  pub fn new(line: &'a [u8]) -> Self {
    Lexer { bytes: line, pos: 0 }
  }

  /// Skip whitespace and comments, advancing the cursor to the next significant byte. A `;` begins a
  /// line comment that consumes the remainder of the line. A `(` begins a block comment consuming up
  /// to and including the matching `)`; an unterminated block comment consumes to end of line, which
  /// mirrors grbl's lenient handling rather than raising an error.
  fn skip_insignificant(&mut self) {
    while self.pos < self.bytes.len() {
      match self.bytes[self.pos] {
        b' ' | b'\t' | b'\r' | b'\n' => self.pos += 1,
        b';' => {
          self.pos = self.bytes.len();
        }
        b'(' => {
          self.pos += 1;
          while self.pos < self.bytes.len() && self.bytes[self.pos] != b')' {
            self.pos += 1;
          }
          // Consume the closing paren when present; an unterminated comment already hit end of line.
          if self.pos < self.bytes.len() {
            self.pos += 1;
          }
        }
        _ => break,
      }
    }
  }

  /// Lex the next word, or `None` at end of line. Returns [`GcodeError`] for a value-less letter or a
  /// malformed number. The letter is uppercased so callers see case-insensitive words.
  pub fn next_word(&mut self) -> Option<Result<Word, GcodeError>> {
    self.skip_insignificant();
    if self.pos >= self.bytes.len() {
      return None;
    }
    let first = self.bytes[self.pos];
    // A significant byte that is not a letter is a value with no command letter (e.g. a stray `5`).
    if !first.is_ascii_alphabetic() {
      return Some(Err(GcodeError::ExpectedCommandLetterValue));
    }
    let letter = first.to_ascii_uppercase();
    self.pos += 1;
    // Whitespace may sit between the letter and its value (`X 5`), so skip it before reading digits.
    while self.pos < self.bytes.len() && matches!(self.bytes[self.pos], b' ' | b'\t') {
      self.pos += 1;
    }
    let start = self.pos;
    self.advance_over_number();
    let token = &self.bytes[start..self.pos];
    if token.is_empty() {
      return Some(Err(GcodeError::ExpectedCommandLetterValue));
    }
    match parse_f32(token) {
      Ok(value) => Some(Ok(Word { letter, value })),
      Err(err) => Some(Err(err)),
    }
  }

  /// Advance the cursor over the bytes that form a number: an optional sign, digits, and at most a
  /// single decimal point. Validation of the collected bytes is left to [`parse_f32`].
  fn advance_over_number(&mut self) {
    if self.pos < self.bytes.len() && matches!(self.bytes[self.pos], b'+' | b'-') {
      self.pos += 1;
    }
    while self.pos < self.bytes.len() && matches!(self.bytes[self.pos], b'0'..=b'9' | b'.') {
      self.pos += 1;
    }
  }
}

/// Parse a GCode numeric token into an `f32` without `core::str` float parsing. Accepts an optional
/// leading sign, decimal digits, and a single optional decimal point with optional fractional digits
/// (`5`, `-12.5`, `+100`, `.5`, `12.`). Exponent notation is intentionally rejected: grbl senders
/// never emit it, and accepting it would silently widen the grammar.
pub fn parse_f32(token: &[u8]) -> Result<f32, GcodeError> {
  let mut i = 0;
  let mut negative = false;
  if i < token.len() && matches!(token[i], b'+' | b'-') {
    negative = token[i] == b'-';
    i += 1;
  }
  let mut int_part: f32 = 0.0;
  let mut saw_digit = false;
  while i < token.len() && token[i].is_ascii_digit() {
    int_part = int_part * 10.0 + f32::from(token[i] - b'0');
    saw_digit = true;
    i += 1;
  }
  let mut value = int_part;
  if i < token.len() && token[i] == b'.' {
    i += 1;
    let mut scale: f32 = 0.1;
    while i < token.len() && token[i].is_ascii_digit() {
      value += f32::from(token[i] - b'0') * scale;
      scale *= 0.1;
      saw_digit = true;
      i += 1;
    }
  }
  // A token with no digits at all (lone sign, lone dot) or trailing junk is malformed.
  if !saw_digit || i != token.len() {
    return Err(GcodeError::BadNumberFormat);
  }
  Ok(if negative { -value } else { value })
}

/// Motion mode (grbl modal group 1). The active motion mode is modal: a line with axis words but no
/// motion word inherits the previously commanded mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotionMode {
  /// G0 — rapid positioning move.
  Rapid,
  /// G1 — linear feed move.
  Linear,
  /// G2 — clockwise arc.
  ArcCw,
  /// G3 — counter-clockwise arc.
  ArcCcw,
}

/// Distance mode (grbl modal group 3): whether axis words are absolute (G90) or incremental (G91).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DistanceMode {
  /// G90 — coordinates are absolute machine/work positions.
  Absolute,
  /// G91 — coordinates are increments relative to the current position.
  Incremental,
}

/// Units mode (grbl modal group 6): the unit that axis/feed words are expressed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Units {
  /// G20 — inches.
  Inch,
  /// G21 — millimeters.
  Millimeter,
}

/// Feed-rate mode (RS274/NGC modal group 5, DOC-10.2). G94 is the power-on/reset default.
///
/// Under **G94** the `F` word is units-per-minute (mm/min for linear travel, deg/min for a pure-rotary move);
/// it is modal and carries forward across lines. Under **G93** the `F` word is *inverse time* — it specifies
/// `1/(move duration in minutes)`, so the move takes `1/F` minutes regardless of its length. Inverse-time `F`
/// is **not** usefully modal: grblHAL requires a fresh `F` on every feed-motion line (G1/G2/G3 and `G38.x`),
/// rejecting one without it as `error:22` (`FeedRateUndefined`). G0 rapids never need an `F` in either mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FeedMode {
  /// G94 — feed is units per minute. The grbl power-on / reset default.
  #[default]
  UnitsPerMin,
  /// G93 — inverse time: `F` is `1/(move duration in minutes)`; the move takes `1/F` minutes regardless of length.
  InverseTime,
}

/// The four `G38.x` probe modes (DOC-09, `docs/tlo-offsets.md` §8, `docs/gcode-streaming.md` §9). Each pairs a
/// direction sense with whether a failed probe ALARMS:
/// - **G38.2** — probe TOWARD the workpiece, stop on contact; **error/ALARM if no contact** within the travel.
/// - **G38.3** — probe toward, stop on contact; **no error** if no contact (the sender checks the `[PRB:]` flag).
/// - **G38.4** — probe AWAY from the workpiece, stop on loss of contact; **error/ALARM if still in contact** at
///   the end of travel.
/// - **G38.5** — probe away, stop on loss of contact; **no error** if it never releases.
///
/// `toward` distinguishes the contact-seeking modes (.2/.3) from the release-seeking modes (.4/.5);
/// `alarm_on_fail` distinguishes the alarming modes (.2/.4) from the silent ones (.3/.5). The executor uses
/// `toward` to choose the stop edge (trigger vs release) and the consumer uses `alarm_on_fail` to decide whether
/// a no-trigger outcome raises ALARM:4/5 or simply finishes with `[PRB:..:0]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProbeKind {
  /// `true` for the contact-seeking modes (G38.2/.3): stop the instant the probe TRIGGERS. `false` for the
  /// release-seeking modes (G38.4/.5): stop the instant the probe RELEASES (loses contact).
  pub toward: bool,
  /// `true` for the alarming modes (G38.2/.4): a probe that reaches the target without the expected edge raises
  /// ALARM (4 if it never moved off its initial state, 5 if it never reached the expected edge in travel).
  /// `false` for the silent modes (G38.3/.5): a no-edge outcome just finishes with a `[PRB:..:0]` and one `ok`.
  pub alarm_on_fail: bool,
}

impl ProbeKind {
  /// The contact-seeking, alarming probe (G38.2) — the touch-plate default in `docs/tlo-offsets.md`.
  pub const G38_2: ProbeKind = ProbeKind { toward: true, alarm_on_fail: true };
  /// The contact-seeking, silent probe (G38.3).
  pub const G38_3: ProbeKind = ProbeKind { toward: true, alarm_on_fail: false };
  /// The release-seeking, alarming probe (G38.4).
  pub const G38_4: ProbeKind = ProbeKind { toward: false, alarm_on_fail: true };
  /// The release-seeking, silent probe (G38.5).
  pub const G38_5: ProbeKind = ProbeKind { toward: false, alarm_on_fail: false };
}

/// Spindle state requested by a line (grbl modal group 7): M3/M4 start the spindle in a direction,
/// M5 stops it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpindleState {
  /// M3 — spindle on, clockwise.
  Clockwise,
  /// M4 — spindle on, counter-clockwise.
  CounterClockwise,
  /// M5 — spindle stop. The power-on / default modal state (the spindle is off until an M3/M4).
  #[default]
  Stop,
}

/// Persistent modal state carried across lines. The parser updates this in place as modal words are
/// consumed, then stamps the active values into each emitted [`PlannerCommand`]. Defaults match grbl
/// power-on: G0, G90 absolute, G21 millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModalState {
  /// Active motion mode (modal group 1).
  pub motion: MotionMode,
  /// Active distance mode (modal group 3).
  pub distance: DistanceMode,
  /// Active units (modal group 6).
  pub units: Units,
  /// Active feed-rate mode (modal group 5): G94 units/min (default) or G93 inverse-time. Sticky across lines.
  pub feed_mode: FeedMode,
  /// Active work coordinate system (modal group 12): 0 = G54 … 5 = G59. Sticky across lines and reported in
  /// `$G` as `G54`…`G59`. The actual offset for the active WCS lives in [`crate::coords::CoordinateSystems`].
  pub wcs: usize,
  /// Active tool-length-offset mode (modal group 8): `true` when a dynamic `G43.1` TLO is in effect, `false`
  /// after `G49`. Tracked for `$G` (`G43.1`/`G49`); the TLO value lives in the coordinate model.
  pub tlo_active: bool,
  /// Last commanded feed rate (F word), in the active units per minute; sticky across lines.
  pub feed: f32,
  /// Last commanded spindle speed (S word), in RPM; sticky across lines.
  pub spindle_speed: f32,
  /// Active spindle direction (modal group 7): `Clockwise`/`CounterClockwise` after M3/M4, `Stop` after M5.
  /// Sticky across lines so the commanded direction survives even when the M-word shares a line with a move
  /// (the per-line emit can carry only one command, so the firmware drives the spindle from this modal value).
  pub spindle: SpindleState,
}

impl Default for ModalState {
  fn default() -> Self {
    ModalState {
      motion: MotionMode::Rapid,
      distance: DistanceMode::Absolute,
      units: Units::Millimeter,
      feed_mode: FeedMode::UnitsPerMin,
      wcs: 0,
      tlo_active: false,
      feed: 0.0,
      spindle_speed: 0.0,
      spindle: SpindleState::Stop,
    }
  }
}

/// Per-axis target words present on a line. `None` means the axis was not mentioned and retains its
/// current position; `Some(v)` is the raw value in the active units (the planner applies units and
/// distance mode). Three axes per DOC-02 (X/Y/Z).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxisWords {
  /// X target word, if present on the line.
  pub x: Option<f32>,
  /// Y target word, if present on the line.
  pub y: Option<f32>,
  /// Z target word, if present on the line.
  pub z: Option<f32>,
  /// A target word (rotation about X), if present on the line. In degrees for a rotary A (the active linear
  /// units' G20/G21 inch scaling is suppressed for an axis marked rotary in `$376`; the planner applies that
  /// per-axis fork). For a non-rotary A it is a 4th linear word like X/Y/Z (DOC-10.1).
  pub a: Option<f32>,
}

/// A coordinate-system / offset operation the parser emits for the Phase B words (G10, G54-G59, G92,
/// G28.1/G30.1, G43.1/G49). These mutate the [`crate::coords::CoordinateSystems`] model the consumer owns,
/// NOT the planner geometry directly — so they carry only the WORK-coordinate words and intent; the consumer
/// resolves any "make the current machine position read this work value" op against the live machine position
/// (which the parser does not have). All axis/offset values are raw, in the active units (the consumer scales
/// inch→mm via [`Units`] before applying), so the parser stays a pure function of the line.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoordinateOp {
  /// `G54`-`G59` select the active work coordinate system. `index` is 0 = G54 … 5 = G59.
  SelectWcs {
    /// The selected WCS index (0 = G54 … 5 = G59).
    index: usize,
  },
  /// `G10 L2 P<n>` set WCS `index` offset directly to the present axis words (the literal new offset).
  SetWcsOffset {
    /// The target WCS index (0 = G54 … 5 = G59), from the `P` word (`P1` = G54).
    index: usize,
    /// The new offset axis words (only present axes are written).
    axes: AxisWords,
    /// Active units for the offset words.
    units: Units,
  },
  /// `G10 L20 P<n>` set WCS `index` offset so the current machine position maps to the present work words.
  SetWcsOffsetToPosition {
    /// The target WCS index (0 = G54 … 5 = G59), from the `P` word (`P1` = G54).
    index: usize,
    /// The work-coordinate target the current machine position should read back as.
    axes: AxisWords,
    /// Active units for the work words.
    units: Units,
  },
  /// `G92` set the dynamic offset so the current machine position reads as the present work words.
  SetG92ToPosition {
    /// The work-coordinate target the current machine position should read back as.
    axes: AxisWords,
    /// Active units for the work words.
    units: Units,
  },
  /// `G92.1` clear the G92 dynamic offset to identity.
  ClearG92,
  /// `G28.1`/`G30.1` store the current machine position as the predefined position `index` (0 = G28, 1 = G30).
  StorePredefined {
    /// 0 = G28 (via G28.1), 1 = G30 (via G30.1).
    index: usize,
  },
  /// `G43.1 Z<value>` apply a dynamic tool-length offset from the Z word (in the active units).
  ApplyTlo {
    /// The Z tool-length-offset value (raw, in `units`).
    z: f32,
    /// Active units for the Z value.
    units: Units,
  },
  /// `G49` cancel the dynamic tool-length offset.
  CancelTlo,
}

/// A validated `$J=` jog command (DOC-08, `docs/gcode-streaming.md` §"jogging"). A jog is an independent,
/// cancelable rapid-style move that, by grbl design, NEVER disturbs the persistent parser modal state — it is
/// parsed in a throwaway modal context seeded from (but discarded back to) the current state, so a program left
/// in `G91`/`G20` makes `$J=X10` incremental/inch without mutating `gc_state`. The fields carry exactly the raw
/// words plus the throwaway modal flags the planner needs to resolve the target; `feed` is mandatory (a jog has
/// no modal feed) and at least one axis must be present (otherwise [`GcodeError::JogNoAxis`]).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct JogCommand {
  /// The jog target axis words present on the `$J=` line (at least one is present; the parser rejects a jog
  /// with no axis word as [`GcodeError::JogNoAxis`]).
  pub axes: AxisWords,
  /// The throwaway distance mode (G90 absolute / G91 incremental) the jog resolves against — seeded from the
  /// current persistent state and overridable by a `G90`/`G91` word on the jog line, but NOT written back.
  pub distance_mode: DistanceMode,
  /// The throwaway units mode (G20 inch / G21 mm) for the axis/feed words — seeded from the current state and
  /// overridable by a `G20`/`G21` word on the jog line, but NOT written back.
  pub units: Units,
  /// The mandatory jog feed rate (`F` word), in `units` per minute. A jog with no `F` is rejected as
  /// [`GcodeError::FeedRateUndefined`]: unlike a G1 move, a jog has no modal feed to fall back on.
  pub feed: f32,
  /// True when a `G53` word on the jog line makes the axis words MACHINE coordinates (the planner must NOT apply
  /// the active work offset), false for an ordinary work-coordinate jog.
  pub machine_coords: bool,
}

/// A validated command emitted to the planner (the payload of the parser→planner channel, DOC-04).
///
/// Each variant carries the raw, unconverted words plus the modal context needed for the planner to
/// resolve geometry. The parser does not convert units or apply coordinate offsets; that is DOC-05.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlannerCommand {
  /// A linear move (G0/G1). `rapid` distinguishes G0 from G1; `feed` and `spindle_speed` carry the
  /// active modal values so the planner needs no back-reference to parser state.
  Move {
    /// True for G0 rapid, false for G1 feed move.
    rapid: bool,
    /// Axis target words present on the line.
    axes: AxisWords,
    /// Active units for the axis/feed words.
    units: Units,
    /// Active distance mode for the axis words.
    distance: DistanceMode,
    /// Active feed rate (modal F). Its meaning depends on `feed_mode`: under G94 it is `units` per minute;
    /// under G93 it is inverse time (`1/(move duration in minutes)`). The planner resolves it accordingly.
    feed: f32,
    /// Active feed-rate mode (modal group 5): G94 units/min or G93 inverse-time. Governs how the planner
    /// interprets `feed` when deriving the block's nominal speed (DOC-10.2).
    feed_mode: FeedMode,
    /// True when this is a `G53` one-shot machine-coordinate move: the axis words are MACHINE positions,
    /// so the planner must NOT apply the active work offset. False for an ordinary work-coordinate move.
    machine_coords: bool,
  },
  /// An arc move (G2/G3). `cw` distinguishes G2 from G3; `i`/`j` are the center offsets in the active
  /// units relative to the start point (grbl IJ arc form). Plane is fixed to G17 (XY) per DOC-04.
  Arc {
    /// True for G2 clockwise, false for G3 counter-clockwise.
    cw: bool,
    /// Axis target words present on the line (arc endpoint).
    axes: AxisWords,
    /// I center offset (X axis) relative to the start point, if present.
    i: Option<f32>,
    /// J center offset (Y axis) relative to the start point, if present.
    j: Option<f32>,
    /// Active units for the axis/offset/feed words.
    units: Units,
    /// Active distance mode for the axis words.
    distance: DistanceMode,
    /// Active feed rate (modal F). Under G94 it is `units` per minute; under G93 it is inverse time for the
    /// whole arc (`1/(arc duration in minutes)`), which the planner distributes across the arc's segments.
    feed: f32,
    /// Active feed-rate mode (modal group 5): G94 units/min or G93 inverse-time (DOC-10.2).
    feed_mode: FeedMode,
    /// True when this is a `G53` one-shot machine-coordinate arc: the endpoint words are MACHINE positions
    /// (the planner must not apply the work offset). False for an ordinary work-coordinate arc.
    machine_coords: bool,
  },
  /// A `G38.x` probe move (DOC-09). The axis words are the probe TARGET in the active work coordinate system —
  /// the planner converts them to a machine target exactly like an absolute/incremental move (the WCO is applied
  /// for an absolute work probe, an incremental probe adds to the current position). `kind` carries the
  /// toward/away + alarm-on-fail semantics; `feed` is the probe feed (`F` word, modal). A probe is a synchronized
  /// motion boundary: the planner flushes look-ahead so the probe starts from rest, and the firmware runs the
  /// probe block on a distinct, probe-watching execution path (it is NOT a normal queued block).
  Probe {
    /// The probe mode (toward/away, alarm-on-fail) from `G38.2`/`.3`/`.4`/`.5`.
    kind: ProbeKind,
    /// The probe target axis words (work coordinates), at least one present — the parser rejects a probe with no
    /// axis word as [`GcodeError::ProbeNoAxis`].
    axes: AxisWords,
    /// Active units for the axis/feed words.
    units: Units,
    /// Active distance mode for the axis words (absolute work target vs incremental from the current position).
    distance: DistanceMode,
    /// Active feed rate (modal F), in `units` per minute — the probe seek speed.
    feed: f32,
  },
  /// G4 dwell for `seconds` (the P word, always seconds regardless of units mode).
  Dwell {
    /// Dwell duration in seconds (the P word).
    seconds: f32,
  },
  /// G28/G30 go to a predefined position. `intermediate` carries any axis words on the line, which
  /// grbl uses as an intermediate point to move through before reaching the stored position.
  GoToPredefined {
    /// True for G28, false for G30.
    is_g28: bool,
    /// Optional intermediate axis words to move through first.
    intermediate: AxisWords,
    /// Active units for the intermediate axis words.
    units: Units,
    /// Active distance mode for the intermediate axis words.
    distance: DistanceMode,
  },
  /// A coordinate-system / offset operation (G10, G54-G59, G92, G28.1/G30.1, G43.1/G49). The planner passes
  /// it through to the consumer (it does not move the machine); the consumer applies it to the shared
  /// [`crate::coords::CoordinateSystems`] and pushes the recomputed WCO back into the planner.
  Coordinate(CoordinateOp),
  /// M3/M4/M5 spindle control. `speed` carries the active modal S value for M3/M4.
  Spindle {
    /// Requested spindle state.
    state: SpindleState,
    /// Active spindle speed (modal S), in RPM.
    speed: f32,
  },
  /// M30 program end: stop motion, stop spindle, and reset modal state to defaults (caller's policy).
  ProgramEnd,
}

/// The supported modal groups. A line may carry at most one word from each group; a second word from
/// an already-set group is a [`GcodeError::ModalGroupViolation`]. Only the groups that can produce a
/// [`PlannerCommand`] need tracking — non-action modal words (G17/G20/G21/G90/G91) update state but
/// each belongs to a distinct group so they never conflict with motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Group {
  Motion,
  Units,
  Distance,
  Plane,
  Spindle,
  Stop,
  /// Feed-rate mode (RS274/NGC modal group 5): G93 / G94. Two on one line is a modal conflict.
  FeedMode,
  /// Work-coordinate-system select (grbl modal group 12): G54-G59.
  Coordinate,
  /// Tool-length-offset mode (grbl modal group 8): G43.1 / G49.
  ToolOffset,
  /// Non-modal group 0 one-shot commands that share a line slot with motion: G10 / G28.1 / G30.1 / G92 / G92.1.
  /// G53 is also group 0 but is a one-shot MODIFIER of a motion word, so it has its own flag, not this slot.
  NonModal,
}

/// Tracks which modal groups have already been set on the current line so a repeat is rejected.
#[derive(Default)]
struct GroupGuard {
  motion: bool,
  units: bool,
  distance: bool,
  plane: bool,
  spindle: bool,
  stop: bool,
  feed_mode: bool,
  coordinate: bool,
  tool_offset: bool,
  non_modal: bool,
}

impl GroupGuard {
  /// Mark `group` as seen on this line, returning [`GcodeError::ModalGroupViolation`] if it was
  /// already set.
  fn claim(&mut self, group: Group) -> Result<(), GcodeError> {
    let slot = match group {
      Group::Motion => &mut self.motion,
      Group::Units => &mut self.units,
      Group::Distance => &mut self.distance,
      Group::Plane => &mut self.plane,
      Group::Spindle => &mut self.spindle,
      Group::Stop => &mut self.stop,
      Group::FeedMode => &mut self.feed_mode,
      Group::Coordinate => &mut self.coordinate,
      Group::ToolOffset => &mut self.tool_offset,
      Group::NonModal => &mut self.non_modal,
    };
    if *slot {
      return Err(GcodeError::ModalGroupViolation);
    }
    *slot = true;
    Ok(())
  }
}

/// Accumulates the meaningful words of a single line before they are folded into a command. Modal
/// words land in `pending_*`; parameter words land in their fields. This fixed struct replaces any
/// need for a growable word buffer. The active motion mode is read from the staged `ModalState`
/// (`next_state.motion`) at emit time, so it is not duplicated here.
#[derive(Default)]
struct LineAccumulator {
  pending_predefined: Option<bool>, // Some(true) = G28, Some(false) = G30.
  pending_dwell: bool,
  /// A pending `G38.x` probe (motion group 1). The axis/feed words are folded in at emit time; a probe with no
  /// axis word is rejected as [`GcodeError::ProbeNoAxis`].
  pending_probe: Option<ProbeKind>,
  pending_spindle: Option<SpindleState>,
  pending_program_end: bool,
  /// A pending non-motion coordinate op (G10/G92/G92.1/G28.1/G30.1/G43.1/G49) that takes the whole line. The
  /// WCS-select (G54-G59) is modal and does NOT use this slot — it updates `next_state.wcs` and emits its own
  /// `SelectWcs` op only when the line carries no other action.
  pending_coordinate: Option<PendingCoordinate>,
  /// G53 one-shot machine-coordinate modifier for THIS line's motion words (group 0; modifies, not replaces).
  machine_coords: bool,
  /// True when a G54-G59 word selected a (possibly new) active WCS on this line. The active WCS is modal, but
  /// a bare select line still emits a [`CoordinateOp::SelectWcs`] so the consumer can push the new WCO into the
  /// planner; this flag tells `emit` to do so when no other action takes the line.
  selected_wcs: bool,
  /// `L` word value (the G10 sub-mode selector: `L2` literal offset, `L20` set-to-position).
  l: Option<f32>,
  /// Whether an `F` word appeared on THIS line. Under G93 inverse-time, a feed move requires a fresh `F` per
  /// line (a prior modal `F` does not satisfy it), so the emit check consults this rather than the modal value.
  saw_feed: bool,
  axes: AxisWords,
  i: Option<f32>,
  j: Option<f32>,
  p: Option<f32>,
}

impl LineAccumulator {
  /// Whether this line carried any axis word (X/Y/Z). Used to reject a directionless `G38.x` probe and to decide
  /// whether a line realizes the active motion mode.
  fn has_axes(&self) -> bool {
    self.axes.x.is_some() || self.axes.y.is_some() || self.axes.z.is_some() || self.axes.a.is_some()
  }
}

/// A coordinate op staged on the current line before its words (axes / P / L) are known, resolved into a
/// concrete [`CoordinateOp`] at emit time. Separated from [`CoordinateOp`] because the P/L/axis words can
/// arrive in any order after the G-word that named the op.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingCoordinate {
  /// `G10` — the L word (2 or 20) and P word are resolved at emit time into a literal or set-to-position op.
  G10,
  /// `G92` — set the dynamic offset to the current position (the axis words are the work target).
  G92Set,
  /// `G92.1` — clear the dynamic offset.
  G92Clear,
  /// `G28.1` / `G30.1` — store the current machine position as predefined `index` (0 = G28, 1 = G30).
  StorePredefined { index: usize },
  /// `G43.1` — apply a dynamic Z tool-length offset from the Z word.
  ApplyTlo,
  /// `G49` — cancel the dynamic tool-length offset.
  CancelTlo,
}

/// The GCode parser. Holds the persistent [`ModalState`] and turns whole lines into at most one
/// [`PlannerCommand`]. The parser is sync and pure; the firmware binary drives it from the USB RX
/// task and forwards emitted commands to the planner channel.
#[derive(Default)]
pub struct Parser {
  state: ModalState,
}

impl Parser {
  /// Create a parser with grbl power-on defaults (G0, G90, G21, F0, S0).
  pub fn new() -> Self {
    Parser { state: ModalState::default() }
  }

  /// The current persistent modal state. Used by `$G` modal-state reporting (DOC-04) and tests.
  pub fn state(&self) -> &ModalState {
    &self.state
  }

  /// Parse one line (CR/LF terminator already removed) and, on success, return the emitted command.
  ///
  /// Returns `Ok(None)` for a line that validly carries no action — a blank line, a comment-only
  /// line, or a line that only changes modal settings (e.g. `G21 G90`). Returns `Ok(Some(cmd))` when
  /// the line commands an action, and `Err` with a grblHAL code on any malformed or invalid input.
  /// Modal state is updated only when the whole line validates, so a rejected line leaves state
  /// untouched.
  pub fn parse_line(&mut self, line: &[u8]) -> Result<Option<PlannerCommand>, GcodeError> {
    let mut lexer = Lexer::new(line);
    let mut guard = GroupGuard::default();
    let mut acc = LineAccumulator::default();
    // Stage modal-state changes locally so a later error on the same line leaves `self.state` intact.
    let mut next_state = self.state;

    while let Some(word) = lexer.next_word() {
      let word = word?;
      self.apply_word(word, &mut guard, &mut acc, &mut next_state)?;
    }

    // A `G38.x` probe MUST carry at least one axis word (the direction to probe); reject one that does not before
    // committing the line's modal state, so a bare `G38.2` errors rather than emitting a directionless probe.
    if acc.pending_probe.is_some() && !acc.has_axes() {
      return Err(GcodeError::ProbeNoAxis);
    }

    // Build the command BEFORE committing `next_state`, passing the pre-line active WCS as the P-less `G10`
    // fallback and surfacing a feed-undefined rejection. A rejected line leaves `self.state` untouched.
    let command = self.emit(&acc, &next_state, self.state.wcs)?;
    self.state = next_state;
    Ok(command)
  }

  /// Parse a `$J=` jog line (the bytes AFTER the `$J=` prefix, CR/LF terminator already removed) into a
  /// [`JogCommand`], using grbl's **seed-from-current-then-discard** modal rule.
  ///
  /// A throwaway [`ModalState`] is SEEDED from `self.state` so the jog inherits the program's current distance
  /// (G90/G91) and units (G20/G21) — a program left in `G91`/`G20` makes `$J=X10` incremental/inch. The jog
  /// line's own `G90`/`G91`/`G20`/`G21`/`G53` words override only WITHIN that throwaway context; every change is
  /// DISCARDED, so the persistent `self.state` is left byte-for-byte untouched (a jog never mutates `gc_state`,
  /// matching grbl — the parser is undisturbed by jogging). A jog MUST carry an `F` feed (there is no modal jog
  /// feed → [`GcodeError::FeedRateUndefined`]) and at least one axis word (→ [`GcodeError::JogNoAxis`]); only
  /// `G90/G91/G20/G21/G53`, the X/Y/Z axis words, and `F` are accepted — any other word is
  /// [`GcodeError::UnsupportedCommand`].
  pub fn parse_jog(&self, line: &[u8]) -> Result<JogCommand, GcodeError> {
    // Seed the throwaway context from the persistent state, then fold the jog line's words into it WITHOUT ever
    // writing back. `seen_feed` tracks the mandatory `F`; `machine_coords` the optional one-shot G53.
    let mut ctx = self.state;
    let mut axes = AxisWords::default();
    let mut seen_feed = false;
    let mut machine_coords = false;
    let mut lexer = Lexer::new(line);
    while let Some(word) = lexer.next_word() {
      let word = word?;
      match word.letter {
        b'X' => axes.x = Some(word.value),
        b'Y' => axes.y = Some(word.value),
        b'Z' => axes.z = Some(word.value),
        b'A' => axes.a = Some(word.value),
        b'F' => {
          ctx.feed = word.value;
          seen_feed = true;
        }
        // Only the distance/units/G53 G-words are legal in a jog; everything else (motion modes, coordinate
        // ops, spindle, …) is rejected so a jog stays a pure axis-target move. `g_code` rejects fractional and
        // out-of-range codes; the integer arms below accept exactly grbl's jog-modifier subset.
        b'G' => match g_code(word.value)? {
          20 => ctx.units = Units::Inch,
          21 => ctx.units = Units::Millimeter,
          90 => ctx.distance = DistanceMode::Absolute,
          91 => ctx.distance = DistanceMode::Incremental,
          53 => machine_coords = true,
          _ => return Err(GcodeError::UnsupportedCommand),
        },
        _ => return Err(GcodeError::UnsupportedCommand),
      }
    }
    if !seen_feed {
      return Err(GcodeError::FeedRateUndefined);
    }
    if axes.x.is_none() && axes.y.is_none() && axes.z.is_none() && axes.a.is_none() {
      return Err(GcodeError::JogNoAxis);
    }
    Ok(JogCommand { axes, distance_mode: ctx.distance, units: ctx.units, feed: ctx.feed, machine_coords })
  }

  /// Fold a single lexed word into the per-line accumulator and staged modal state, enforcing modal
  /// group exclusivity and rejecting unsupported words.
  fn apply_word(
    &self,
    word: Word,
    guard: &mut GroupGuard,
    acc: &mut LineAccumulator,
    next_state: &mut ModalState,
  ) -> Result<(), GcodeError> {
    match word.letter {
      b'G' => self.apply_g_word(word.value, guard, acc, next_state),
      b'M' => self.apply_m_word(word.value, guard, acc, next_state),
      b'X' => {
        acc.axes.x = Some(word.value);
        Ok(())
      }
      b'Y' => {
        acc.axes.y = Some(word.value);
        Ok(())
      }
      b'Z' => {
        acc.axes.z = Some(word.value);
        Ok(())
      }
      b'A' => {
        acc.axes.a = Some(word.value);
        Ok(())
      }
      b'I' => {
        acc.i = Some(word.value);
        Ok(())
      }
      b'J' => {
        acc.j = Some(word.value);
        Ok(())
      }
      b'P' => {
        acc.p = Some(word.value);
        Ok(())
      }
      b'L' => {
        acc.l = Some(word.value);
        Ok(())
      }
      b'F' => {
        next_state.feed = word.value;
        acc.saw_feed = true;
        Ok(())
      }
      b'S' => {
        next_state.spindle_speed = word.value;
        Ok(())
      }
      // Tool select (`T<n>`): accepted as a no-op. This machine has no automatic tool changer and does not
      // implement `M6`, so there is nothing for a tool word to act on — but CAM posts (e.g. Vectric) emit `T1`
      // before starting the spindle, and rejecting it would abort the whole program with `error:20`. grbl itself
      // only stores the pending tool until an `M6` consumes it; here it is simply consumed.
      b'T' => Ok(()),
      // Line number (`N<n>`): a sequence label some posts prefix to every line. It carries no machine action, so
      // it is accepted and ignored — grbl uses it only for error reporting, which this firmware does by other means.
      b'N' => Ok(()),
      _ => Err(GcodeError::UnsupportedCommand),
    }
  }

  /// Apply a `G` word. Fractional codes (G28.1, G30.1, G43.1, G59.1-.3, G92.1) are dispatched first by their
  /// exact value; the remaining whole-number codes go through the integer match. Any code outside the
  /// supported subset is [`GcodeError::UnsupportedCommand`].
  fn apply_g_word(
    &self,
    value: f32,
    guard: &mut GroupGuard,
    acc: &mut LineAccumulator,
    next_state: &mut ModalState,
  ) -> Result<(), GcodeError> {
    // Resolve the fractional Phase B codes before the integer path: G code values like 28.1 / 43.1 / 92.1 are
    // exact-matched against a scaled integer (×10) so f32 round-off cannot mis-classify them.
    if let Some(result) = self.apply_fractional_g_word(value, guard, acc, next_state) {
      return result;
    }
    match g_code(value)? {
      0 => {
        guard.claim(Group::Motion)?;
        next_state.motion = MotionMode::Rapid;
        Ok(())
      }
      1 => {
        guard.claim(Group::Motion)?;
        next_state.motion = MotionMode::Linear;
        Ok(())
      }
      2 => {
        guard.claim(Group::Motion)?;
        next_state.motion = MotionMode::ArcCw;
        Ok(())
      }
      3 => {
        guard.claim(Group::Motion)?;
        next_state.motion = MotionMode::ArcCcw;
        Ok(())
      }
      4 => {
        guard.claim(Group::Motion)?;
        acc.pending_dwell = true;
        Ok(())
      }
      10 => {
        // G10 is a non-modal group-0 command; the L/P/axis words are resolved at emit time.
        guard.claim(Group::NonModal)?;
        acc.pending_coordinate = Some(PendingCoordinate::G10);
        Ok(())
      }
      17 => {
        // G17 XY plane is the only supported plane; claim its group so `G17 G17` still violates.
        guard.claim(Group::Plane)?;
        Ok(())
      }
      20 => {
        guard.claim(Group::Units)?;
        next_state.units = Units::Inch;
        Ok(())
      }
      21 => {
        guard.claim(Group::Units)?;
        next_state.units = Units::Millimeter;
        Ok(())
      }
      // G93 inverse-time / G94 units-per-minute feed mode (modal group 5). Two on one line is a conflict.
      93 => {
        guard.claim(Group::FeedMode)?;
        next_state.feed_mode = FeedMode::InverseTime;
        Ok(())
      }
      94 => {
        guard.claim(Group::FeedMode)?;
        next_state.feed_mode = FeedMode::UnitsPerMin;
        Ok(())
      }
      28 => {
        guard.claim(Group::Motion)?;
        acc.pending_predefined = Some(true);
        Ok(())
      }
      30 => {
        guard.claim(Group::Motion)?;
        acc.pending_predefined = Some(false);
        Ok(())
      }
      // G53 one-shot machine-coordinate modifier (non-modal group 0): the next move's words are machine
      // coordinates. It modifies a motion word rather than taking the line, so it sets a flag and does NOT
      // claim the motion group. grbl requires an explicit G0/G1 on the same line; the planner honors the flag.
      53 => {
        acc.machine_coords = true;
        Ok(())
      }
      // G49 cancel the dynamic tool-length offset (modal group 8 tool-offset).
      49 => {
        guard.claim(Group::ToolOffset)?;
        next_state.tlo_active = false;
        acc.pending_coordinate = Some(PendingCoordinate::CancelTlo);
        Ok(())
      }
      // G54-G59 select the active work coordinate system (modal group 12). Index 0 = G54 … 5 = G59.
      54..=59 => {
        guard.claim(Group::Coordinate)?;
        next_state.wcs = (g_code(value)? - 54) as usize;
        acc.selected_wcs = true;
        Ok(())
      }
      90 => {
        guard.claim(Group::Distance)?;
        next_state.distance = DistanceMode::Absolute;
        Ok(())
      }
      91 => {
        guard.claim(Group::Distance)?;
        next_state.distance = DistanceMode::Incremental;
        Ok(())
      }
      // G92 (whole) — set the dynamic offset to the current position (group 0 non-modal).
      92 => {
        guard.claim(Group::NonModal)?;
        acc.pending_coordinate = Some(PendingCoordinate::G92Set);
        Ok(())
      }
      _ => Err(GcodeError::UnsupportedCommand),
    }
  }

  /// Dispatch the fractional Phase B `G` codes (G28.1, G30.1, G43.1, G49 has no fraction but G43.1 does, G92.1).
  /// Returns `Some(result)` when `value` matched a fractional code (claiming the right group and staging the op)
  /// and `None` when it is not a fractional code this subset knows, so the integer path can handle it. Matching
  /// is done on `round(value × 10)` so f32 representation error (e.g. 28.1 stored as 28.0999994) cannot
  /// mis-classify a code; a value within tolerance of an integer (`×10` ends in 0) is left for the integer path.
  fn apply_fractional_g_word(
    &self,
    value: f32,
    guard: &mut GroupGuard,
    acc: &mut LineAccumulator,
    next_state: &mut ModalState,
  ) -> Option<Result<(), GcodeError>> {
    let scaled = libm::roundf(value * 10.0);
    // Reject only genuine fractional codes here; a whole-number code (×10 divisible by 10) belongs to the
    // integer path. Guard against representation error the same way `g_code` does.
    if libm::fabsf(value * 10.0 - scaled) > 1e-2 || (scaled as i32) % 10 == 0 {
      return None;
    }
    let code = scaled as i32;
    Some(match code {
      // G28.1 / G30.1 store the current machine position as the predefined position (group 0 non-modal).
      281 => guard
        .claim(Group::NonModal)
        .map(|()| acc.pending_coordinate = Some(PendingCoordinate::StorePredefined { index: 0 })),
      301 => guard
        .claim(Group::NonModal)
        .map(|()| acc.pending_coordinate = Some(PendingCoordinate::StorePredefined { index: 1 })),
      // G43.1 dynamic tool-length offset (group 8 tool-offset). The Z value is resolved at emit time, and the
      // modal `tlo_active` flag is set so `$G` reports `G43.1`.
      431 => guard.claim(Group::ToolOffset).map(|()| {
        acc.pending_coordinate = Some(PendingCoordinate::ApplyTlo);
        next_state.tlo_active = true;
      }),
      // G92.1 clear the dynamic offset (group 0 non-modal).
      921 => guard
        .claim(Group::NonModal)
        .map(|()| acc.pending_coordinate = Some(PendingCoordinate::G92Clear)),
      // G38.2/.3/.4/.5 probe moves (motion group 1). They share the motion slot like G0-G4, so a probe on the
      // same line as another motion word is a modal-group violation. The axis/feed words are folded in at emit
      // time; the toward/away + alarm-on-fail semantics ride in the staged [`ProbeKind`].
      382 => guard.claim(Group::Motion).map(|()| acc.pending_probe = Some(ProbeKind::G38_2)),
      383 => guard.claim(Group::Motion).map(|()| acc.pending_probe = Some(ProbeKind::G38_3)),
      384 => guard.claim(Group::Motion).map(|()| acc.pending_probe = Some(ProbeKind::G38_4)),
      385 => guard.claim(Group::Motion).map(|()| acc.pending_probe = Some(ProbeKind::G38_5)),
      _ => Err(GcodeError::UnsupportedCommand),
    })
  }

  /// Apply an `M` word from the supported subset (M3/M4/M5 spindle, M2/M30 program end). M3/M4/M5 update BOTH the
  /// per-line `pending_spindle` (for a spindle-only line's single emit) and the sticky modal `next_state.spindle`
  /// (modal group 7) so the commanded direction survives a line that also carries a move.
  fn apply_m_word(
    &self,
    value: f32,
    guard: &mut GroupGuard,
    acc: &mut LineAccumulator,
    next_state: &mut ModalState,
  ) -> Result<(), GcodeError> {
    match g_code(value)? {
      3 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::Clockwise);
        next_state.spindle = SpindleState::Clockwise;
        Ok(())
      }
      4 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::CounterClockwise);
        next_state.spindle = SpindleState::CounterClockwise;
        Ok(())
      }
      5 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::Stop);
        next_state.spindle = SpindleState::Stop;
        Ok(())
      }
      // M2 (program end) and M30 (program end + rewind) both end the program. This firmware has no pallet/rewind
      // distinction, so both reset the parser to its start-of-program state via the same `ProgramEnd`. M2 had been
      // missing, which aborted any CAM file that ends with `M2` (most do) on `error:20` at the final line.
      2 | 30 => {
        guard.claim(Group::Stop)?;
        acc.pending_program_end = true;
        Ok(())
      }
      _ => Err(GcodeError::UnsupportedCommand),
    }
  }

  /// Build the line's [`PlannerCommand`] from the accumulator and staged state, applying grbl's modal
  /// precedence: an explicit non-motion action (program end, dwell, predefined position, coordinate
  /// offset) takes the line; otherwise axis words realize the active motion mode; otherwise a spindle
  /// word emits a spindle command; a line with none of these yields `None`.
  ///
  /// `pre_line_wcs` is the active WCS committed BEFORE the line began; it is the fallback for a P-less
  /// `G10` so a same-line `G54`-`G59` select does not retarget the offset write (see [`resolve_coordinate`]).
  ///
  /// Returns [`GcodeError::FeedRateUndefined`] (error:22) when the line realizes a move that requires a feed
  /// (a `G38.x` probe, or a G1/G2/G3 feed move) but no feed is defined (`feed <= 0`). G0 rapids do not need a
  /// feed and are unaffected, matching grbl. The check sits at each affected emit point so it respects the
  /// precedence above (e.g. a `G10`/dwell line that happens to have feed 0 is never rejected).
  ///
  /// Per the DOC-04 subset each line emits at most one [`PlannerCommand`], so when a spindle word
  /// shares a line with a move (uncommon for PCB milling) the move is emitted and the S value / state
  /// persist in modal state for the spindle task to act on — the two are not fused into one command.
  fn emit(
    &self,
    acc: &LineAccumulator,
    state: &ModalState,
    pre_line_wcs: usize,
  ) -> Result<Option<PlannerCommand>, GcodeError> {
    if acc.pending_program_end {
      return Ok(Some(PlannerCommand::ProgramEnd));
    }
    if acc.pending_dwell {
      return Ok(Some(PlannerCommand::Dwell { seconds: acc.p.unwrap_or(0.0) }));
    }
    if let Some(is_g28) = acc.pending_predefined {
      return Ok(Some(PlannerCommand::GoToPredefined {
        is_g28,
        intermediate: acc.axes,
        units: state.units,
        distance: state.distance,
      }));
    }
    if let Some(pending) = acc.pending_coordinate {
      return Ok(Some(PlannerCommand::Coordinate(resolve_coordinate(pending, acc, state, pre_line_wcs))));
    }
    // A `G38.x` probe takes the line ahead of an ordinary move: it carries axis words but is NOT a Move (it has
    // its own toward/away + alarm semantics). `parse_line` already guaranteed at least one axis word is present.
    // A probe MUST have a defined feed (the seek speed); with feed 0 it would crawl, so grbl rejects it (error:22).
    if let Some(kind) = acc.pending_probe {
      // A probe is LINEAR-ONLY: a rotary `A` word is rejected (decision, 2026-06-21). Probing while a rotary axis
      // moves is metrologically unsound, so the A axis is never part of a probe target — even a redundant `A`
      // equal to the current position is rejected, keeping the contract unambiguous. Checked first so a probe that
      // names A reads as the rotary-axis error regardless of any feed-mode / feed problem on the same line.
      if acc.axes.a.is_some() {
        return Err(GcodeError::ProbeRotaryAxisWord);
      }
      // A probe is rejected under G93 inverse-time feed mode (DOC-10.2 decision): a probe needs a well-defined
      // units/min CONTACT speed, but inverse-time ties speed to the move's distance, which for a probe is the
      // arbitrary no-contact overshoot — so the seek speed would be meaningless. The operator must be in G94 to
      // probe. Checked before the feed-undefined test so a `G93` probe reads as the probe-specific error, not 22.
      if state.feed_mode == FeedMode::InverseTime {
        return Err(GcodeError::ProbeInverseTimeUnsupported);
      }
      // A probe always consumes a feed (the seek speed); a feed-undefined probe (no modal F yet) is rejected.
      if feed_is_undefined(state, acc.saw_feed, true) {
        return Err(GcodeError::FeedRateUndefined);
      }
      // With G93 rejected above, `feed_mode` is always G94 here, so `feed` is provably a units/min contact speed
      // (no `feed_mode` field is carried — the probe is never inverse-time).
      return Ok(Some(PlannerCommand::Probe {
        kind,
        axes: acc.axes,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
      }));
    }
    if acc.has_axes() {
      // G1/G2/G3 feed moves require a defined feed; G0 rapids run at rapid rate and do not. Reject a feed move
      // with no defined feed (error:22) rather than crawling at feed 0. Under G94 a modal feed (set on an earlier
      // line) satisfies the check; under G93 the inverse-time `F` must appear on THIS line (`feed_is_undefined`).
      if feed_is_undefined(state, acc.saw_feed, state.motion != MotionMode::Rapid) {
        return Err(GcodeError::FeedRateUndefined);
      }
      return Ok(Some(self.motion_command(acc, state)));
    }
    if let Some(spindle) = acc.pending_spindle {
      return Ok(Some(PlannerCommand::Spindle { state: spindle, speed: state.spindle_speed }));
    }
    // A bare WCS select (no move on the line) emits a SelectWcs op so the consumer can push the new WCO into
    // the planner. When a select SHARES a line with a move, the move takes the line; the consumer keeps the
    // active WCS in sync with the parser's modal `wcs` before planning, so the move still uses the right offset.
    if acc.selected_wcs {
      return Ok(Some(PlannerCommand::Coordinate(CoordinateOp::SelectWcs { index: state.wcs })));
    }
    Ok(None)
  }

  /// Construct the move/arc command for a line carrying axis words, using the active motion mode and threading
  /// the G53 one-shot machine-coordinate flag (`acc.machine_coords`) through to the planner.
  fn motion_command(&self, acc: &LineAccumulator, state: &ModalState) -> PlannerCommand {
    match state.motion {
      MotionMode::Rapid => PlannerCommand::Move {
        rapid: true,
        axes: acc.axes,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
        feed_mode: state.feed_mode,
        machine_coords: acc.machine_coords,
      },
      MotionMode::Linear => PlannerCommand::Move {
        rapid: false,
        axes: acc.axes,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
        feed_mode: state.feed_mode,
        machine_coords: acc.machine_coords,
      },
      MotionMode::ArcCw => PlannerCommand::Arc {
        cw: true,
        axes: acc.axes,
        i: acc.i,
        j: acc.j,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
        feed_mode: state.feed_mode,
        machine_coords: acc.machine_coords,
      },
      MotionMode::ArcCcw => PlannerCommand::Arc {
        cw: false,
        axes: acc.axes,
        i: acc.i,
        j: acc.j,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
        feed_mode: state.feed_mode,
        machine_coords: acc.machine_coords,
      },
    }
  }
}

/// Resolve a staged [`PendingCoordinate`] into a concrete [`CoordinateOp`], folding in the line's P/L/axis
/// words. The P word selects the WCS for `G10` (grbl `P1` = G54, so `index = P − 1`); the L word picks the
/// `G10` sub-mode (`L2` literal offset, `L20` set-to-position, with `L2` the grbl default if absent). For an
/// out-of-range / missing P the WCS defaults to `active_wcs_fallback`, which is the system that was active
/// BEFORE the line began (the parser's committed modal `wcs`), matching grbl's "P0 / absent = active system"
/// rule: a same-line `G54`-`G59` select does NOT retarget a P-less G10 (so `G56 G10 L2 X5` writes the pre-line
/// system, not G56). An explicit, in-range P overrides the fallback entirely.
fn resolve_coordinate(
  pending: PendingCoordinate,
  acc: &LineAccumulator,
  state: &ModalState,
  active_wcs_fallback: usize,
) -> CoordinateOp {
  match pending {
    PendingCoordinate::G10 => {
      let index = wcs_index_from_p(acc.p, active_wcs_fallback);
      // L20 sets the offset so the current position reads the work words; L2 (and the default) sets it literally.
      let is_l20 = matches!(acc.l, Some(l) if libm::roundf(l) as i32 == 20);
      if is_l20 {
        CoordinateOp::SetWcsOffsetToPosition { index, axes: acc.axes, units: state.units }
      } else {
        CoordinateOp::SetWcsOffset { index, axes: acc.axes, units: state.units }
      }
    }
    PendingCoordinate::G92Set => CoordinateOp::SetG92ToPosition { axes: acc.axes, units: state.units },
    PendingCoordinate::G92Clear => CoordinateOp::ClearG92,
    PendingCoordinate::StorePredefined { index } => CoordinateOp::StorePredefined { index },
    PendingCoordinate::ApplyTlo => CoordinateOp::ApplyTlo { z: acc.axes.z.unwrap_or(0.0), units: state.units },
    PendingCoordinate::CancelTlo => CoordinateOp::CancelTlo,
  }
}

/// Map a `G10 P<n>` word to a WCS index: grbl numbers `P1` = G54 … `P6` = G59, so `index = P − 1`. A missing
/// or out-of-range P (`P0`, or `P > 6`) falls back to the currently active WCS (`fallback`), matching grbl's
/// "absent / P0 = active coordinate system" convention.
fn wcs_index_from_p(p: Option<f32>, fallback: usize) -> usize {
  match p {
    Some(value) => {
      let n = libm::roundf(value) as i32;
      if (1..=crate::coords::WCS_COUNT as i32).contains(&n) {
        (n - 1) as usize
      } else {
        fallback
      }
    }
    None => fallback,
  }
}

/// Whether a motion line that *would consume* a feed must be rejected with [`GcodeError::FeedRateUndefined`].
///
/// `needs_feed` is false for a G0 rapid (which runs at the rapid rate and never needs an `F`), true for a G1/G2/G3
/// feed move or a `G38.x` probe. A feed-consuming line is undefined when there is no positive modal feed, OR —
/// under **G93 inverse-time** — when no `F` appeared on THIS line (`saw_feed`), since an inverse-time `F`
/// describes the single move's duration and a prior modal `F` does not carry it (grblHAL's per-line rule).
fn feed_is_undefined(state: &ModalState, saw_feed: bool, needs_feed: bool) -> bool {
  if !needs_feed {
    return false;
  }
  if state.feed <= 0.0 {
    return true;
  }
  state.feed_mode == FeedMode::InverseTime && !saw_feed
}

/// Convert a G/M word value to its integer code, rejecting non-integer codes (e.g. `G1.5`). grbl
/// codes in this subset are all whole numbers; a fractional code is unsupported, not a bad number.
fn g_code(value: f32) -> Result<u16, GcodeError> {
  // Round to nearest to absorb f32 representation error (e.g. 90.0 stored as 89.9999994), then verify
  // the value really was an in-range non-negative integer before trusting the rounded code.
  let rounded = libm::roundf(value);
  if !(0.0..=255.0).contains(&rounded) || libm::fabsf(value - rounded) > 1e-3 {
    return Err(GcodeError::UnsupportedCommand);
  }
  Ok(rounded as u16)
}

#[cfg(test)]
mod tests {
  use super::*;

  // ---- parse_f32: numeric grammar vectors -------------------------------------------------------

  #[test]
  fn parse_f32_integer() {
    assert_eq!(parse_f32(b"100"), Ok(100.0));
  }

  #[test]
  fn parse_f32_negative_fraction() {
    assert_eq!(parse_f32(b"-12.5"), Ok(-12.5));
  }

  #[test]
  fn parse_f32_explicit_plus() {
    assert_eq!(parse_f32(b"+3.25"), Ok(3.25));
  }

  #[test]
  fn parse_f32_leading_dot() {
    assert_eq!(parse_f32(b".5"), Ok(0.5));
  }

  #[test]
  fn parse_f32_trailing_dot() {
    assert_eq!(parse_f32(b"12."), Ok(12.0));
  }

  #[test]
  fn parse_f32_rejects_lone_sign() {
    assert_eq!(parse_f32(b"-"), Err(GcodeError::BadNumberFormat));
  }

  #[test]
  fn parse_f32_rejects_lone_dot() {
    assert_eq!(parse_f32(b"."), Err(GcodeError::BadNumberFormat));
  }

  #[test]
  fn parse_f32_rejects_double_dot() {
    assert_eq!(parse_f32(b"1.2.3"), Err(GcodeError::BadNumberFormat));
  }

  #[test]
  fn parse_f32_rejects_trailing_junk() {
    assert_eq!(parse_f32(b"12x"), Err(GcodeError::BadNumberFormat));
  }

  // ---- Lexer: tokenization, whitespace, comments, case ------------------------------------------

  fn lex_all(line: &[u8]) -> heapless::Vec<Word, 16> {
    let mut lexer = Lexer::new(line);
    let mut out = heapless::Vec::new();
    while let Some(w) = lexer.next_word() {
      out.push(w.expect("test lines lex without error")).expect("under capacity");
    }
    out
  }

  #[test]
  fn lex_simple_move() {
    let words = lex_all(b"G1 X10 Y-5 F100");
    assert_eq!(words.len(), 4);
    assert_eq!(words[0], Word { letter: b'G', value: 1.0 });
    assert_eq!(words[1], Word { letter: b'X', value: 10.0 });
    assert_eq!(words[2], Word { letter: b'Y', value: -5.0 });
    assert_eq!(words[3], Word { letter: b'F', value: 100.0 });
  }

  #[test]
  fn lex_is_case_insensitive() {
    let words = lex_all(b"g1 x10 y-5");
    assert_eq!(words[0].letter, b'G');
    assert_eq!(words[1].letter, b'X');
    assert_eq!(words[2].letter, b'Y');
  }

  #[test]
  fn lex_tolerates_missing_and_extra_whitespace() {
    let words = lex_all(b"  G1X10   Y -5\t");
    assert_eq!(words.len(), 3);
    assert_eq!(words[1], Word { letter: b'X', value: 10.0 });
    assert_eq!(words[2], Word { letter: b'Y', value: -5.0 });
  }

  #[test]
  fn lex_strips_block_comment_inline() {
    let words = lex_all(b"G1 (rapid to start) X10");
    assert_eq!(words.len(), 2);
    assert_eq!(words[1], Word { letter: b'X', value: 10.0 });
  }

  #[test]
  fn lex_strips_semicolon_comment_to_end_of_line() {
    let words = lex_all(b"G1 X10 ; move and the rest is ignored Y99");
    assert_eq!(words.len(), 2);
    assert_eq!(words[1], Word { letter: b'X', value: 10.0 });
  }

  #[test]
  fn lex_tolerates_unterminated_block_comment() {
    // grbl swallows an unterminated `(` to end of line rather than erroring.
    let words = lex_all(b"G1 X10 (oops no close paren Y99");
    assert_eq!(words.len(), 2);
    assert_eq!(words[1], Word { letter: b'X', value: 10.0 });
  }

  #[test]
  fn lex_empty_line_yields_no_words() {
    assert_eq!(lex_all(b"").len(), 0);
    assert_eq!(lex_all(b"   \t ").len(), 0);
  }

  #[test]
  fn lex_value_without_letter_is_error() {
    let mut lexer = Lexer::new(b"5 X10");
    assert_eq!(lexer.next_word(), Some(Err(GcodeError::ExpectedCommandLetterValue)));
  }

  #[test]
  fn lex_letter_without_value_is_error() {
    let mut lexer = Lexer::new(b"X");
    assert_eq!(lexer.next_word(), Some(Err(GcodeError::ExpectedCommandLetterValue)));
  }

  // ---- Parser: end-to-end PlannerCommand emission -----------------------------------------------

  #[test]
  fn parse_rapid_move_is_default_motion() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"X10 Y20").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Move {
        rapid: true,
        axes: AxisWords { x: Some(10.0), y: Some(20.0), z: None, a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 0.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
    );
  }

  #[test]
  fn parse_linear_move_carries_modal_feed() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G1 X10 F250").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Move {
        rapid: false,
        axes: AxisWords { x: Some(10.0), y: None, z: None, a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 250.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
    );
  }

  #[test]
  fn parse_motion_mode_is_modal_across_lines() {
    let mut parser = Parser::new();
    parser.parse_line(b"G1 F100").expect("valid");
    // A bare axis line inherits the G1 mode set on the previous line.
    let cmd = parser.parse_line(b"X5").expect("valid");
    assert!(matches!(cmd, Some(PlannerCommand::Move { rapid: false, .. })));
  }

  #[test]
  fn parse_feed_is_sticky_across_lines() {
    let mut parser = Parser::new();
    parser.parse_line(b"G1 X0 F300").expect("valid");
    let cmd = parser.parse_line(b"X10").expect("valid");
    if let Some(PlannerCommand::Move { feed, .. }) = cmd {
      assert_eq!(feed, 300.0);
    } else {
      panic!("expected a move");
    }
  }

  #[test]
  fn parse_units_and_distance_modal_words_emit_nothing() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G21 G90").expect("valid");
    assert_eq!(cmd, None);
    assert_eq!(parser.state().units, Units::Millimeter);
    assert_eq!(parser.state().distance, DistanceMode::Absolute);
  }

  #[test]
  fn parse_inch_incremental_propagate_into_move() {
    let mut parser = Parser::new();
    parser.parse_line(b"G20 G91").expect("valid");
    let cmd = parser.parse_line(b"G1 X1 F10").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Move {
        rapid: false,
        axes: AxisWords { x: Some(1.0), y: None, z: None, a: None },
        units: Units::Inch,
        distance: DistanceMode::Incremental,
        feed: 10.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
    );
  }

  #[test]
  fn parse_arc_cw_with_offsets() {
    let mut parser = Parser::new();
    parser.parse_line(b"G1 F100").expect("valid");
    let cmd = parser.parse_line(b"G2 X10 Y0 I5 J0").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Arc {
        cw: true,
        axes: AxisWords { x: Some(10.0), y: Some(0.0), z: None, a: None },
        i: Some(5.0),
        j: Some(0.0),
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 100.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
    );
  }

  #[test]
  fn parse_dwell_reads_p_seconds() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G4 P2.5").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Dwell { seconds: 2.5 }));
  }

  #[test]
  fn parse_go_to_predefined_g28() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G28 Z5").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::GoToPredefined {
        is_g28: true,
        intermediate: AxisWords { x: None, y: None, z: Some(5.0), a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
      })
    );
  }

  // ---- Phase B: coordinate-system & offset words ------------------------------------------------

  #[test]
  fn parse_g92_set_emits_coordinate_op() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G92 X0 Y0").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetG92ToPosition {
        axes: AxisWords { x: Some(0.0), y: Some(0.0), z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g92_1_clears_g92() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G92.1").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::ClearG92)));
  }

  #[test]
  fn parse_wcs_select_is_modal_and_emits_select_op() {
    let mut parser = Parser::new();
    // G55 selects WCS index 1 and, on a bare select line, emits a SelectWcs op.
    let cmd = parser.parse_line(b"G55").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::SelectWcs { index: 1 })));
    assert_eq!(parser.state().wcs, 1);
    // The selection is modal: a following move inherits it without re-emitting a select.
    let cmd = parser.parse_line(b"G0 X1").expect("valid");
    assert!(matches!(cmd, Some(PlannerCommand::Move { machine_coords: false, .. })));
  }

  #[test]
  fn parse_all_six_wcs_indices() {
    for (line, index) in [
      (b"G54".as_slice(), 0usize),
      (b"G55", 1),
      (b"G56", 2),
      (b"G57", 3),
      (b"G58", 4),
      (b"G59", 5),
    ] {
      let mut parser = Parser::new();
      let cmd = parser.parse_line(line).expect("valid");
      assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::SelectWcs { index })));
    }
  }

  #[test]
  fn parse_g10_l2_sets_offset_for_p_indexed_wcs() {
    let mut parser = Parser::new();
    // G10 L2 P1 = G54 (index 0). The X/Y words are the literal new offset.
    let cmd = parser.parse_line(b"G10 L2 P1 X10 Y20").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffset {
        index: 0,
        axes: AxisWords { x: Some(10.0), y: Some(20.0), z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g10_l20_sets_offset_to_position() {
    let mut parser = Parser::new();
    // G10 L20 P2 = G55 (index 1). The X word is the work target the current position should read as.
    let cmd = parser.parse_line(b"G10 L20 P2 X0").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffsetToPosition {
        index: 1,
        axes: AxisWords { x: Some(0.0), y: None, z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g10_without_p_targets_active_wcs() {
    let mut parser = Parser::new();
    parser.parse_line(b"G56").expect("valid"); // active WCS = index 2.
    // G10 L2 with no P writes the active WCS (grbl P0/absent = active system).
    let cmd = parser.parse_line(b"G10 L2 X5").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffset {
        index: 2,
        axes: AxisWords { x: Some(5.0), y: None, z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g10_without_p_targets_pre_line_active_wcs_not_same_line_select() {
    let mut parser = Parser::new();
    // Start with G54 active. A line that BOTH selects G56 and issues a P-less `G10 L2` must write the system that
    // was active BEFORE the line began (G54, index 0), per grbl — NOT the G56 staged by the same-line select.
    let cmd = parser.parse_line(b"G56 G10 L2 X5").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffset {
        index: 0,
        axes: AxisWords { x: Some(5.0), y: None, z: None, a: None },
        units: Units::Millimeter,
      }))
    );
    // The same-line select still commits modally for subsequent lines (G56 is now active).
    assert_eq!(parser.state().wcs, 2);
  }

  #[test]
  fn parse_g10_l20_without_p_targets_pre_line_active_wcs_not_same_line_select() {
    let mut parser = Parser::new();
    // The L20 form has the same P-less fallback rule: `G55 G10 L20 X0` writes the pre-line active G54, not G55.
    let cmd = parser.parse_line(b"G55 G10 L20 X0").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffsetToPosition {
        index: 0,
        axes: AxisWords { x: Some(0.0), y: None, z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g10_with_explicit_p_is_unaffected_by_same_line_select() {
    let mut parser = Parser::new();
    // An explicit P word overrides the fallback entirely: `G56 G10 L2 P3 X5` writes P3 = G56 (index 2) regardless
    // of which system was active, proving the fix only changes the P-ABSENT fallback.
    let cmd = parser.parse_line(b"G56 G10 L2 P3 X5").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Coordinate(CoordinateOp::SetWcsOffset {
        index: 2,
        axes: AxisWords { x: Some(5.0), y: None, z: None, a: None },
        units: Units::Millimeter,
      }))
    );
  }

  #[test]
  fn parse_g28_1_stores_predefined_g28() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G28.1").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::StorePredefined { index: 0 })));
  }

  #[test]
  fn parse_g30_1_stores_predefined_g30() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G30.1").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::StorePredefined { index: 1 })));
  }

  #[test]
  fn parse_g43_1_applies_z_tlo_and_sets_modal() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G43.1 Z-14.442").expect("valid");
    // The Z word is parsed by the no_std lexer; assert on the resolved op shape and a tolerant Z value (the
    // exact f32 of -14.442 is not representable, so compare within an epsilon rather than for bit equality).
    match cmd {
      Some(PlannerCommand::Coordinate(CoordinateOp::ApplyTlo { z, units: Units::Millimeter })) => {
        assert!((z + 14.442).abs() < 1e-3, "TLO Z was {z}");
      }
      other => panic!("expected ApplyTlo, got {other:?}"),
    }
    assert!(parser.state().tlo_active);
  }

  #[test]
  fn parse_g49_cancels_tlo_and_clears_modal() {
    let mut parser = Parser::new();
    parser.parse_line(b"G43.1 Z1").expect("valid");
    assert!(parser.state().tlo_active);
    let cmd = parser.parse_line(b"G49").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Coordinate(CoordinateOp::CancelTlo)));
    assert!(!parser.state().tlo_active);
  }

  #[test]
  fn parse_g53_one_shot_marks_move_machine_coords() {
    let mut parser = Parser::new();
    // G53 modifies the move on its OWN line: the words are machine coordinates.
    let cmd = parser.parse_line(b"G53 G0 X10 Y20").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Move {
        rapid: true,
        axes: AxisWords { x: Some(10.0), y: Some(20.0), z: None, a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 0.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: true,
      })
    );
    // G53 is one-shot: the next move is back to work coordinates.
    let cmd = parser.parse_line(b"G0 X0").expect("valid");
    assert!(matches!(cmd, Some(PlannerCommand::Move { machine_coords: false, .. })));
  }

  // ---- Phase C: G38.x probe parsing -------------------------------------------------------------

  #[test]
  fn parse_g38_2_probe_toward_with_alarm() {
    let mut parser = Parser::new();
    // The touch-plate default: G38.2 Z-5 F50 — probe toward, alarm on no contact. The work-coordinate Z target
    // and the modal feed ride into the probe command; units/distance carry the active modes.
    let cmd = parser.parse_line(b"G38.2 Z-5 F50").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Probe {
        kind: ProbeKind::G38_2,
        axes: AxisWords { x: None, y: None, z: Some(-5.0), a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 50.0,
      })
    );
    assert_eq!((ProbeKind::G38_2.toward, ProbeKind::G38_2.alarm_on_fail), (true, true));
  }

  #[test]
  fn parse_all_four_g38_modes_map_to_their_kind() {
    for (line, expected) in [
      // Each probe carries a feed: a probe now REQUIRES a defined feed (error:22 otherwise), so the F word keeps
      // these focused on the kind mapping rather than tripping the feed guard.
      (b"G38.2 Z-1 F10".as_slice(), ProbeKind::G38_2),
      (b"G38.3 Z-1 F10", ProbeKind::G38_3),
      (b"G38.4 Z1 F10", ProbeKind::G38_4),
      (b"G38.5 Z1 F10", ProbeKind::G38_5),
    ] {
      let mut parser = Parser::new();
      let cmd = parser.parse_line(line).expect("valid");
      match cmd {
        Some(PlannerCommand::Probe { kind, .. }) => assert_eq!(kind, expected, "{line:?} -> {expected:?}"),
        other => panic!("expected a probe for {line:?}, got {other:?}"),
      }
    }
    // The toward/away + alarm matrix the four modes encode (toward, alarm_on_fail).
    assert_eq!((ProbeKind::G38_2.toward, ProbeKind::G38_2.alarm_on_fail), (true, true));
    assert_eq!((ProbeKind::G38_3.toward, ProbeKind::G38_3.alarm_on_fail), (true, false));
    assert_eq!((ProbeKind::G38_4.toward, ProbeKind::G38_4.alarm_on_fail), (false, true));
    assert_eq!((ProbeKind::G38_5.toward, ProbeKind::G38_5.alarm_on_fail), (false, false));
  }

  #[test]
  fn parse_probe_carries_inch_and_incremental_modes() {
    let mut parser = Parser::new();
    parser.parse_line(b"G20 G91").expect("valid");
    let cmd = parser.parse_line(b"G38.2 Z-0.2 F2").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Probe {
        kind: ProbeKind::G38_2,
        axes: AxisWords { x: None, y: None, z: Some(-0.2), a: None },
        units: Units::Inch,
        distance: DistanceMode::Incremental,
        feed: 2.0,
      })
    );
  }

  #[test]
  fn parse_probe_with_no_axis_word_is_rejected() {
    let mut parser = Parser::new();
    // A bare G38.2 has no direction to probe; grbl rejects it with error:26 (no axis words in block).
    assert_eq!(parser.parse_line(b"G38.2 F50"), Err(GcodeError::ProbeNoAxis));
    assert_eq!(GcodeError::ProbeNoAxis.code(), 26);
  }

  #[test]
  fn parse_probe_conflicts_with_another_motion_word() {
    let mut parser = Parser::new();
    // G38 is motion group 1, so it cannot share a line with G0/G1 (a modal-group violation).
    assert_eq!(parser.parse_line(b"G1 G38.2 Z-5"), Err(GcodeError::ModalGroupViolation));
  }

  // ---- feed-rate required for probes and G1/G2/G3 moves (grbl error:22) -------------------------

  #[test]
  fn parse_probe_with_no_feed_is_error_22() {
    let mut parser = Parser::new();
    // A probe with an axis word but no modal feed is undefined (it would crawl at feed 0); grbl rejects it as
    // error:22, not accept it. The axis-present check passes, so this proves the feed guard, not [`ProbeNoAxis`].
    assert_eq!(parser.parse_line(b"G38.2 Z-5"), Err(GcodeError::FeedRateUndefined));
  }

  #[test]
  fn parse_probe_with_feed_on_same_line_is_ok() {
    let mut parser = Parser::new();
    // The same probe WITH an F word on the line is accepted (feed is defined).
    assert!(matches!(parser.parse_line(b"G38.2 Z-5 F50"), Ok(Some(PlannerCommand::Probe { .. }))));
  }

  #[test]
  fn parse_probe_under_g93_inverse_time_is_rejected() {
    let mut parser = Parser::new();
    // Enter inverse-time feed mode (a bare G93 line carries no motion — Ok(None)).
    assert_eq!(parser.parse_line(b"G93"), Ok(None));
    // A probe while G93 is active is rejected (DOC-10.2 decision): inverse-time has no well-defined contact speed.
    // The fresh F on the line does NOT rescue it — the rejection is about the feed MODE, not a missing feed.
    assert_eq!(parser.parse_line(b"G38.2 Z-10 F50"), Err(GcodeError::ProbeInverseTimeUnsupported));
    // It maps to the feed-family wire code 22.
    assert_eq!(GcodeError::ProbeInverseTimeUnsupported.code(), 22);
    // The rejected line left the modal state intact: still G93, so re-issuing the probe still rejects (no silent
    // fallback to G94), proving the parser did not commit a partial state on the error.
    assert_eq!(parser.parse_line(b"G38.2 Z-10 F50"), Err(GcodeError::ProbeInverseTimeUnsupported));
  }

  #[test]
  fn parse_probe_with_rotary_a_word_is_rejected() {
    let mut parser = Parser::new();
    // A probe naming the rotary A axis is rejected (linear-only): probing through a rotary axis is unsound.
    assert_eq!(parser.parse_line(b"G38.2 A90 F50"), Err(GcodeError::ProbeRotaryAxisWord));
    assert_eq!(GcodeError::ProbeRotaryAxisWord.code(), 33);
    // Even a redundant A alongside a linear word is rejected — the contract is "no A in a probe, period".
    assert_eq!(parser.parse_line(b"G38.2 Z-10 A0 F50"), Err(GcodeError::ProbeRotaryAxisWord));
    // The A-word rejection takes precedence over a same-line feed-mode problem (it is checked first).
    parser.parse_line(b"G93").expect("valid");
    assert_eq!(parser.parse_line(b"G38.2 A90 F50"), Err(GcodeError::ProbeRotaryAxisWord));
    // A purely linear probe (no A) is still accepted.
    parser.parse_line(b"G94").expect("valid");
    assert!(matches!(parser.parse_line(b"G38.2 Z-10 F50"), Ok(Some(PlannerCommand::Probe { .. }))));
  }

  #[test]
  fn parse_probe_under_g94_after_g93_is_ok() {
    let mut parser = Parser::new();
    // G93 then back to G94: the probe is accepted again, and its feed is a units/min contact speed.
    assert_eq!(parser.parse_line(b"G93"), Ok(None));
    assert_eq!(parser.parse_line(b"G94"), Ok(None));
    assert!(matches!(
      parser.parse_line(b"G38.2 Z-10 F50"),
      Ok(Some(PlannerCommand::Probe { feed, .. })) if feed == 50.0
    ));
  }

  #[test]
  fn parse_g1_feed_move_with_no_feed_is_error_22() {
    let mut parser = Parser::new();
    // A G1 feed move with no prior modal F has feed 0 — undefined; grbl rejects it as error:22.
    assert_eq!(parser.parse_line(b"G1 X1"), Err(GcodeError::FeedRateUndefined));
  }

  #[test]
  fn parse_g1_feed_move_with_prior_modal_feed_is_ok() {
    let mut parser = Parser::new();
    // A feed set on an EARLIER line persists (modal F), so a later bare G1 move is accepted.
    parser.parse_line(b"G1 X0 F100").expect("valid");
    assert!(matches!(parser.parse_line(b"G1 X1"), Ok(Some(PlannerCommand::Move { rapid: false, .. }))));
  }

  #[test]
  fn parse_g1_feed_move_with_feed_on_same_line_is_ok() {
    let mut parser = Parser::new();
    // A feed word on the SAME line as the move satisfies the requirement (no prior modal feed needed).
    assert!(matches!(parser.parse_line(b"G1 X1 F100"), Ok(Some(PlannerCommand::Move { rapid: false, .. }))));
  }

  #[test]
  fn parse_g0_rapid_with_no_feed_is_ok() {
    let mut parser = Parser::new();
    // G0 rapids do NOT need a feed (they run at rapid rate), so a rapid with no F is accepted — matching grbl.
    assert!(matches!(parser.parse_line(b"G0 X1"), Ok(Some(PlannerCommand::Move { rapid: true, .. }))));
  }

  #[test]
  fn parse_default_rapid_with_no_feed_is_ok() {
    let mut parser = Parser::new();
    // The power-on default motion mode is G0 rapid, so a bare axis line with no feed is also a rapid and accepted.
    assert!(matches!(parser.parse_line(b"X1"), Ok(Some(PlannerCommand::Move { rapid: true, .. }))));
  }

  #[test]
  fn parse_arc_with_no_feed_is_error_22() {
    let mut parser = Parser::new();
    // G2/G3 arcs are feed moves like G1, so an arc with no defined feed is rejected as error:22.
    assert_eq!(parser.parse_line(b"G2 X10 Y0 I5 J0"), Err(GcodeError::FeedRateUndefined));
  }

  #[test]
  fn parse_probe_does_not_become_the_modal_motion_mode() {
    let mut parser = Parser::new();
    parser.parse_line(b"G1 F100").expect("valid");
    // A probe runs but must NOT change the modal motion mode away from G1 (grbl leaves group 1 as the probe is a
    // one-shot synchronized move, and the next bare axis line should still feed-move under G1).
    parser.parse_line(b"G38.2 Z-5 F50").expect("valid");
    let cmd = parser.parse_line(b"X5").expect("valid");
    assert!(matches!(cmd, Some(PlannerCommand::Move { rapid: false, .. })), "motion mode stays G1 after a probe");
  }

  #[test]
  fn parse_spindle_on_carries_modal_speed() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"M3 S1000").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::Spindle { state: SpindleState::Clockwise, speed: 1000.0 })
    );
  }

  #[test]
  fn parse_spindle_stop() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"M5").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::Spindle { state: SpindleState::Stop, speed: 0.0 }));
  }

  #[test]
  fn spindle_direction_is_modal_and_survives_a_combined_move_line() {
    // M3/M4/M5 commit the direction to modal state (group 7), not just the per-line accumulator, so a spindle
    // word that SHARES a line with a move still records the commanded direction — the move wins the single emit
    // (the firmware drives the spindle from this modal value, see DOC-07). Regression guard for the combined-line
    // case where the standalone spindle emit is suppressed by `has_axes()`.
    let mut parser = Parser::new();
    assert_eq!(parser.state().spindle, SpindleState::Stop, "spindle starts stopped");
    // A combined `M3 S1000 G1 X10`: the emit is the Move, but the modal spindle still latches Clockwise + S1000.
    let cmd = parser.parse_line(b"M3 S1000 G1 X10 F100").expect("valid");
    assert!(matches!(cmd, Some(PlannerCommand::Move { .. })), "the move wins the single per-line emit");
    assert_eq!(parser.state().spindle, SpindleState::Clockwise);
    assert_eq!(parser.state().spindle_speed, 1000.0);
    // A bare following `M4` reverses the modal direction; speed is sticky.
    parser.parse_line(b"M4").expect("valid");
    assert_eq!(parser.state().spindle, SpindleState::CounterClockwise);
    assert_eq!(parser.state().spindle_speed, 1000.0);
    // M5 returns the modal direction to Stop.
    parser.parse_line(b"M5").expect("valid");
    assert_eq!(parser.state().spindle, SpindleState::Stop);
  }

  #[test]
  fn parse_program_end_m30() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"M30").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::ProgramEnd));
  }

  #[test]
  fn parse_program_end_m2() {
    // M2 ends the program exactly like M30; most CAM posts (e.g. Vectric) terminate with M2, and it had been
    // rejected as `error:20`.
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"M2").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::ProgramEnd));
  }

  #[test]
  fn parse_tool_select_is_accepted_as_noop() {
    // A standalone tool select (Vectric posts emit `T1` before the spindle starts) must not abort the program;
    // it carries no motion, so the line yields no command.
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"T1").expect("tool select is accepted");
    assert_eq!(cmd, None);
  }

  #[test]
  fn parse_line_number_prefix_is_ignored() {
    // A leading sequence number must be ignored, not rejected, and must not change the motion on the line.
    let mut with_n = Parser::new();
    let mut without = Parser::new();
    let a = with_n.parse_line(b"N10 G0 X5").expect("line number accepted");
    let b = without.parse_line(b"G0 X5").expect("valid");
    assert_eq!(a, b, "a leading line number must not change the parsed command");
  }

  #[test]
  fn parse_comment_only_line_emits_nothing() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"(just a comment)").expect("valid"), None);
    assert_eq!(parser.parse_line(b"; trailing only").expect("valid"), None);
  }

  // ---- Parser: error paths and modal-group validation -------------------------------------------

  #[test]
  fn parse_rejects_unsupported_g_word() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G99"), Err(GcodeError::UnsupportedCommand));
  }

  #[test]
  fn parse_rejects_unsupported_letter() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"Q5"), Err(GcodeError::UnsupportedCommand));
  }

  #[test]
  fn parse_rejects_two_motion_words() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G0 G1 X5"), Err(GcodeError::ModalGroupViolation));
  }

  #[test]
  fn parse_rejects_conflicting_units() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G20 G21"), Err(GcodeError::ModalGroupViolation));
  }

  #[test]
  fn parse_rejects_bad_number() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"X1.2.3"), Err(GcodeError::BadNumberFormat));
  }

  #[test]
  fn parse_rejects_value_without_letter() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G1 5"), Err(GcodeError::ExpectedCommandLetterValue));
  }

  #[test]
  fn parse_error_leaves_modal_state_unchanged() {
    let mut parser = Parser::new();
    parser.parse_line(b"G1 F100").expect("valid");
    // This line sets G91 then hits an unsupported word; state must not retain the G91.
    let _ = parser.parse_line(b"G91 G99");
    assert_eq!(parser.state().distance, DistanceMode::Absolute);
    assert_eq!(parser.state().motion, MotionMode::Linear);
  }

  #[test]
  fn g_code_absorbs_f32_representation_error() {
    // 90.0 may not be exactly representable through the parse path; rounding must still yield G90.
    assert_eq!(g_code(89.9999994), Ok(90));
    assert_eq!(g_code(90.0), Ok(90));
  }

  #[test]
  fn g_code_rejects_fractional_code() {
    assert_eq!(g_code(1.5), Err(GcodeError::UnsupportedCommand));
  }

  #[test]
  fn error_codes_match_grblhal() {
    assert_eq!(GcodeError::ExpectedCommandLetterValue.code(), 1);
    assert_eq!(GcodeError::BadNumberFormat.code(), 2);
    // grbl's dedicated "Modal group violation" code — NOT the `error:9` state-lock code.
    assert_eq!(GcodeError::ModalGroupViolation.code(), 21);
    assert_eq!(GcodeError::UnsupportedCommand.code(), 20);
    assert_eq!(GcodeError::FeedRateUndefined.code(), 22);
    // A probe/jog missing its axis word is grbl's "No axis words in block" (26), not the integer-value code 23.
    assert_eq!(GcodeError::ProbeNoAxis.code(), 26);
    assert_eq!(GcodeError::JogNoAxis.code(), 26);
  }

  // ---- Phase D: `$J=` jog parsing (seed-from-current-then-discard) ------------------------------

  #[test]
  fn parse_jog_absolute_default_with_feed() {
    let parser = Parser::new();
    // Default state is G90 absolute, G21 mm; a plain `$J=X10 Y5 F600` resolves against those.
    let jog = parser.parse_jog(b"X10 Y5 F600").expect("valid jog");
    assert_eq!(
      jog,
      JogCommand {
        axes: AxisWords { x: Some(10.0), y: Some(5.0), z: None, a: None },
        distance_mode: DistanceMode::Absolute,
        units: Units::Millimeter,
        feed: 600.0,
        machine_coords: false,
      }
    );
  }

  #[test]
  fn parse_jog_line_g91_g20_override_within_throwaway_context() {
    let parser = Parser::new();
    // The jog line's own G91/G20 override the (default G90/G21) seeded context for THIS jog only.
    let jog = parser.parse_jog(b"G91 G20 X1 F30").expect("valid jog");
    assert_eq!(jog.distance_mode, DistanceMode::Incremental);
    assert_eq!(jog.units, Units::Inch);
    assert_eq!(jog.feed, 30.0);
    assert_eq!(jog.axes, AxisWords { x: Some(1.0), y: None, z: None, a: None });
  }

  #[test]
  fn parse_jog_g53_marks_machine_coordinates() {
    let parser = Parser::new();
    let jog = parser.parse_jog(b"G53 Z-1 F100").expect("valid jog");
    assert!(jog.machine_coords);
    assert_eq!(jog.axes, AxisWords { x: None, y: None, z: Some(-1.0), a: None });
  }

  #[test]
  fn parse_jog_seeds_distance_from_current_program_state() {
    let mut parser = Parser::new();
    // Leave the program in G91 incremental; a jog with no distance word inherits it (grbl seed-from-current).
    parser.parse_line(b"G91").expect("valid");
    let jog = parser.parse_jog(b"X10 F600").expect("valid jog");
    assert_eq!(jog.distance_mode, DistanceMode::Incremental);
  }

  #[test]
  fn parse_jog_seeds_units_from_current_program_state() {
    let mut parser = Parser::new();
    // Leave the program in G20 inch; a jog with no units word inherits it.
    parser.parse_line(b"G20").expect("valid");
    let jog = parser.parse_jog(b"X1 F30").expect("valid jog");
    assert_eq!(jog.units, Units::Inch);
  }

  #[test]
  fn parse_jog_does_not_mutate_persistent_modal_state() {
    let mut parser = Parser::new();
    // Start the program in a known modal state, then jog with line words that WOULD change it if applied.
    parser.parse_line(b"G90 G21 G1 F100").expect("valid");
    let before = *parser.state();
    let _ = parser.parse_jog(b"G91 G20 X10 F600").expect("valid jog");
    // grbl: jogging is independent of modal state by design — the persistent state is byte-for-byte unchanged.
    assert_eq!(*parser.state(), before);
    assert_eq!(parser.state().distance, DistanceMode::Absolute);
    assert_eq!(parser.state().units, Units::Millimeter);
    assert_eq!(parser.state().feed, 100.0);
  }

  #[test]
  fn parse_jog_missing_feed_is_error_22() {
    let parser = Parser::new();
    // grbl rejects a jog with no F (feed undefined) as error:22 — there is no modal jog feed to fall back on.
    assert_eq!(parser.parse_jog(b"X10"), Err(GcodeError::FeedRateUndefined));
  }

  #[test]
  fn parse_jog_missing_axis_is_error_23() {
    let parser = Parser::new();
    // A jog with a feed but no axis word has no direction to move — error:26 (no axis words in block).
    assert_eq!(parser.parse_jog(b"F600"), Err(GcodeError::JogNoAxis));
  }

  #[test]
  fn parse_jog_rejects_unsupported_word() {
    let parser = Parser::new();
    // A jog may carry only G90/G91/G20/G21/G53, X/Y/Z, and F. A motion mode (G1), spindle (M3), or any other
    // G-word is rejected so a jog stays a pure axis-target move.
    assert_eq!(parser.parse_jog(b"G1 X10 F600"), Err(GcodeError::UnsupportedCommand));
    assert_eq!(parser.parse_jog(b"M3 X10 F600"), Err(GcodeError::UnsupportedCommand));
    assert_eq!(parser.parse_jog(b"X10 S1000 F600"), Err(GcodeError::UnsupportedCommand));
  }

  #[test]
  fn parse_jog_is_case_insensitive_and_tolerates_whitespace() {
    let parser = Parser::new();
    let jog = parser.parse_jog(b"  g91 x10  f600 ").expect("valid jog");
    assert_eq!(jog.distance_mode, DistanceMode::Incremental);
    assert_eq!(jog.axes, AxisWords { x: Some(10.0), y: None, z: None, a: None });
    assert_eq!(jog.feed, 600.0);
  }

  // ---- Feed mode G93/G94 (DOC-10.2, modal group 5) ----------------------------------------------

  #[test]
  fn feed_mode_defaults_to_g94_units_per_min() {
    // grbl power-on default is G94. A fresh parser is in units-per-minute until a G93 appears (DOC-10.2 test 9).
    let parser = Parser::new();
    assert_eq!(parser.state().feed_mode, FeedMode::UnitsPerMin);
  }

  #[test]
  fn g93_sets_inverse_time_and_g94_restores_units_per_min() {
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G93"), Ok(None));
    assert_eq!(parser.state().feed_mode, FeedMode::InverseTime);
    assert_eq!(parser.parse_line(b"G94"), Ok(None));
    assert_eq!(parser.state().feed_mode, FeedMode::UnitsPerMin);
  }

  #[test]
  fn feed_mode_is_sticky_across_lines() {
    // The feed mode is modal: it survives lines that do not mention it until the opposite word appears.
    let mut parser = Parser::new();
    parser.parse_line(b"G93").expect("valid");
    parser.parse_line(b"G21 G90").expect("valid");
    assert_eq!(parser.state().feed_mode, FeedMode::InverseTime);
  }

  #[test]
  fn g93_g94_modal_conflict_on_one_line() {
    // Two modal-group-5 words on one line is a conflict (DOC-10.2 test 8); the rejected line leaves state intact.
    let mut parser = Parser::new();
    assert_eq!(parser.parse_line(b"G93 G94"), Err(GcodeError::ModalGroupViolation));
    assert_eq!(parser.state().feed_mode, FeedMode::UnitsPerMin);
  }

  #[test]
  fn g93_feed_move_requires_f_on_its_own_line() {
    // Under G93 the inverse-time F is per-move: a feed move with an F passes, but a later feed move with no F on
    // its own line is rejected (error:22) even though a prior modal F exists (DOC-10.2 test 6).
    let mut parser = Parser::new();
    parser.parse_line(b"G93").expect("valid");
    assert!(matches!(parser.parse_line(b"G1 X10 F2"), Ok(Some(PlannerCommand::Move { rapid: false, .. }))));
    assert_eq!(parser.parse_line(b"X20"), Err(GcodeError::FeedRateUndefined));
  }

  #[test]
  fn g93_rapid_does_not_require_f() {
    // G0 rapids never consume a feed, so a G93-active rapid needs no F in either mode (grbl behavior).
    let mut parser = Parser::new();
    parser.parse_line(b"G93").expect("valid");
    assert!(matches!(parser.parse_line(b"G0 X10"), Ok(Some(PlannerCommand::Move { rapid: true, .. }))));
  }

  #[test]
  fn g93_rejects_a_probe_regardless_of_feed() {
    // DOC-10.2 decision: a G38.x probe is rejected outright under G93 inverse-time — a probe needs a well-defined
    // units/min contact speed, which inverse-time cannot express. The rejection is the SAME with or without an F
    // (it is about the feed MODE, not a missing feed), so neither line below reaches the feed-undefined check.
    let mut parser = Parser::new();
    parser.parse_line(b"G93").expect("valid");
    assert_eq!(parser.parse_line(b"G38.2 Z-5"), Err(GcodeError::ProbeInverseTimeUnsupported));
    assert_eq!(parser.parse_line(b"G38.2 Z-5 F50"), Err(GcodeError::ProbeInverseTimeUnsupported));
    // Switching back to G94 makes the probe valid again.
    parser.parse_line(b"G94").expect("valid");
    assert!(matches!(parser.parse_line(b"G38.2 Z-5 F50"), Ok(Some(PlannerCommand::Probe { .. }))));
  }

  #[test]
  fn g94_feed_move_still_uses_modal_feed_fallback() {
    // Regression: G94 (the default) keeps the modal-F fallback — a bare move after an F-bearing line is fine.
    let mut parser = Parser::new();
    parser.parse_line(b"G1 X0 F100").expect("valid");
    assert!(matches!(parser.parse_line(b"X10"), Ok(Some(PlannerCommand::Move { feed, .. })) if feed == 100.0));
  }
}
