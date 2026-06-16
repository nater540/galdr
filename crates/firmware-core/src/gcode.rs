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
  /// `error:9` — more than one word from the same modal group appeared on a single line (e.g. two
  /// motion words `G0 G1`). Maps to grbl's "G-code locked out / modal group violation" family.
  ModalGroupViolation,
  /// `error:20` — an unsupported or unrecognized command word for the implemented GCode subset.
  UnsupportedCommand,
}

impl GcodeError {
  /// The numeric grblHAL status code for this error, suitable for an `error:N` response line.
  pub fn code(self) -> u8 {
    match self {
      GcodeError::ExpectedCommandLetterValue => 1,
      GcodeError::BadNumberFormat => 2,
      GcodeError::ModalGroupViolation => 9,
      GcodeError::UnsupportedCommand => 20,
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

/// Spindle state requested by a line (grbl modal group 7): M3/M4 start the spindle in a direction,
/// M5 stops it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpindleState {
  /// M3 — spindle on, clockwise.
  Clockwise,
  /// M4 — spindle on, counter-clockwise.
  CounterClockwise,
  /// M5 — spindle stop.
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
  /// Last commanded feed rate (F word), in the active units per minute; sticky across lines.
  pub feed: f32,
  /// Last commanded spindle speed (S word), in RPM; sticky across lines.
  pub spindle_speed: f32,
}

impl Default for ModalState {
  fn default() -> Self {
    ModalState {
      motion: MotionMode::Rapid,
      distance: DistanceMode::Absolute,
      units: Units::Millimeter,
      feed: 0.0,
      spindle_speed: 0.0,
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
    /// Active feed rate (modal F), in `units` per minute.
    feed: f32,
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
    /// Active feed rate (modal F), in `units` per minute.
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
  /// G92 set coordinate offset. The axis words define the offset applied so the current position
  /// reads as the given values; resolution against current position is the planner's job.
  SetCoordinateOffset {
    /// Axis words defining the new offset.
    axes: AxisWords,
    /// Active units for the offset words.
    units: Units,
  },
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
/// need for a growable word buffer.
#[derive(Default)]
struct LineAccumulator {
  pending_motion: Option<MotionMode>,
  pending_predefined: Option<bool>, // Some(true) = G28, Some(false) = G30.
  pending_dwell: bool,
  pending_set_offset: bool,
  pending_spindle: Option<SpindleState>,
  pending_program_end: bool,
  axes: AxisWords,
  i: Option<f32>,
  j: Option<f32>,
  p: Option<f32>,
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

    let command = self.emit(&acc, &next_state);
    self.state = next_state;
    Ok(command)
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
      b'M' => self.apply_m_word(word.value, guard, acc),
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
      b'F' => {
        next_state.feed = word.value;
        Ok(())
      }
      b'S' => {
        next_state.spindle_speed = word.value;
        Ok(())
      }
      _ => Err(GcodeError::UnsupportedCommand),
    }
  }

  /// Apply a `G` word. The value is matched on its integer code; a non-integer or out-of-subset code
  /// is [`GcodeError::UnsupportedCommand`].
  fn apply_g_word(
    &self,
    value: f32,
    guard: &mut GroupGuard,
    acc: &mut LineAccumulator,
    next_state: &mut ModalState,
  ) -> Result<(), GcodeError> {
    match g_code(value)? {
      0 => {
        guard.claim(Group::Motion)?;
        acc.pending_motion = Some(MotionMode::Rapid);
        next_state.motion = MotionMode::Rapid;
        Ok(())
      }
      1 => {
        guard.claim(Group::Motion)?;
        acc.pending_motion = Some(MotionMode::Linear);
        next_state.motion = MotionMode::Linear;
        Ok(())
      }
      2 => {
        guard.claim(Group::Motion)?;
        acc.pending_motion = Some(MotionMode::ArcCw);
        next_state.motion = MotionMode::ArcCw;
        Ok(())
      }
      3 => {
        guard.claim(Group::Motion)?;
        acc.pending_motion = Some(MotionMode::ArcCcw);
        next_state.motion = MotionMode::ArcCcw;
        Ok(())
      }
      4 => {
        guard.claim(Group::Motion)?;
        acc.pending_dwell = true;
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
      92 => {
        guard.claim(Group::Motion)?;
        acc.pending_set_offset = true;
        Ok(())
      }
      _ => Err(GcodeError::UnsupportedCommand),
    }
  }

  /// Apply an `M` word from the supported subset (M3/M4/M5 spindle, M30 program end).
  fn apply_m_word(&self, value: f32, guard: &mut GroupGuard, acc: &mut LineAccumulator) -> Result<(), GcodeError> {
    match g_code(value)? {
      3 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::Clockwise);
        Ok(())
      }
      4 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::CounterClockwise);
        Ok(())
      }
      5 => {
        guard.claim(Group::Spindle)?;
        acc.pending_spindle = Some(SpindleState::Stop);
        Ok(())
      }
      30 => {
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
  /// Per the DOC-04 subset each line emits at most one [`PlannerCommand`], so when a spindle word
  /// shares a line with a move (uncommon for PCB milling) the move is emitted and the S value / state
  /// persist in modal state for the spindle task to act on — the two are not fused into one command.
  fn emit(&self, acc: &LineAccumulator, state: &ModalState) -> Option<PlannerCommand> {
    if acc.pending_program_end {
      return Some(PlannerCommand::ProgramEnd);
    }
    if acc.pending_dwell {
      return Some(PlannerCommand::Dwell { seconds: acc.p.unwrap_or(0.0) });
    }
    if let Some(is_g28) = acc.pending_predefined {
      return Some(PlannerCommand::GoToPredefined {
        is_g28,
        intermediate: acc.axes,
        units: state.units,
        distance: state.distance,
      });
    }
    if acc.pending_set_offset {
      return Some(PlannerCommand::SetCoordinateOffset { axes: acc.axes, units: state.units });
    }
    let has_axes = acc.axes.x.is_some() || acc.axes.y.is_some() || acc.axes.z.is_some();
    if has_axes {
      return Some(self.motion_command(acc, state));
    }
    if let Some(spindle) = acc.pending_spindle {
      return Some(PlannerCommand::Spindle { state: spindle, speed: state.spindle_speed });
    }
    None
  }

  /// Construct the move/arc command for a line carrying axis words, using the active motion mode.
  fn motion_command(&self, acc: &LineAccumulator, state: &ModalState) -> PlannerCommand {
    match state.motion {
      MotionMode::Rapid => PlannerCommand::Move {
        rapid: true,
        axes: acc.axes,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
      },
      MotionMode::Linear => PlannerCommand::Move {
        rapid: false,
        axes: acc.axes,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
      },
      MotionMode::ArcCw => PlannerCommand::Arc {
        cw: true,
        axes: acc.axes,
        i: acc.i,
        j: acc.j,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
      },
      MotionMode::ArcCcw => PlannerCommand::Arc {
        cw: false,
        axes: acc.axes,
        i: acc.i,
        j: acc.j,
        units: state.units,
        distance: state.distance,
        feed: state.feed,
      },
    }
  }
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
        axes: AxisWords { x: Some(10.0), y: Some(20.0), z: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 0.0,
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
        axes: AxisWords { x: Some(10.0), y: None, z: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 250.0,
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
        axes: AxisWords { x: Some(1.0), y: None, z: None },
        units: Units::Inch,
        distance: DistanceMode::Incremental,
        feed: 10.0,
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
        axes: AxisWords { x: Some(10.0), y: Some(0.0), z: None },
        i: Some(5.0),
        j: Some(0.0),
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 100.0,
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
        intermediate: AxisWords { x: None, y: None, z: Some(5.0) },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
      })
    );
  }

  #[test]
  fn parse_set_coordinate_offset_g92() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"G92 X0 Y0").expect("valid");
    assert_eq!(
      cmd,
      Some(PlannerCommand::SetCoordinateOffset {
        axes: AxisWords { x: Some(0.0), y: Some(0.0), z: None },
        units: Units::Millimeter,
      })
    );
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
  fn parse_program_end_m30() {
    let mut parser = Parser::new();
    let cmd = parser.parse_line(b"M30").expect("valid");
    assert_eq!(cmd, Some(PlannerCommand::ProgramEnd));
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
    assert_eq!(GcodeError::ModalGroupViolation.code(), 9);
    assert_eq!(GcodeError::UnsupportedCommand.code(), 20);
  }
}
