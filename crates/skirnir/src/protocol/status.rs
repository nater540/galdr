//! Structured parsing of a `<...>` status-report body into typed fields.
//!
//! The response layer ([`crate::protocol::response::parse_line`]) already strips the angle brackets and hands
//! us the verbatim body, e.g. `Idle|MPos:0.000,0.000,0.000|FS:0,0|Ov:100,100,100`. This module turns that
//! body into a [`StatusReport`] the UI's DRO/overrides/pin panels render directly. It is pure and
//! synchronous — no async, no UI — so the field grammar is unit-tested in isolation.
//!
//! Per `docs/gcode-streaming.md` the grammar is:
//! `<State{:substate}|MPos:|WPos:<axes>{|Bf:..}{|Ln:..}{|FS:..}{|Pn:..}{|WCO:..}{|Ov:..}{|A:..}{|...}>`.
//! State is always first; position (`MPos:` or `WPos:`, never both) is always second. We parse the fields we
//! render and, following grblHAL's sender guidance, ignore tags we do not recognise rather than failing, so
//! forward-compatible firmware extensions never break the DRO.

/// The machine's high-level run state, taken from the leading token of a status report. Substate digits (e.g.
/// `Hold:0`, `Run:2`) are carried in [`MachineState::substate`] so the UI can distinguish them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunState {
  /// `Idle` — ready, no motion, no program running.
  Idle,
  /// `Run` — executing motion from the planner.
  Run,
  /// `Hold` — feed hold in effect.
  Hold,
  /// `Jog` — executing a `$J=` jog.
  Jog,
  /// `Alarm` — controller is in an alarm state and refuses G-code.
  Alarm,
  /// `Door` — safety door is open / interlock active.
  Door,
  /// `Check` — `$C` check mode (parses without moving).
  Check,
  /// `Home` — running the homing cycle.
  Home,
  /// `Sleep` — `$SLP` sleep state.
  Sleep,
  /// `Tool` — grblHAL tool-change state.
  Tool,
  /// A state token we did not recognise. Surfaced rather than dropped so nothing is silently lost.
  Unknown,
}

/// Cheaply peek just the leading run-state token of a status-report body, without the full structural decode of
/// [`parse_status`]. The state element is always first (`State{:substate}|MPos:…`), so this splits off the first
/// field and the optional substate and maps only that — no position/`Pn:`/override parsing. The engine uses this
/// at the poll rate to adapt the 5↔10 Hz status cadence, leaving the full decode to the reducer's single pass so
/// the body is not parsed twice per report (finding #9).
pub fn peek_run_state(body: &str) -> RunState {
  let token = body.split('|').next().unwrap_or("");
  let state = token.split_once(':').map(|(state, _sub)| state).unwrap_or(token);
  RunState::from_token(state)
}

impl RunState {
  /// Map the leading state token (already split off any `:substate`) to a [`RunState`].
  fn from_token(token: &str) -> Self {
    match token {
      "Idle" => RunState::Idle,
      "Run" => RunState::Run,
      "Hold" => RunState::Hold,
      "Jog" => RunState::Jog,
      "Alarm" => RunState::Alarm,
      "Door" => RunState::Door,
      "Check" => RunState::Check,
      "Home" => RunState::Home,
      "Sleep" => RunState::Sleep,
      "Tool" => RunState::Tool,
      _ => RunState::Unknown,
    }
  }
}

/// The leading state element: a [`RunState`] plus an optional substate code (the digits after the colon, e.g.
/// `Hold:0` → substate `Some(0)`, `Run:2` → `Some(2)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MachineState {
  /// The high-level run state.
  pub state: RunState,
  /// The substate code if the token carried one (`Hold:0`, `Door:1`, `Run:2`, ...), else `None`.
  pub substate: Option<u32>,
}

/// The decoded `Pn:` input-pin signal set: which firmware input pins are currently asserted. grblHAL reports
/// `Pn:` as a string of single-letter signal codes listing only the ASSERTED pins, and omits the whole field
/// when nothing is asserted — so an absent `Pn:` means every pin here is clear. We model the full grblHAL
/// letter set (`docs/gcode-streaming.md` §4) as named booleans, with the X/Y/Z limit switches as first-class
/// fields the endstop UI reads directly; the rotary/extra limits and the auxiliary signals (door, reset, hold,
/// probe, e-stop, ...) are carried too so probe/door/etc. can be surfaced later without revisiting the parser.
/// Per grblHAL's sender guidance the decode is order-independent and silently ignores letters it does not know,
/// so a firmware that grows the signal list never breaks this struct.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PinState {
  /// `X` — the X limit switch is asserted.
  pub limit_x: bool,
  /// `Y` — the Y limit switch is asserted.
  pub limit_y: bool,
  /// `Z` — the Z limit switch is asserted.
  pub limit_z: bool,
  /// `A` — the A (rotary) limit switch is asserted.
  pub limit_a: bool,
  /// `B` — the B (rotary) limit switch is asserted.
  pub limit_b: bool,
  /// `C` — the C (rotary) limit switch is asserted.
  pub limit_c: bool,
  /// `U` — the U limit switch is asserted (grblHAL extra linear axis).
  pub limit_u: bool,
  /// `V` — the V limit switch is asserted (grblHAL extra linear axis).
  pub limit_v: bool,
  /// `W` — the W limit switch is asserted (grblHAL extra linear axis).
  pub limit_w: bool,
  /// `P` — the probe input is asserted (triggered).
  pub probe: bool,
  /// `O` — the probe is reported disconnected (grblHAL probe-connected sensing).
  pub probe_disconnected: bool,
  /// `D` — the safety-door input is asserted (door open / interlock).
  pub door: bool,
  /// `R` — the reset input is asserted.
  pub reset: bool,
  /// `H` — the feed-hold input is asserted.
  pub feed_hold: bool,
  /// `S` — the cycle-start input is asserted.
  pub cycle_start: bool,
  /// `E` — the emergency-stop input is asserted.
  pub e_stop: bool,
  /// `L` — the block-delete input is asserted.
  pub block_delete: bool,
  /// `T` — the optional-stop input is asserted.
  pub optional_stop: bool,
  /// `M` — a motor warning is asserted.
  pub motor_warning: bool,
  /// `F` — a motor fault is asserted.
  pub motor_fault: bool,
  /// `Q` — single-step (single-block) input is asserted.
  pub single_step: bool,
}

impl PinState {
  /// Decode the asserted-pin letters of a `Pn:` field (e.g. `['X', 'Y', 'Z']`) into a typed [`PinState`].
  /// Order-independent and forward-compatible: a letter we do not model is ignored rather than treated as an
  /// error, and an empty iterator yields the all-clear default (mirroring an absent `Pn:` field). Letters are
  /// matched case-sensitively, exactly as grblHAL emits them (all-uppercase).
  pub fn from_letters<I: IntoIterator<Item = char>>(letters: I) -> Self {
    let mut pins = PinState::default();
    for letter in letters {
      match letter {
        'X' => pins.limit_x = true,
        'Y' => pins.limit_y = true,
        'Z' => pins.limit_z = true,
        'A' => pins.limit_a = true,
        'B' => pins.limit_b = true,
        'C' => pins.limit_c = true,
        'U' => pins.limit_u = true,
        'V' => pins.limit_v = true,
        'W' => pins.limit_w = true,
        'P' => pins.probe = true,
        'O' => pins.probe_disconnected = true,
        'D' => pins.door = true,
        'R' => pins.reset = true,
        'H' => pins.feed_hold = true,
        'S' => pins.cycle_start = true,
        'E' => pins.e_stop = true,
        'L' => pins.block_delete = true,
        'T' => pins.optional_stop = true,
        'M' => pins.motor_warning = true,
        'F' => pins.motor_fault = true,
        'Q' => pins.single_step = true,
        // An unknown letter (a future grblHAL signal) is ignored, never an error — the list is expected to grow.
        _ => {}
      }
    }
    pins
  }

  /// Whether any X/Y/Z limit switch is asserted, for a single at-a-glance "an endstop is hit" signal.
  pub fn any_xyz_limit(self) -> bool {
    self.limit_x || self.limit_y || self.limit_z
  }
}

/// Whether a reported position vector is machine- or work-coordinate. A report carries exactly one of the
/// two; the other is derived via `WPos = MPos − WCO` once a [`StatusReport::wco`] is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionKind {
  /// `MPos:` — machine coordinates.
  Machine,
  /// `WPos:` — work coordinates.
  Work,
}

/// A parsed status report: the fields skirnir's UI renders. Absent optional fields are `None`/empty; an
/// unparseable numeric is dropped from its vector rather than failing the whole report. The full grbl field
/// set is large — we model the ones the DRO, overrides, feed/speed, and pin panels need, and ignore the rest.
#[derive(Debug, Clone, PartialEq)]
pub struct StatusReport {
  /// The leading state element (always present).
  pub machine_state: MachineState,
  /// Whether [`Self::position`] is machine or work coordinates.
  pub position_kind: PositionKind,
  /// The reported axis positions, in report order (X, Y, Z, then any rotary axes).
  pub position: Vec<f64>,
  /// `WCO:` work-coordinate offset per axis, if this report carried one. grbl pushes it intermittently and on
  /// change; the UI caches the last-seen value to derive the missing position kind.
  pub wco: Option<Vec<f64>>,
  /// `FS:` feed and spindle. `(feed, programmed_rpm, actual_rpm)`; `actual_rpm` is `None` when not reported.
  /// A bare `F:` (feed only, no spindle) yields `(feed, 0.0, None)`.
  pub feed_speed: Option<(f64, f64, Option<f64>)>,
  /// `Ov:` override percentages `(feed, rapid, spindle)`, each an integer percent as reported.
  pub overrides: Option<(u32, u32, u32)>,
  /// `Pn:` asserted input-pin signal letters (e.g. `P`, `X`, `D`), in report order. Empty when no `Pn:` field
  /// (which the firmware omits entirely when nothing is asserted). This is the raw, verbatim letter list; the
  /// typed decode is [`StatusReport::pin_state`], which the endstop/probe/door UI reads. Both come from the
  /// same parse, so they cannot disagree.
  pub pins: Vec<char>,
  /// `Bf:` buffer state `(planner_blocks_free, rx_bytes_free)`. Diagnostics only — never used for flow
  /// control (the host counts characters itself), but some UIs display it.
  pub buffer: Option<(u32, u32)>,
  /// `Ln:` current line number, if the running program carried line numbers.
  pub line: Option<u32>,
}

impl StatusReport {
  /// The typed input-pin set decoded from the raw [`Self::pins`] letters. An empty `Pn:` (the firmware omits
  /// the field when nothing is asserted) yields the all-clear default, so the UI can read this unconditionally.
  pub fn pin_state(&self) -> PinState {
    PinState::from_letters(self.pins.iter().copied())
  }
}

/// Parse one status-report body (no angle brackets) into a [`StatusReport`]. Always succeeds with at least a
/// [`MachineState`]; unrecognised fields are ignored. An empty body yields an `Unknown` state with no
/// position, which the UI can treat as a malformed report.
pub fn parse_status(body: &str) -> StatusReport {
  let mut fields = body.split('|');

  // The first field is always the state token, optionally `State:substate`.
  let machine_state = fields.next().map(parse_machine_state).unwrap_or(MachineState {
    state: RunState::Unknown,
    substate: None,
  });

  // Position kind defaults to Machine until a position field overrides it; an absent position leaves the
  // vector empty (a malformed report), which the UI renders as dashes rather than stale numbers.
  let mut position_kind = PositionKind::Machine;
  let mut position = Vec::new();
  let mut wco = None;
  let mut feed_speed = None;
  let mut overrides = None;
  let mut pins = Vec::new();
  let mut buffer = None;
  let mut line = None;

  for field in fields {
    let (tag, value) = match field.split_once(':') {
      Some(pair) => pair,
      // A field with no colon (unexpected) is ignored rather than mis-parsed.
      None => continue,
    };
    match tag {
      "MPos" => {
        position_kind = PositionKind::Machine;
        position = parse_floats(value);
      }
      "WPos" => {
        position_kind = PositionKind::Work;
        position = parse_floats(value);
      }
      "WCO" => wco = Some(parse_floats(value)),
      "FS" => feed_speed = parse_feed_speed(value),
      // `F:` is the feed-only variant emitted when spindle reporting is masked off; spindle reads as 0.
      "F" => feed_speed = value.trim().parse::<f64>().ok().map(|f| (f, 0.0, None)),
      "Ov" => overrides = parse_overrides(value),
      "Pn" => pins = value.chars().filter(|c| !c.is_whitespace()).collect(),
      "Bf" => buffer = parse_pair_u32(value),
      "Ln" => line = value.trim().parse::<u32>().ok(),
      // Every other tag (A, WCS, MPG, H, D, Sc, TLR, FW, In, ...) is ignored for now.
      _ => {}
    }
  }

  StatusReport {
    machine_state,
    position_kind,
    position,
    wco,
    feed_speed,
    overrides,
    pins,
    buffer,
    line,
  }
}

/// Parse the leading `State{:substate}` token into a [`MachineState`].
fn parse_machine_state(token: &str) -> MachineState {
  match token.split_once(':') {
    Some((state, sub)) => MachineState {
      state: RunState::from_token(state),
      substate: sub.trim().parse::<u32>().ok(),
    },
    None => MachineState {
      state: RunState::from_token(token),
      substate: None,
    },
  }
}

/// Parse a comma-separated list of floats, silently dropping any element that does not parse. grbl always
/// emits well-formed fixed-point numbers, but dropping a bad element keeps a single typo from blanking the
/// whole DRO.
fn parse_floats(value: &str) -> Vec<f64> {
  value.split(',').filter_map(|n| n.trim().parse::<f64>().ok()).collect()
}

/// Parse `FS:feed,rpm{,actual_rpm}` into `(feed, rpm, actual_rpm)`. Requires at least feed and rpm; a missing
/// or malformed pair yields `None` so the UI keeps its last-known feed/speed rather than showing garbage.
fn parse_feed_speed(value: &str) -> Option<(f64, f64, Option<f64>)> {
  let mut parts = value.split(',');
  let feed = parts.next()?.trim().parse::<f64>().ok()?;
  let rpm = parts.next()?.trim().parse::<f64>().ok()?;
  let actual = parts.next().and_then(|a| a.trim().parse::<f64>().ok());
  Some((feed, rpm, actual))
}

/// Parse `Ov:feed,rapid,spindle` into a three-integer percentage tuple. All three must be present and parse.
fn parse_overrides(value: &str) -> Option<(u32, u32, u32)> {
  let mut parts = value.split(',');
  let feed = parts.next()?.trim().parse::<u32>().ok()?;
  let rapid = parts.next()?.trim().parse::<u32>().ok()?;
  let spindle = parts.next()?.trim().parse::<u32>().ok()?;
  Some((feed, rapid, spindle))
}

/// Parse a `a,b` pair of unsigned integers (used by `Bf:`). Both must be present and parse.
fn parse_pair_u32(value: &str) -> Option<(u32, u32)> {
  let mut parts = value.split(',');
  let a = parts.next()?.trim().parse::<u32>().ok()?;
  let b = parts.next()?.trim().parse::<u32>().ok()?;
  Some((a, b))
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_the_minimal_idle_report() {
    let report = parse_status("Idle|MPos:0.000,0.000,0.000|FS:0,0");
    assert_eq!(report.machine_state, MachineState { state: RunState::Idle, substate: None });
    assert_eq!(report.position_kind, PositionKind::Machine);
    assert_eq!(report.position, vec![0.0, 0.0, 0.0]);
    assert_eq!(report.feed_speed, Some((0.0, 0.0, None)));
  }

  #[test]
  fn parses_a_running_report_with_work_position_and_overrides() {
    let report = parse_status("Run|WPos:-1.500,2.250,-0.100|FS:500,12000|Ov:110,100,90");
    assert_eq!(report.machine_state.state, RunState::Run);
    assert_eq!(report.position_kind, PositionKind::Work);
    assert_eq!(report.position, vec![-1.5, 2.25, -0.1]);
    assert_eq!(report.feed_speed, Some((500.0, 12000.0, None)));
    assert_eq!(report.overrides, Some((110, 100, 90)));
  }

  #[test]
  fn peek_run_state_matches_the_full_decode_without_parsing_the_body() {
    // The cheap leading-token peek (finding #9) must produce exactly the state the full `parse_status` would, for
    // bodies with and without a substate and trailing fields, so the engine's poll-rate gating never disagrees
    // with the reducer's decode. An empty / malformed leading token is `Unknown`, same as the full path.
    for body in ["Run|MPos:0,0,0|FS:500,0", "Hold:0|WPos:0,0,0", "Idle", "Jog:1|MPos:1,2,3", "Bogus|MPos:0,0,0",
      ""]
    {
      assert_eq!(peek_run_state(body), parse_status(body).machine_state.state, "peek must match full decode: {body:?}");
    }
  }

  #[test]
  fn carries_a_substate_code() {
    assert_eq!(parse_status("Hold:0|MPos:0,0,0").machine_state, MachineState {
      state: RunState::Hold,
      substate: Some(0),
    });
    assert_eq!(parse_status("Run:2|WPos:0,0,0").machine_state, MachineState {
      state: RunState::Run,
      substate: Some(2),
    });
  }

  #[test]
  fn parses_wco_for_deriving_the_other_position() {
    let report = parse_status("Idle|MPos:10.000,20.000,5.000|WCO:1.000,2.000,3.000");
    assert_eq!(report.wco, Some(vec![1.0, 2.0, 3.0]));
  }

  #[test]
  fn parses_actual_spindle_rpm_when_present() {
    let report = parse_status("Run|MPos:0,0,0|FS:300,10000,9850");
    assert_eq!(report.feed_speed, Some((300.0, 10000.0, Some(9850.0))));
  }

  #[test]
  fn parses_feed_only_speed_field() {
    let report = parse_status("Run|MPos:0,0,0|F:450");
    assert_eq!(report.feed_speed, Some((450.0, 0.0, None)));
  }

  #[test]
  fn collects_asserted_pin_letters() {
    let report = parse_status("Alarm|MPos:0,0,0|Pn:PXZ");
    assert_eq!(report.pins, vec!['P', 'X', 'Z']);
  }

  #[test]
  fn decodes_all_three_xyz_limits() {
    let pins = parse_status("Alarm|MPos:0,0,0|Pn:XYZ").pin_state();
    assert!(pins.limit_x && pins.limit_y && pins.limit_z);
    assert!(pins.any_xyz_limit());
    // Nothing else should be asserted by an XYZ list.
    assert!(!pins.probe && !pins.door && !pins.limit_a);
  }

  #[test]
  fn decodes_a_single_limit_leaving_the_others_clear() {
    let pins = parse_status("Alarm|MPos:0,0,0|Pn:X").pin_state();
    assert!(pins.limit_x);
    assert!(!pins.limit_y && !pins.limit_z);
    assert!(pins.any_xyz_limit());
  }

  #[test]
  fn decodes_a_mixed_limit_and_auxiliary_list() {
    // `Pn:PXYZD` mixes the probe, the X/Y/Z limits, and the door — each must land on its own field.
    let pins = parse_status("Alarm|MPos:0,0,0|Pn:PXYZD").pin_state();
    assert!(pins.probe && pins.limit_x && pins.limit_y && pins.limit_z && pins.door);
    assert!(!pins.probe_disconnected && !pins.e_stop);
  }

  #[test]
  fn an_absent_pn_field_clears_every_pin() {
    // grblHAL omits `Pn:` entirely when nothing is asserted, so the typed set must read all-clear.
    let pins = parse_status("Run|MPos:0,0,0|FS:500,0").pin_state();
    assert_eq!(pins, PinState::default());
    assert!(!pins.any_xyz_limit());
  }

  #[test]
  fn decodes_the_full_auxiliary_signal_set() {
    // Exercise every modelled non-limit signal letter so the decode table cannot silently lose one.
    let pins = parse_status("Door|MPos:0,0,0|Pn:OPDRHSELTMFQ").pin_state();
    assert!(pins.probe_disconnected && pins.probe && pins.door && pins.reset && pins.feed_hold);
    assert!(pins.cycle_start && pins.e_stop && pins.block_delete && pins.optional_stop);
    assert!(pins.motor_warning && pins.motor_fault && pins.single_step);
    // No limit letter was present, so the X/Y/Z limits stay clear.
    assert!(!pins.any_xyz_limit());
  }

  #[test]
  fn decodes_the_rotary_and_extra_limit_letters() {
    let pins = parse_status("Alarm|MPos:0,0,0|Pn:ABCUVW").pin_state();
    assert!(pins.limit_a && pins.limit_b && pins.limit_c);
    assert!(pins.limit_u && pins.limit_v && pins.limit_w);
    // These are not X/Y/Z, so the at-a-glance XYZ summary stays false.
    assert!(!pins.any_xyz_limit());
  }

  #[test]
  fn pin_decode_is_order_independent() {
    // The wire order of the letters must not matter — `ZYX` decodes identically to `XYZ`.
    assert_eq!(parse_status("Alarm|MPos:0,0,0|Pn:ZYX").pin_state(),
      parse_status("Alarm|MPos:0,0,0|Pn:XYZ").pin_state());
  }

  #[test]
  fn unknown_pin_letters_are_ignored_not_fatal() {
    // A future signal letter (`K`, here) must be ignored while the known letters around it still decode — the
    // list is expected to grow, so an unmodelled code is never an error.
    let pins = parse_status("Alarm|MPos:0,0,0|Pn:XKY").pin_state();
    assert!(pins.limit_x && pins.limit_y);
    assert!(!pins.limit_z);
  }

  #[test]
  fn pin_state_survives_an_odd_field_order() {
    // The `Pn:` field may arrive anywhere after the (fixed) state + position, so decode it from a report whose
    // tags are shuffled — order-independence at the report level, not just within the letter list.
    let pins = parse_status("Run|WPos:0,0,0|Ov:100,100,100|Pn:Z|FS:0,0|WCO:0,0,0").pin_state();
    assert!(pins.limit_z);
    assert!(!pins.limit_x && !pins.limit_y);
  }

  #[test]
  fn parses_buffer_and_line_fields() {
    let report = parse_status("Run|MPos:0,0,0|Bf:35,1023|Ln:42");
    assert_eq!(report.buffer, Some((35, 1023)));
    assert_eq!(report.line, Some(42));
  }

  #[test]
  fn handles_a_rotary_fourth_axis() {
    let report = parse_status("Idle|MPos:1.0,2.0,3.0,90.0");
    assert_eq!(report.position, vec![1.0, 2.0, 3.0, 90.0]);
  }

  #[test]
  fn unknown_fields_are_ignored_not_fatal() {
    let report = parse_status("Idle|MPos:0,0,0|WCS:G54|MPG:0|FW:grblHAL|TLR:1");
    assert_eq!(report.machine_state.state, RunState::Idle);
    assert_eq!(report.position, vec![0.0, 0.0, 0.0]);
  }

  #[test]
  fn unknown_state_token_is_surfaced_not_dropped() {
    let report = parse_status("Frobnicate|MPos:0,0,0");
    assert_eq!(report.machine_state.state, RunState::Unknown);
  }

  #[test]
  fn empty_body_yields_an_unknown_state_with_no_position() {
    let report = parse_status("");
    assert_eq!(report.machine_state.state, RunState::Unknown);
    assert!(report.position.is_empty());
  }
}
