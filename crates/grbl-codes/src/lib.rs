//! Shared grblHAL wire vocabulary.
//!
//! This crate holds the parts of the grblHAL wire protocol that are a fixed, spec-defined vocabulary and so
//! must be *identical* on both sides of the link — the firmware that emits them and the host sender that parses
//! them. Right now that is the machine run-state token set (the leading `<State|…>` field of a `<...>` status
//! report). Single-sourcing it here means the emitter (`firmware_core::protocol`) and the parser
//! (`skirnir::protocol`) cannot drift on the spelling of a token.
//!
//! What deliberately does NOT live here: the `error:N` / `ALARM:N` *text* tables. Those are intentionally
//! different on each side — the firmware's tables are exactly the codes it emits (its `$EE`/`$EA` authority),
//! while the host keeps a broader fallback table for any grbl-ish controller and learns the real text from
//! `$EE`/`$EA` at runtime. Merging them would change behavior; instead a cross-crate test locks their overlap.
//! See `docs/architecture-refactor.md` (D1).

#![no_std]
#![deny(unsafe_code)]

/// A grblHAL machine run-state token — the leading `<State|…>` field of a `<...>` status report, without any
/// `:substate` digits. This is the pure wire vocabulary shared by the firmware emitter and the host parser.
/// Each crate keeps its own richer state type (the firmware's carries hold/alarm payloads; the host's carries an
/// `Unknown` catch-all) and converts to/from this shared token set, so the token *strings* exist in exactly one
/// place. The variants and their order mirror grblHAL's status-report state set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RunState {
  /// `Idle` — ready, no motion queued or executing.
  Idle,
  /// `Run` — executing a queued motion block.
  Run,
  /// `Hold` — feed hold in effect (the `:0`/`:1` substate is appended by the emitter, not encoded here).
  Hold,
  /// `Jog` — executing a `$J=` jog.
  Jog,
  /// `Alarm` — halted in an alarm (the `:<code>` substate is appended by the emitter).
  Alarm,
  /// `Door` — safety door open / interlock active.
  Door,
  /// `Check` — `$C` check mode: parse/validate without moving.
  Check,
  /// `Home` — running the homing cycle.
  Home,
  /// `Sleep` — `$SLP` sleep state.
  Sleep,
  /// `Tool` — grblHAL tool-change state, held awaiting a cycle-start resume.
  Tool,
}

impl RunState {
  /// Every run-state token, in grblHAL's canonical order. Lets both sides enumerate the full vocabulary (e.g.
  /// the cross-crate lock test) without restating the set.
  pub const ALL: [RunState; 10] = [
    RunState::Idle,
    RunState::Run,
    RunState::Hold,
    RunState::Jog,
    RunState::Alarm,
    RunState::Door,
    RunState::Check,
    RunState::Home,
    RunState::Sleep,
    RunState::Tool,
  ];

  /// The status-report token string for this state (e.g. `"Idle"`). Substate digits (`Hold:0`, `Alarm:2`) are
  /// appended by the caller — this returns only the bare state token.
  pub const fn as_token(self) -> &'static str {
    match self {
      RunState::Idle => "Idle",
      RunState::Run => "Run",
      RunState::Hold => "Hold",
      RunState::Jog => "Jog",
      RunState::Alarm => "Alarm",
      RunState::Door => "Door",
      RunState::Check => "Check",
      RunState::Home => "Home",
      RunState::Sleep => "Sleep",
      RunState::Tool => "Tool",
    }
  }

  /// Parse a leading state token (already split off any `:substate`) into a [`RunState`], or `None` if the token
  /// is not a recognised grblHAL state. Callers that want a total mapping (e.g. a UI that surfaces unknown
  /// tokens rather than dropping them) map the `None` to their own catch-all.
  pub fn from_token(token: &str) -> Option<RunState> {
    Some(match token {
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
      _ => return None,
    })
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn every_token_round_trips() {
    for state in RunState::ALL {
      assert_eq!(RunState::from_token(state.as_token()), Some(state));
    }
  }

  #[test]
  fn unknown_token_is_none() {
    assert_eq!(RunState::from_token("Bogus"), None);
    assert_eq!(RunState::from_token(""), None);
    // A token carrying its substate must be split BEFORE this call — the bare token does not parse.
    assert_eq!(RunState::from_token("Hold:0"), None);
  }
}
