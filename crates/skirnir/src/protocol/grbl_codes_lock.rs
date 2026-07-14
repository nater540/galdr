//! Cross-crate lock (D1): skirnir's static grbl fallback tables and run-state vocabulary must stay consistent
//! with the firmware authority (`firmware-core`).
//!
//! The error/alarm text tables are deliberately NOT shared — skirnir keeps a broader fallback for any grbl-ish
//! controller and learns the real text from `$EE`/`$EA` at runtime, while the firmware's tables are exactly the
//! codes it emits. But for the codes the firmware DOES declare, skirnir's static fallback should read identically
//! to the firmware's `$EE`/`$EA` text, so the display does not visibly change when runtime enrichment arrives.
//! Before this lock the only "verbatim match" tests lived inside skirnir and compared skirnir's copy against
//! skirnir's own literals — they never actually pinned skirnir to the firmware, so the two could drift silently
//! (they had, on alarm 4/5, historically). This module is the real cross-crate pin: it reaches into
//! `firmware-core` (a dev-dependency, test-only) and asserts the overlap matches.
//!
//! The run-state token set IS shared (`grbl_codes`), so its half of the lock is structural — this just proves
//! skirnir recognises every shared token and maps it consistently.

use firmware_core::protocol::{AlarmCode, ERROR_CODES};

use super::codes::{alarm_text, error_text};
use super::status::{RunState, peek_run_state};

/// The generic gloss [`error_text`] returns for a code it has no specific row for. Hitting it means the code is
/// outside skirnir's static table, so the lock does not apply (the runtime `$EE` CodeBook enriches it instead).
const GENERIC_ERROR_NAME: &str = "G-code error";
/// The generic gloss [`alarm_text`] returns for an alarm it has no specific row for (see above).
const GENERIC_ALARM_NAME: &str = "Controller locked";

#[test]
fn skirnir_error_fallback_matches_firmware_authority() {
  for code in ERROR_CODES {
    let text = error_text(u32::from(code.id));
    if text.name == GENERIC_ERROR_NAME {
      continue; // No specific skirnir row for this id; the runtime `$EE` enumeration enriches it.
    }
    assert_eq!(text.name.as_ref(), code.name, "error:{} name drifted from the firmware authority", code.id);
    assert_eq!(
      text.description.as_ref(),
      code.description,
      "error:{} description drifted from the firmware authority",
      code.id
    );
  }
}

#[test]
fn skirnir_alarm_fallback_matches_firmware_authority() {
  for &alarm in AlarmCode::ALL {
    let text = alarm_text(u32::from(alarm.code()));
    if text.name == GENERIC_ALARM_NAME {
      continue; // No specific skirnir row for this id; the runtime `$EA` enumeration enriches it.
    }
    assert_eq!(text.name.as_ref(), alarm.name(), "ALARM:{} name drifted from the firmware authority", alarm.code());
    assert_eq!(
      text.description.as_ref(),
      alarm.description(),
      "ALARM:{} description drifted from the firmware authority",
      alarm.code()
    );
  }
}

#[test]
fn skirnir_recognises_every_shared_run_state_token() {
  // Every token the firmware can emit (the shared `grbl_codes` vocabulary) must parse to a concrete host
  // `RunState`, never `Unknown`. Both sides key off `grbl_codes`, so this also proves the emitter's and parser's
  // token sets are identical — a token the firmware could emit but skirnir would drop to `Unknown` cannot exist.
  for shared in grbl_codes::RunState::ALL {
    let parsed = peek_run_state(shared.as_token());
    assert_ne!(parsed, RunState::Unknown, "skirnir did not recognise run-state token {:?}", shared.as_token());
    assert_eq!(parsed, RunState::from(shared), "run-state token {:?} mapped inconsistently", shared.as_token());
  }
}
