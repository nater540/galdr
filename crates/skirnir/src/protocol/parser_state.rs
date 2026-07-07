//! Parsing of the grblHAL `$G` / `[GC:]` parser-state line for the fields skirnir renders.
//!
//! The `$G` query answers with a bracketed push message, e.g. `[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]`. The
//! response layer ([`crate::protocol::response::parse_line`]) strips the square brackets and hands the generic
//! `[...]` body to the reducer; this module recognises the `GC:` sub-kind and extracts the modal words skirnir
//! cares about. Today that is just the active tool (`T<n>`) — the AUTHORITATIVE source of "what tool is loaded"
//! per the streaming contract — but the parser is written to scan the whole word list so future modal fields
//! (units, plane, distance mode) can join without reopening it.
//!
//! Following grblHAL's sender guidance we ignore words we do not model rather than failing, so a firmware that
//! adds or reorders modal words never breaks the parse. The function is pure and synchronous (no async, no UI)
//! so the grammar is unit-tested in isolation, mirroring [`crate::protocol::status`].

/// The parser-state fields skirnir renders. Only the active tool is modelled today; the struct exists so the
/// reducer stores one typed value and future modal fields slot in beside it without changing call sites.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ParserState {
  /// The active tool number from the `T<n>` modal word (`T0` = no tool). `None` when the line carried no `T`
  /// word at all (an older firmware, or a malformed line), so the UI can distinguish "no tool" (`Some(0)`) from
  /// "not reported" (`None`) and avoid showing a stale or fabricated number.
  pub tool: Option<u32>,
  /// The active work-coordinate system index from the `G54`…`G59` modal word (`G54` = 0 … `G59` = 5). `None` when
  /// the line carried no WCS word (a malformed/partial line). The height-map WCS-mismatch warning reads this to
  /// tell whether the mesh was probed under the same WCS the job runs in.
  pub wcs: Option<usize>,
}

/// Parse a `[GC:...]` body (the bracket-stripped text, INCLUDING the leading `GC:` tag) into a [`ParserState`],
/// or `None` when the body is not a `GC:` line. The modal words are space-separated; we scan them for the ones we
/// model and ignore the rest. A `T` word with a non-numeric tail is treated as absent rather than fabricating a
/// tool — the line still reaches the console verbatim through the generic message path, so nothing is lost.
pub fn parse_gc_body(body: &str) -> Option<ParserState> {
  let words = body.strip_prefix("GC:")?;
  let mut state = ParserState::default();
  for word in words.split_whitespace() {
    // The tool word is `T` followed by the integer tool number. Other modal words (`G..`, `M..`, `F..`, `S..`)
    // are not modelled yet and fall through untouched.
    if let Some(number) = word.strip_prefix('T')
      && let Ok(tool) = number.parse::<u32>()
    {
      state.tool = Some(tool);
    }
    // The WCS modal word is `G54`…`G59` (index 0…5). Extended systems `G59.1`…`G59.3` are not modelled — an
    // exact match keeps them out. This is the authoritative active WCS the mesh-mismatch warning compares against.
    state.wcs = state.wcs.or(match word {
      "G54" => Some(0),
      "G55" => Some(1),
      "G56" => Some(2),
      "G57" => Some(3),
      "G58" => Some(4),
      "G59" => Some(5),
      _ => None,
    });
  }
  Some(state)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_non_gc_body_is_not_parser_state() {
    // Only a `GC:`-tagged body is a parser-state line; every other bracketed message must fall through.
    assert_eq!(parse_gc_body("MSG:'$H'|'$X' to unlock"), None);
    assert_eq!(parse_gc_body("OPT:VNS,100,1024"), None);
    assert_eq!(parse_gc_body("PRB:0.000,0.000,0.000:0"), None);
  }

  #[test]
  fn the_active_tool_is_extracted_from_a_full_gc_line() {
    // The canonical grblHAL `$G` answer: the tool word sits among the modal words and is the authoritative tool.
    let state = parse_gc_body("GC:G0 G54 G17 G21 G90 G94 M5 M9 T3 F0 S0").expect("a GC line parses");
    assert_eq!(state.tool, Some(3));
  }

  #[test]
  fn the_active_wcs_is_extracted_from_the_g54_to_g59_word() {
    // The WCS modal word maps G54…G59 → 0…5; the mismatch warning compares this against the mesh's probed WCS.
    assert_eq!(parse_gc_body("GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0").expect("parses").wcs, Some(0));
    assert_eq!(parse_gc_body("GC:G0 G56 T0 F0 S0").expect("parses").wcs, Some(2));
    assert_eq!(parse_gc_body("GC:G0 G59 T0 F0 S0").expect("parses").wcs, Some(5));
    // Extended systems (G59.1…) are not modelled, and a line with no WCS word reports `None`.
    assert_eq!(parse_gc_body("GC:G0 G59.1 T0 F0 S0").expect("parses").wcs, None);
    assert_eq!(parse_gc_body("GC:G17 G21 F0 S0").expect("parses").wcs, None);
  }

  #[test]
  fn t0_means_no_tool_and_is_distinct_from_an_absent_word() {
    // `T0` is an explicit "no tool loaded" — `Some(0)`, NOT `None`. A line with no `T` word at all is `None`, so
    // the UI can tell "reported as none" from "never reported".
    assert_eq!(parse_gc_body("GC:G0 G54 T0 F0 S0").expect("parses").tool, Some(0));
    assert_eq!(parse_gc_body("GC:G0 G54 F0 S0").expect("parses").tool, None);
  }

  #[test]
  fn a_malformed_tool_word_is_ignored_not_fabricated() {
    // A non-numeric `T` tail (corrupt line) must not fabricate a tool; it is dropped and stays `None` so the DRO
    // never shows a bogus number. The verbatim line still reaches the console via the generic message path.
    assert_eq!(parse_gc_body("GC:G0 Tx F0 S0").expect("parses").tool, None);
  }

  #[test]
  fn the_last_tool_word_wins_and_high_numbers_parse() {
    // Defensive: a duplicate `T` word takes the last value (grbl emits one, but a scan must be deterministic), and
    // a multi-digit tool number parses (some setups carry large tool-table indices).
    assert_eq!(parse_gc_body("GC:T1 T27").expect("parses").tool, Some(27));
  }
}
