//! Inbound line de-framing with grblHAL terminator rules.
//!
//! grblHAL treats `CRLF` (`\r\n`) and `LFCR` (`\n\r`) as a *single* line terminator — counting them as two
//! is the classic legacy "double-ok" bug. A bare `CR` or bare `LF` is also a terminator. This deframer
//! accumulates incoming bytes and yields complete lines (terminator stripped), correctly handling a
//! terminator that is split across two reads: it remembers whether the previous chunk ended on a lone
//! `\r`/`\n` and swallows the complementary byte at the start of the next chunk.

/// Stateful reassembler that turns arbitrary inbound byte chunks into whole lines. One instance lives for
/// the life of a connection; it carries the partial-line buffer and the cross-read pairing state.
#[derive(Debug, Default)]
pub struct LineReassembler {
  /// Bytes of the current, not-yet-terminated line.
  buffer: Vec<u8>,
  /// The terminator byte (`\r` or `\n`) that just ended a line, when the *next* byte might be its pair
  /// (`\n` or `\r`) and should be swallowed rather than starting a fresh line. `None` otherwise.
  pending_pair: Option<u8>,
}

impl LineReassembler {
  /// Create an empty reassembler.
  pub fn new() -> Self {
    Self::default()
  }

  /// Feed a chunk of inbound bytes; returns every complete line found (terminator stripped, as `String`,
  /// lossily decoded since firmware output is ASCII). Partial trailing data is retained for the next call.
  /// Empty lines are returned as empty strings — the caller decides what an empty line means.
  pub fn push(&mut self, chunk: &[u8]) -> Vec<String> {
    let mut lines = Vec::new();
    for &byte in chunk {
      // If the previous byte was a lone terminator, a complementary terminator here completes a CRLF/LFCR
      // pair and is swallowed without emitting a second (empty) line. `take()` clears the pending state either
      // way; only the matching-complement case is swallowed, otherwise `byte` is processed below as usual.
      if let Some(prev) = self.pending_pair.take()
        && ((prev == b'\r' && byte == b'\n') || (prev == b'\n' && byte == b'\r'))
      {
        continue;
      }

      match byte {
        b'\r' | b'\n' => {
          lines.push(String::from_utf8_lossy(&self.buffer).into_owned());
          self.buffer.clear();
          // Remember this terminator so a complementary byte in the next position pairs with it.
          self.pending_pair = Some(byte);
        }
        _ => self.buffer.push(byte),
      }
    }
    lines
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn one_chunk(bytes: &[u8]) -> Vec<String> {
    LineReassembler::new().push(bytes)
  }

  #[test]
  fn crlf_is_a_single_terminator() {
    assert_eq!(one_chunk(b"ok\r\n"), vec!["ok".to_string()]);
  }

  #[test]
  fn lfcr_is_a_single_terminator() {
    assert_eq!(one_chunk(b"ok\n\r"), vec!["ok".to_string()]);
  }

  #[test]
  fn bare_lf_terminates() {
    assert_eq!(one_chunk(b"a\nb\n"), vec!["a".to_string(), "b".to_string()]);
  }

  #[test]
  fn bare_cr_terminates() {
    assert_eq!(one_chunk(b"a\rb\r"), vec!["a".to_string(), "b".to_string()]);
  }

  #[test]
  fn two_real_lines_separated_by_crlf_yield_two_lines_not_four() {
    // Two lines, each CRLF-terminated, must produce exactly two lines — the double-ok regression guard.
    assert_eq!(one_chunk(b"ok\r\nerror:9\r\n"), vec!["ok".to_string(), "error:9".to_string()]);
  }

  #[test]
  fn terminator_split_across_two_reads_is_one_terminator() {
    let mut r = LineReassembler::new();
    let mut lines = r.push(b"ok\r");
    lines.extend(r.push(b"\nerror:1\r\n"));
    assert_eq!(lines, vec!["ok".to_string(), "error:1".to_string()]);
  }

  #[test]
  fn line_split_across_two_reads_reassembles() {
    let mut r = LineReassembler::new();
    assert!(r.push(b"<Idle|MPos:0.0").is_empty());
    assert_eq!(r.push(b"00>\n"), vec!["<Idle|MPos:0.000>".to_string()]);
  }

  #[test]
  fn genuinely_empty_line_is_emitted_once() {
    // A blank line (just CRLF) is one empty string, not two.
    assert_eq!(one_chunk(b"\r\n"), vec!["".to_string()]);
  }

  #[test]
  fn trailing_partial_line_is_held_until_terminated() {
    let mut r = LineReassembler::new();
    assert!(r.push(b"partial").is_empty());
    assert_eq!(r.push(b"\n"), vec!["partial".to_string()]);
  }
}
