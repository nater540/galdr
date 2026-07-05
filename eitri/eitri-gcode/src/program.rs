//! The line buffer a postprocessor's hooks append to, plus final rendering to text.
//!
//! A [`Program`] is just an ordered list of already-formatted lines and the [`OutputFormat`] that produced them. The
//! emitter drives the hook sequence; each hook pushes zero, one, or many lines. Rendering joins the lines with the
//! configured terminator and appends a final one so the file ends in a newline.

use crate::format::OutputFormat;

/// A G-code program under construction. Hooks append lines via [`Program::push`] / [`Program::comment`]; the emitter
/// hands the finished buffer back and [`Program::render`] serializes it.
#[derive(Debug, Clone)]
pub struct Program {
  format: OutputFormat,
  lines: Vec<String>,
}

impl Program {
  /// A new, empty program that will be rendered with `format`.
  pub fn new(format: OutputFormat) -> Program {
    Program { format, lines: Vec::new() }
  }

  /// The output formatting this program renders with — used by hooks to format coordinates and feeds.
  pub fn format(&self) -> &OutputFormat {
    &self.format
  }

  /// Append one already-formatted line (no terminator; trailing whitespace is trimmed).
  pub fn push(&mut self, line: impl Into<String>) {
    let line = line.into();
    let trimmed = line.trim_end();
    self.lines.push(trimmed.to_string());
  }

  /// Append a comment line in the program's configured style; a no-op when comments are disabled.
  pub fn comment(&mut self, text: &str) {
    if let Some(line) = self.format.comment(text) {
      self.lines.push(line);
    }
  }

  /// The emitted lines so far (without terminators).
  pub fn lines(&self) -> &[String] {
    &self.lines
  }

  /// Number of lines emitted.
  pub fn len(&self) -> usize {
    self.lines.len()
  }

  /// Whether nothing has been emitted yet.
  pub fn is_empty(&self) -> bool {
    self.lines.is_empty()
  }

  /// Serialize to a single string: every line followed by the configured terminator (so the file ends in a newline).
  pub fn render(&self) -> String {
    let ending = self.format.line_ending.as_str();
    let mut out = String::new();
    for line in &self.lines {
      out.push_str(line);
      out.push_str(ending);
    }
    out
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::format::LineEnding;

  #[test]
  fn render_terminates_every_line_including_the_last() {
    let mut prog = Program::new(OutputFormat::default());
    prog.push("G21");
    prog.push("G0 X1.0000");
    assert_eq!(prog.render(), "G21\nG0 X1.0000\n");
  }

  #[test]
  fn crlf_ending_is_honoured() {
    let mut prog = Program::new(OutputFormat { line_ending: LineEnding::Crlf, ..Default::default() });
    prog.push("M2");
    assert_eq!(prog.render(), "M2\r\n");
  }

  #[test]
  fn push_trims_trailing_whitespace() {
    let mut prog = Program::new(OutputFormat::default());
    prog.push("G0 X1.0000   ");
    assert_eq!(prog.lines(), &["G0 X1.0000".to_string()]);
  }
}
