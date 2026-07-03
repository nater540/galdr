//! The console MDI (manual-command) input's recall history: a pure, egui-free ring of previously sent lines
//! navigated with ↑/↓, shell-style. The view layer only translates key presses into [`MdiHistory::up`]/
//! [`MdiHistory::down`] calls and swaps the field text; every decision — ordering, the consecutive-duplicate
//! collapse, the draft stash, the cap — lives here where it is unit-tested without a window.
//!
//! Semantics (matching a shell's readline):
//! - Submitting a line [`push`](MdiHistory::push)es it to the newest slot; a line identical to the newest is not
//!   duplicated (jog-style repeats stay one entry). Any navigation in progress is reset.
//! - The FIRST ↑ stashes the operator's in-progress draft, then walks from the newest entry backward; further ↑
//!   steps older and clamps at the oldest (repeated ↑ at the top stays there rather than wrapping).
//! - ↓ walks back toward the newest; stepping past it restores the stashed draft and ends the recall, so the
//!   operator gets their half-typed line back rather than losing it.

/// How many sent lines the recall keeps. Old entries fall off the front; a session's worth of manual commands
/// comfortably fits, and the buffer stays trivially small.
const HISTORY_CAP: usize = 100;

/// The MDI recall history plus the transient navigation state (where ↑/↓ currently points, and the stashed
/// draft the recall will restore).
#[derive(Debug, Clone, Default)]
pub struct MdiHistory {
  /// Sent lines, oldest first.
  entries: Vec<String>,
  /// The entry ↑/↓ currently points at, or `None` when not navigating.
  cursor: Option<usize>,
  /// The in-progress draft stashed by the first ↑, restored by ↓ stepping past the newest entry.
  draft: String,
}

impl MdiHistory {
  /// Record a submitted line as the newest entry. Collapses a consecutive duplicate (submitting the same line
  /// twice keeps one entry), resets any navigation in progress, and drops the oldest entry past the cap. Blank
  /// lines are not recorded — the submit gate never sends them anyway.
  pub fn push(&mut self, line: &str) {
    self.cursor = None;
    self.draft.clear();
    if line.trim().is_empty() {
      return;
    }
    if self.entries.last().is_some_and(|last| last == line) {
      return;
    }
    self.entries.push(line.to_string());
    if self.entries.len() > HISTORY_CAP {
      self.entries.remove(0);
    }
  }

  /// Step to the previous (older) entry: the recall's ↑. The first step stashes `current` as the draft and
  /// yields the newest entry; further steps walk older and clamp at the oldest. `None` when there is no history
  /// to recall (the field is left untouched).
  pub fn up(&mut self, current: &str) -> Option<String> {
    if self.entries.is_empty() {
      return None;
    }
    let next = match self.cursor {
      None => {
        self.draft = current.to_string();
        self.entries.len() - 1
      }
      Some(index) => index.saturating_sub(1),
    };
    self.cursor = Some(next);
    Some(self.entries[next].clone())
  }

  /// Step to the next (newer) entry: the recall's ↓. Stepping past the newest entry ends the navigation and
  /// restores the stashed draft. `None` when not navigating (a stray ↓ leaves the field untouched).
  pub fn down(&mut self) -> Option<String> {
    let index = self.cursor?;
    if index + 1 < self.entries.len() {
      self.cursor = Some(index + 1);
      Some(self.entries[index + 1].clone())
    } else {
      self.cursor = None;
      Some(std::mem::take(&mut self.draft))
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn up_recalls_newest_first_then_walks_older_and_clamps() {
    let mut h = MdiHistory::default();
    h.push("$$");
    h.push("G0 X1");
    h.push("G0 X2");
    assert_eq!(h.up("").as_deref(), Some("G0 X2"), "the first ↑ recalls the newest line");
    assert_eq!(h.up("").as_deref(), Some("G0 X1"));
    assert_eq!(h.up("").as_deref(), Some("$$"));
    // Clamped at the oldest: repeated ↑ stays there rather than wrapping or vanishing.
    assert_eq!(h.up("").as_deref(), Some("$$"));
  }

  #[test]
  fn down_walks_back_and_restores_the_draft_past_the_newest() {
    let mut h = MdiHistory::default();
    h.push("$$");
    h.push("G0 X1");
    // The operator had "G53 G0" half-typed, then pressed ↑ twice…
    assert_eq!(h.up("G53 G0").as_deref(), Some("G0 X1"));
    assert_eq!(h.up("G53 G0").as_deref(), Some("$$"));
    // …and ↓ walks back down, ending with the stashed draft restored.
    assert_eq!(h.down().as_deref(), Some("G0 X1"));
    assert_eq!(h.down().as_deref(), Some("G53 G0"), "stepping past the newest restores the draft");
    assert_eq!(h.down(), None, "a stray ↓ after the recall ended leaves the field alone");
  }

  #[test]
  fn up_with_no_history_is_inert() {
    let mut h = MdiHistory::default();
    assert_eq!(h.up("half-typed"), None);
    assert_eq!(h.down(), None);
  }

  #[test]
  fn push_collapses_consecutive_duplicates_and_ignores_blanks() {
    let mut h = MdiHistory::default();
    h.push("G0 X1");
    h.push("G0 X1");
    h.push("");
    h.push("   ");
    assert_eq!(h.up("").as_deref(), Some("G0 X1"));
    // Only one entry exists: another ↑ clamps on it instead of finding a duplicate or a blank.
    assert_eq!(h.up("").as_deref(), Some("G0 X1"));
    // A NON-consecutive repeat is kept — recalling "$$, G0 X1, $$" in order matters more than dedupe.
    h.push("$$");
    h.push("G0 X1");
    assert_eq!(h.up("").as_deref(), Some("G0 X1"));
    assert_eq!(h.up("").as_deref(), Some("$$"));
    assert_eq!(h.up("").as_deref(), Some("G0 X1"));
  }

  #[test]
  fn push_resets_an_in_progress_navigation_and_caps_the_buffer() {
    let mut h = MdiHistory::default();
    for n in 0..(HISTORY_CAP + 10) {
      h.push(&format!("G0 X{n}"));
    }
    // The oldest entries fell off: walking all the way up lands on the capped-out oldest survivor.
    let mut last = String::new();
    for _ in 0..(HISTORY_CAP + 20) {
      if let Some(entry) = h.up("") {
        last = entry;
      }
    }
    assert_eq!(last, "G0 X10", "the cap drops the oldest entries");
    // Submitting mid-navigation resets the cursor: the next ↑ recalls the (new) newest line again.
    h.up("");
    h.push("$H");
    assert_eq!(h.up("").as_deref(), Some("$H"));
  }
}
