//! The explicit-Save staging store for the firmware settings dialog: edits are buffered locally as a dirty map
//! (`$<n>` number → staged value text) and only flushed to the controller when the operator presses Save.
//!
//! This is the host-side fix for the silent-drop bug the old per-field commit had: there, an edit only reached
//! the firmware if the operator pressed Enter in the field, so typing a value and then clicking elsewhere
//! discarded it. Staging happens on *any* edit (Enter or focus-loss), so a value the operator typed is never
//! lost — it is held here, visibly marked modified, until an explicit Save writes it (or a confirmed Discard
//! clears it). grbl has no batch/transaction, so Save still emits one independent `$<n>=<value>` line per staged
//! setting; this type only owns the staging boundary and the ordered flush, not the wire transaction.
//!
//! It is pure (no egui, no I/O) so the staging policy — what counts as dirty, what Save sends, and whether a
//! refresh/close needs a discard confirmation — is unit-tested without a window or a live link. The view binds
//! its rows to [`SettingsStaging::staged_value`]/[`SettingsStaging::is_dirty`] for the modified marker, and the
//! shell flushes [`SettingsStaging::write_lines`] through the streaming engine on Save.

use std::collections::BTreeMap;

use crate::protocol::setting_write_line;

/// The locally-staged settings edits: setting number → staged value text, kept in a [`BTreeMap`] so a flush is
/// always in ascending `$<n>` order (predictable, matching the firmware's own `$$` order). A row is "dirty"
/// exactly when it has an entry here; staging a value equal to the live one removes the entry rather than
/// holding a no-op, so the dirty set never contains writes that would not change anything.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SettingsStaging {
  staged: BTreeMap<u32, String>,
}

impl SettingsStaging {
  /// An empty staging store (nothing edited yet).
  pub fn new() -> Self {
    Self::default()
  }

  /// Stage an edit of setting `number` to `value`, given the firmware's current `live` value (if known). The
  /// value is trimmed; an edit that, after trimming, equals the live value clears the staging for that setting
  /// (it is no longer a change) — so clicking into a field and back out, or typing the value already shown, never
  /// leaves a phantom modified marker or a no-op write. An empty (whitespace-only) edit is also treated as "no
  /// change": it clears any staging rather than staging a blank the firmware would reject.
  pub fn stage(&mut self, number: u32, value: &str, live: Option<&str>) {
    let trimmed = value.trim();
    if trimmed.is_empty() || live == Some(trimmed) {
      self.staged.remove(&number);
      return;
    }
    self.staged.insert(number, trimmed.to_string());
  }

  /// The staged value for `number`, if it has been edited, else `None`. The view shows this in place of the live
  /// value while a row is dirty so the field reflects the pending edit rather than the controller's old value.
  pub fn staged_value(&self, number: u32) -> Option<&str> {
    self.staged.get(&number).map(String::as_str)
  }

  /// Whether setting `number` has a staged (unsaved) edit. Drives the row's "modified" marker.
  pub fn is_dirty(&self, number: u32) -> bool {
    self.staged.contains_key(&number)
  }

  /// Whether nothing is staged. The Save button is disabled and a refresh/close needs no confirmation when true.
  pub fn is_empty(&self) -> bool {
    self.staged.is_empty()
  }

  /// How many settings have staged edits — the `N` in the "Discard N unsaved change(s)?" confirmation copy.
  pub fn len(&self) -> usize {
    self.staged.len()
  }

  /// Drop every staged edit without writing. Called by a confirmed Discard, by Save once the writes are issued,
  /// and on disconnect so a reconnect never resumes editing the previous board's settings.
  pub fn clear(&mut self) {
    self.staged.clear();
  }

  /// The ordered `$<n>=<value>` write lines a Save must send, one per staged setting, in ascending `$<n>` order.
  /// Built through the shared [`setting_write_line`] so the wire form lives in one tested place. The shell sends
  /// each through the streaming engine (then `$$` to re-confirm) and clears staging; grbl has no batch, so order
  /// between independent writes does not matter — ascending is chosen only for predictability.
  pub fn write_lines(&self) -> Vec<String> {
    self.staged.iter().map(|(number, value)| setting_write_line(*number, value)).collect()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn starts_empty() {
    let staging = SettingsStaging::new();
    assert!(staging.is_empty());
    assert_eq!(staging.len(), 0);
    assert!(staging.write_lines().is_empty());
  }

  #[test]
  fn staging_a_changed_value_marks_it_dirty_and_holds_the_edit() {
    // The core fix: an edit (from typing/focus-loss, not only Enter) is held here so it is never silently dropped.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    assert!(staging.is_dirty(0), "a changed value is dirty");
    assert_eq!(staging.staged_value(0), Some("12"), "the field shows the staged edit, not the live value");
    assert_eq!(staging.len(), 1);
  }

  #[test]
  fn staging_the_live_value_is_a_no_op_and_clears_any_prior_edit() {
    // Editing back to the live value (or clicking in and out without a change) must not leave a phantom marker.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.stage(0, " 10 ", Some("10"));
    assert!(!staging.is_dirty(0), "re-typing the live value clears the dirty marker");
    assert!(staging.is_empty(), "no real change remains staged");
  }

  #[test]
  fn an_empty_edit_clears_rather_than_staging_a_blank() {
    // A whitespace-only edit is "no change", not a blank write the firmware would reject.
    let mut staging = SettingsStaging::new();
    staging.stage(5, "1", Some("0"));
    staging.stage(5, "   ", Some("0"));
    assert!(!staging.is_dirty(5));
    assert!(staging.is_empty());
  }

  #[test]
  fn a_first_ever_value_with_no_live_value_still_stages() {
    // A setting with no live value yet (only metadata) still stages a real edit.
    let mut staging = SettingsStaging::new();
    staging.stage(5, "1", None);
    assert!(staging.is_dirty(5));
    assert_eq!(staging.staged_value(5), Some("1"));
  }

  #[test]
  fn write_lines_are_ascending_and_use_the_shared_builder() {
    // Save emits one `$<n>=<value>` per staged setting in ascending order, formed by `setting_write_line`.
    let mut staging = SettingsStaging::new();
    staging.stage(110, "800", Some("500"));
    staging.stage(0, "12", Some("10"));
    staging.stage(22, "1", Some("0"));
    assert_eq!(staging.write_lines(), vec!["$0=12".to_string(), "$22=1".to_string(), "$110=800".to_string()]);
  }

  #[test]
  fn restaging_a_setting_replaces_its_value_not_duplicates_it() {
    // Editing the same field twice keeps one entry with the latest value.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.stage(0, "13", Some("10"));
    assert_eq!(staging.len(), 1);
    assert_eq!(staging.staged_value(0), Some("13"));
  }

  #[test]
  fn clear_drops_every_staged_edit() {
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.stage(110, "800", Some("500"));
    staging.clear();
    assert!(staging.is_empty(), "Discard / Save / disconnect wipe every staged edit");
  }
}
