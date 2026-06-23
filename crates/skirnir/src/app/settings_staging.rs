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

use std::collections::{BTreeMap, BTreeSet};

use crate::protocol::setting_write_line;

/// The locally-staged settings edits: setting number → staged value text, kept in a [`BTreeMap`] so a flush is
/// always in ascending `$<n>` order (predictable, matching the firmware's own `$$` order). A row is "dirty"
/// exactly when it has an entry here; staging a value equal to the live one removes the entry rather than
/// holding a no-op, so the dirty set never contains writes that would not change anything.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SettingsStaging {
  staged: BTreeMap<u32, String>,
  /// Edits issued by a Save and awaiting the post-write `$$` re-dump's confirmation: `$<n>` → the value we
  /// wrote. A setting stays in `staged` (dirty, visible) while it is here; [`Self::confirm`] removes it from
  /// `staged` only when the re-dump shows it actually took the written value, and flags it [`Self::rejected`]
  /// otherwise — so a firmware-refused setting (out-of-range/read-only → `error:N`) is never silently dropped.
  pending: BTreeMap<u32, String>,
  /// Settings whose last Save was rejected (the re-dump came back unchanged). They remain dirty so the operator
  /// keeps seeing their edit; the shell surfaces these `$<n>` to the console so the failure is not silent.
  rejected: BTreeSet<u32>,
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
    // A fresh edit supersedes any prior Save still awaiting confirmation for this setting, and clears a stale
    // rejected flag — the operator is acting on the row again, so neither the old save's confirmation nor its
    // failure should still apply.
    self.pending.remove(&number);
    self.rejected.remove(&number);
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

  /// Drop every staged edit (and any pending/rejected bookkeeping) without writing. Called by a confirmed
  /// Discard and on disconnect so a reconnect never resumes editing the previous board's settings.
  pub fn clear(&mut self) {
    self.staged.clear();
    self.pending.clear();
    self.rejected.clear();
  }

  /// The ordered `$<n>=<value>` write lines a Save must send, one per staged setting, in ascending `$<n>` order.
  /// Built through the shared [`setting_write_line`] so the wire form lives in one tested place. The shell sends
  /// each through the streaming engine (then `$$` to re-confirm); grbl has no batch, so order between independent
  /// writes does not matter — ascending is chosen only for predictability. Prefer [`Self::begin_save`], which
  /// also arms the per-setting confirmation so a rejected write is not silently dropped.
  pub fn write_lines(&self) -> Vec<String> {
    self.staged.iter().map(|(number, value)| setting_write_line(*number, value)).collect()
  }

  /// Issue a Save: return the ordered `$<n>=<value>` write lines AND arm each staged edit for confirmation by the
  /// post-write `$$` re-dump. Crucially this does NOT drop the staged edits — they stay dirty and visible until
  /// [`Self::confirm`] sees the re-dump prove each one took (or refuse it). This is the Bug 6 fix: clearing on
  /// Save before any `ok`/`error` returned silently lost a firmware-rejected setting; now a rejection survives.
  pub fn begin_save(&mut self) -> Vec<String> {
    self.rejected.clear();
    self.pending = self.staged.clone();
    self.write_lines()
  }

  /// Fold one re-dumped live `$<n>=<value>` value into a pending Save's confirmation. Only a setting currently
  /// awaiting confirmation (armed by [`Self::begin_save`] and not since re-edited) reacts: if the re-dumped
  /// `live` value equals the value we wrote, the write took — the edit is cleared (no longer dirty). If it does
  /// not, the firmware refused it (clamp/read-only → `error:N`, value reverted): the edit stays staged and dirty
  /// and is flagged [`Self::rejected`], so it remains visible to the operator instead of vanishing.
  pub fn confirm(&mut self, number: u32, live: &str) {
    let Some(written) = self.pending.get(&number) else {
      return;
    };
    // A re-edit after Save (which dropped the pending entry) is handled by the early return above; here the
    // pending value is still the one we wrote, so the re-dump is the authoritative verdict on it.
    if written == live.trim() {
      self.staged.remove(&number);
      self.rejected.remove(&number);
    } else {
      self.rejected.insert(number);
    }
    self.pending.remove(&number);
  }

  /// The `$<n>` numbers whose last Save was rejected by the firmware (the re-dump came back unchanged). The view
  /// uses this to flag the rejected rows; ascending order, matching the rest of the staging API. Non-draining —
  /// the flags persist (the rows stay dirty) until the operator re-edits or discards. See [`Self::take_rejected`]
  /// for the shell's one-shot console reporting.
  pub fn rejected(&self) -> impl Iterator<Item = u32> + '_ {
    self.rejected.iter().copied()
  }

  /// Whether setting `number`'s last Save was rejected by the firmware, for the view's per-row "rejected" marker.
  pub fn is_rejected(&self, number: u32) -> bool {
    self.rejected.contains(&number)
  }

  /// Drain and return the rejected `$<n>` set so the shell reports each refused setting to the console exactly
  /// once. The settings stay staged/dirty (the operator's edit remains visible for a retry); only the
  /// not-yet-reported flag is consumed here, so a rejection is announced once rather than every frame.
  pub fn take_rejected(&mut self) -> Vec<u32> {
    std::mem::take(&mut self.rejected).into_iter().collect()
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
  fn begin_save_returns_the_write_lines_but_keeps_rows_dirty_until_confirmed() {
    // Bug 6: Save must NOT drop the staged edits the instant the writes are issued — a setting the firmware
    // rejects (error:N) would then silently vanish. `begin_save` hands back the write lines and marks the edits
    // pending-confirmation, but they stay dirty and visible until the post-write `$$` re-dump confirms each one.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.stage(110, "800", Some("500"));
    let lines = staging.begin_save();
    assert_eq!(lines, vec!["$0=12".to_string(), "$110=800".to_string()]);
    // The rows remain dirty and show their staged values — nothing is dropped yet.
    assert!(staging.is_dirty(0) && staging.is_dirty(110), "edits stay dirty until the re-dump confirms them");
    assert_eq!(staging.staged_value(0), Some("12"));
    assert!(!staging.is_empty());
  }

  #[test]
  fn confirm_clears_an_accepted_setting_and_keeps_a_rejected_one_dirty() {
    // The heart of Bug 6: after Save's `$$` re-dump, the firmware shows the new value for an accepted write and
    // the OLD value for a rejected one. Confirming with the re-dumped live value must clear the accepted row and
    // keep the rejected row staged/dirty so the operator still sees their failed edit, rather than it vanishing.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10")); // will be accepted (re-dump shows 12)
    staging.stage(110, "999999", Some("500")); // will be rejected (re-dump still shows 500)
    staging.begin_save();
    // The re-dump lands: $0 took the new value, $110 was refused and reverted.
    staging.confirm(0, "12");
    staging.confirm(110, "500");
    assert!(!staging.is_dirty(0), "an accepted setting clears once the re-dump confirms it took");
    assert!(staging.is_dirty(110), "a rejected setting stays dirty and visible, not silently dropped");
    assert_eq!(staging.staged_value(110), Some("999999"), "the rejected edit is still shown to the operator");
    assert_eq!(staging.rejected().collect::<Vec<_>>(), vec![110], "the rejected `$N` is reportable to the console");
  }

  #[test]
  fn confirm_for_an_unsaved_setting_is_ignored() {
    // A `$$` re-dump value for a setting that was never saved (e.g. an unrelated dump) must not touch staging.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.confirm(0, "10"); // no begin_save yet: this is just an ambient dump, not a save confirmation.
    assert!(staging.is_dirty(0), "an ambient dump value must not clear a freshly-staged, unsaved edit");
  }

  #[test]
  fn re_staging_a_pending_setting_supersedes_the_pending_save() {
    // If the operator edits a setting again after Save but before the re-dump lands, the new edit takes over: the
    // row is dirty with the new value and is no longer pending the old save's confirmation.
    let mut staging = SettingsStaging::new();
    staging.stage(0, "12", Some("10"));
    staging.begin_save();
    staging.stage(0, "15", Some("10")); // re-edited after Save.
    // The old save's confirmation must not clear the freshly re-staged value.
    staging.confirm(0, "12");
    assert!(staging.is_dirty(0), "a re-edit after Save supersedes the pending confirmation");
    assert_eq!(staging.staged_value(0), Some("15"));
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
