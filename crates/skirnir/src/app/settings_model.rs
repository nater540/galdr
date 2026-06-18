//! The live firmware-settings model: an ordered, mergeable view of the `$<n>=<value>` values a `$$` dump
//! reports, enriched by the `[SETTING:...]` metadata a `$ES` enumeration reports.
//!
//! This is the host-side counterpart to the firmware's settings authority. It is pure (no egui, no I/O) so the
//! merge/edit logic is unit-tested without a window or a live link: the reducer folds parsed settings traffic
//! in via [`SettingsModel::apply_value`]/[`SettingsModel::apply_meta`], the view renders the ordered
//! [`SettingsModel::rows`], and an operator edit becomes a `$<n>=<value>` write the shell sends.
//!
//! ## Why the text `$<n>=<value>` path, not binary `$PBX`
//! `docs/gcode-streaming.md` designates two settings channels: the universal text path (`$$` to read, a single
//! `$<n>=<value>` to write, validated with `ok`/`error:N`) and the grblHAL/galdr `$PBX` binary bulk channel
//! that transfers a whole CRC-framed protobuf record. The interactive per-field editor the design shows — a
//! list of `$NNN` keys with live values the operator nudges one at a time — maps onto the text path exactly,
//! needs no frame/CRC plumbing, and lets the firmware validate each write. The `$PBX` bulk path is a
//! whole-record read-modify-write optimisation whose frame codec (magic/version/CRC32) lives in `firmware-core`
//! and is not pulled into skirnir; it is a deliberate follow-up, not modelled here. The doc's directive — "do
//! not hardcode a settings UI" — is honoured by learning the row labels from `$ES` rather than a static table.

use std::collections::BTreeMap;

use crate::protocol::{SettingMeta, SettingValue};

/// One row of the settings panel: a setting's number, live value, and (once a `$ES` enumeration has been
/// received) its human label, unit, and advertised bounds. A value can arrive before or after its metadata,
/// so either may be present alone; the row is keyed by number and the two streams merge onto it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingRow {
  /// The grbl setting number (`$0`, `$100`, ...).
  pub number: u32,
  /// The live value text from the last `$<n>=<value>` seen, or `None` if only metadata has arrived so far.
  pub value: Option<String>,
  /// The metadata from `$ES`, or `None` if only a value has arrived so far. Carries the name/unit/bounds.
  pub meta: Option<SettingMeta>,
}

impl SettingRow {
  /// The label to show for this setting: the enumerated name when known, else a bare `$<n>` fallback so a row
  /// that has a value but no metadata yet still reads sensibly.
  pub fn label(&self) -> String {
    match &self.meta {
      Some(meta) if !meta.name.is_empty() => meta.name.clone(),
      _ => format!("${}", self.number),
    }
  }

  /// The unit string to show beside the value, or empty when unknown or unitless.
  pub fn unit(&self) -> &str {
    self.meta.as_ref().map(|m| m.unit.as_str()).unwrap_or("")
  }
}

/// The ordered live settings, merged from `$<n>=<value>` values and `[SETTING:...]` metadata. Keyed by setting
/// number in a [`BTreeMap`] so iteration is always in ascending `$n` order (the firmware's own `$$` order),
/// regardless of the order lines arrive in. A `$$` dump replaces values as it streams; metadata is sticky.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SettingsModel {
  rows: BTreeMap<u32, SettingRow>,
}

impl SettingsModel {
  /// An empty model (no settings known yet).
  pub fn new() -> Self {
    Self::default()
  }

  /// Whether any setting (value or metadata) is known yet. The panel shows a "request settings" affordance
  /// until this is true.
  pub fn is_empty(&self) -> bool {
    self.rows.is_empty()
  }

  /// How many settings are known.
  pub fn len(&self) -> usize {
    self.rows.len()
  }

  /// Merge one live `$<n>=<value>` value into the model, creating the row if new or updating its value if it
  /// already exists. Metadata already merged onto the row is preserved.
  pub fn apply_value(&mut self, value: SettingValue) {
    let row = self.rows.entry(value.number).or_insert_with(|| SettingRow {
      number: value.number,
      value: None,
      meta: None,
    });
    row.value = Some(value.value);
  }

  /// Merge one `$ES` enumeration row into the model, creating the row if new or attaching its metadata if the
  /// value already arrived. A live value already on the row is preserved.
  pub fn apply_meta(&mut self, meta: SettingMeta) {
    let row = self.rows.entry(meta.number).or_insert_with(|| SettingRow {
      number: meta.number,
      value: None,
      meta: None,
    });
    row.meta = Some(meta);
  }

  /// The current value text for setting `number`, if known. Used to seed an edit field.
  pub fn value_of(&self, number: u32) -> Option<&str> {
    self.rows.get(&number).and_then(|r| r.value.as_deref())
  }

  /// The settings rows in ascending `$n` order, for the panel to render.
  pub fn rows(&self) -> impl Iterator<Item = &SettingRow> {
    self.rows.values()
  }

  /// Drop every known setting. Called on disconnect so a reconnect starts from a clean slate rather than
  /// showing the previous board's settings (report-derived state must not survive a disconnect).
  pub fn clear(&mut self) {
    self.rows.clear();
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn value(number: u32, value: &str) -> SettingValue {
    SettingValue { number, value: value.to_string() }
  }

  fn meta(number: u32, name: &str, unit: &str) -> SettingMeta {
    SettingMeta {
      number,
      group: 0,
      name: name.to_string(),
      unit: unit.to_string(),
      min: None,
      max: None,
    }
  }

  #[test]
  fn starts_empty() {
    let model = SettingsModel::new();
    assert!(model.is_empty());
    assert_eq!(model.len(), 0);
    assert!(model.rows().next().is_none());
  }

  #[test]
  fn values_merge_in_ascending_number_order_regardless_of_arrival() {
    // A `$$` dump can arrive in any order on the wire; the model always iterates ascending `$n`.
    let mut model = SettingsModel::new();
    model.apply_value(value(100, "250.000"));
    model.apply_value(value(0, "10"));
    model.apply_value(value(11, "0.010"));
    let numbers: Vec<u32> = model.rows().map(|r| r.number).collect();
    assert_eq!(numbers, vec![0, 11, 100]);
    assert_eq!(model.value_of(11), Some("0.010"));
  }

  #[test]
  fn a_later_value_overwrites_an_earlier_one() {
    // Re-dumping (or editing then re-reading) updates the live value in place.
    let mut model = SettingsModel::new();
    model.apply_value(value(0, "10"));
    model.apply_value(value(0, "12"));
    assert_eq!(model.value_of(0), Some("12"));
    assert_eq!(model.len(), 1, "the same setting is one row, not two");
  }

  #[test]
  fn metadata_and_value_merge_onto_one_row_in_either_order() {
    // Value first, then metadata.
    let mut model = SettingsModel::new();
    model.apply_value(value(0, "10"));
    model.apply_meta(meta(0, "Step pulse time", "microseconds"));
    let row = model.rows().next().expect("a row");
    assert_eq!(row.value.as_deref(), Some("10"));
    assert_eq!(row.label(), "Step pulse time");
    assert_eq!(row.unit(), "microseconds");

    // Metadata first, then value — same merged result.
    let mut model = SettingsModel::new();
    model.apply_meta(meta(0, "Step pulse time", "microseconds"));
    model.apply_value(value(0, "10"));
    let row = model.rows().next().expect("a row");
    assert_eq!(row.value.as_deref(), Some("10"));
    assert_eq!(row.label(), "Step pulse time");
  }

  #[test]
  fn label_falls_back_to_the_dollar_number_without_metadata() {
    // A value with no enumeration yet still reads sensibly as `$110`.
    let mut model = SettingsModel::new();
    model.apply_value(value(110, "500.000"));
    let row = model.rows().next().expect("a row");
    assert_eq!(row.label(), "$110");
    assert_eq!(row.unit(), "");
  }

  #[test]
  fn clear_wipes_every_row() {
    let mut model = SettingsModel::new();
    model.apply_value(value(0, "10"));
    model.apply_meta(meta(0, "Step pulse time", "microseconds"));
    model.clear();
    assert!(model.is_empty());
  }
}
