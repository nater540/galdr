//! Pure, egui-free view state: everything the views render that is not the live widget scratch (drafts,
//! scroll positions) owned by `views::UiState`. Kept engine-free too — the shell folds engine outcomes into
//! this, so the state transitions are unit-tested without a window or a session.
//!
//! The load-bearing piece is the op lifecycle ([`OpView`]): while a command runs off-thread the session is
//! *away* (moved into the worker), so the views render from this snapshot state — the tree rows, the log, the
//! progress — and every session-touching control is disabled. The reducer functions here are what the shell
//! calls as progress events and outcomes drain in.

use eitri_project::{ObjectId, ObjectKind};

/// A capped number of log lines kept in memory (the dock shows the tail; an unbounded log would grow forever
/// on a long session).
pub const LOG_CAPACITY: usize = 500;

/// The severity of a log line, mapped to a status colour by the views (always alongside the text, never colour
/// alone).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogKind {
  /// Neutral information (opens, progress messages).
  Info,
  /// A completed operation.
  Ok,
  /// A cancelled operation or a soft notice.
  Warn,
  /// A failed operation or an unusable file.
  Error,
}

/// One line in the log dock.
#[derive(Debug, Clone, PartialEq)]
pub struct LogLine {
  /// Severity, for the coloured marker.
  pub kind: LogKind,
  /// The (already localized) text.
  pub text: String,
}

/// One row of the project tree — a snapshot of an object's display facts, rebuilt from the session whenever
/// the collection changes. The tree renders from this even while the session is away on a worker thread.
#[derive(Debug, Clone, PartialEq)]
pub struct TreeRow {
  /// The object's stable id.
  pub id: ObjectId,
  /// Its display name.
  pub name: String,
  /// Its kind, for the icon and the parameter panel dispatch.
  pub kind: ObjectKind,
  /// Whether the object is shown on the canvas (the tree row's eye toggle).
  pub visible: bool,
}

/// The in-flight-operation view: what the progress UI renders and what gates the session-touching controls.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum OpView {
  /// No operation running; the session is home and controls are live.
  #[default]
  Idle,
  /// A command is running on the worker; `done`/`total` are the engine's progress units (0/0 until the first
  /// `Advanced` event → the views render an indeterminate bar).
  Running {
    /// The localized operation label.
    label: String,
    /// Units complete.
    done: u64,
    /// Total units (0 = unknown).
    total: u64,
  },
}

/// The pure view state the shell owns and the views read.
#[derive(Debug, Clone, Default)]
pub struct ViewState {
  /// The tree snapshot, in display order.
  pub tree: Vec<TreeRow>,
  /// The selected object, if any (cleared automatically when the object disappears).
  pub selected: Option<ObjectId>,
  /// The log tail, capped at [`LOG_CAPACITY`].
  pub log: Vec<LogLine>,
  /// The op lifecycle.
  pub op: OpView,
  /// Whether the engine history has an edit to undo (snapshotted with the tree — the session may be away).
  pub can_undo: bool,
  /// Whether the engine history has an edit to redo.
  pub can_redo: bool,
}

impl ViewState {
  /// Whether a command is currently running (the session is away; session-touching controls must be disabled).
  pub fn busy(&self) -> bool {
    matches!(self.op, OpView::Running { .. })
  }

  /// Append a log line, evicting the oldest beyond [`LOG_CAPACITY`].
  pub fn log_line(&mut self, kind: LogKind, text: impl Into<String>) {
    self.log.push(LogLine { kind, text: text.into() });
    if self.log.len() > LOG_CAPACITY {
      let excess = self.log.len() - LOG_CAPACITY;
      self.log.drain(..excess);
    }
  }

  /// Begin the running-op state with a localized label. Progress starts unknown (0/0 → indeterminate bar).
  pub fn op_started(&mut self, label: impl Into<String>) {
    self.op = OpView::Running { label: label.into(), done: 0, total: 0 };
  }

  /// Fold an engine progress advance into the running state. Ignored when idle (a late event from a finished
  /// op must not resurrect the bar).
  pub fn op_advanced(&mut self, new_done: u64, new_total: u64) {
    if let OpView::Running { done, total, .. } = &mut self.op {
      *done = new_done;
      *total = new_total;
    }
  }

  /// End the running-op state (the shell logs the outcome separately).
  pub fn op_finished(&mut self) {
    self.op = OpView::Idle;
  }

  /// The running op's progress as a `0..=1` fraction, or `None` while the total is unknown (indeterminate).
  pub fn op_fraction(&self) -> Option<f32> {
    match &self.op {
      OpView::Running { done, total, .. } if *total > 0 => Some((*done as f32 / *total as f32).clamp(0.0, 1.0)),
      _ => None,
    }
  }

  /// Replace the tree snapshot and drop a selection that no longer resolves — the reducer the shell calls
  /// after every collection change (op done, undo/redo, delete).
  pub fn set_tree(&mut self, rows: Vec<TreeRow>, can_undo: bool, can_redo: bool) {
    if let Some(selected) = self.selected
      && !rows.iter().any(|row| row.id == selected)
    {
      self.selected = None;
    }
    self.tree = rows;
    self.can_undo = can_undo;
    self.can_redo = can_redo;
  }

  /// The selected row, if the selection still resolves.
  pub fn selected_row(&self) -> Option<&TreeRow> {
    let id = self.selected?;
    self.tree.iter().find(|row| row.id == id)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn row(id: u64, kind: ObjectKind) -> TreeRow {
    TreeRow { id: ObjectId(id), name: format!("fixture-{id}"), kind, visible: true }
  }

  #[test]
  fn the_op_lifecycle_runs_started_advanced_finished() {
    let mut view = ViewState::default();
    assert!(!view.busy());
    assert_eq!(view.op_fraction(), None);

    view.op_started("Isolation routing");
    assert!(view.busy(), "a started op gates the session-touching controls");
    assert_eq!(view.op_fraction(), None, "0/0 renders an indeterminate bar, not a full one");

    view.op_advanced(1, 4);
    assert_eq!(view.op_fraction(), Some(0.25));
    view.op_advanced(4, 4);
    assert_eq!(view.op_fraction(), Some(1.0));

    view.op_finished();
    assert!(!view.busy());
    assert_eq!(view.op_fraction(), None);
  }

  #[test]
  fn a_late_progress_event_after_finish_does_not_resurrect_the_bar() {
    // The worker's progress channel may still hold buffered events when the outcome lands; draining them after
    // `op_finished` must be a no-op, or a finished op would flash "running" again.
    let mut view = ViewState::default();
    view.op_started("x");
    view.op_finished();
    view.op_advanced(3, 9);
    assert!(!view.busy());
    assert_eq!(view.op_fraction(), None);
  }

  #[test]
  fn the_log_caps_at_capacity_evicting_the_oldest() {
    let mut view = ViewState::default();
    for i in 0..(LOG_CAPACITY + 10) {
      view.log_line(LogKind::Info, format!("line {i}"));
    }
    assert_eq!(view.log.len(), LOG_CAPACITY);
    assert_eq!(view.log[0].text, "line 10", "the oldest lines are evicted, the tail is kept");
    assert_eq!(view.log.last().map(|l| l.text.as_str()), Some(&*format!("line {}", LOG_CAPACITY + 9)));
  }

  #[test]
  fn set_tree_drops_a_selection_that_no_longer_resolves_and_keeps_one_that_does() {
    let mut view = ViewState::default();
    view.set_tree(vec![row(1, ObjectKind::Gerber), row(2, ObjectKind::CncJob)], true, false);
    view.selected = Some(ObjectId(2));
    assert_eq!(view.selected_row().map(|r| r.id), Some(ObjectId(2)));
    assert!(view.can_undo && !view.can_redo);

    // The job is deleted (undoable): the refreshed tree no longer carries id 2 → the selection clears rather
    // than pointing at a ghost the parameter panel would then fail to resolve.
    view.set_tree(vec![row(1, ObjectKind::Gerber)], true, true);
    assert_eq!(view.selected, None, "a vanished selection must clear");
    assert_eq!(view.selected_row(), None);

    // A selection that still resolves survives a refresh.
    view.selected = Some(ObjectId(1));
    view.set_tree(vec![row(1, ObjectKind::Gerber), row(3, ObjectKind::Geometry)], false, false);
    assert_eq!(view.selected, Some(ObjectId(1)), "a still-present selection survives the refresh");
  }
}
