//! [`History`] — snapshot-based undo/redo over an [`ObjectCollection`].
//!
//! The model is deliberately snapshot-based rather than command-based: an edit clones the current collection onto a
//! bounded past stack, then mutates in place. Undo/redo just move whole snapshots between the past, present, and
//! future stacks. This trades a little memory for correctness and simplicity — there is no per-operation inverse to
//! get wrong — and the memory cost is kept low because the collection's heavy immutable payloads (embedded source,
//! rendered G-code) are behind `Arc`, so a snapshot clone bumps refcounts instead of copying kilobytes.

use crate::collection::ObjectCollection;

/// The default cap on retained undo snapshots. Beyond this the oldest history is dropped.
pub const DEFAULT_HISTORY_LIMIT: usize = 64;

/// An undo/redo history around a single [`ObjectCollection`].
#[derive(Debug, Clone)]
pub struct History {
  present: ObjectCollection,
  past: Vec<ObjectCollection>,
  future: Vec<ObjectCollection>,
  limit: usize,
}

impl History {
  /// Start a history from an initial collection state, using the default snapshot limit.
  pub fn new(initial: ObjectCollection) -> History {
    History::with_limit(initial, DEFAULT_HISTORY_LIMIT)
  }

  /// Start a history with an explicit snapshot limit (minimum 1).
  pub fn with_limit(initial: ObjectCollection, limit: usize) -> History {
    History { present: initial, past: Vec::new(), future: Vec::new(), limit: limit.max(1) }
  }

  /// The current collection state.
  pub fn current(&self) -> &ObjectCollection {
    &self.present
  }

  /// Apply an edit: snapshot the present onto the undo stack, run the mutation, and clear the redo stack (a new edit
  /// invalidates any redo future). The oldest snapshot is dropped once the limit is exceeded.
  pub fn edit<R>(&mut self, mutate: impl FnOnce(&mut ObjectCollection) -> R) -> R {
    self.past.push(self.present.clone());
    if self.past.len() > self.limit {
      self.past.remove(0);
    }
    self.future.clear();
    mutate(&mut self.present)
  }

  /// Mutate the present state WITHOUT taking a new undo snapshot. Used to coalesce a continuous gesture (dragging an
  /// object across the stock) into the single snapshot its first step already pushed, so the whole drag undoes at
  /// once rather than one pixel at a time. The redo future is left untouched (the gesture's first [`Self::edit`]
  /// already cleared it).
  pub fn amend<R>(&mut self, mutate: impl FnOnce(&mut ObjectCollection) -> R) -> R {
    mutate(&mut self.present)
  }

  /// Whether there is a prior state to undo to.
  pub fn can_undo(&self) -> bool {
    !self.past.is_empty()
  }

  /// Whether there is a redo state to move forward to.
  pub fn can_redo(&self) -> bool {
    !self.future.is_empty()
  }

  /// Undo the last edit, moving the present onto the redo stack. Returns `false` (a no-op) if there is nothing to undo.
  pub fn undo(&mut self) -> bool {
    match self.past.pop() {
      Some(previous) => {
        let current = std::mem::replace(&mut self.present, previous);
        self.future.push(current);
        true
      }
      None => false,
    }
  }

  /// Redo the last undone edit, moving the present back onto the undo stack. Returns `false` if there is nothing to redo.
  pub fn redo(&mut self) -> bool {
    match self.future.pop() {
      Some(next) => {
        let current = std::mem::replace(&mut self.present, next);
        self.past.push(current);
        true
      }
      None => false,
    }
  }
}
