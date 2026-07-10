//! [`History`] — snapshot-based undo/redo over the project *document*: the [`ObjectCollection`] **and** the
//! [`WorkSetup`] (datum / stock), snapshotted together so a datum change undoes in lockstep with the staleness it
//! flags on dependent jobs.
//!
//! The model is deliberately snapshot-based rather than command-based: an edit clones the current document onto a
//! bounded past stack, then mutates in place. Undo/redo just move whole snapshots between the past, present, and
//! future stacks. This trades a little memory for correctness and simplicity — there is no per-operation inverse to
//! get wrong — and the memory cost is kept low because the collection's heavy immutable payloads (embedded source,
//! rendered G-code) are behind `Arc`, so a snapshot clone bumps refcounts instead of copying kilobytes.

use crate::collection::ObjectCollection;
use crate::datum::WorkSetup;

/// The default cap on retained undo snapshots. Beyond this the oldest history is dropped.
pub const DEFAULT_HISTORY_LIMIT: usize = 64;

/// One undo snapshot: the whole document state at a point in time.
#[derive(Debug, Clone, Default)]
struct Snapshot {
  /// The objects and groups.
  collection: ObjectCollection,
  /// The datum / stock work-setup.
  setup: WorkSetup,
}

/// An undo/redo history around the project document ([`ObjectCollection`] + [`WorkSetup`]).
#[derive(Debug, Clone)]
pub struct History {
  present: Snapshot,
  past: Vec<Snapshot>,
  future: Vec<Snapshot>,
  limit: usize,
}

impl History {
  /// Start a history from an initial collection state (native-frame work-setup), using the default snapshot limit.
  pub fn new(initial: ObjectCollection) -> History {
    History::with_document(initial, WorkSetup::default(), DEFAULT_HISTORY_LIMIT)
  }

  /// Start a history with an explicit snapshot limit (minimum 1) and the native-frame work-setup.
  pub fn with_limit(initial: ObjectCollection, limit: usize) -> History {
    History::with_document(initial, WorkSetup::default(), limit)
  }

  /// Start a history from a full document state (collection + work-setup) — the loader's entry point, so a loaded
  /// project's datum is the present without a spurious opening undo entry.
  pub fn with_document(collection: ObjectCollection, setup: WorkSetup, limit: usize) -> History {
    History { present: Snapshot { collection, setup }, past: Vec::new(), future: Vec::new(), limit: limit.max(1) }
  }

  /// The current collection state.
  pub fn current(&self) -> &ObjectCollection {
    &self.present.collection
  }

  /// The current work-setup (datum / stock).
  pub fn setup(&self) -> &WorkSetup {
    &self.present.setup
  }

  /// Push the present onto the bounded undo stack and clear the redo future (a new edit invalidates any redo).
  fn snapshot(&mut self) {
    self.past.push(self.present.clone());
    if self.past.len() > self.limit {
      self.past.remove(0);
    }
    self.future.clear();
  }

  /// Apply an edit to the object collection: snapshot the present, run the mutation, clear the redo stack. The oldest
  /// snapshot is dropped once the limit is exceeded.
  pub fn edit<R>(&mut self, mutate: impl FnOnce(&mut ObjectCollection) -> R) -> R {
    self.snapshot();
    mutate(&mut self.present.collection)
  }

  /// Mutate the present collection WITHOUT taking a new undo snapshot. Used to coalesce a continuation of a gesture
  /// (the frames of an object drag, or a layer auto-joining its import group) into the single snapshot the gesture's
  /// first step already pushed, so the whole gesture undoes at once. The redo future is left untouched (the first
  /// [`Self::edit`] already cleared it).
  pub fn amend<R>(&mut self, mutate: impl FnOnce(&mut ObjectCollection) -> R) -> R {
    // Amending before any snapshot exists would fold the mutation into the initial state with no way to undo it — a
    // misuse (every legitimate caller amends into a snapshot its gesture's first `edit()` already pushed). Flag it
    // loudly in debug; in release, degrade to a real snapshot so the mutation stays undoable rather than silently
    // corrupting undo.
    debug_assert!(self.can_undo(), "History::amend requires a prior edit() in the same gesture to coalesce into");
    if !self.can_undo() {
      self.snapshot();
    }
    mutate(&mut self.present.collection)
  }

  /// Commit a work-setup (datum / stock) change, mutating the setup **and** the collection (to flag dependent jobs
  /// stale) in one snapshot. When `coalesce` is set the change folds into the current undo entry instead of pushing
  /// its own — the caller asserts this commit continues an in-progress gesture (a stock-spinner drag re-commits every
  /// frame, and the open-time auto-fit folds into the import) — falling back to a fresh snapshot if there is no entry
  /// to coalesce into. `coalesce = false` always starts a fresh entry, so discrete setup actions stay independently
  /// undoable.
  pub fn commit_setup<R>(&mut self, coalesce: bool, mutate: impl FnOnce(&mut ObjectCollection, &mut WorkSetup) -> R) -> R {
    if !(coalesce && self.can_undo()) {
      self.snapshot();
    }
    mutate(&mut self.present.collection, &mut self.present.setup)
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

#[cfg(test)]
mod tests {
  use super::*;
  use crate::datum::Stock;
  use crate::id::ObjectId;

  fn a_stock(thickness: f64) -> WorkSetup {
    WorkSetup::from_stock(Some(Stock::fit((0.0, 0.0, 10.0, 10.0), thickness)))
  }

  #[test]
  fn commit_setup_snapshots_the_datum_and_undoes_it() {
    let mut history = History::new(ObjectCollection::new());
    assert_eq!(*history.setup(), WorkSetup::default(), "a fresh history is in the native frame");
    history.commit_setup(false, |_c, setup| *setup = a_stock(1.6));
    assert_eq!(history.setup().stock.map(|s| s.thickness), Some(1.6), "the setup is committed");
    assert!(history.undo(), "the setup change is undoable");
    assert_eq!(*history.setup(), WorkSetup::default(), "undo reverts the datum");
  }

  #[test]
  fn coalescing_setup_commits_fold_into_one_entry_but_discrete_ones_stay_separate() {
    // A drag continuation (coalesce = true) folds into the current entry; the first frame (coalesce = false) started
    // it. The whole run undoes at once.
    let mut history = History::new(ObjectCollection::new());
    history.commit_setup(false, |_c, setup| *setup = a_stock(1.0)); // drag start: fresh entry
    history.commit_setup(true, |_c, setup| *setup = a_stock(2.0)); // continuation: folds in
    history.commit_setup(true, |_c, setup| *setup = a_stock(3.0)); // continuation: folds in
    assert_eq!(history.setup().stock.map(|s| s.thickness), Some(3.0), "the last drag value is in force");
    assert!(history.undo(), "one undo reverts the whole coalesced run");
    assert_eq!(*history.setup(), WorkSetup::default(), "back to native in a single step");

    // Two discrete commits (coalesce = false) are independent undo entries.
    let mut history = History::new(ObjectCollection::new());
    history.commit_setup(false, |_c, setup| *setup = a_stock(1.0));
    history.commit_setup(false, |_c, setup| *setup = a_stock(2.0));
    assert!(history.undo(), "undo the second discrete commit");
    assert_eq!(history.setup().stock.map(|s| s.thickness), Some(1.0), "reverts only the later change");
  }

  #[test]
  fn a_coalesce_with_no_prior_entry_falls_back_to_a_fresh_snapshot() {
    // Defensive: a coalescing commit with an empty past must still be undoable, not fold into the base state.
    let mut history = History::new(ObjectCollection::new());
    history.commit_setup(true, |_c, setup| *setup = a_stock(1.0));
    assert!(history.undo(), "the fallback snapshot makes it undoable");
    assert_eq!(*history.setup(), WorkSetup::default());
  }

  #[test]
  fn amend_after_an_edit_coalesces_without_a_new_snapshot() {
    let mut history = History::new(ObjectCollection::new());
    let _ = history.edit(|c| c.create_group("g".to_string()));
    assert!(history.can_undo(), "the edit pushed one snapshot");
    // The amend folds into that snapshot — still exactly one undo step for the whole gesture.
    history.amend(|c| {
      let _ = c.add_to_group("g", ObjectId(1));
    });
    history.undo();
    assert!(!history.can_undo(), "the edit + amend undo together as one step");
  }

  #[test]
  #[should_panic(expected = "requires a prior edit")]
  fn amend_without_a_prior_edit_panics_in_debug() {
    // A first-call amend has no snapshot to coalesce into — the guard catches the misuse rather than silently
    // making an un-undoable mutation (docs follow-up minor M3).
    let mut history = History::new(ObjectCollection::new());
    history.amend(|c| {
      let _ = c.create_group("orphan".to_string());
    });
  }
}
