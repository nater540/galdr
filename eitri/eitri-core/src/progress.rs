//! Progress reporting and cooperative cancellation.
//!
//! Long-running CAM operations (isolating thousands of polygons, painting, drilling) must stay interruptible and
//! must not block the UI. These are the framework-agnostic seams every `eitri-cam` operation will accept: a
//! [`ProgressReporter`] the operation pushes events into, and a [`CancelToken`] it polls to bail out early.
//!
//! We use `std::sync::mpsc` for progress (rather than crossbeam) deliberately: it is dependency-free, its `Sender`
//! is `Send + Clone` which is all a fan-in progress channel needs, and the API here hides the channel type so a
//! later switch to crossbeam (for `select!` / multiple consumers) is a non-breaking change. Cancellation is an
//! `Arc<AtomicBool>` — cheap to clone across `rayon` worker threads and lock-free to poll in a hot loop.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};

use crate::error::{Error, Result};

/// An event emitted by a running operation. `done`/`total` counts are operation-defined units (polygons, hits,
/// passes) so a UI can render a determinate bar; `Message` carries a human-readable status line.
#[derive(Debug, Clone, PartialEq)]
pub enum ProgressEvent {
  /// The operation has begun; `label` names it for display.
  Started { label: String },
  /// Incremental progress: `done` of `total` units complete.
  Advanced { done: u64, total: u64 },
  /// A free-form status message.
  Message(String),
  /// The operation finished (successfully or by cancellation — inspect the operation's `Result` for which).
  Finished,
}

/// The producer half: an operation reports progress through this. Construct a connected pair with
/// [`ProgressReporter::channel`], or a no-op reporter with [`ProgressReporter::silent`] for tests and headless runs.
#[derive(Debug, Clone, Default)]
pub struct ProgressReporter {
  /// `None` for a silent reporter; sends are best-effort and a dropped receiver is not an error.
  tx: Option<Sender<ProgressEvent>>,
}

impl ProgressReporter {
  /// Create a reporter and its receiver. The caller (typically the UI) drains the receiver.
  pub fn channel() -> (ProgressReporter, Receiver<ProgressEvent>) {
    let (tx, rx) = channel();
    (ProgressReporter { tx: Some(tx) }, rx)
  }

  /// A reporter that discards every event — for callers that do not care about progress.
  pub fn silent() -> ProgressReporter {
    ProgressReporter { tx: None }
  }

  /// Emit an event. Best-effort: if the receiver has been dropped the event is silently discarded, because a
  /// consumer that stopped listening must never make the producing operation fail.
  pub fn emit(&self, event: ProgressEvent) {
    if let Some(tx) = &self.tx {
      let _ = tx.send(event);
    }
  }

  /// Convenience for the common case: report `done` of `total` units complete.
  pub fn advance(&self, done: u64, total: u64) {
    self.emit(ProgressEvent::Advanced { done, total });
  }
}

/// A cooperative cancellation flag. Clone it freely (clones share one flag); an operation polls
/// [`CancelToken::is_cancelled`] / [`CancelToken::check`] and any holder calls [`CancelToken::cancel`] to request
/// a stop. Cancellation is one-way — once set it stays set.
#[derive(Debug, Clone, Default)]
pub struct CancelToken {
  flag: Arc<AtomicBool>,
}

impl CancelToken {
  /// A fresh, not-yet-cancelled token.
  pub fn new() -> CancelToken {
    CancelToken::default()
  }

  /// Request cancellation. Idempotent.
  pub fn cancel(&self) {
    self.flag.store(true, Ordering::Relaxed);
  }

  /// Whether cancellation has been requested.
  pub fn is_cancelled(&self) -> bool {
    self.flag.load(Ordering::Relaxed)
  }

  /// Return `Err(Error::Cancelled)` if cancellation was requested, else `Ok(())`. Call this at loop boundaries so
  /// an operation unwinds cleanly through `?` the moment it is cancelled.
  pub fn check(&self) -> Result<()> {
    if self.is_cancelled() { Err(Error::Cancelled) } else { Ok(()) }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn reporter_delivers_events_in_order() {
    let (reporter, rx) = ProgressReporter::channel();
    reporter.emit(ProgressEvent::Started { label: "isolate".into() });
    reporter.advance(1, 4);
    reporter.advance(4, 4);
    reporter.emit(ProgressEvent::Finished);
    // `try_iter` drains what has been buffered without blocking; a blocking `iter` would deadlock here because
    // `reporter` still holds a live `Sender`, so the channel never signals end-of-stream while it is in scope.
    let got: Vec<ProgressEvent> = rx.try_iter().collect();
    assert_eq!(got, vec![
      ProgressEvent::Started { label: "isolate".into() },
      ProgressEvent::Advanced { done: 1, total: 4 },
      ProgressEvent::Advanced { done: 4, total: 4 },
      ProgressEvent::Finished,
    ]);
  }

  #[test]
  fn silent_reporter_never_panics_without_a_receiver() {
    let reporter = ProgressReporter::silent();
    reporter.emit(ProgressEvent::Started { label: "x".into() });
    reporter.advance(1, 1);
  }

  #[test]
  fn dropped_receiver_does_not_fail_the_producer() {
    let (reporter, rx) = ProgressReporter::channel();
    drop(rx);
    // Emitting after the consumer is gone must be a no-op, not a panic or error.
    reporter.advance(1, 2);
  }

  #[test]
  fn cancel_token_is_shared_across_clones() {
    let token = CancelToken::new();
    let worker = token.clone();
    assert!(!worker.is_cancelled());
    assert!(worker.check().is_ok());
    token.cancel();
    assert!(worker.is_cancelled());
    assert!(matches!(worker.check(), Err(Error::Cancelled)));
  }
}
