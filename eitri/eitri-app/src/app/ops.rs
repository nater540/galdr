//! The off-thread operation runner: long engine commands (parses, CAM ops, project loads) run on a worker
//! thread so the UI never blocks, with progress streamed over [`ProgressReporter::channel`] and cancellation
//! via a fresh [`CancelToken`] per run.
//!
//! The threading model is ownership handoff, not locking: the whole [`Session`] MOVES into the worker for the
//! duration of one [`OpRequest`] (its commands take `&mut self`, so a lock would serialize the UI anyway) and
//! comes back inside the [`OpOutcome`]. While it is away the shell renders from the snapshot state in
//! [`super::view_state`] and disables every session-touching control — enforced by the shell holding a
//! [`SessionSlot`] that is either `Home(Session)` or `Away(RunningOp)`, never both, so "use the session while
//! it is on the worker" is unrepresentable.
//!
//! Everything here is egui-free and unit-tested against real engine fixtures.

use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};

use eitri_core::{CancelToken, ProgressEvent, ProgressReporter};
use eitri_gcode::{DrillJob, IsolationJob};
use eitri_project::{DrillSpec, IsolationSpec, ObjectId};
use eitri_script::{ScriptError, Session};

/// One engine command the worker runs. File contents arrive as strings (the shell reads the file before
/// spawning, so a read error is reported immediately rather than from a worker).
#[derive(Debug, Clone)]
pub enum OpRequest {
  /// Open Gerber source as a new object.
  OpenGerber {
    /// The display name (the file stem).
    name: String,
    /// The raw Gerber text.
    source: String,
  },
  /// Open Excellon source as a new object.
  OpenExcellon {
    /// The display name.
    name: String,
    /// The raw Excellon text.
    source: String,
  },
  /// Import SVG source as a geometry object.
  ImportSvg {
    /// The display name.
    name: String,
    /// The raw SVG text.
    source: String,
  },
  /// Import DXF source as a geometry object.
  ImportDxf {
    /// The display name.
    name: String,
    /// The raw DXF text.
    source: String,
  },
  /// Import existing G-code as a geometry object (recovered cut polylines).
  ImportGcode {
    /// The display name.
    name: String,
    /// The raw G-code text.
    source: String,
  },
  /// Isolation-route a copper/geometry source into a CNC job.
  Isolate {
    /// The source object.
    source: ObjectId,
    /// The isolation parameters from the panel drafts.
    spec: IsolationSpec,
    /// The emission parameters (depths, feeds, spindle).
    job: IsolationJob,
  },
  /// Plan drilling for an Excellon source and emit the program.
  Drill {
    /// The source object.
    source: ObjectId,
    /// The drilling parameters from the panel drafts.
    spec: DrillSpec,
    /// The emission parameters (travel height, spindle).
    job: DrillJob,
  },
  /// Load a project from its JSON, REPLACING the session on success (the old one is discarded).
  LoadProject {
    /// The project file's JSON body.
    json: String,
  },
}

impl OpRequest {
  /// The i18n key of this operation's display label (`tr!`d by the shell at emit time, so the log follows the
  /// active locale).
  pub fn label_key(&self) -> &'static str {
    match self {
      OpRequest::OpenGerber { .. } => "op-open-gerber",
      OpRequest::OpenExcellon { .. } => "op-open-excellon",
      OpRequest::ImportSvg { .. } => "op-import-svg",
      OpRequest::ImportDxf { .. } => "op-import-dxf",
      OpRequest::ImportGcode { .. } => "op-import-gcode",
      OpRequest::Isolate { .. } => "op-isolate",
      OpRequest::Drill { .. } => "op-drill",
      OpRequest::LoadProject { .. } => "op-load-project",
    }
  }
}

/// What a successful request produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpOutput {
  /// A new object was committed (open/import/CAM ops); the shell selects it.
  Object(ObjectId),
  /// The session was replaced by a loaded project.
  ProjectLoaded,
}

/// The worker's result: the session comes home (the loaded one for a successful `LoadProject`, the original
/// otherwise — including on failure, so nothing is ever lost) plus the command's outcome.
pub struct OpOutcome {
  /// The session, back from the worker.
  pub session: Session,
  /// What happened.
  pub result: Result<OpOutput, ScriptError>,
}

/// A handle to one in-flight request: the cancel button's token, the progress drain, and the outcome slot.
pub struct RunningOp {
  label_key: &'static str,
  cancel: CancelToken,
  progress_rx: Receiver<ProgressEvent>,
  outcome_rx: Receiver<OpOutcome>,
}

impl RunningOp {
  /// The i18n key of the running operation's label.
  pub fn label_key(&self) -> &'static str {
    self.label_key
  }

  /// Request cooperative cancellation; the worker bails at its next check and the outcome arrives as
  /// `Err(Cancelled)`.
  pub fn cancel(&self) {
    self.cancel.cancel();
  }

  /// Drain every progress event buffered since the last frame (non-blocking).
  pub fn drain_progress(&self) -> Vec<ProgressEvent> {
    self.progress_rx.try_iter().collect()
  }

  /// Take the outcome if the worker has finished (non-blocking). `None` while it is still running. A worker
  /// that panicked (channel disconnected without a send) also yields `None` — the shell's [`SessionSlot`]
  /// handles that as a permanently-away session and reports it, rather than blocking forever.
  pub fn try_outcome(&self) -> Option<OpOutcome> {
    self.outcome_rx.try_recv().ok()
  }

  /// Whether the worker died without delivering an outcome (a panic). Distinct from "still running".
  pub fn is_poisoned(&self) -> bool {
    matches!(self.outcome_rx.try_recv(), Err(TryRecvError::Disconnected))
  }
}

/// Move `session` into a worker thread and run `request` on it. A FRESH cancel token and a fresh progress
/// channel are installed per run (cancellation is one-way on a token, so reuse would poison later runs).
pub fn spawn(mut session: Session, request: OpRequest) -> RunningOp {
  let (progress, progress_rx) = ProgressReporter::channel();
  let cancel = CancelToken::new();
  session.set_progress(progress);
  session.set_cancel(cancel.clone());
  let label_key = request.label_key();
  let (outcome_tx, outcome_rx) = channel();
  std::thread::spawn(move || run_request(session, request, outcome_tx));
  RunningOp { label_key, cancel, progress_rx, outcome_rx }
}

/// The worker body: run the request, restore a silent reporter (the UI's receiver dies with the [`RunningOp`]),
/// and send the session home. A send failure means the UI dropped the handle — the session is then discarded,
/// which is the only sane teardown.
fn run_request(mut session: Session, request: OpRequest, outcome_tx: Sender<OpOutcome>) {
  let result = match request {
    OpRequest::OpenGerber { name, source } => session.open_gerber_str(name, source).map(OpOutput::Object),
    OpRequest::OpenExcellon { name, source } => session.open_excellon_str(name, source).map(OpOutput::Object),
    OpRequest::ImportSvg { name, source } => session.import_svg_str(name, source).map(OpOutput::Object),
    OpRequest::ImportDxf { name, source } => session.import_dxf_str(name, source).map(OpOutput::Object),
    OpRequest::ImportGcode { name, source } => session.import_gcode_str(name, source).map(OpOutput::Object),
    OpRequest::Isolate { source, spec, job } => session.isolate(source, spec, job).map(OpOutput::Object),
    OpRequest::Drill { source, spec, job } => session.drill(source, spec, job).map(OpOutput::Object),
    OpRequest::LoadProject { json } => match Session::load_project_str(&json) {
      // The loaded session replaces the working one; the old session is dropped here, exactly like FlatCAM's
      // open-project semantics. Failure keeps the original untouched.
      Ok(loaded) => {
        session = loaded;
        Ok(OpOutput::ProjectLoaded)
      }
      Err(err) => Err(err),
    },
  };
  session.set_progress(ProgressReporter::silent());
  let _ = outcome_tx.send(OpOutcome { session, result });
}

/// The shell's session holder: exactly one of "the session is home" or "a request is in flight". Methods keep
/// the invariant; the illegal state (both, neither) is unrepresentable.
pub enum SessionSlot {
  /// The session is on the UI thread, available for immediate commands (select, delete, undo, export).
  Home(Session),
  /// The session is on a worker; only progress/cancel/outcome polling is possible.
  Away(RunningOp),
}

impl SessionSlot {
  /// Borrow the session if it is home.
  pub fn session(&self) -> Option<&Session> {
    match self {
      SessionSlot::Home(session) => Some(session),
      SessionSlot::Away(_) => None,
    }
  }

  /// Mutably borrow the session if it is home.
  pub fn session_mut(&mut self) -> Option<&mut Session> {
    match self {
      SessionSlot::Home(session) => Some(session),
      SessionSlot::Away(_) => None,
    }
  }

  /// The in-flight handle, if a request is running.
  pub fn running(&self) -> Option<&RunningOp> {
    match self {
      SessionSlot::Home(_) => None,
      SessionSlot::Away(op) => Some(op),
    }
  }

  /// Launch `request` if the session is home, replacing this slot with the in-flight handle. Returns whether
  /// the launch happened (`false` = already busy; the caller's controls should have been disabled).
  pub fn launch(&mut self, request: OpRequest) -> bool {
    // Two-step replace: take the session out only when home, then install the running handle.
    match std::mem::replace(self, SessionSlot::Away(dummy_running())) {
      SessionSlot::Home(session) => {
        *self = SessionSlot::Away(spawn(session, request));
        true
      }
      away => {
        *self = away;
        false
      }
    }
  }

  /// Poll the worker: if the outcome has arrived, bring the session home (installing it back into this slot)
  /// and return the command's result, plus any progress events still buffered when it landed — the handle (and
  /// its progress receiver) is dropped here, so those trailing events would otherwise be silently lost and a
  /// final `Advanced(total, total)` would never reach the bar. `None` while the worker is still running.
  pub fn poll(&mut self) -> Option<(Vec<ProgressEvent>, Result<OpOutput, ScriptError>)> {
    let (trailing, OpOutcome { session, result }) = match self {
      SessionSlot::Home(_) => return None,
      SessionSlot::Away(op) => {
        let outcome = op.try_outcome()?;
        // The worker has exited, so everything it will ever send is already buffered: this drain is complete.
        (op.drain_progress(), outcome)
      }
    };
    *self = SessionSlot::Home(session);
    Some((trailing, result))
  }
}

/// Whether a polled result was a cooperative cancellation — logged as a warning, not an error.
pub fn was_cancelled(result: &Result<OpOutput, ScriptError>) -> bool {
  matches!(result, Err(ScriptError::Engine(eitri_core::Error::Cancelled)))
}

/// A placeholder `RunningOp` used only inside [`SessionSlot::launch`]'s two-step replace; never observable.
fn dummy_running() -> RunningOp {
  let (_progress, progress_rx) = ProgressReporter::channel();
  let (_tx, outcome_rx) = channel();
  RunningOp { label_key: "op-running", cancel: CancelToken::new(), progress_rx, outcome_rx }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_project::{DirectionSpec, ObjectKind};
  use std::time::{Duration, Instant};

  const GERBER: &str = include_str!("../../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
  const EXCELLON: &str = include_str!("../../../fixtures/synthetic/excellon/metric_leading.drl");

  fn iso_spec() -> IsolationSpec {
    IsolationSpec { tool_diameter: 0.2, passes: 1, overlap: 0.0, combine: false, direction: DirectionSpec::Climb }
  }

  /// Poll a slot until the worker delivers, with a generous budget so a slow CI machine cannot flake.
  fn wait_outcome(slot: &mut SessionSlot) -> Result<OpOutput, ScriptError> {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
      if let Some((_trailing, result)) = slot.poll() {
        return result;
      }
      assert!(Instant::now() < deadline, "the worker never delivered an outcome");
      std::thread::sleep(Duration::from_millis(2));
    }
  }

  #[test]
  fn an_open_gerber_request_runs_off_thread_and_brings_the_session_home() {
    let mut slot = SessionSlot::Home(Session::new("fixture-board"));
    assert!(slot.launch(OpRequest::OpenGerber { name: "fixture-top".into(), source: GERBER.into() }));
    assert!(slot.session().is_none(), "the session is away while the worker runs");
    assert!(!slot.launch(OpRequest::OpenGerber { name: "again".into(), source: GERBER.into() }),
      "a second launch while busy must be refused, not queued");

    let result = wait_outcome(&mut slot);
    let id = match result.expect("the fixture gerber opens") {
      OpOutput::Object(id) => id,
      other => panic!("expected an object, got {other:?}"),
    };
    let session = slot.session().expect("the session is home after poll");
    assert_eq!(session.kind(id).unwrap(), ObjectKind::Gerber);
    assert_eq!(session.len(), 1);
  }

  #[test]
  fn an_isolation_request_streams_progress_and_commits_a_job() {
    let mut session = Session::new("fixture-board");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let mut slot = SessionSlot::Home(session);
    assert!(slot.launch(OpRequest::Isolate { source: gerber, spec: iso_spec(), job: IsolationJob::default() }));

    // Drain progress while waiting: the engine emits Started/Advanced/Finished through the channel. Events
    // are drained BEFORE each poll so the drain sees everything the worker sent before its outcome landed.
    let mut events = Vec::new();
    let result = loop {
      if let SessionSlot::Away(op) = &slot {
        events.extend(op.drain_progress());
      }
      if let Some((trailing, result)) = slot.poll() {
        // The trailing drain closes the race: events buffered between our drain and the outcome landing are
        // handed over by `poll` rather than dying with the receiver.
        events.extend(trailing);
        break result;
      }
      std::thread::sleep(Duration::from_millis(1));
    };
    assert!(
      events.iter().any(|e| matches!(e, ProgressEvent::Started { .. })),
      "the isolation op must announce itself over the progress channel: {events:?}",
    );
    let id = match result.expect("isolation succeeds") {
      OpOutput::Object(id) => id,
      other => panic!("expected a job object, got {other:?}"),
    };
    assert_eq!(slot.session().unwrap().kind(id).unwrap(), ObjectKind::CncJob);
  }

  #[test]
  fn cancelling_a_running_op_surfaces_as_cancelled_and_the_next_op_still_runs() {
    let mut session = Session::new("fixture-board");
    let gerber = session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let mut slot = SessionSlot::Home(session);
    assert!(slot.launch(OpRequest::Isolate { source: gerber, spec: iso_spec(), job: IsolationJob::default() }));
    // Cancel immediately — the op may still win the race and complete; both endings are legitimate, but a
    // CANCELLED ending must be classified as a cancellation, and either way the session must come home usable.
    if let SessionSlot::Away(op) = &slot {
      op.cancel();
    }
    let result = wait_outcome(&mut slot);
    if result.is_err() {
      assert!(was_cancelled(&result), "the only acceptable failure here is Cancelled: {:?}",
        result.as_ref().err().map(|e| e.to_string()));
    }

    // The follow-up op must run cleanly: `spawn` installs a FRESH token, so the poisoned one is gone.
    assert!(slot.launch(OpRequest::OpenExcellon { name: "fixture-drills".into(), source: EXCELLON.into() }));
    let second = wait_outcome(&mut slot);
    let id = match second.expect("the next op must not inherit the cancelled token") {
      OpOutput::Object(id) => id,
      other => panic!("expected an object, got {other:?}"),
    };
    assert_eq!(slot.session().unwrap().kind(id).unwrap(), ObjectKind::Excellon);
  }

  #[test]
  fn a_failed_load_project_keeps_the_original_session() {
    let mut session = Session::new("keeper");
    session.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let mut slot = SessionSlot::Home(session);
    assert!(slot.launch(OpRequest::LoadProject { json: "this is not a project".into() }));
    let result = wait_outcome(&mut slot);
    assert!(result.is_err(), "garbage JSON must not load");
    let session = slot.session().expect("home again");
    assert_eq!(session.name(), "keeper", "a failed load must keep the original session");
    assert_eq!(session.len(), 1, "with its objects intact");
  }

  #[test]
  fn a_successful_load_project_replaces_the_session() {
    let mut donor = Session::new("donor");
    donor.open_gerber_str("fixture-top", GERBER).expect("fixture opens");
    let json = donor.save_project().expect("serialises");

    let mut slot = SessionSlot::Home(Session::new("original"));
    assert!(slot.launch(OpRequest::LoadProject { json }));
    let result = wait_outcome(&mut slot);
    assert_eq!(result.expect("the donor project loads"), OpOutput::ProjectLoaded);
    let session = slot.session().expect("home again");
    assert_eq!(session.name(), "donor", "the loaded project replaces the session");
    assert_eq!(session.len(), 1);
  }

  #[test]
  fn every_request_maps_to_a_distinct_label_key() {
    let keys = [
      OpRequest::OpenGerber { name: String::new(), source: String::new() }.label_key(),
      OpRequest::OpenExcellon { name: String::new(), source: String::new() }.label_key(),
      OpRequest::ImportSvg { name: String::new(), source: String::new() }.label_key(),
      OpRequest::ImportDxf { name: String::new(), source: String::new() }.label_key(),
      OpRequest::ImportGcode { name: String::new(), source: String::new() }.label_key(),
      OpRequest::Isolate { source: ObjectId(0), spec: iso_spec(), job: IsolationJob::default() }.label_key(),
      OpRequest::Drill {
        source: ObjectId(0),
        spec: DrillSpec { depth: -1.0, feed: 100.0, retract: 2.0, peck: None, dwell: None },
        job: DrillJob::default(),
      }
      .label_key(),
      OpRequest::LoadProject { json: String::new() }.label_key(),
    ];
    let mut unique: Vec<&str> = keys.to_vec();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(unique.len(), keys.len(), "labels must be distinguishable in the log");
  }
}
