//! End-to-end tests of the Rhai binding: a real script drives the whole pipeline, a misused command surfaces as a
//! script error rather than a panic, and a cancelled script stops.
//!
//! The whole file is gated on the `scripting` feature — without Rhai there is no binding to exercise.
#![cfg(feature = "scripting")]

use eitri_gcode::check_grbl_conformance;
use eitri_script::bindings::{ScriptSession, new_engine};
use eitri_script::{Session, eval};
use rhai::{EvalAltResult, Scope};

const GERBER: &str = include_str!("../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
const EXCELLON: &str = include_str!("../../fixtures/synthetic/excellon/metric_leading.drl");

/// A script that opens a Gerber from source, isolates it, and returns the rendered G-code — the canonical pipeline.
const ISOLATE_SCRIPT: &str = r#"
  let g = cam.open_gerber_str("top", src);
  let spec = isolation_spec(0.2, 1, 0.0, false, "climb");
  let job = cut_job(0.1, 0.1, 120.0, 60.0, 2.0, 10000.0);
  let iso = cam.isolate(g, spec, job);
  cam.write_gcode(iso)
"#;

#[test]
fn script_isolates_a_gerber_to_conformant_gcode() {
  let session = ScriptSession::new(Session::new("board"));
  let handle = session.clone();

  // Drive it through the lower-level engine so the fixture source can be injected as a scope constant.
  let engine = new_engine();
  let mut scope = Scope::new();
  scope.push("cam", session);
  scope.push_constant("src", GERBER.to_string());
  let gcode: String = engine.eval_with_scope(&mut scope, ISOLATE_SCRIPT).expect("script runs end to end");

  assert!(gcode.contains("G1"), "the script produced cutting moves");
  assert!(check_grbl_conformance(&gcode).is_empty(), "script output is grbl-conformant");
  // The session was mutated through the shared handle: the Gerber plus the CNC job.
  assert_eq!(handle.borrow().len(), 2);
}

#[test]
fn a_wrong_kind_command_surfaces_as_a_script_error_not_a_panic() {
  let session = ScriptSession::new(Session::new("board"));
  let engine = new_engine();
  let mut scope = Scope::new();
  scope.push("cam", session);
  scope.push_constant("src", EXCELLON.to_string());

  // Isolating a drill file is a wrong-kind error; it must come back as a catchable Rhai error.
  let script = r#"
    let d = cam.open_excellon_str("drills", src);
    let spec = isolation_spec(0.2, 1, 0.0, false, "climb");
    let job = cut_job(0.1, 0.1, 120.0, 60.0, 2.0, 10000.0);
    cam.isolate(d, spec, job)
  "#;
  let result = engine.eval_with_scope::<rhai::Dynamic>(&mut scope, script);
  let err = result.expect_err("wrong-kind command must fail");
  let message = err.to_string();
  assert!(message.contains("Excellon"), "the script error explains the kind mismatch: {message}");
}

#[test]
fn an_unknown_milling_direction_is_a_clean_script_error() {
  let session = ScriptSession::new(Session::new("board"));
  let err = eval(&session, r#"isolation_spec(0.2, 1, 0.0, false, "sideways")"#).expect_err("bad direction fails");
  assert!(err.to_string().contains("milling direction"), "names the offending argument: {err}");
}

#[test]
fn a_cancelled_token_aborts_a_running_script() {
  let session = ScriptSession::new(Session::new("board"));
  // Trip the token before running: the operation callback then aborts the otherwise-infinite loop at once, so the
  // test is fully deterministic and cannot hang.
  session.borrow().cancel_token().cancel();

  let result = eval(&session, "let i = 0; while true { i += 1; } i");
  let err = result.expect_err("a cancelled script must not run to completion");
  assert!(
    matches!(*err, EvalAltResult::ErrorTerminated(..)),
    "cancellation aborts via the operation callback, got {err:?}"
  );
}

#[test]
fn a_full_two_layer_script_saves_a_project() {
  // Exercises open → isolate → object management → save all from one script, reading the JSON back out.
  let session = ScriptSession::new(Session::new("board"));
  let engine = new_engine();
  let mut scope = Scope::new();
  scope.push("cam", session.clone());
  scope.push_constant("src", GERBER.to_string());

  let script = r#"
    let g = cam.open_gerber_str("top", src);
    let spec = isolation_spec(0.2, 2, 0.25, true, "conventional");
    let job = cut_job(0.15, 0.05, 100.0, 50.0, 3.0, 12000.0);
    let iso = cam.isolate(g, spec, job);
    cam.rename(iso, "top-iso");
    cam.save_project()
  "#;
  let json: String = engine.eval_with_scope(&mut scope, script).expect("script saves the project");
  assert!(json.contains("\"schema_version\""), "output is a versioned project document");
  assert!(json.contains("top-iso"), "the renamed job is persisted");
  assert!(session.borrow().id_of("top-iso").is_some(), "rename applied to the live session");
}
