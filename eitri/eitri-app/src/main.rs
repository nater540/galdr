//! The `eitri-app` binary entry point: launch the egui window.
//!
//! `expect` is acceptable on this startup path, where a failure to build the window is genuinely
//! unrecoverable; the library itself never panics on a runtime condition.

fn main() -> eframe::Result<()> {
  eitri_app::app::run()
}
