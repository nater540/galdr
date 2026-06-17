//! Minimal CLI smoke entry for `skirnir`.
//!
//! This is a placeholder driver, not the product: the egui UI is a later task. For now it opens a serial
//! port, hands it to the engine, optionally streams a G-code file, and prints lifecycle/response events to
//! stdout so the streaming engine can be exercised against real firmware with no GUI dependency.
//!
//! Usage: `skirnir <serial-path> [gcode-file] [baud]`, e.g. `skirnir /dev/ttyACM0 part.gcode 115200`.
//!
//! `expect` is used freely here: this is the startup path of a binary where a missing port or unreadable
//! file is genuinely unrecoverable. The library itself never panics on a runtime condition.

#[cfg(feature = "serial")]
mod cli {
  use anyhow::{Context, Result};
  use skirnir::engine::{Command, Engine, Event};
  use skirnir::transport::serial::SerialTransport;

  /// The default USB-CDC line rate. The ESP32-S3 native USB ignores it, but the host driver wants a value.
  const DEFAULT_BAUD: u32 = 115_200;

  /// Open the port, optionally load a program, and pump events to stdout until the engine disconnects.
  pub async fn run() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().context("usage: skirnir <serial-path> [gcode-file] [baud]")?;
    let gcode_path = args.next();
    let baud = args.next().and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_BAUD);

    let transport = SerialTransport::open(&path, baud).with_context(|| format!("opening serial port {path}"))?;
    let mut handle = Engine::connect(transport);
    println!("connected to {path} at {baud} baud");

    // If a file was given, read and stream it. Failure to read the file is a startup error.
    if let Some(gcode_path) = gcode_path {
      let body = std::fs::read_to_string(&gcode_path).with_context(|| format!("reading {gcode_path}"))?;
      let lines: Vec<String> = body.lines().map(str::to_string).collect();
      println!("streaming {} lines from {gcode_path}", lines.len());
      handle.send(Command::StreamProgram(lines));
    }

    // Drain events until the engine task ends (disconnect / EOF). This is a smoke loop, not a UI.
    while let Some(event) = handle.recv().await {
      match event {
        Event::StateChanged(state) => println!("[state] {state:?}"),
        Event::Response(response) => println!("[resp ] {response:?}"),
        Event::Progress { sent, acked, total } => println!("[prog ] {acked}/{sent} acked, {total} total"),
        Event::Fault(err) => eprintln!("[fault] {err}"),
        Event::Disconnected(reason) => {
          match reason {
            Some(err) => eprintln!("[down ] disconnected: {err}"),
            None => println!("[down ] disconnected cleanly"),
          }
          break;
        }
      }
    }
    Ok(())
  }
}

#[cfg(feature = "serial")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
  cli::run().await
}

// Without the `serial` feature there is no real transport to open, so the binary is a no-op stub. The
// library and its loopback-driven tests are unaffected — this only guards the hardware entry point.
#[cfg(not(feature = "serial"))]
fn main() {
  eprintln!("skirnir was built without the `serial` feature; rebuild with --features serial to use the CLI.");
}
