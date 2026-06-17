//! The `skirnir` binary entry point.
//!
//! Two entry paths share one binary, selected by build features and a flag:
//!
//! - **GUI (default).** Built with the `gui` feature, `skirnir` with no positional args launches the egui
//!   window. This is the product.
//! - **CLI smoke path (`--cli`).** Built with the `serial` feature, `skirnir --cli <port> [gcode] [baud]`
//!   opens a port, hands it to the engine, optionally streams a file, and prints lifecycle/response events to
//!   stdout. It exercises the streaming engine against real firmware with no window — useful in headless
//!   environments and CI hardware tests.
//!
//! `expect` is used freely on the startup paths of this binary, where a missing port, an unreadable file, or
//! a failure to build the window/runtime is genuinely unrecoverable. The library itself never panics on a
//! runtime condition.

// The CLI smoke path. Compiled whenever the real serial transport is available; reached via `--cli`.
#[cfg(feature = "serial")]
mod cli {
  use anyhow::{Context, Result};
  use skirnir::engine::{Command, Engine, Event};
  use skirnir::transport::serial::SerialTransport;

  /// The default USB-CDC line rate. The ESP32-S3 native USB ignores it, but the host driver wants a value.
  const DEFAULT_BAUD: u32 = 115_200;

  /// Open the port, optionally load a program, and pump events to stdout until the engine disconnects. `args`
  /// is the tail after the `--cli` flag.
  pub async fn run(mut args: impl Iterator<Item = String>) -> Result<()> {
    let path = args.next().context("usage: skirnir --cli <serial-path> [gcode-file] [baud]")?;
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
      handle.send(Command::StreamProgram(lines.into()));
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

  /// Run the CLI smoke path on a fresh current-thread runtime. Kept self-contained so the GUI path owns its
  /// own multi-threaded runtime independently.
  pub fn main(args: impl Iterator<Item = String>) -> Result<()> {
    let runtime = tokio::runtime::Builder::new_current_thread()
      .enable_all()
      .build()
      .expect("failed to build the tokio runtime");
    runtime.block_on(run(args))
  }
}

fn main() -> anyhow::Result<()> {
  // Peek the first positional arg to choose the entry path. `--cli` selects the smoke loop; anything else
  // (including no args) launches the GUI when it is compiled in.
  let mut args = std::env::args().skip(1).peekable();
  let cli_requested = args.peek().map(|a| a == "--cli").unwrap_or(false);
  if cli_requested {
    args.next(); // consume the flag
  }

  // `--cli` always routes to the smoke path when the serial transport is compiled in.
  #[cfg(feature = "serial")]
  if cli_requested {
    return cli::main(args);
  }

  // Default path: launch the GUI when compiled in. The explicit `return` is load-bearing across feature sets:
  // the non-gui `cfg` blocks below are real alternative tails, so this branch must return rather than fall
  // through — hence the `needless_return` allow (it is only "needless" in the gui-only build clippy sees).
  #[cfg(feature = "gui")]
  {
    let _ = &mut args;
    #[allow(clippy::needless_return)]
    return skirnir::app::run().map_err(|err| anyhow::anyhow!("skirnir window error: {err}"));
  }

  // No GUI compiled in. With serial, the binary is CLI-only; otherwise there is nothing to run. The `return`
  // is the explicit tail of this cfg combo (mirroring the gui branch above); it only reads as "needless" in the
  // serial-only build clippy sees, so the allow is scoped here too.
  #[cfg(all(feature = "serial", not(feature = "gui")))]
  {
    eprintln!("built without the `gui` feature; pass `--cli <port>` to use the streaming smoke path.");
    #[allow(clippy::needless_return)]
    return cli::main(args);
  }
  #[cfg(all(not(feature = "gui"), not(feature = "serial")))]
  {
    let _ = args;
    eprintln!("skirnir was built without `gui` or `serial`; nothing to run. Rebuild with default features.");
    Ok(())
  }
}
