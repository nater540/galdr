//! The `skirnir` binary entry point.
//!
//! Two entry paths share one binary, selected by build features and a flag:
//!
//! - **GUI (default).** Built with the `gui` feature, `skirnir` with no positional args launches the egui
//!   window. This is the product.
//! - **Headless harness (`--cli`).** Built with the `serial` feature,
//!   `skirnir --cli <port> [gcode] [baud] [--idle-timeout S] [--timeout S]` opens a port, hands it to the engine,
//!   optionally streams a file, and prints lifecycle/response events to stdout. It drives the SAME streaming
//!   engine as the GUI against real firmware with no window — useful for headless bug-repro and CI hardware
//!   tests. It exits with a scriptable code (0 = program fully acknowledged / clean close, 2 = stall/timeout,
//!   3 = a firmware `error:N`/`ALARM:N` occurred, 4 = transport I/O failure, 130 = Ctrl-C), and an idle timeout
//!   guarantees it can never hang indefinitely. Ctrl-C sends a soft reset + disconnect so the machine is left safe.
//!
//! `expect` is used freely on the startup paths of this binary, where a missing port, an unreadable file, or
//! a failure to build the window/runtime is genuinely unrecoverable. The library itself never panics on a
//! runtime condition.

// The CLI smoke path. Compiled whenever the real serial transport is available; reached via `--cli`.
#[cfg(feature = "serial")]
mod cli {
  use anyhow::{Context, Result};
  use skirnir::engine::{Command, Engine, Event};
  use skirnir::protocol::{RealtimeCommand, Response};
  use skirnir::transport::serial::SerialTransport;
  use std::time::Duration;
  use tokio::time::{Instant, sleep, sleep_until};

  /// The default USB-CDC line rate. The ESP32-S3 native USB ignores it, but the host driver wants a value.
  const DEFAULT_BAUD: u32 = 115_200;

  /// Default inactivity window: if the engine emits NO event (no `ok`, status, progress, or disconnect) for this
  /// long, the controller is presumed wedged and the run exits non-zero. A legitimately busy program keeps
  /// emitting acks and periodic status, so this only trips on a genuine stall — and guarantees the headless run
  /// can never hang forever the way an ad-hoc serial script can. Override with `--idle-timeout <secs>` (0 = off).
  const DEFAULT_IDLE_TIMEOUT_SECS: u64 = 30;

  /// A sentinel "never" deadline for a disabled timeout, so its `select!` arm can be unconditional yet inert.
  fn never() -> Instant {
    Instant::now() + Duration::from_secs(86_400 * 365)
  }

  /// How a headless run ended, before folding in whether any firmware error/alarm was seen.
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  enum Outcome {
    /// Every streamed line was acknowledged (`acked == total`).
    Completed,
    /// The transport closed cleanly (clean EOF / requested disconnect).
    CleanDisconnect,
    /// The transport closed with an I/O failure.
    IoDisconnect,
    /// No activity within the idle window, or the overall deadline elapsed — a presumed stall.
    Timeout,
    /// The operator interrupted with Ctrl-C.
    Interrupted,
  }

  /// Map a run [`Outcome`] (plus whether any `error:N`/`ALARM:N` was observed) to a process exit code, so the
  /// harness is scriptable: 0 = success, 2 = stall/timeout, 3 = the firmware reported an error or alarm, 4 =
  /// transport I/O failure, 130 = interrupted (the conventional SIGINT code).
  ///
  /// A firmware `error:N`/`ALARM:N` yields 3 even when the run later ends as a `Timeout` (Bug 5): after an error
  /// the firmware error-holds and stops acking, so `acked` never reaches `total` and the run falls through to the
  /// idle timeout — but the rejected line is the real failure and must not be masked as a mere stall. Interrupt
  /// (130) and an I/O disconnect (4) keep their own codes, since those describe how the run was cut short rather
  /// than a firmware rejection within it.
  fn exit_code(outcome: Outcome, had_error: bool) -> i32 {
    match outcome {
      Outcome::Interrupted => 130,
      Outcome::IoDisconnect => 4,
      // Any other ending after a firmware error/alarm is an error (3), including a post-error stall.
      _ if had_error => 3,
      Outcome::Timeout => 2,
      Outcome::Completed | Outcome::CleanDisconnect => 0,
    }
  }

  /// The parsed CLI invocation: the positional `port`/`gcode`/`baud` plus the timeout flags.
  struct CliArgs {
    port: String,
    gcode_path: Option<String>,
    baud: u32,
    idle_timeout_secs: u64,
    overall_timeout_secs: Option<u64>,
  }

  /// Parse `--cli`'s tail: positionals `<port> [gcode] [baud]` in order, plus `--idle-timeout <secs>` (0 = off)
  /// and `--timeout <secs>` (overall cap) in any position. Pure so it is unit-testable without a port.
  fn parse_args(args: impl Iterator<Item = String>) -> Result<CliArgs> {
    let argv: Vec<String> = args.collect();
    let mut positionals: Vec<String> = Vec::new();
    let mut idle_timeout_secs = DEFAULT_IDLE_TIMEOUT_SECS;
    let mut overall_timeout_secs = None;
    let mut i = 0;
    while i < argv.len() {
      match argv[i].as_str() {
        "--idle-timeout" => {
          i += 1;
          idle_timeout_secs = argv.get(i).and_then(|s| s.parse().ok()).context("--idle-timeout needs <secs>")?;
        }
        "--timeout" => {
          i += 1;
          overall_timeout_secs = Some(argv.get(i).and_then(|s| s.parse().ok()).context("--timeout needs <secs>")?);
        }
        other => positionals.push(other.to_string()),
      }
      i += 1;
    }
    let mut pos = positionals.into_iter();
    let port = pos.next().context("usage: skirnir --cli <serial-path> [gcode-file] [baud] [--idle-timeout S] [--timeout S]")?;
    let gcode_path = pos.next();
    let baud = pos.next().and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_BAUD);
    Ok(CliArgs { port, gcode_path, baud, idle_timeout_secs, overall_timeout_secs })
  }

  /// Open the port, optionally stream a program, and pump engine events to stdout until the program completes,
  /// the link drops, a timeout trips, or Ctrl-C is pressed. Returns the process exit code (see [`exit_code`]).
  /// Setup failures (bad args, unopenable port, unreadable file) return `Err` instead, which the caller maps to 1.
  pub async fn run(args: impl Iterator<Item = String>) -> Result<i32> {
    let cli = parse_args(args)?;
    let transport =
      SerialTransport::open(&cli.port, cli.baud).with_context(|| format!("opening serial port {}", cli.port))?;
    let mut handle = Engine::connect(transport);
    println!("connected to {} at {} baud", cli.port, cli.baud);

    // If a file was given, read and stream it. Failure to read the file is a startup error.
    if let Some(gcode_path) = &cli.gcode_path {
      let body = std::fs::read_to_string(gcode_path).with_context(|| format!("reading {gcode_path}"))?;
      let lines: Vec<String> = body.lines().map(str::to_string).collect();
      println!("streaming {} lines from {gcode_path}", lines.len());
      handle.send(Command::StreamProgram(lines.into()));
    }

    let idle = (cli.idle_timeout_secs > 0).then(|| Duration::from_secs(cli.idle_timeout_secs));
    let overall_deadline = cli.overall_timeout_secs.map(|s| Instant::now() + Duration::from_secs(s));
    let mut idle_deadline = idle.map(|d| Instant::now() + d);
    let mut had_error = false;

    // Drive the run. Each engine event resets the idle deadline; a stall (no events), the overall cap, or Ctrl-C
    // each end the loop with a distinct outcome. Completion is "every streamed line acknowledged".
    let outcome = loop {
      let idle_at = idle_deadline.unwrap_or_else(never);
      let overall_at = overall_deadline.unwrap_or_else(never);
      tokio::select! {
        maybe = handle.recv() => match maybe {
          None => break Outcome::CleanDisconnect,
          Some(event) => {
            if let Some(d) = idle {
              idle_deadline = Some(Instant::now() + d);
            }
            match event {
              Event::StateChanged(state) => println!("[state] {state:?}"),
              Event::Progress { sent, acked, total } => {
                println!("[prog ] {acked}/{sent} acked, {total} total");
                if total > 0 && acked == total {
                  println!("[done ] all {total} lines acknowledged");
                  break Outcome::Completed;
                }
              }
              Event::Response(resp) => match resp {
                Response::Error(n) => {
                  had_error = true;
                  eprintln!("[error] error:{n}");
                }
                Response::Alarm(n) => {
                  had_error = true;
                  eprintln!("[alarm] ALARM:{n}");
                }
                other => println!("[resp ] {other:?}"),
              },
              Event::Fault(err) => eprintln!("[fault] {err}"),
              Event::Disconnected(reason) => {
                break match reason {
                  Some(err) => {
                    eprintln!("[down ] disconnected: {err}");
                    Outcome::IoDisconnect
                  }
                  None => {
                    println!("[down ] disconnected cleanly");
                    Outcome::CleanDisconnect
                  }
                };
              }
            }
          }
        },
        _ = sleep_until(idle_at) => {
          eprintln!("[stall] no engine activity for {}s — controller presumed wedged", cli.idle_timeout_secs);
          break Outcome::Timeout;
        }
        _ = sleep_until(overall_at) => {
          eprintln!("[stall] overall timeout elapsed");
          break Outcome::Timeout;
        }
        _ = tokio::signal::ctrl_c() => {
          eprintln!("[abort] interrupt — sending soft reset + disconnect");
          handle.send(Command::Realtime(RealtimeCommand::SoftReset));
          handle.send(Command::Disconnect);
          sleep(Duration::from_millis(250)).await; // let the real-time byte flush before we tear down
          break Outcome::Interrupted;
        }
      }
    };

    // On a still-live link (completion or timeout), request a clean disconnect so the port is released promptly
    // for the next tool; a disconnect outcome already closed it. Brief grace so the command reaches the engine.
    if matches!(outcome, Outcome::Completed | Outcome::Timeout) {
      handle.send(Command::Disconnect);
      sleep(Duration::from_millis(150)).await;
    }
    let code = exit_code(outcome, had_error);
    println!("[exit ] outcome={outcome:?} had_error={had_error} code={code}");
    Ok(code)
  }

  /// Run the headless harness on a fresh current-thread runtime, then exit the process with the run's code so it
  /// is scriptable. Setup errors propagate as `Err` (the caller prints them and exits 1).
  pub fn main(args: impl Iterator<Item = String>) -> Result<()> {
    let runtime = tokio::runtime::Builder::new_current_thread()
      .enable_all()
      .build()
      .expect("failed to build the tokio runtime");
    let code = runtime.block_on(run(args))?;
    std::process::exit(code);
  }

  #[cfg(test)]
  mod tests {
    use super::*;

    #[test]
    fn exit_code_maps_outcomes() {
      assert_eq!(exit_code(Outcome::Completed, false), 0);
      assert_eq!(exit_code(Outcome::CleanDisconnect, false), 0);
      assert_eq!(exit_code(Outcome::Completed, true), 3, "a completed run that saw error:N fails");
      assert_eq!(
        exit_code(Outcome::Timeout, true),
        3,
        "a run that saw error:N then stalled is an error (3), not a stall (2) — the firmware error-holds and \
         stops acking, so acked never reaches total; the rejected line must not be masked as a timeout",
      );
      assert_eq!(exit_code(Outcome::Timeout, false), 2);
      assert_eq!(exit_code(Outcome::IoDisconnect, false), 4);
      assert_eq!(exit_code(Outcome::Interrupted, false), 130);
    }

    #[test]
    fn parse_args_reads_positionals_and_flags() {
      let a = parse_args(["/dev/ttyACM0", "job.nc", "115200"].iter().map(|s| s.to_string())).expect("valid");
      assert_eq!(a.port, "/dev/ttyACM0");
      assert_eq!(a.gcode_path.as_deref(), Some("job.nc"));
      assert_eq!(a.baud, 115_200);
      assert_eq!(a.idle_timeout_secs, DEFAULT_IDLE_TIMEOUT_SECS);
      assert_eq!(a.overall_timeout_secs, None);

      let b = parse_args(["--idle-timeout", "5", "/dev/x", "--timeout", "600"].iter().map(|s| s.to_string()))
        .expect("flags in any position");
      assert_eq!(b.port, "/dev/x");
      assert_eq!(b.idle_timeout_secs, 5);
      assert_eq!(b.overall_timeout_secs, Some(600));
      assert_eq!(b.gcode_path, None);
    }

    #[test]
    fn parse_args_requires_a_port() {
      assert!(parse_args(std::iter::empty()).is_err());
    }
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
