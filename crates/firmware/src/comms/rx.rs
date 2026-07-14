//! The USB receive path (DOC-08 reader/framer halves): the `usb_rx` reader task, the `line_assembler` framer
//! task, the `frame_byte` per-byte framer step, and the `trim_ascii` line helper — extracted verbatim from
//! `comms.rs` (architecture-refactor A1, step 3). `usb_rx` reads USB bytes and dispatches real-time commands
//! immediately (never blocking on line flow control); `line_assembler` drains `RX_PIPE` through the
//! `StreamEngine` framer and back-pressures the host via `LINE_QUEUE`. Real-time dispatch itself
//! (`dispatch_realtime`) and the `error` ack helper still live in `comms.rs` until later steps and are reached
//! via `super::`. `comms.rs` re-exports this module (`pub(crate) use rx::*;`) so the `main.rs` task spawns
//! (`comms::usb_rx` / `comms::line_assembler`) and the `trim_ascii` call sites keep resolving unqualified.

use core::sync::atomic::Ordering;

use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_hal::usb_serial_jtag::UsbSerialJtagRx;
use esp_hal::Async;

use firmware_core::protocol::{classify_realtime, EngineEvent, StreamEngine};

use super::{
  dispatch_realtime, error, Line, LINE_QUEUE, LINE_RESET, LINES_FRAMED, RX_ACTIVITY, RX_PIPE, RX_PIPE_OVERFLOW,
};

/// The USB receive task — the *reader half*. It reads USB bytes and, per byte, intercepts real-time
/// commands ([`classify_realtime`]) and dispatches them through their Signals IMMEDIATELY; every other byte
/// is pushed into [`RX_PIPE`] for the [`line_assembler`] to frame. This task NEVER blocks on line flow
/// control: real-time dispatch is non-blocking, and the pipe write is non-blocking (and, for a compliant
/// host, never full — see [`RX_PIPE_CAPACITY`]). So a feed-hold / soft-reset arriving during sustained
/// streaming is acted on within one byte, not after a whole move's worth of back-pressure clears.
#[embassy_executor::task]
pub async fn usb_rx(mut rx: UsbSerialJtagRx<'static, Async>) -> ! {
  // A modest read chunk: the USB Serial/JTAG FIFO is 64 bytes, so reads return promptly.
  let mut buf = [0u8; 64];
  loop {
    crate::crash::record_comms_stage(crate::crash::CommsTask::UsbRx, crate::crash::CommsStage::RxRead);
    let n = match rx.read(&mut buf).await {
      Ok(n) => n,
      // A persistent USB read error must not become a tight spin that starves the other core-0 tasks: back
      // off briefly before retrying. The connection persists across host reopen on native USB, so we do not
      // tear down state here.
      Err(_) => {
        Timer::after(USB_RX_ERROR_BACKOFF).await;
        continue;
      }
    };
    // Host-activity heartbeat: advance once per non-empty read (by the byte count, harmlessly) so the watchdog can
    // tell a host is actively driving the firmware. Bumping per read rather than per byte is enough for the "did it
    // advance since the last tick" check and stays off the per-byte path. `usb_rx` keeps draining the FIFO even when
    // the comms PROCESSING path is wedged (the real-board failure mode), which is EXACTLY why RX activity is the
    // right "host present" signal to gate the comms-stall feed-withhold on.
    if n > 0 {
      RX_ACTIVITY.fetch_add(n as u32, Ordering::Relaxed);
    }
    for &byte in &buf[..n] {
      match classify_realtime(byte) {
        // A real-time byte: dispatch its action and do NOT let it enter a line or the byte buffer.
        Some(cmd) => dispatch_realtime(cmd),
        // An ordinary line byte: hand it to the line-assembly half. `try_write` never blocks, keeping the
        // reader free for the next real-time byte; for a compliant host the pipe (sized to the advertised
        // RX buffer) is never full, so no byte is lost. If a misbehaving host overruns its character-count
        // window the overflowing byte is dropped — the resulting framed line errors, which is the correct
        // push-back for a host that ignored flow control, and real-time dispatch stays alive throughout.
        //
        // OBSERVE-ONLY probe (task #22): COUNT a dropped byte but keep the SILENT DROP unchanged. `try_write`
        // returns `Ok(n)` (bytes accepted, 0 or 1 here) or `Err` (pipe full). A dropped byte is `Ok(0)` or `Err` —
        // both mean the single byte did not enter the pipe. We deliberately do NOT convert this to an `error:N`
        // this build: that would hold the stream and mask the skip we are trying to observe (see `RX_PIPE_OVERFLOW`).
        None => match RX_PIPE.try_write(&[byte]) {
          Ok(1) => {}
          _ => {
            RX_PIPE_OVERFLOW.fetch_add(1, Ordering::Relaxed);
          }
        },
      }
    }
  }
}

/// The line-assembly half: drain [`RX_PIPE`] one byte at a time through the [`StreamEngine`] line framer
/// and forward each completed line (blank lines included, as an empty [`Line`]) to [`LINE_QUEUE`]. Blocking
/// on a full `LINE_QUEUE` is CORRECT back-pressure: it stops draining the pipe, the pipe fills, the reader's
/// `try_write` starts refusing bytes, and the host's character-counting throttles. A soft reset
/// ([`LINE_RESET`]) drops any partially framed line so a fresh stream is not contaminated by a half-line
/// from before the reset. Reading a single byte per iteration keeps the reset cancellation point exact (no
/// pre-buffered chunk to discard) and is not a throughput concern — the assembler is bounded by the USB
/// byte rate, not by per-byte overhead.
#[embassy_executor::task]
pub async fn line_assembler() -> ! {
  let mut engine = StreamEngine::new();
  let mut byte = [0u8; 1];
  loop {
    crate::crash::record_comms_stage(crate::crash::CommsTask::LineAssembler, crate::crash::CommsStage::LineWaitByte);
    // Race the next byte against a soft reset so a `0x18` mid-line drops the partial line promptly. `read`
    // is cancel-safe (it consumes from the pipe only on completion), so losing this race loses no byte.
    match select(RX_PIPE.read(&mut byte), LINE_RESET.wait()).await {
      Either::First(0) => {}
      Either::First(_) => frame_byte(byte[0], &mut engine).await,
      Either::Second(()) => engine.soft_reset(),
    }
  }
}

/// Feed one byte to the line framer and act on the resulting [`EngineEvent`]: forward a completed line
/// (blank included) to the single in-order consumer, or emit `error:15` immediately for an over-length
/// line. The framer performs no real-time classification or error-hold — those live in the reader half and
/// the consumer respectively (see [`StreamEngine`]).
async fn frame_byte(byte: u8, engine: &mut StreamEngine) {
  match engine.ingest(byte) {
    EngineEvent::None => {}
    EngineEvent::AcceptLine(line) => {
      // Copy the borrowed line into an owned buffer for the channel. A line longer than the buffer cannot
      // occur: the framer already enforced MAX_LINE_LEN, so this never truncates. A blank line is forwarded
      // as an empty `Line`, so the consumer owns both its bare `ok` and the error-hold recovery.
      let mut owned = Line::new();
      let _ = owned.extend_from_slice(line);
      // OBSERVE-ONLY probe (task #22): a line was successfully FRAMED and is about to be handed to the consumer.
      // Counted here (not at the consumer) so it is the true "line-IN" leg — every framed line, before any
      // back-pressure wait, so a divergence from `ACKS_EMITTED` pins a loss to the consume/ack stage, not framing.
      LINES_FRAMED.fetch_add(1, Ordering::Relaxed);
      // Block here if the consumer is briefly behind: this back-pressures the host stream (correct flow
      // control) without dropping a line. Real-time bytes already bypassed this path entirely.
      crate::crash::record_comms_stage(crate::crash::CommsTask::LineAssembler, crate::crash::CommsStage::LineSendQueue);
      LINE_QUEUE.send(owned).await;
    }
    EngineEvent::Reject(code) => error(code).await,
  }
}

/// Backoff before retrying a USB read after a read error, so a persistent error does not become a tight
/// spin that starves the other core-0 tasks. Short enough that a transient glitch barely delays reception,
/// long enough to yield the executor on a sustained fault.
const USB_RX_ERROR_BACKOFF: Duration = Duration::from_millis(5);

/// Trim leading/trailing ASCII whitespace from a line, since a sender may pad `$` commands with spaces.
/// `core` lacks a stable slice trim for `&[u8]`, so this is a small explicit helper.
pub(crate) fn trim_ascii(line: &[u8]) -> &[u8] {
  let mut start = 0;
  let mut end = line.len();
  while start < end && (line[start] == b' ' || line[start] == b'\t') {
    start += 1;
  }
  while end > start && (line[end - 1] == b' ' || line[end - 1] == b'\t') {
    end -= 1;
  }
  &line[start..end]
}
