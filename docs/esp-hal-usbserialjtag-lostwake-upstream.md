# esp-hal upstream issue — `UsbSerialJtagTx` async write can lose its TX-done wake (APPROVED to file 2026-07-11; file manually at https://github.com/esp-rs/esp-hal/issues/new — `gh` was unavailable in-session)

Prepared 2026-07-11 by the Galdr firmware team (embedded-bug-hunter). This is a ready-to-file draft for
`esp-rs/esp-hal`; it is Option C of `docs/streaming-lockup-investigation.md` §18 — the durable upstream root fix,
separate from our in-tree Option B (poll-based `usb_tx`). All line references are esp-hal `1.1.1`,
`src/usb_serial_jtag.rs`.

---

## Title
`UsbSerialJtagTx` async write can lose its TX-done wake → the write future hangs until an unrelated event (ESP32-S3)

## Summary
On the async `UsbSerialJtagTx`, a completed USB transmission can fail to wake its write future, so an
`embedded-io-async` `write`/`flush` future never resolves even though the host has drained the FIFO. Under a
`with_timeout`/`select` (the standard way to bound a device→host write) the problem is aggravated because the write
future has no `Drop` and leaves the TX interrupt armed when cancelled. On an ESP32-S3 streaming responses over
USB-Serial-JTAG under Embassy, this manifests as an intermittent, permanent TX stall.

## Root cause (two independent defects)
The completion protocol for `UsbSerialJtagWriteFuture` is: `new()` sets `int_ena.serial_in_empty` (arm); `poll()`
registers a single shared `WAKER_TX` (`AtomicWaker`) and returns `Ready` **iff `int_ena.serial_in_empty` is clear**; the
ISR `async_interrupt_handler` clears `int_ena.serial_in_empty` and calls `WAKER_TX.wake()` on TX-empty.

1. **No `Drop` on `UsbSerialJtagWriteFuture` (lines 706–747).** There is no `Drop` impl in the file. If the future is
   dropped after `new()` armed the interrupt — e.g. it is the losing branch of a `with_timeout`/`select` — the
   `int_ena.serial_in_empty` enable bit stays SET. The arm/event/waker state is then desynced and a subsequent write can
   arm-an-already-armed bit and hang. (Captured signature on our board: `int_ena.serial_in_empty == 1` at the stall —
   armed but unserviced.)

2. **Edge-only completion via a single, non-counting `AtomicWaker` + a mask bit (lines 703, 733–747, 932–961).**
   `WAKER_TX` holds only the latest waker and no count; completion is signalled solely by the ISR clearing the enable
   bit. If the wake is raced or lost, the future never completes even though the host HAS drained the FIFO
   (`ep1_conf.serial_in_ep_data_free == 1`), because `poll()` only re-checks the enable bit, not the hardware
   FIFO-drained state. (Captured signature: `int_ena.serial_in_empty == 0`, `serial_in_ep_data_free == 1`, future never
   resolved — the ISR ran but the completion was lost with no retained state to recover from.)

For contrast, ESP-IDF's own `usb_serial_jtag` driver
(`components/esp_driver_usb_serial_jtag/src/usb_serial_jtag.c`) does not have this class: it stages TX in a ring buffer,
the SERIAL_IN_EMPTY ISR drains the ring into the FIFO and keeps re-firing while data remains, and completion is a
*retained* `tx_idle_sem` — the signal is data-occupancy-driven and self-healing, not a single edge.

## Proposed fix (small, self-healing)
1. **Add a `Drop` to `UsbSerialJtagWriteFuture`** that disarms the interrupt it armed:
   `int_ena.serial_in_empty.clear_bit()`. This makes cancellation (timeout/select) safe.
2. **Make `poll()` data-driven** — return `Ready` when EITHER the enable bit is clear OR
   `ep1_conf.serial_in_ep_data_free` is set. Reading the hardware FIFO-drained bit means a lost/raced edge self-heals on
   the next poll instead of hanging.

Both are localized to `UsbSerialJtagWriteFuture` / its `poll` and fix the class for every user without an API change.

## Reproduction context
ESP32-S3, esp-hal 1.1.1, esp-rtos + Embassy, single async task streaming ~4–130 B responses over USB-Serial-JTAG with
each write/flush wrapped in `embassy_time::with_timeout`. Intermittent (non-deterministic) permanent TX stall; both
signatures above captured via RTC_FAST breadcrumbs. Full investigation: (link to be added by the user if filed).

## Workaround (what we're doing meanwhile)
Bypass the async write future entirely: poll `write_byte_nb`/`flush_tx_nb` (which re-read `serial_in_ep_data_free`) with
`yield_now()` between polls — no waker, nothing to lose. (This is our in-tree Option B.)
