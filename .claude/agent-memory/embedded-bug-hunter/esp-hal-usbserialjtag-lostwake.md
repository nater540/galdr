---
name: esp-hal-usbserialjtag-lostwake
description: Root of the usb_tx lost-TX-wake is an esp-hal UsbSerialJtagTx design defect (edge-signalled completion + no Drop); grblHAL/IDF prevent it with a retained signal; Option B poll-based usb_tx is the low-risk fix.
metadata:
  type: reference
---

Source-verified 2026-07-11 (esp-hal 1.1.1 `src/usb_serial_jtag.rs`, esp-idf `usb_serial_jtag.c`, grblHAL/ESP32
`main/usb_serial.c`). Full analysis = docs/streaming-lockup-investigation.md §18. See [[firmware-two-2s-timeouts]] /
[[firmware-rwdt-superwdt-facts]] for the wedge this causes.

**The lost-TX-wake is an esp-hal `UsbSerialJtagTx` DEFECT, not an Embassy/our-code limitation.** `write_async` writes
≤64 B chunks straight to the EP1 FIFO then awaits `UsbSerialJtagWriteFuture`, whose completion signal is "the ISR
CLEARED `int_ena.serial_in_empty`" + ONE shared `WAKER_TX` `AtomicWaker`. Three source-confirmed fragilities:
1. `UsbSerialJtagWriteFuture` has NO `Drop` (grep: zero Drop impls in the file) → a `with_timeout` drop leaves
   `int_ena` ARMED = the captured Signature-A `iena=1` write-stage lost wake.
2. Single latest-only, non-counting `AtomicWaker` → a raced/dropped wake (the classic `iena=0 empty=0 free=1`: ISR ran,
   host drained, future never completed) strands the write with no retained state.
3. Completion = a mask bit, not data occupancy → payload already in the FIFO, nothing retained to re-drive.

**Prevention pattern (both references use a RETAINED signal, never a single edge to a latest-only waker):**
- ESP-IDF `usb_serial_jtag` driver (SAME peripheral): `usb_serial_jtag_write_bytes`→`xRingbufferSend` into a TX ring;
  the SERIAL_IN_EMPTY ISR drains the ring→FIFO and gives a retained `tx_idle_sem`; pending bytes live in the ring and
  the ISR re-fires until drained (self-healing).
- grblHAL ESP32 actual uses TinyUSB CDC (native USB-OTG, NOT USB-Serial-JTAG): `_usb_write()` POLLS
  `tud_cdc_write_available()` + flush + a yield callback — NO TX-empty ISR, so no wake to lose.

**Fix options (PLAN-FIRST; user decides):**
- **Option B (recommended, low-risk, in-our-code):** make `usb_tx` POLL-based — write ≤64 B chunks, re-read
  `ep1_conf.serial_in_ep_data_free` with `yield_now().await` between chunks instead of awaiting the future. = our
  validated TIER-1 poll-after-arm logic promoted to the primary loop; no waker to lose. No esp-hal dependency.
- **Option C (cheapest upstream root fix):** esp-hal patch — add `Drop` to `UsbSerialJtagWriteFuture` disarming
  `int_ena`, and make `poll()` also return Ready on the hw `serial_in_ep_data_free` bit (data-driven).
- Option A (HIGH effort): reimplement IDF's ring-buffer+ISR TX in Rust — belongs UPSTREAM in esp-hal.
Keep §17.17 executor-liveness reset→ALARM:11 as defense-in-depth regardless. Contract note: grblHAL advertises 512 B RX
(we advertise 1024 B); nothing in the driver layer contradicts our ok/error/CRLF/error-hold/realtime contract.
