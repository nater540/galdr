---
name: project-rmt-clock-and-tx-completion
description: Verified-from-source esp-hal 1.0.0 RMT facts for the S3 step path — clock divider chain, single-block 48-symbol TX, blocking wait() completion mechanism, and why these are NOT the first-move-wedge cause.
metadata:
  type: project
---

Investigated 2026-06-16 (branch `firmware/core-pipeline-and-streaming`) for the "first G0 X5 wedges core 1"
bug. All three static-RMT hypotheses (clock divider, 48-symbol memsize boundary, end-marker) were DISPROVEN
from esp-hal 1.0.0 source. Non-obvious facts a future author needs (so nobody re-treads this):

**RMT clock chain on S3 is correct as written (two dividers, both right):** S3 default RMT clock source is
`Apb` (esp-metadata-generated `for_each_rmt_clock_source` default(Apb)), and `Clocks::get().apb_clock` is
`Rate::from_mhz(80)` (clock/mod.rs). `Rmt::new(RMT, Rate::from_mhz(80))` calls `configure_clock(Apb, 80MHz)`
→ `div = (src/freq)-1 = (80/80)-1 = 0` written to `sclk_div_num` → source passes through at 80 MHz. THEN the
per-channel `TxChannelConfig::with_clk_divider(80)` → `set_divider(80)` divides 80 MHz by 80 = 1 MHz = 1
tick/µs, matching `MotionConfig.tick_hz = 1_000_000`. So 1 µs/tick is real; the divider is NOT doubled and TX
is NOT frozen by a wrong clock.

**S3 RMT `channel_ram_size = 48` (esp-metadata-0.8.0 devices/esp32s3.toml). A full 48-symbol burst (47 events +
1 end marker) fits ONE block (`memsize=1`) exactly with NO wraparound.** `Channel::transmit` calls
`RmtWriter::write(initial=true)` which writes `min(data.len(), memsize)=48` symbols up front and sets
`state=Done`, `remaining_data` empty. `start_send` sets threshold `codes()/2 = 24`. The blocking `wait()` loop
(`poll_internal`) only ever re-invokes `write` on a `Threshold` event, which returns immediately (state==Done).
So 48 symbols is the SAFE max for the single-shot blocking path — no boundary mishandling.

**Blocking `wait()` needs NO ISR:** `get_tx_status()` reads `int_raw()` (the RAW status reg, S3 chip_specific
~rmt.rs:2440) for `ch_tx_end`/`ch_tx_err`/`ch_tx_thr_event`. `wait()` loops until `Event::End` (TX-END raw
bit, set by HW at the end-marker symbol) or `Event::Error`. So a never-returning `wait()` means TX-END genuinely
never fired (a real hardware/encoding fault), NOT priority inversion. `is_end_marker()` = `length1==0 ||
length2==0`; the encoder's end_marker is the only zero-length symbol and is placed last — correct.

**The actual wire symptom is mis-diagnosable:** `status_responder` computes `running = EXECUTOR_RUNNING ||
queued` where `queued = blocks_free < BLOCK_QUEUE_LEN (32)`. So `Bf:31` (1 block still queued) ALONE forces a
permanent `Run` regardless of EXECUTOR_RUNNING, and `FS:0` means `LIVE_PROGRAMMED_FEED_MM_MIN` was never
published (run_block never reached). `Run`+`FS:0`+`Bf:31` is therefore consistent with the executor stalling
BEFORE the pop/run_block — NOT necessarily inside emit_burst's `wait()`. Localize with defmt, don't assume RMT.

**defmt tracing added to motion.rs** behind a local `mtrace!` macro (cfg-gated: `defmt::trace!` with feature,
empty otherwise). Trace chain: executor-loop-entered → hold-park → popping/lock-acquired → block-popped →
feed-published → emit_burst → per-axis transmit Ok/Err → per-axis wait begin/ok/err → run_block-returned. The
"Diagnosing a core-1 stall" module doc holds the last-trace-line→fault decision tree. Build:
`just build --features defmt`, flash, `just monitor`, send `G0 X5`, read last RTT line.

See [[project-motion-executor]], [[project-motion-executor-review-fixes]], [[project-motion-contracts]].
</content>
</invoke>
