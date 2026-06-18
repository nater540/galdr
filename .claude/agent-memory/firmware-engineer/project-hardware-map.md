---
name: project-hardware-map
description: Galdr ESP32-S3 hardware map — GPIO/peripheral allocation for RMT step gen, TMC2209 UART, LEDC spindle, USB CDC, plus gotchas.
metadata:
  type: project
---

ESP32-S3 devkit, dual Xtensa LX7 @ 240 MHz, hardware single-precision FPU. Authoritative detail is DOC-00 in
`docs/00-architecture.md`; this is a quick index.

**Step/dir:** X_STEP GPIO1=RMT TX ch0, Y_STEP GPIO2=ch1, Z_STEP GPIO4=ch2; ch3 (GPIO18) spare/4th axis. Dir: X GPIO5,
Y GPIO6, Z GPIO7. STEP_EN GPIO8 (TMC ENN active-low). Keep RMT `mem_block_symbols <= 48` (one block) per channel.
esp-hal 1.0 has no RMT DMA backend — use the interrupt path.

**TMC2209:** shared half-duplex single-wire UART1 on GPIO9, 115200 baud, 1k series resistor; every TX byte echoes on
RX (discard). Node addr 0=X,1=Y,2=Z,3=spare via MS1/MS2. Datagrams CRC8-ATM (poly 0x07, init 0x00, LSB-first).
Adafruit 6121 breakout uses **0.05 Ohm** sense resistors (NOT 0.11) — verify before computing IRUN/IHOLD CS.

**Spindle (WS55-220):** no logic-PWM input. LEDC ch0 (low-speed timer0) PWM on GPIO13 -> external RC + op-amp -> 0-10V
(mandatory conditioning circuit). SPIN_EN GPIO14 (to GND = run), SPIN_DIR GPIO15. Direction reversal forces M5 +
spin-down dwell first; ALARM/soft-reset force spindle off; feed-hold does NOT stop spindle.

**Limits/control:** X_LIM GPIO10, Y_LIM GPIO11, Z_LIM GPIO12 (NC switches, internal pull-up, rising-edge IRQ).
FHOLD GPIO16, CYCSTART GPIO17 optional.

**USB:** USB Serial/JTAG controller (fixed-function CDC-ACM, internal PHY GPIO19/20) — NOT otg_fs. No eFuse burn, no
custom descriptors. Native USB can't be hard-reset by host: emit banner on boot + every soft-reset (0x18); answer
0x87/$I+.

**Cores:** Core 1 (APP_CPU) runs ONLY `motion_executor` on a high-priority InterruptExecutor (SoftwareInterrupt<1>,
Priority3). Core 0 (PRO_CPU) runs all else on thread-mode executor. embassy-time on SYSTIMER.

**Gotchas:** strapping pins GPIO0/3/45/46 must not be driven at boot; GPIO19/20 reserved for USB. RX buffer = 1024
bytes (grblHAL norm), report truthfully in `[OPT:]` and `Bf:`. CRLF/LFCR = single terminator (avoid double-ok).

See [[project-galdr-overview]] and [[project-build-constraints]].
