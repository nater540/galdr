# grblHAL GCode Streaming Protocol: A Complete Implementation Reference for Rust/Embassy on ESP32-S3

## TL;DR
- grblHAL is a 32-bit port/rewrite of grbl 1.1f — its own README states it "is a port/rewrite of grbl 1.1f and should be compatible with GCode senders compliant with the specifications for that version." So the streaming protocol is fundamentally grbl 1.1: line-oriented G-code with a single line terminator, acknowledged by `ok`/`error:N`, with flow control done by the *host* tracking in-flight bytes against the controller's serial RX buffer (character counting) or by simple send-response. Your firmware must reproduce grbl 1.1's exact text responses plus grblHAL's bracketed extensions.
- The biggest grblHAL-specific deltas to implement: a much larger RX buffer (typically 1024 bytes vs. grbl's 128, reported in `[OPT:...]` so hosts read it at runtime), CRLF/LFCR treated as a single line terminator, top-bit-set real-time command equivalents (0x80=`?`, 0x81=`~`, 0x82=`!`, 0x83=`$G`, 0x87=full report, 0x8C=auto-report toggle), the extended `$I`/`$I+` build-info report, runtime enumeration of settings/errors/alarms (`$ES`/`$EE`/`$EA`), persistent error state after an error during a job, and an optional auto-report interval (`$481`).
- For the Z-probe / touch-off workflow (e.g. PCB surface probing), implement `G38.2/.3/.4/.5` with the `[PRB:x,y,z:success]` push message; `.2`/`.4` raise `ALARM:4/5` on failure (halting), `.3`/`.5` never alarm. The startup/connect handshake must tolerate that an ESP32-S3 on native USB cannot be hard-reset by the host, so you must emit the welcome banner on boot and answer `0x87`/`$I+` so senders like ioSender can detect readiness.

## Key Findings

1. **grblHAL ≈ grbl 1.1f at the protocol level.** Implement grbl 1.1 first, then layer the grblHAL extensions. Senders compliant with grbl 1.1f are expected to work.

2. **Three response classes.** Your firmware emits exactly three kinds of output: (a) **response messages** — `ok` or `error:N`, one per accepted line, the only thing that drives flow control; (b) **push messages** — `<...>` status reports, `[...]` bracketed messages, `$N=...`/`$n=val` settings lines, the `Grbl`/`GrblHAL` banner, and `>...:ok` startup-line echoes; (c) **real-time command responses**, triggered by single bytes that bypass the line buffer.

3. **Flow control = host counts bytes, not the firmware.** The firmware just provides `ok`/`error` per line and never throttles the host explicitly (XON/XOFF was removed). The host either waits for `ok` (simple) or tracks the sum of in-flight line lengths against the RX buffer size (character counting). Your firmware's only obligations: emit exactly one `ok`/`error:N` per line consumed, and never emit a spurious `ok`.

4. **RX buffer is bigger and advertised.** Classic grbl hardcodes a 128-byte serial RX buffer — the gnea/grbl v1.1 wiki states "Grbl has a 128 character serial receive buffer, and the host PC can send up to 128 characters without overflowing the buffer" (and a 16-block planner buffer). grblHAL typically uses 1024 bytes (driver-dependent) and reports both the planner (block) buffer size and RX buffer size in the `[OPT:...]` line so hosts size their send-ahead window dynamically.

5. **Line endings.** grblHAL treats `CRLF` (`\r\n`) and `LFCR` (`\n\r`) as a *single* terminator (legacy grbl counts them as two and emits two `ok`s — the classic "double-ok" streaming bug). A bare `CR` or bare `LF` is also a terminator.

6. **Persistent error state (grblHAL-specific, safety-relevant).** When a G-code line produces an error, grblHAL keeps subsequent G-code lines in an error state until a reset, an empty line, or a `$` system command is issued — reducing the risk of a dangerous move after a bad line. This differs from legacy grbl, which only dumps the single offending block.

## Details

### 1. Transport layer (USB CDC serial)

- **Baud rate is irrelevant on native USB CDC.** CDC-ACM is packetized over USB bulk transfers; the "baud rate" is a legacy field the host sets (senders default to 115200, 8-N-1) but it does not gate throughput on a native-USB MCU like the ESP32-S3. Hosts still open the port with a nominal rate for compatibility. Do not rely on baud-derived timing.
- **Line endings:** accept `\n`, `\r`, `\r\n`, and `\n\r`. Treat CRLF/LFCR as one line terminator (grblHAL behavior). Strip the terminator, parse the line, emit one `ok`/`error:N`. Keep terminator handling consistent.
- **Max line length:** grbl's classic line buffer is 80 characters (`LINE_BUFFER_SIZE`), and an over-length line yields `error:15` (line length exceeded) in the broader grbl family; grblHAL's line buffer is larger, but still enforce a maximum and return the line-overflow error rather than silently truncating. Comments in `()` or after `;` are stripped by the parser.
- **Connection / readiness:** on power-up or soft reset the controller prints the welcome banner (see §5). A host treats receipt of the banner as "controller reset and ready." Because the ESP32-S3 on native USB may not be hard-resettable by the host (toggling DTR/RTS drops the USB connection rather than resetting the MCU), you MUST emit the banner on every boot and on every soft reset (0x18), and you must also answer `0x87`/`$I+` so a sender can detect readiness without a banner.

### 2. Flow control — the character-counting protocol

Two host strategies; your firmware behaves identically for both — the difference is entirely host-side.

- **Simple send-response:** host sends one line, waits for `ok`/`error`, sends the next. Robust, simplest, slightly slower because the RX buffer is usually empty. grbl officially *recommends* this method.
- **Character counting (send-ahead):** host keeps a running sum of the byte-lengths (including the terminator) of every line sent but not yet acknowledged. It sends more lines while `sum < RX_BUFFER_SIZE` (classically `RX_BUFFER_SIZE - 1`, i.e. ≤127 on legacy grbl). Each `ok`/`error` lets the host subtract the oldest unacknowledged line's length. This keeps your RX buffer full so the planner never starves — important for the dense, short segments of PCB isolation milling.

**grblHAL-specific buffer sizing.** Do not hardcode 128. grblHAL reports the RX buffer size in the `[OPT:...]` line of the `$I` response: `[OPT:<options>,<block buffer size>,<RX buffer size>{,<axes>{,<tool table entries>}}]`. On 32-bit drivers the RX buffer is typically **1024 bytes** (real `$I` outputs show `[OPT:VNMSL,100,1024,3,0]` and `[OPT:VNMSL,35,1024,5,0]`), but it is a per-driver compile-time constant — some forks report 255. A correct host reads the second comma-separated OPT value (planner/block buffer size) and the third (RX buffer size) at connect and sizes its send-ahead window to the reported RX value. For your firmware: pick an RX buffer size (1024 is the grblHAL norm and a good fit on the ESP32-S3, which has ample RAM) and report it accurately in OPT and as the second field of `Bf:`.

- **The `Bf:` status field.** Format `Bf:<planner blocks free>,<RX bytes free>`. grbl v1.1 changed these from "in use" to "available/free": the wiki notes "The buffer state values changed from showing 'in-use' blocks or bytes to 'available'. This change does not require the GUI to know how many block/bytes Grbl has been compiled with." Example `Bf:15,128` or `Bf:35,1023`. **Hosts must NOT use `Bf:` for flow control** — during a stream the reported buffer is out of date by the time the host receives it; the `ok`/`error` responses are the authoritative flow-control signal. `Bf:` is for diagnostics/GUI niceties, off by default (enabled via `$10` mask bit 1). Emit it correctly when enabled because some GUIs display it.
- **Settings writes break character counting.** On AVR grbl, EEPROM writes disable the serial RX interrupt — the wiki warns: "When Grbl stores data to EEPROM, the AVR requires all interrupts to be disabled during this write process, including the serial RX ISR. This means that if a g-code or Grbl $ command writes to EEPROM, the data sent during the write may be lost." So settings (`$x=`, `$N`, `$RST`, `G10`, `G28.1`/`G30.1`) must be streamed with simple send-response. On the ESP32-S3 with flash/NVS emulation, ensure your NVS write path does not lose RX bytes; if it can, document that settings must be sent send-response, matching host expectations.

### 3. Real-time commands

Real-time commands are single bytes intercepted the instant they arrive — picked out of the byte stream *before* the line buffer, never entering a line, never getting an `ok`, executing within tens of milliseconds. Your RX path must scan every incoming byte and divert these before appending to the line buffer.

**Core single-byte real-time commands:**
- `0x18` (Ctrl-X) — **soft reset** / abort. Halts motion, resets parser/planner, re-emits the welcome banner. On grblHAL a soft reset clears homed status only if the machine was in motion.
- `?` — status report request (responds with `<...>`).
- `~` — cycle start / resume.
- `!` — feed hold.
- `0x84` — safety door (suspends into DOOR state, kills spindle/coolant).
- `0x85` — jog cancel (feed-hold + flush planner; ignored if not jogging).
- `0x90`–`0x9E`, `0xA0`/`0xA1` — overrides (below).

**grblHAL-specific control bytes and top-bit-set equivalents:**
- `0x19` (Ctrl-Y) — **stop**: similar to 0x18 reset but does *not* perform a full warm reset, leaving more internal state intact.
- `0x1B` then `0x14` (`<ESC><Ctrl-T>`) — two-byte **hard reset** sequence on drivers that support it. NOTE: native-USB/network connections drop on hard reset.
- `0x80` = `?`, `0x81` = `~`, `0x82` = `!`, `0x83` = `$G` (parser-state report), `0x87` = **full real-time report** (includes all change-only elements plus the alarm substate). These top-bit-set forms exist because grblHAL does *not* honor the printable `?`/`~`/`!` while reading `$`-command or message input (so those characters can appear in passwords/strings). Support is advertised by `RT+`/`RT-` in the `NEWOPT` tag.
- `0x88` toggle optional-stop, `0x89` toggle single-block, `0x8A` toggle fan0, `0x8B` toggle MPG mode, `0x8C` toggle auto real-time report mode, `0xA3` tool-change acknowledge, `0xA4` toggle probe-connected (not yet functional).

**Override real-time commands (extended ASCII, identical to grbl 1.1):**
- Feed override: `0x90` set 100%, `0x91` +10%, `0x92` −10%, `0x93` +1%, `0x94` −1%.
- Rapid override: `0x95` 100%, `0x96` 50%, `0x97` 25%.
- Spindle override: `0x99` 100%, `0x9A` +10%, `0x9B` −10%, `0x9C` +1%, `0x9D` −1%, `0x9E` spindle stop toggle.
- Coolant: `0xA0` flood toggle, `0xA1` mist toggle.
Feed/spindle override range is clamped (typically 10%–200%); if the value would not change, the command is ignored.

**Interleaving with streaming:** because these bypass the line buffer, a host can send `?` or `!` at any instant mid-line without corrupting the stream. Your byte-scanner must remove the real-time byte, not count it toward the line, and not emit an `ok` for it (override/status commands produce their own push output).

### 4. Status reports

**Full format (grblHAL):**
```
<State{:substate}|MPos:|WPos:<axis positions>{|Bf:blocks,bytes}{|Ln:n}{|FS:feed,rpm{,actual_rpm}}{|Pn:signals}{|WCO:offsets}{|WCS:Gn}{|Ov:f,r,s}{|A:accessory}{|MPG:0|1}{|H:0|1{,mask}}{|D:0|1}{|Sc:axes}{|TLR:0|1}{|FW:grblHAL}{|In:result}>
```
- **State** (always first): `Idle, Run, Hold, Jog, Alarm, Door, Check, Home, Sleep, Tool` (`Tool` is grblHAL's tool-change state). Substates: `Hold:0` (complete/ready to resume), `Hold:1` (in progress), `Door:0..3`, `Run:1` (feed hold pending), `Run:2` (probing), `Alarm:<code>` (added when full report requested via 0x87).
- **Position** (always second): either `MPos:` (machine) or `WPos:` (work) — never both. `WPos = MPos − WCO`.
- `Bf:` (planner blocks free, RX bytes free), `Ln:` (line number, only if present in g-code), `FS:` (feed, programmed RPM, optionally actual RPM), `Pn:` (asserted input pins), `WCO:`, `Ov:` (feed,rapid,spindle override %), `A:` (accessory: `S`=spindle CW, `C`=spindle CCW, `M`=mist, `F`=flood, `T`=tool change pending).
- `Pn:` signal letters (grblHAL-extended): `P` probe, `O` probe disconnected, `X/Y/Z/A/B/C/U/V/W` limit switches, `D` door, `R` reset, `H` feed hold, `S` cycle start, `E` e-stop, `L` block delete, `T` optional stop, `M` motor warning, `F` motor fault, `Q` single-step.
- grblHAL-only elements: `WCS:Gn` (on coordinate-system change), `MPG:0|1` (pendant control handoff), `SD:` (SD streaming status/percent), `H:` (homing status + homed-axis mask), `D:` (radius/diameter lathe mode), `Sc:` (scaled axes), `TLR:`, `FW:grblHAL` (only in 0x87 full report, only if compatibility level < 2), `In:` (last M66 result).

**WCO reporting rules.** Per the gnea/grbl v1.1 wiki, `WCO:` is reported: "In every 10 or 30 (configurable 1-255) status reports, depending on if Grbl is in a motion state or not. Immediately in the next report, if an offset value has changed. In the first report after a reset/power-cycle." This lets a host that only ever sees `MPos:` compute `WPos` and vice-versa. The `$10` mask controls which fields appear; in grbl 1.1+ you can no longer arbitrarily mask out MPos/WPos (one is always present).

**`$10` status report mask bits (grblHAL):** bit0 machine position, bit1 buffer state (`Bf:`), bit2 line numbers (`Ln:`), bit3 feed & speed (`FS:`), bit4 pin state (`Pn:`), bit5 work coord offset (`WCO:`), bit6 overrides (`Ov:`), bit7 probe coordinates, bit8 sync-on-WCO-change, bit9 parser state (push `$G` after a status request), bit10 alarm substate, bit11 run substate. A common grblHAL value is `$10=511`.

**Polling.** grbl recommends hosts poll `?` at **no more than 5–10 Hz** (the grbl docs say poll "no more than 5-10Hz to avoid overwhelming Grbl"); above that there are diminishing returns and higher CPU load. The `?` is answered immediately *except while homing*, when status requests are queued and not answered (so a sender's DRO will not update during `$H` — document this).

**Auto-report interval (grblHAL-specific):** setting **`$481` (`Setting_AutoReportInterval`)** — grblHAL's `config.h` defines it verbatim as "$481 - Setting_AutoReportInterval · Auto status report interval, allowed range is 100 - 1000. Set to 0 to disable", with `#define DEFAULT_AUTOREPORT_INTERVAL 0` (units = milliseconds, default disabled). (Note `$480` is `Setting_Fan0OffDelay`, *not* the report interval — do not confuse them.) When enabled, the controller pushes status reports on its own without `?` polling; the mode can also be toggled at runtime with real-time byte `0x8C`. Implementing `$481` lets your firmware drive the host DRO without poll traffic.

### 5. Startup / handshake sequence

**Banner.** On power-up and on every soft reset, emit the welcome line. grblHAL emits `GrblHAL 1.1f ['$' or '$HELP' for help]` (legacy grbl emits `Grbl 1.1f ['$' for help]`). If `COMPATIBILITY_LEVEL >= 1`, grblHAL reports itself as `Grbl` instead of `GrblHAL` for sender compatibility. A host detects readiness by seeing this banner.

**grblHAL connect challenge (critical for ESP32-S3 native USB).** Because many grblHAL boards cannot be hard-reset on connect (native USB / network drops the link; some UART bridges lack DTR/RTS reset wiring), a sender cannot rely on the banner appearing on connect. The grblHAL-recommended sender startup sequence is:
1. On connect, listen ~400–500 ms for incoming traffic. If a real-time *report* arrives, a secondary sender (MPG/pendant) holds the stream — wait and watch for `|MPG:0` to signal release.
2. If the welcome banner arrives, assume the controller was reset and is ready.
3. Send `0x87` and wait ~250 ms for a full real-time report; receiving one proves an extended (grblHAL) controller is present and reveals its state.
4. Request the extended build info with `$I+` to learn capabilities.

Your firmware should: always emit the banner on boot/soft-reset; always answer `0x87` with a full report even when otherwise locked; and answer `$I`/`$I+`.

**`$I` build info.** Legacy grbl `$I` returns `[VER:1.1f.<build>:<string>]` and `[OPT:...]`. grblHAL extends it. A real grblHAL `$I` looks like:
```
[VER:1.1f.20250225:]
[OPT:VNMSL,100,1024,3,0]
[AXS:3:XYZ]
[NEWOPT:ENUMS,RT+,ES,REBOOT,SED,RTC,WIFI,FS,SD]
[FIRMWARE:grblHAL]
[SIGNALS:HSEP]
[NVS STORAGE:*FLASH 4K]
[FREE MEMORY:130K]
[DRIVER:ESP32]
[DRIVER VERSION:250129]
[BOARD:MKS DLC32 2.x]
[AUX IO:1,2,0,0]
...
ok
```
`$I+` adds the `[AXS:]`, `[NEWOPT:]`, `[FIRMWARE:]`, `[SIGNALS:]` lines (the extended report). `[OPT:]` fields = options string, block buffer size, RX buffer size, axis count, tool-table entries. `NEWOPT` advertises capabilities (`ENUMS`, `RT+`, `ES` e-stop, `TC` manual tool change, `ATC`, `SD`, `WIFI`, `HOME`, `LATHE`, `PROBES=n`, `NOPROBE`, etc.). Every multi-line request response ends with `ok`.

**`$N` startup blocks.** `$N` lists stored startup lines (`$N0=...`, `$N1=...`); `$N0=<gcode>` stores one (validated, returns `ok`/`error`). They run at init (or immediately after a successful homing cycle if homing is enabled) and are echoed back as `>G54G20:ok` style lines. They do *not* run if the controller comes up in ALARM or exits ALARM via `$X`.

**ALARM on connect.** If homing is required, or e-stop/limit asserted, the controller comes up in ALARM and prints e.g. `[MSG:'$H'|'$X' to unlock]` (homing-required has no alarm substate; e-stop is `ALARM:10`, homing-required is `ALARM:11`). In locked alarm states (codes 1, 2, 10) the controller responds only to real-time report requests until a soft reset; other alarm states still allow `$` commands. A host must detect ALARM on connect and prompt the user to `$H` (home) or `$X` (unlock) rather than streaming.

### 6. Settings and configuration (`$`-commands)

- **`$$`** dumps all settings, one per line: `$<n>=<value>`. grblHAL can append a human description in parentheses when requested, e.g. `$0=10.0 (Step pulse time, μs)`. The dump ends with `ok`.
- **Writing:** `$<n>=<value>` → `ok` (stored to NVS) or `error:N` (e.g. `error:3` invalid statement, value out of range). On ESP32-S3 use NVS/flash emulation.
- **`$RST=` resets:** `$RST=$` restores `$$` settings to defaults; `$RST=#` zeros G54–G59 work offsets and G28/G30 positions; `$RST=*` clears/restores all NVS data (settings, parameters, startup lines, build info). The controller auto-resets after a `$RST`. grblHAL also has `$RST=&` for driver-specific settings. Each can be individually disabled at compile time.
- **grblHAL extended settings beyond legacy grbl:** legacy grbl uses `$0`–`$132`. grblHAL adds many: `$4`/`$5` become per-axis bitmasks (enable/limit inversion per axis), `$14`/`$17` control-signal masks, `$22` homing as an exclusive bitmask (bit0 enable, bit1 single-axis, bit2 home-on-startup, bit3 set-origin, bit6 override-locks, bit7 keep-homed-on-reset), `$28` G73 retract, `$39` enable printable real-time chars, `$40` soft-limit jogging, `$43` homing locate cycles, `$44`–`$49` homing pass axis masks, `$60`–`$65` (restore overrides, door handling, sleep, laser-during-hold, force-alarm-on-startup, feed override during probe), `$62` sleep enable, `$70` network services mask, `$73`–`$77` WiFi, `$307` websocket port, `$330` admin password, `$339` sensorless homing, `$340` spindle at-speed tolerance, `$341` tool-change mode (0 normal, 1 manual, 2 manual@G59.3, 3 auto-touch-off@G59.3, 4 ignore M6), `$481` auto-report interval, `$482` timezone, `$484` unlock-after-estop, and more. (The brief's "$340 spindle type" is actually "spindle at speed tolerance"; spindle type/enumeration is handled by `$SPINDLES`/`$SPINDLESH`.)
- **Runtime enumeration (grblHAL, strongly recommended over hardcoding):** `$ES`/`$ESG`/`$ESH` enumerate settings (id, group, name, unit, datatype, format, min, max, reboot-required, null-allowed); `$EE`/`$EEG` enumerate error codes; `$EA`/`$EAG` enumerate alarm codes; `$EG` enumerates setting groups; `$SED=<n>` returns a description for setting n. Formats include grbl-CSV-compatible and grblHAL tab-separated. The grblHAL wiki explicitly tells senders: "Do *not* hardcode a settings user interface" — fetch enumerations from the controller (advertised by `ENUMS`/`SED` in NEWOPT), falling back to the published CSV files.
- **Galdr spindle delays (`$392`/`$393`):** `$392` is the spindle-on / spin-up delay in seconds (grbl-aligned): after an M3/M4, Galdr inserts a synchronized dwell of this length before the first cutting move so the spindle reaches speed before it cuts (default `0`). `$393` is a Galdr-specific M3↔M4 reverse spin-down dwell in seconds: a running spindle is forced to a stop and parked this long before the opposite direction is energized (default `1.5`; a configured value below ~0.5 s is floored in firmware so a reversal is never instantaneous). Both appear in `$$`/`$ES`.

Example enumeration lines:
```
[ALARMCODE:1|Hard limit|Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended.]
[ERRORCODE:1|Expected command letter|G-code words consist of a letter and a value. Letter was not found.]
[SETTING:0|18|Step pulse time|microseconds|6|#0.0|2.0|]
[SETTINGGROUP:1|0|General]
```

### 7. Error and alarm handling during streaming

- **`error:N` mid-job.** grbl reports `error:N` and *dumps that block*. A host should **halt the stream immediately** — professional practice, because the dumped block may have carried positioning/feed that following lines depend on. **grblHAL difference:** the error condition *persists* for all subsequent G-code lines until a reset, an empty line, or a `$` command is issued — a safety measure to prevent dangerous moves after a bad line. (Legacy grbl keeps accepting following lines.) The character-counting caveat: because lines are already stuffed in the RX buffer, the firmware keeps consuming them; a host cannot "unsend" them — the classic mitigation is to pre-check the whole file with `$C` check mode (parses/error-checks without moving), and grblHAL's persistent-error behavior further contains the damage.
- **`ALARM:N`.** An alarm halts everything and blocks G-code until cleared. Critical alarms (hard limit `ALARM:1`, soft limit `ALARM:2`, abort/e-stop, reset-while-moving `ALARM:3`) print `[MSG:Reset to continue]` and require a soft reset; the machine enters the `Alarm` state. Streaming must stop. Probe-fail alarms are `ALARM:4` (probe already triggered at start) and `ALARM:5` (probe not tripped within travel). Homing alarms: `ALARM:6` reset during homing, `ALARM:8`/`ALARM:9` homing fail. grblHAL adds `ALARM:10` (e-stop) and `ALARM:11` (homing required).
- **`$X` kill alarm lock.** Valid when in a (non-critical-locked) alarm state; clears the lock and replies `[MSG:Caution: Unlocked]` then `ok`. It does *not* run startup lines (reset afterward for those). `$X` is invalid/limited while a critical event (limit/e-stop still asserted) holds the controller locked — in those states only a soft reset (after clearing the physical cause) works. `$H` runs the homing cycle and is the proper way to clear a homing-required alarm.
- **How hosts handle it.** UGS historically had a bug where it kept streaming after a reset/abort that did not raise an alarm (e.g. during a `G4` dwell), sending inch-mode g-code to a controller that had reverted to mm — the fix: when the host sees the welcome banner mid-stream, it must stop streaming (the banner means the controller reset and its modal state may have changed). ioSender's Stop button does a staged stop (first stops feeding new lines, second resets). Lesson for your firmware: always emit the banner on soft reset so hosts can detect it and abort; raise an alarm on reset-while-moving so the host cannot blindly continue.
- **Galdr G4 / M30 (synchronized).** `G4` and the `$392` spin-up are real synchronized boundaries: the firmware blocks the stream — no `ok` for that line until prior motion has fully drained *and* the dwell has elapsed — so the host's character-counting naturally stalls (this is correct flow control, not a hang). A `0x18` mid-dwell aborts the line and resets. `M30` likewise drains motion, then stops the spindle, resets modal state to defaults, re-selects G54, and clears overrides + coolant before the `ok` (`[MSG:Pgm End]`-equivalent rewind) — a host should treat M30 like a reset of modal state. In `$C` check mode neither dwell blocks and the spindle never energizes (validation moves/actuates nothing).

- **Galdr extension — inline error/alarm context (`[MSG:error:N …]` / `[MSG:ALARM:N …]`).** To make a rejection legible even on a plain terminal that does not fetch the `$EE`/`$EA` enumeration, the firmware emits a context **push** line carrying the short code name next to the response/alarm line. The two paths order it differently, by design: for an error the context comes **before** the response (`[MSG:error:21 Modal group violation]` then `error:21`) so a terminal that stops reading at the `error:N` still sees the gloss; for an alarm the halting `ALARM:N` comes **first** (it is the primary signal of the halt) and the context follows (`ALARM:1` then `[MSG:ALARM:1 Hard limit]`), ahead of the standard `[MSG:'$H'|'$X' to unlock]` / `[MSG:Reset to continue]` prompt. This is a pure push message — it is **not** a flow-control response (only the byte-exact `error:N`/`ok` move the host's character-count window), and a sender that decodes the code itself ignores it. The name is sourced from the same `ERROR_CODES`/`AlarmCode` tables that back `$EE`/`$EA`, so the inline text always matches the enumeration. One exception: a line rejected purely because the stream is held in the post-error state is emitted **bare** (no context), because its reused halt code (`error:1`) does not describe why that particular line was refused — annotating it would mislead. `skirnir` decodes `error:N`/`ALARM:N` itself (built-in table enriched from `$EE`/`$EA`) and therefore suppresses this redundant annotation from its console.

- **Galdr note — code choices.** The firmware emits grbl-canonical codes rather than collapsing distinct failures: a modal-group clash (e.g. two motion words on one line) is `error:21` "Modal group violation" — *not* `error:9`, which is reserved for "G-code locked out during alarm or jog state"; a probe/jog command with no axis word is `error:26` "No axis words in block" — *not* `error:23` (which means "requires an integer value"); a degenerate arc target is `error:33` "Invalid target". Every code the firmware can emit (parser, planner, settings, and the `$`-dispatch/state-lock paths) has a row in the `$EE` table, so a sender that builds its display from the enumeration always has matching text.

### 8. Coordinate systems and modal state

- **`$G` parser-state report:** `[GC:<modal state>]`, e.g. `[GC:G0 G54 G17 G21 G90 G94 G49 G98 G50 M5 M9 T0 F0 S0.]`. grblHAL extends the reported set with codes legacy grbl omitted: `G5 G7 G8 G43 G49 G50 G51 G73 G81 G82 G83 G85 G86 G89 G96 G97 G98 G99 M1 M50 M51 M53 M56 M60`. `$10` bit9 makes grblHAL *push* the `$G` report automatically after a status request whenever modal state changed; real-time byte `0x83` requests it on demand. A host should handle (or opt out of) this push. **Galdr** reports the live spindle modal word (group 7) — `M3`/`M4`/`M5` — in this line (not a fixed `M5`); the `S` word is the modal programmed speed, which can be set without starting the spindle (so `M5 ... S8000` is valid).
- **Restoring modal state after reset:** after a soft reset the parser returns to defaults (G54, G17, G21/G20 per config, G90, G94, M5, M9). A host that reset mid-job must re-establish modal state (units, plane, coordinate system, feed) before resuming — this is why detecting the banner and stopping is critical. Jogging (`$J=`) is independent of modal state by design, so it never disturbs the parser.
- **`$#` NGC parameters:** returns `[G54:...]`…`[G59:...]`, `[G28:...]`, `[G30:...]`, `[G92:...]`, `[TLO:...]`, `[PRB:...:success]`; grblHAL adds `[G59.1:]`/`[G59.2:]`/`[G59.3:]`, `[G51:]` (scaling), `[HOME:<positions>:<mask>]`, and `[T:<n>,<offsets>]` tool table entries.
- **`[MSG:...]` messages:** informational push messages, always in `[]`. Examples: `[MSG:'$H'|'$X' to unlock]`, `[MSG:Caution: Unlocked]`, `[MSG:Reset to continue]`, `[MSG:Check Limits]`, `[MSG:Check Door]`, `[MSG:Pgm End]` (M2/M30), `[MSG:Enabled]`/`[MSG:Disabled]` (check mode). The startup-line execution echo is the exception that uses `>` not `[`: `>G54G20:ok`.

### 9. Probing (G38.x) — for the PCB Z-probe

- **Commands:** `G38.2` probe toward, stop on contact, **error/alarm if no contact**; `G38.3` probe toward, stop on contact, **no error if no contact**; `G38.4` probe *away*, stop on loss of contact, error if it stays in contact; `G38.5` probe away, no error. For Z touch-off use `G38.2 Z-5 F50` (often with `G91` for incremental); for height-mapping a PCB you repeat probe-up/move/probe cycles.
- **`[PRB:...]` result:** `[PRB:<x>,<y>,<z>{,<a>...}:<success>]`, machine-coordinate position at the trigger instant, with a trailing `:1` (success) or `:0` (fail). Examples: `[PRB:-1.015,0.000,0.000:1]`, `[PRB:0.000,0.000,0.000:0]`. This is a push message emitted immediately after the probe completes (grblHAL can disable the immediate push; the value is always retrievable via `$#`). NOTE: the reported point is where the probe *triggered*; the machine decelerates past it, so the actual stop position differs slightly — account for this in height maps (overtravel is proportional to feed rate).
- **Failure behavior:** `G38.2`/`G38.4` failure → `ALARM:4`/`ALARM:5`, halts, requires `$X`/reset. `G38.3`/`G38.5` failure → no alarm, `[PRB:...:0]` only. For robust PCB auto-leveling, prefer `G38.2` (so a missed contact stops the job rather than crashing).
- **Reading back:** the host parses the `[PRB:...]` line directly after each probe (PCB height-mappers capture the Z from the third PRB field), or queries `$#` to read the last probe result.

### 10. grblHAL protocol extensions vs legacy grbl (summary for parsing)

Bracketed push-message types you must produce/parse: `[GC:]` parser state, `[G54:]`…`[G59.3:]`/`[G28:]`/`[G30:]`/`[G92:]` parameters, `[TLO:]` tool length offset, `[PRB:]` probe, `[MSG:]` messages, `[VER:]`/`[OPT:]`/`[AXS:]`/`[NEWOPT:]`/`[FIRMWARE:]`/`[SIGNALS:]`/`[DRIVER:]`/`[BOARD:]`/`[PLUGIN:]` build info, `[HOME:]`, `[ALARMCODE:]`/`[ERRORCODE:]`/`[SETTING:]`/`[SETTINGGROUP:]` enumerations, `[PINSTATE:]`/`[PORT:]`/`[SPINDLE:]` hardware enumerations. General parsing rules grblHAL publishes for senders: do not assume tag/element order; ignore unknown tags and unknown values; expect comma-separated lists to grow; expect single-character lists (pin states) to expand. Following these means future extensions won't break a parser.

Streaming-model differences vs legacy grbl, consolidated: larger RX buffer reported in OPT (don't assume 128); CRLF/LFCR = one terminator; persistent error state after an error; `0x19` stop vs `0x18` reset; top-bit-set real-time commands (0x80–0x8C, advertised by RT+); printable `?`/`~`/`!` ignored while reading `$`/message input; optional auto-report (`$481`/`0x8C`); `Tool` state and tool-change protocol (`0xA3` ack); homing status tracking (`H:` element, `Home` state pushed before homing); runtime code/settings enumeration.

### 11. Host software reference implementations

- **ioSender** (terjeio/ioSender, C#, by grblHAL's author Terje Io) is the reference grblHAL sender. It parses the `[OPT:...]` line to extract block-buffer and RX-buffer sizes (documented in issue #356, which shows it reading blockBufferSize then rxBufferSize then axis count from the comma-separated OPT values) and sizes its streaming around the controller-reported buffer. Its "Aggressive Buffering" option fills the controller RX buffer as full as possible (character-counting send-ahead) for maximum throughput; off, it is closer to send-response. It builds its settings UI dynamically from the controller's enumerations, and implements grblHAL probing, tool change, and up-to-6-axis DRO. A grblHAL simulator (buildable via the Web Builder) lets you test ioSender without hardware.
- **Universal Gcode Sender (UGS)** (winder/Universal-G-Code-Sender, Java) uses its `GrblController`/`GrblCommunicator` with a buffered streaming model and detects grbl capabilities/version at connect (`GrblUtils.getGrblStatusCapabilities`). Known gotcha (issue #1341): UGS continued streaming after a reset/abort that didn't alarm — the fix hinges on detecting the welcome banner mid-stream and stopping. It auto-uses grbl 1.1 jog mode when detected.
- **bCNC** (vlachoudis/bCNC, Python) uses character-counting streaming and is widely used for probing/auto-leveling; a documented gotcha is its tool-change macro sending `G38.5` that alarms on some setups.
- **Common bugs/gotchas your firmware should be defensive about:** the double-`ok` bug from CRLF on legacy grbl (grblHAL fixes by treating CRLF as one terminator — make sure you do too); hosts mis-counting bytes if the host's line length differs from what the firmware buffers (count the terminator consistently); senders that crash on `[OPT:...]` when axis count parses wrong (emit OPT in the exact documented field order); not answering `?` during homing (expected — document it); and senders assuming a hard reset on connect (emit the banner on boot and answer 0x87/`$I+`).

## Recommendations

**Stage 1 — Minimum viable grbl 1.1 streaming (get a sender to connect and run a job):**
1. Implement the byte-level RX scanner first: divert real-time bytes (`0x18`, `?`, `~`, `!`, `0x84`, `0x85`, `0x90`–`0xA1`, and grblHAL `0x80`–`0x8C`, `0x19`) before the line buffer; accumulate the rest into a line buffer terminated by CR/LF/CRLF/LFCR (one terminator).
2. Emit exactly one `ok` or `error:N` per consumed line. Implement the welcome banner on boot and on `0x18`. Implement `?` → a minimal `<Idle|MPos:0.000,0.000,0.000|FS:0,0>` report.
3. Implement `$$`, `$<n>=<value>`, `$G` → `[GC:...]`, `$I`/`$I+`, `$#`, `$H`, `$X`, `$RST=*`. End every multi-line response with `ok`.
4. Choose RX buffer = 1024 bytes; report it accurately in `[OPT:...]` and `Bf:`.
*Benchmark to advance:* ioSender and UGS connect, show the DRO updating, and run a simple G-code file to completion via simple send-response.

**Stage 2 — grblHAL fidelity and robustness:**
5. Add the full status-report element set with correct change-only/intermittent rules (especially WCO refresh on change + after reset, and the `$10` mask). Implement `0x87` full report and persistent-error-after-error semantics.
6. Implement the alarm state machine (ALARM:1–11 with `[MSG:Reset to continue]`/`[MSG:'$H'|'$X' to unlock]`), the locked-state rule (respond only to report requests on codes 1/2/10), and emit the banner on every soft reset so hosts abort streaming correctly.
7. Implement runtime enumerations `$ES`/`$EE`/`$EA`/`$EG`/`$SED` and advertise `ENUMS,RT+,SED` in `NEWOPT` — this is what makes modern ioSender/gSender build their UI without hardcoding.
*Benchmark to advance:* character-counting (Aggressive Buffering) streaming of a dense file runs without buffer overflow or planner starvation; a mid-job `error` halts cleanly.

**Stage 3 — probing and polish:**
8. Implement `G38.2/.3/.4/.5` with `[PRB:x,y,z:success]` push and correct ALARM:4/5 behavior; verify height-map probing in ioSender/bCNC reads back PRB Z values.
9. Implement `$481` auto-report + `0x8C` toggle for low-latency DRO; implement the jog protocol (`$J=`) so jogging never disturbs modal state.
10. Verify with the **grblHAL simulator** and against ioSender, UGS, and bCNC before hardware.

**Thresholds that change the plan:** If you set `COMPATIBILITY_LEVEL`-equivalent behavior to report as `Grbl` (not `GrblHAL`), drop the grblHAL-only `$I+` lines and `FW:` element and you'll work with more legacy senders but lose advanced UI. If your NVS write path can drop RX bytes, you must document "send settings with send-response only" and reject character-counted setting writes.

## Caveats
- The grblHAL "For sender developers" wiki page is explicitly marked **DRAFT** (last edited Nov 2023); its connect-handshake advice is provisional, though the `0x87`/`0x8C`/banner facts are corroborated elsewhere. Treat the exact 400–500 ms / 250 ms timings as guidance, not a hard spec.
- The RX buffer default of 1024 is empirically confirmed on multiple ARM drivers and is the grblHAL norm, but it is a **per-driver compile-time constant**, not a guaranteed universal default — that's exactly why the protocol advertises it in `[OPT:...]` and why hosts must read it at runtime. Choose your ESP32-S3 value freely; just report it truthfully.
- Settings numbers above are accurate as of recent builds (2024–2025), but grblHAL adds settings over time and many are driver/plugin-conditional; rely on `$ES`/CSV enumeration as the source of truth rather than a static list. `$10=511` and specific masks vary by build; verify exact bit meanings against `$ES` for your target build.
- Some details (max line length, exact line-overflow error number) come from the broader grbl family and the AVR line-buffer constant; grblHAL's larger buffers change the numbers, so enforce a limit and return the documented overflow error rather than copying 80 verbatim.
- "Baud rate irrelevance" applies to *native USB CDC*; if you ever route through a UART-USB bridge, baud does matter for that link. The ESP32-S3 native USB path is the assumed transport here.