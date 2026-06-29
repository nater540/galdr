---
name: coordination-two-bughunters
description: Two embedded-bug-hunter agents (blue "bughunter" + yellow "bughunter-2") run on the streaming-lockup investigation; blue owns the §13 record + A/B diagnosis. Check ownership before editing the doc or claiming tasks.
metadata:
  type: project
---

The streaming-lockup investigation has had TWO embedded-bug-hunter instances active at once: blue **bughunter**
(the established owner of `docs/streaming-lockup-investigation.md` §13 + the A/B diagnosis) and yellow
**bughunter-2** (invoked separately by team-lead on the same brief).

**Why:** team-lead invoked a second bug-hunter (bughunter-2) on the new 2026-06-25-night A/B hardware evidence
without flagging that blue bughunter already owned the investigation. The task #19 re-broadcast bughunter-2
received carried a STALE description, so both walked the same ground (independently CONVERGED — Sig A =
write-stage lost-wake the flush-only `6024126` fix misses; Sig B = watchdog dead-zone, B-1 leading). The
convergence corroborated the diagnosis but risked clobbering §13 (both edited it: blue's §13.7 spec superseded
bughunter-2's §13.4-13.6).

**RESOLVED (team-lead, option a, 2026-06-26):** blue **bughunter** is SOLE owner of
`docs/streaming-lockup-investigation.md` (§13 + §13.7) + the primary diagnosis/decode/writeback + tasks #18/#19.
**bughunter-2** stays ONLY as an independent SECOND DECODER — decode each capture in parallel, report your read,
flag divergence from blue LOUDLY (a non-deterministic fault's single capture is not sole basis), but do NOT edit
the doc or churn task ownership. On the implementer side the duplicate firmware agent was stood down:
**fwengineer-2** solely owns the board + tree.

**How to apply:** before editing the §13 record or claiming tasks #18/#19, STOP — blue owns them. A second
bug-hunter instance contributes as a cross-check decoder, NOT by re-editing the shared doc. If you find yourself
re-walking the diagnosis, you are duplicating — check ownership first. Related: [[firmware-two-2s-timeouts]] (the
A/B diagnosis itself), [[streaming-lockup-doc]].
