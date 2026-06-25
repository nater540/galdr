# Skirnir Engineer Memory

- [Test feature split](skirnir-test-feature-split.md) — gui-gated tests (views/shell/kittest) need `--features gui`; default run skips them silently; never `--workspace`
- [Stream-time ETA seam](skirnir-stream-time-eta-seam.md) — dock ETA via shell `stream_time()`: physics `EtaTimeline` vs acked-rate; reducer stays free of `Instant::now()`
