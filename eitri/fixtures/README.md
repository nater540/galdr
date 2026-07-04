# Eitri fixtures

Parser and CAM test corpora live here. Populated in Phase 2 onward:

- `gerber/` — real RS-274X files from KiCad, Eagle, Altium, and older EDA tools, to stress the aperture-macro
  interpreter and zero-omitted coordinate decoding (see `docs/eitri-porting-plan.md` §4).
- `excellon/` — drill files with declared and inferred number formats, metric/imperial, leading/trailing
  zero suppression (§5).
- `gcode/` — known-good NC output for postprocessor golden-file tests (§8, §12).

Empty for Phase 1 (core + geo only); the directory exists so later phases have a settled home for corpora.
