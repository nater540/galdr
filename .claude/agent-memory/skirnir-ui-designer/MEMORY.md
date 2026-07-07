# skirnir-ui-designer memory index

- [Snapshot harness](snapshot-harness.md) — GPU snapshot suite: run/regenerate commands, kittest 8px inset, fixture rules
- [egui 0.34 layout gotchas](egui-034-layout-gotchas.md) — RTL horizontal(), TextEdit sizing, panel resize band, dual dock ids, glyph coverage, no rustfmt
- [egui-elegance verdict](egui-elegance-verdict.md) — passed on adopting the widget crate (theming coherence); rationale + when to revisit
- [egui_tiles verdict](egui-tiles-verdict.md) — REVERSED: tiles 0.15 now hosts the viewport/console split after 3 failed panel fixes; shape + persistence choices
- [egui disabled styling](egui-disabled-styling.md) — explicit RichText/fill colors defeat egui's disabled fade; hand-style gated states and snapshot them
- [View data threading](skirnir-view-data-threading.md) — shell-owned data reaches views via ShellPanelsData borrows, never UiState mirrors; modal staging + disconnect teardown
- [Setup dialogs](setup-dialogs.md) — right column decluttered: overrides inline, 5 probing wizards moved to launched dialog windows; running wizard forces its dialog open
- [eitri-app mirror](eitri-app-mirror.md) — eitri/eitri-app mirrors skirnir UI (eframe 0.35); SessionSlot op runner, contained() vs 0.35 panel bleed, snapshot cmds
