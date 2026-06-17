# Vendored fonts

skirnir bundles two open-source typefaces, compiled into the binary via `include_bytes!` (see
`src/app/fonts.rs`). They back the egui font stack: Roboto is the proportional UI face, JetBrains Mono is the
monospace face used for the DRO digits, the console, and the status strip (the design calls for tabular figures
in those readouts, which JetBrains Mono provides).

## Roboto

- Faces: `Roboto-Regular.ttf` (400), `Roboto-Medium.ttf` (500), `Roboto-Bold.ttf` (700).
- Source: googlefonts Roboto v2.136 hinted static release
  (https://github.com/googlefonts/roboto-2/releases/tag/v2.136).
- License: SIL Open Font License 1.1 — `roboto/LICENSE.txt`. Roboto was relicensed from Apache-2.0 to OFL-1.1
  in 2024; the vendored OFL text is authoritative.
- Copyright 2011 The Roboto Project Authors.

## JetBrains Mono

- Faces: `JetBrainsMono-Regular.ttf` (400), `JetBrainsMono-Medium.ttf` (500), `JetBrainsMono-Bold.ttf` (700).
- Source: JetBrains Mono v2.304 release
  (https://github.com/JetBrains/JetBrainsMono/releases/tag/v2.304).
- License: SIL Open Font License 1.1 — `jetbrains-mono/LICENSE.txt`.
- Copyright 2020 The JetBrains Mono Project Authors.

Both licenses permit bundling and redistribution within an application. Keep the `LICENSE.txt` files alongside
the font binaries so the repository stays license-compliant.
