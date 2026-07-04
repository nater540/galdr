# Real Gerber fixtures (per producing tool)

Drop **real** sample Gerber sets here, one subdirectory per producing EDA tool, e.g.:

```
gerber/kicad/      *.gbr from KiCad
gerber/altium/     *.gbr from Altium
gerber/eagle/      *.gbr from Eagle / older tools
```

Note the producing tool (and ideally its version) alongside each set. These exercise the parser against real-world
files — different tools stress different corners (aperture macros, `G36/G37` pours, zero-suppression variants). The
"parse a real KiCad Gerber into a copper `MultiPolygon`" acceptance check runs over the KiCad set once it lands.

Hand-authored spec-corner fixtures live in `../synthetic/gerber/`, not here.
