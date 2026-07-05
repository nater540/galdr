# Real Excellon fixtures (per producing tool)

Drop **real** sample Excellon drill files here, one subdirectory per producing EDA tool, e.g.:

```
excellon/kicad/     *.drl from KiCad
excellon/altium/    *.txt/*.drl from Altium
excellon/eagle/     from Eagle / older tools
```

Note the producing tool (and version). These exercise the number-format inference against real headers — declared
vs undeclared units, leading vs trailing zero suppression, and metric vs imperial — where real files break parsers.

Hand-authored spec-corner fixtures live in `../synthetic/excellon/`, not here.
