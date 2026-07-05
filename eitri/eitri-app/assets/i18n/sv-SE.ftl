# eitri-app — Svenska. Full översättning av varje en-US-nyckel (paritet bevakas av ett test).

app-title = Eitri

# ── Verktygsfält ────────────────────────────────────────────────────────────────────────────────────────
btn-open-gerber = Öppna Gerber…
btn-open-excellon = Öppna Excellon…
btn-import = Importera
btn-import-svg = SVG…
btn-import-dxf = DXF…
btn-import-gcode = G-kod…
btn-project-open = Öppna projekt…
btn-project-save = Spara projekt…
btn-undo = Ångra
btn-redo = Gör om
btn-zoom-fit = Anpassa vy
tip-settings = Programinställningar
tip-undo = Ångra senaste ändringen
tip-redo = Gör om den senast ångrade ändringen
tip-zoom-fit = Zooma arbetsytan till den inlästa geometrin

# ── Projektträd ─────────────────────────────────────────────────────────────────────────────────────────
tree-title = Projekt
tree-empty = Inga objekt ännu.
tree-empty-hint = Öppna en Gerber- eller Excellon-fil för att börja.
kind-gerber = Gerber
kind-excellon = Excellon
kind-geometry = Geometri
kind-cncjob = CNC-jobb
btn-delete-object = Ta bort objekt

# ── Parameterpaneler ────────────────────────────────────────────────────────────────────────────────────
params-title = Parametrar
params-empty = Välj ett objekt för att redigera dess parametrar.
params-busy = En operation körs — parametrarna låses upp när den är klar.
params-isolation = Isolationsfräsning
params-tool-diameter = Verktygsdia. (mm)
params-passes = Pass
params-overlap = Överlapp
params-combine = Kombinera pass
params-direction = Riktning
direction-climb = Medfräsning
direction-conventional = Motfräsning
params-cut-depth = Skärdjup (mm)
params-pass-depth = Per pass (mm)
params-cut-feed = Skärmatning (mm/min)
params-plunge-feed = Nedmatning (mm/min)
params-travel-z = Transport-Z (mm)
params-spindle-rpm = Spindel (varv/min)
btn-run-isolation = Isolera
tip-run-isolation = Beräkna isolationsbanor och skapa ett CNC-jobb
params-drill = Borrning
params-drill-depth = Djup (mm)
params-drill-feed = Matning (mm/min)
params-drill-retract = Retur-Z (mm)
btn-run-drill = Borra
tip-run-drill = Ordna borrhålen och skapa ett borrjobb
params-job = CNC-jobb
job-lines = { $count ->
    [one] 1 rad G-kod
   *[other] { $count } rader G-kod
  }
job-dialect = Dialekt: { $dialect }
btn-export-gcode = Exportera G-kod…
tip-export-gcode = Skriv jobbets G-kod till en fil
params-geometry = Geometri
geometry-shapes = { $polygons } polygoner · { $polylines } polylinjer
excellon-hits = { $hits } borrhål · { $tools } verktyg
gerber-info = Kopparytor tolkade och cachade

# ── Operationer / förlopp ───────────────────────────────────────────────────────────────────────────────
op-running = Kör: { $label }
btn-cancel = Avbryt
tip-cancel = Begär att den pågående operationen avbryts
op-cancelled = { $label } avbröts
op-failed = { $label } misslyckades: { $reason }
op-done = { $label } klar
op-open-gerber = Öppna Gerber
op-open-excellon = Öppna Excellon
op-import-svg = Importera SVG
op-import-dxf = Importera DXF
op-import-gcode = Importera G-kod
op-isolate = Isolationsfräsning
op-drill = Borrplanering
op-load-project = Öppna projekt

# ── Docka ───────────────────────────────────────────────────────────────────────────────────────────────
dock-log = Logg
dock-gcode = G-kod
gcode-empty = Välj ett CNC-jobb för att förhandsvisa dess G-kod.
log-ready = Redo.

# ── Arbetsyta ───────────────────────────────────────────────────────────────────────────────────────────
canvas-empty-title = Inget att visa ännu
canvas-empty-hint = Öppna en Gerber-, Excellon-, SVG- eller DXF-fil så visas den här.

# ── Statusrad ───────────────────────────────────────────────────────────────────────────────────────────
status-objects = { $count ->
    [0] inga objekt
    [one] 1 objekt
   *[other] { $count } objekt
  }
status-units = mm
status-idle = vilande

# ── Filer / fel ─────────────────────────────────────────────────────────────────────────────────────────
file-filter-gerber = Gerber
file-filter-excellon = Excellon
file-filter-svg = SVG
file-filter-dxf = DXF
file-filter-gcode = G-kod
file-filter-project = Eitri-projekt
error-open-file = Kunde inte läsa { $path }: { $reason }
error-save-file = Kunde inte skriva { $path }: { $reason }
export-done = Skrev { $path }
project-saved = Projektet sparades till { $path }
project-loaded = Projekt inläst: { $path }

# ── Programinställningar ────────────────────────────────────────────────────────────────────────────────
app-settings-title = Programinställningar
app-settings-language = Språk
app-settings-theme = Tema
app-settings-font-scale = Textskala
app-settings-new-theme-hint = nytt temanamn
app-settings-create = Skapa från aktuellt
app-settings-create-hint = Spara de aktiva färgerna som ett redigerbart tema
app-settings-save = Spara
app-settings-save-hint = Skriv konfigurationsfilen
app-settings-unsaved = osparade ändringar
app-settings-builtin-hint = Inbyggda teman är skrivskyddade — skapa en kopia ovan för att redigera färger.
theme-group-chrome = Ytor & ramar
theme-group-accents = Accenter
theme-group-text = Text
theme-group-status = Status
theme-group-canvas = Arbetsyta
theme-group-other = Övrigt
