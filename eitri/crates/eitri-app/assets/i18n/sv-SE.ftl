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
vis-hide = Dölj { $name }
vis-show = Visa { $name }

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

# ── Operationsväljare + panelerna för fler operationer ──────────────────────────────────────────────────
params-operation = Operation
op-choice-isolate = Isolera
op-choice-paint = Målning
op-choice-noncopper = Röj ej-koppar
op-choice-cutout = Utfräsning
op-choice-panelize = Panelisering
op-choice-mirror = Spegling
op-choice-film = Filmexport
params-paint = Ytröjning
params-margin = Marginal (mm)
params-strategy = Strategi
strategy-concentric = Koncentrisk
strategy-seed = Frö
strategy-raster = Raster
params-raster-angle = Rastervinkel (°)
params-finish-pass = Slutpass
btn-run-paint = Måla
tip-run-paint = Röj kopparytan och skapa ett CNC-jobb
params-noncopper = Ej-kopparröjning
params-boundary = Gräns
boundary-bbox = Begränsningsruta
boundary-object = Objektsiluett
boundary-object-none = välj ett objekt…
params-boundary-margin = Marginal för ruta (mm)
params-boundary-source = Gränsobjekt
btn-run-noncopper = Röj ej-koppar
tip-run-noncopper = Röj allt utom kopparn innanför gränsen och skapa ett CNC-jobb
error-boundary-object = Välj ett gränsobjekt med geometri innan du kör ej-kopparröjningen.
params-cutout = Utfräsning av kort
params-outline = Kontur
outline-rectangle = Rektangel
outline-silhouette = Objektsiluett
params-rect-min-x = Rekt. min X (mm)
params-rect-min-y = Rekt. min Y (mm)
params-rect-max-x = Rekt. max X (mm)
params-rect-max-y = Rekt. max Y (mm)
params-tab-width = Flikbredd (mm)
params-tab-count = Flikar
btn-seed-bounds = Använd objektets mått
tip-seed-bounds = Fyll rektangeln från det valda objektets utbredning
btn-run-cutout = Fräs ut
tip-run-cutout = Fräs kortets kontur med hållflikar och skapa ett CNC-jobb
error-outline-object = Det valda objektet har ingen siluettgeometri att fräsa runt.
params-panelize = Panelisering
params-rows = Rader
params-cols = Kolumner
params-spacing-mode = Avstånd
spacing-gap = Mellanrum
spacing-pitch = Delning
params-spacing-x = Avstånd X (mm)
params-spacing-y = Avstånd Y (mm)
btn-run-panelize = Panelisera
tip-run-panelize = Ordna objektet i ett rutnät som ett nytt geometriobjekt
params-mirror = Spegling (dubbelsidig)
params-mirror-axis = Axel
axis-vertical = Vertikal (vänd X)
axis-horizontal = Horisontell (vänd Y)
params-mirror-value = Linje vid (mm)
btn-seed-center = Objektets centrum
tip-seed-center = Placera spegellinjen vid det valda objektets centrum
btn-run-mirror = Spegla
tip-run-mirror = Spegla objektet kring linjen som ett nytt geometriobjekt

# ── Uppställningsnoden (ämne & arbetsnolla) ─────────────────────────────────────────────────────────────
tree-setup = Uppställning
setup-kind = Jobbuppställning — ämne & arbetsnolla
setup-hint = Materialblocket som bearbetas och var arbets-X0 Y0 Z0 sitter på det. Objekten flyttas aldrig — bara koordinaterna som matas ut i G-koden förskjuts.
params-stock = Ämne
params-stock-x = Storlek X (mm)
params-stock-y = Storlek Y (mm)
params-stock-thickness = Tjocklek (mm)
btn-fit-stock = Anpassa till kort
tip-fit-stock = Storleksätt ämnet efter det valda objektets utbredning
params-datum = Nollpunkt (X0 Y0)
datum-corner-bl = Nedre vänster
datum-corner-br = Nedre höger
datum-corner-tl = Övre vänster
datum-corner-tr = Övre höger
datum-corner-center = Mitten
params-z-zero = Z-nolla
z-zero-top = Ämnets ovansida
z-zero-bottom = Ämnets undersida
work-zero-label = Arbetsnolla (mm)
work-zero-native = Ursprungsram — ingen förskjutning
btn-stock-native = Ursprungsram (inget ämne)
tip-stock-native = Mata ut G-kod i källfilens eget koordinatsystem
stock-set = Ämne satt — arbetsnolla vid X { $x } Y { $y } Z { $z }
stock-cleared = Ämne rensat — utmatning i ursprungliga koordinater
params-film = Fotofilm
params-film-kind = Typ
film-positive = Positiv
film-negative = Negativ
params-film-scale = Skala
params-film-mirror = Spegla
params-film-border = Ram (mm)
btn-export-film = Exportera film-SVG…
tip-export-film = Skriv en positiv/negativ film av kopparn som en SVG-fil

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
op-paint = Målning
op-noncopper = Röjning av ej-koppar
op-cutout = Utfräsning
op-panelize = Panelisering
op-mirror = Spegling
op-load-project = Öppna projekt

# ── Docka ───────────────────────────────────────────────────────────────────────────────────────────────
dock-log = Logg
dock-gcode = G-kod
gcode-empty = Välj ett CNC-jobb för att förhandsvisa dess G-kod.
log-ready = Redo.

# ── Arbetsyta ───────────────────────────────────────────────────────────────────────────────────────────
canvas-label = Arbetsyta
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

# ── Verktygsdatabas ─────────────────────────────────────────────────────────────────────────────────────
btn-tools = Verktyg
tip-tools = Hantera verktygsbiblioteket
tool-db-title = Verktygsdatabas
tool-db-empty = Inga verktyg ännu. Lägg till ett för att bygga biblioteket.
tool-db-add = Lägg till verktyg
tool-db-remove = Ta bort verktyg
tool-db-save-hint = Skriv verktygsbiblioteket till disk
tool-db-select = Välj ett verktyg för att redigera det.
tool-db-name = Namn
tool-db-diameter = Diameter (mm)
tool-db-iso-defaults = Isolationsstandard
tool-db-drill-defaults = Borrstandard
tool-db-new-name = nytt verktyg
btn-seed-tool = Hämta från verktyg
seed-tool-hint = välj ett verktyg…
