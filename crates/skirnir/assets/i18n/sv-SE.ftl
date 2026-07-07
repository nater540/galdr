# Skirnir UI-strängar — sv-SE (svensk översättning av en-US.ftl).
#
# Inbäddas i binären via `include_str!` (se `i18n::BUNDLED_LOCALES`). Varje nyckel här motsvarar en nyckel i
# `en-US.ftl`; saknas en nyckel faller `tr!` tillbaka på `en-US`. Interpolation använder Fluent-placeables:
# `{ $name }` ersätts av det namngivna argument som skickas via `tr!`.
#
# ÖVERSÄTTS INTE (medvetet kvar på engelska/protokoll): GCode/grbl-protokolltecken (`$$`, `$X`, `G10 L2`,
# `ALARM:`, `error:`, `0x18`, `?`, `$ES`), enheter (`mm`, `mm/min`, `RPM`, `°`, `s`), axelbokstäver samt den
# rullande konsolloggen med maskintrafik.

app-title = Skirnir

# Översta verktygsfältet — raden med navigerings-/åtgärdsknappar längst upp i fönstret.
btn-connect = Anslut
btn-disconnect = Koppla från
btn-cancel = Avbryt
btn-identify = Identifiera
btn-open = Öppna…
btn-home = ⌂ Referens
btn-settings = Inställningar
port-choose = Välj port…
tip-refresh-ports = Uppdatera portar
tip-identify = Sök efter grblHAL på vald port (skickar ?/$I)
tip-open = Ladda ett G-kodprogram
tip-home = Kör referenskörning ($H)
tip-settings = Firmware-inställningar ($$)
tip-app-settings = Programinställningar (språk, tema, textskala)

# Programstyrning (Kör/Pausa/Stoppa plus det fristående Nödstopp och Simulera).
transport-run = ▶ Kör
transport-running = ▶ Körs
transport-resume = ▶ Återuppta
transport-hold = ⏸ Pausa
transport-stop = ■ Stopp
transport-abort = ⏹ Nödstopp
transport-simulate = ≈ Simulera
transport-autolevel = ⌗ Nivå
tip-hold = Matningspaus (!)
tip-stop = Stoppa jobbet rent (0x86) — bromsa, töm kön, återgå till Inaktiv
tip-abort = Nödåterställning (0x18) — avbryter till LARM och återställer styrenheten
tip-simulate = Uppskatta jobbtid från maskininställningarna (ingen rörelse — endast värd)
tip-autolevel = Höjdkorrigera programmet mot den avkända höjdkartan före sändning (kräver en mesh)
tip-close-dialog = Stäng

# Maskintillståndets bricka (verktygsfält + statusrad). Versaler enligt designen: etiketten är främsta signalen.
badge-disconnected = FRÅNKOPPLAD
badge-connecting = ANSLUTER
badge-idle = INAKTIV
badge-run = KÖR
badge-jog = JOGG
badge-hold = PAUS
badge-home = REFERENS
badge-door = LUCKA
badge-check = KONTROLL
badge-sleep = VILA
badge-tool = VERKTYGSBYTE
badge-alarm = LARM
badge-error = FEL

# Positionsvisning (DRO).
hdr-dro = Positionsvisning
lbl-limits = GRÄNSLÄGEN
lbl-tool = VERKTYG
dro-tool-none = inget
tip-limit-asserted = Gränslägesbrytare utlöst
tip-limit-clear = Gränsläge fritt
btn-zero-xyz = Nolla XYZ

# Joggpanel.
hdr-jog = Jogg
jog-esc-cancel = esc · avbryt
tip-jog-cancel = Avbryt jogg (0x85)
jog-step-label = Steg (mm · A°)
jog-cont = kont
tip-jog-cont = Kontinuerlig jogg: håll en riktning för att röra, släpp för att stanna

# Åsidosättningspanel.
hdr-overrides = Åsidosättningar
lbl-feed = Matning
lbl-spindle = Spindel
ov-rapid = Snabbmatning { $pct }%
ov-realized-feed = Verklig F
ov-realized-speed = Verklig S

# Inställnings- & avkänningsmeny (högerkolumn): kompakta genvägar till avkännings-/inställningsdialogerna.
hdr-setup = Inställning & avkänning
setup-intro = Guider för nollställning och avkänning. Var och en öppnas i eget fönster.
setup-running = pågår

# Avkänningspanel (Z-nollning).
hdr-probe = Avkänn Z · ingen platta
probe-intro = Z-nollning utan mätplatta. Sänk tills kontakt, sätt Z = 0.
lbl-depth = Djup
lbl-plate = Platta
btn-probe-z = Avkänn Z → sätt arbetsnollpunkt
msg-probing-awaiting = Avkänner… inväntar resultat
msg-probing-awaiting-dot = Avkänner… inväntar resultat.
probe-contact = Kontakt vid [{ $coords }]
probe-work-z-set = Arbets-Z satt.
probe-work-z-unchanged = Arbets-Z oförändrat.
probe-failed = Avkänning misslyckades: { $reason }

# Guide för rotationscentrumsökare (DOC-11 §1.2).
hdr-rotary-center = Rotationscentrumsökare
rotary-intro = Hitta A-centrumlinjen från en dorn med känd diameter fastspänd koncentriskt.
lbl-dowel-dia = Dorn ⌀
lbl-a-angle = A-vinkel
lbl-side-probe-z = Sidoavkänning Z
tip-side-probe-z = Maskin-Z som sidoberöringarna (X/Y) sänks till — måste ligga inom dornens Z-utsträckning. Ett felaktigt värde krockar med detaljen eller missar flanken.
rotary-side-confirm = Sidoavkänning Z { $z } mm är inställt för denna dorn
btn-start-center = Starta centrumsökare
btn-apply-saved-center = Tillämpa sparat centrum
rotary-step-enter-dowel = Jogga till −Y-ytans ansättning, avkänn sedan.
btn-probe-y-left = Avkänn Y (vänster sida)
rotary-step-ready-yright = Jogga till +Y-ytans ansättning, avkänn sedan.
btn-probe-y-right = Avkänn Y (höger sida)
rotary-step-move-yc = Flytta till Y-centrum innan toppen avkänns.
btn-move-yc = Flytta till Y-centrum
rotary-step-moved-yc = Vid Y-centrum. Avkänn dornens topp.
btn-probe-z-top = Avkänn Z (dornens topp)
rotary-step-review = Centrum hittat. Skriv det till aktivt WCS (endast Y/Z).
btn-write-center = Skriv centrum → WCS (G10 L2)

# Bänkinställda rotationsparametrar (hopfällbar sektion).
hdr-bench-params = Bänkparametrar
lbl-clearance-z = Frigång Z
tip-clearance-z = Maskin-Z-återgång ovanför dornen före A-indexeringen (G53).
lbl-settle = Vila
tip-settle = Vila efter A-indexeringen så att glapp/svängning dämpas ut före avkänningen.
tip-bench-feed = Avkänningsmatning för G38.2-beröringen.
tip-bench-depth = Hur långt avkänningen matas fram för att söka kontakt innan den ger upp (larmar).

# Rotationsavläsningar + beräknat centrum.
rotary-reading-dowel = Dorn ⌀ { $dia } mm · A { $angle }°
rotary-reading-y-left = Y vänster { $v }
rotary-reading-y-right = Y höger { $v }
rotary-reading-y-center = Y centrum { $v }
rotary-reading-z-top = Z topp { $v }
rotary-reading-z-center = Z centrum { $v }

# Väljare för rotationens Z-referens.
lbl-z0-datum = Arbets-Z0-referens
z-datum-axis = Axelns centrumlinje
z-datum-top = Övre yta
z-datum-axis-desc = Z0 vid rotationsaxeln (Z_topp − D/2).
z-datum-top-desc = Z0 vid den avkända övre ytan (Z_topp).
z-datum-g10 = G10 sätter Z { $z }

# Datumsökare (kant / hörn / Z-yta).
hdr-datum = Datumsökare
datum-intro = Hitta arbetsnollan: en enskild kant, ett hörn (in-/utvändigt) eller en Z-yta.
datum-corner-label = Hörn (X/Y)
datum-inside = Invändigt (ficka) hörn
btn-find-corner = Hitta hörn → WCS
datum-edge-label = Enskild kant
lbl-datum-axis = Axel
lbl-datum-dir = Ansättning
btn-find-edge = Hitta kant → WCS
datum-step-enter = Jogga till ansättningen, avkänn sedan.
btn-datum-probe = Avkänn
datum-step-ready-y = Jogga till Y-sidans ansättning, avkänn sedan Y.
btn-datum-probe-y = Avkänn Y-sida
datum-step-review = Datum hittat. Skriv det till aktivt WCS.
btn-datum-write = Skriv datum → WCS (G10 L2)
datum-target-corner-in = Invändigt hörn
datum-target-corner-out = Utvändigt hörn
datum-target-edge = { $axis }-kant
datum-reading-edge = Kant { $v }
datum-reading-x = X { $v }
datum-reading-y = Y { $v }

# Datumsökarens bänkinställda parametrar (fällbar sektion).
hdr-datum-bench = Datumbänkparametrar
lbl-tip-dia = Spets ⌀
lbl-xy-clearance = XY-frigång
lbl-probe-distance = Avkänningssträcka
lbl-latch-distance = Spärrsträcka
lbl-datum-probe-feed = Avkänningsmatning
lbl-latch-feed = Spärrmatning
lbl-corner-offset = Hörnförskjutning

# Höjdkarta-insamlingspanel (Del B2/B3).
hdr-mesh = Höjdkarta
mesh-intro = Avkänn ett rutnät för att kompensera skevt material / ojämn koppar.
lbl-mesh-min = Min
lbl-mesh-max = Max
lbl-mesh-spacing = Avstånd
mesh-grid-size = { $nx } × { $ny } punkter
btn-mesh-auto-bounds = Auto från programgränser
btn-mesh-start = Starta insamling
btn-mesh-apply = Tillämpa sparad höjdkarta
btn-mesh-clear = Rensa sparad höjdkarta
mesh-saved = En höjdkarta är sparad.
mesh-correct-rapids = Höjdkorrigera snabbmatningar (G0 Z)
mesh-step-ready = Jogga fritt, avkänn sedan varje rutnätspunkt.
btn-mesh-probe = Avkänn nästa punkt
mesh-progress = { $done } / { $total } punkter
mesh-done = Höjdkarta klar ({ $n } punkter).
hdr-mesh-bench = Rutnätsavkänningsparametrar
lbl-mesh-clearance = Frigångs-Z
lbl-mesh-feed = Avkänningsmatning
lbl-mesh-depth = Avkänningsdjup

# Verifiera/mät-panel (DOC-11 §2): 180°-vändningsverifiering och kastrapport.
hdr-verify = Verifiera · mät
verify-intro = 180°-vändningsverifiering eller N-vinkelkast, avkänner −Y.
lbl-start-a = Start-A
lbl-runout-n = Kast N
btn-start-flip = Starta 180°-vändningsverifiering
btn-start-runout = Starta kastrapport
verify-title-flip = 180°-vändningsverifiering
verify-title-runout = Kastrapport
verify-title-generic = Verifiera
verify-ready = Jogga ansättningen för beröring { $current } av { $total }, avkänn sedan.
btn-probe-angle = Avkänn denna vinkel
verify-flip-need-two = Vändningsverifiering kräver två avläsningar.
verify-residual = Kvarstående excentricitet { $mm } mm.
verify-apply-desc = Tillämpa förskjuter aktivt WCS-ursprung på denna axel med residualen.
btn-apply-correction = Tillämpa korrigering → WCS (G10 L2)
verify-runout-result = TIR { $tir } mm · excentricitet { $ecc } mm ({ $count } pkt)
verify-runout-readonly = Skrivskyddad — ingen offset skriven.
verify-runout-need-two = Kast kräver minst två avläsningar.

# Gemensamma guidestatusrader.
msg-aborted = Avbruten: { $reason }
reason-cancelled = avbruten
btn-cancel-wizard = Avbryt

# Nedre docka: flikar, ihopfällning och strömningsförloppet.
tab-console = Konsol
tab-program = Program
tip-expand-dock = Expandera docka
tip-collapse-dock = Fäll ihop docka
eta-default-settings = (standardinställningar)
eta-pauses = { $count ->
    [one] pausar vid { $count } rad
   *[other] pausar vid { $count } rader
  }

# Konsolflikens innehåll.
console-auto-scroll = autorulla
console-verbose = utförlig
btn-clear = Rensa
btn-send = Skicka
mdi-hint-disconnected = anslut för att skicka kommandon

# Nedre statusrad.
status-wco-set = WCO satt
status-line = Rad { $acked } / { $total } · { $pct }%

# Larm-/strömningsfelsbanner.
banner-alarm = ⚠ ALARM:{ $code }
banner-error = ⚠ error:{ $code } — strömning stoppad
btn-dismiss = Stäng
btn-soft-reset = Mjuk återställning
tip-soft-reset = Mjuk återställning (0x18)
btn-unlock = Lås upp $X
tip-unlock = Rensa larmspärren

# Banner för manuellt verktygsbyte.
banner-tool-with = 🔧 Verktygsbyte: sätt i T{ $tool }, återuppta sedan
banner-tool-generic = 🔧 Verktygsbyte: sätt i verktyget, återuppta sedan
banner-tool-detail = Maskinen är pausad för ett manuellt verktygsbyte (M6). Sätt i verktyget och tryck Återuppta (cykelstart) för att fortsätta.
tip-resume-tool = Återuppta efter verktygsbytet (cykelstart, ~)

# Firmware-inställningsfönster ($$ / $ES).
settings-window-title = Inställningar
lbl-connection = Anslutning
lbl-baud = Baud
hdr-firmware-settings = Firmware-inställningar
settings-stage-note = Ändringar köas lokalt — Spara skriver dem. Vissa inställningar (t.ex. $22 referenskörning) träder i kraft först efter nästa återställning.
settings-save = Spara
settings-save-n = Spara ({ $count })
tip-settings-save = Skriv alla köade inställningar till styrenheten
settings-refresh = Uppdatera ($$)
tip-settings-refresh = Hämta $$-värden och $ES-etiketter från styrenheten
settings-none = Inga inställningar laddade — Uppdatera för att hämta styrenhetens $$ / $ES.
tip-setting-edit = Klicka för att redigera
setting-unit = Enhet: { $unit }
setting-range = Intervall: { $min }..{ $max }
setting-range-min = Intervall: ≥ { $min }
setting-range-max = Intervall: ≤ { $max }
settings-discard-title = Kasta osparade ändringar?
settings-discard-body = Kasta { $count } osparad(e) ändring(ar)?
btn-discard = Kasta
btn-keep-editing = Avbryt (fortsätt redigera)

# Programinställningsdialog — språk/tema/textskala (värdsidan, skilt från firmwarens Inställningar).
app-settings-title = Programinställningar
app-settings-language = Språk
app-settings-theme = Tema
app-settings-font-scale = Textskala
app-settings-new-theme-hint = nytt temanamn…
app-settings-create = Skapa från aktuellt
app-settings-create-hint = Spara de aktiva färgerna som ett nytt redigerbart tema
app-settings-builtin-hint = Inbyggda teman är skrivskyddade — skapa en kopia för att anpassa färgerna.
app-settings-save = Spara till config.json
app-settings-save-hint = Skriv de aktuella inställningarna till konfigurationsfilen (skriver om hela filen)
app-settings-unsaved = osparade ändringar

# Rubriker för temats färgredigerare (visas med versaler).
theme-group-accents = accenter
theme-group-text = text
theme-group-states = maskintillstånd
theme-group-alarm = larmyta
theme-group-console = konsol
theme-group-toolpath = verktygsbana
theme-group-chrome = ram & ytor
theme-group-other = övrigt

# Strömningsförlopp — interpolerar den bekräftade raden och programmets längd. (Ännu inte kopplad till en vy,
# men behålls som färdig sträng och täcks av tester.)
stream-progress = Strömmar rad { $current } av { $total }

# Resultat av portsökning — en pluralväljare baserad på antalet hittade portar. (Färdig sträng; testtäckt.)
ports-found = { $count ->
    [0] Inga serieportar hittades
    [one] { $count } serieport hittades
   *[other] { $count } serieportar hittades
  }

# Återställbart fel när en port inte kan öppnas. (Färdig sträng; testtäckt.)
error-port-open = Kunde inte öppna porten { $port }: { $reason }
