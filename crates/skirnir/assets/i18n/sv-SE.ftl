# Skirnir UI-strängar — sv-SE (svensk översättning av en-US.ftl).
#
# Inbäddas i binären via `include_str!` (se `i18n::BUNDLED_LOCALES`). Varje nyckel här motsvarar en nyckel i
# `en-US.ftl`; saknas en nyckel faller `tr!` tillbaka på `en-US`. Interpolation använder Fluent-placeables:
# `{ $name }` ersätts av det namngivna argument som skickas via `tr!`.

app-title = Skirnir

# Översta verktygsfältet — raden med navigerings-/åtgärdsknappar längst upp i fönstret (kopplade till `tr!`).
btn-connect = Anslut
btn-disconnect = Koppla från
btn-cancel = Avbryt
btn-identify = Identifiera
btn-open = Öppna…
btn-home = ⌂ Hemma
btn-settings = Inställningar

# Programstyrning (det sammanfogade Kör/Pausa/Stoppa-fältet — ännu inte kopplat till `tr!`).
btn-run = Kör
btn-hold = Pausa
btn-stop = Stoppa

# Anslutnings-/strömningstillstånd som visas i rubriken.
status-disconnected = Frånkopplad
status-connecting = Ansluter
status-idle = Inaktiv
status-streaming = Strömmar
status-hold = Pausad
status-alarm = Larm

# Strömningsförlopp — interpolerar den bekräftade raden och programmets längd.
stream-progress = Strömmar rad { $current } av { $total }

# Resultat av portsökning — en pluralväljare baserad på antalet hittade portar.
ports-found = { $count ->
    [0] Inga serieportar hittades
    [one] { $count } serieport hittades
   *[other] { $count } serieportar hittades
  }

# Återställbart fel när en port inte kan öppnas.
error-port-open = Kunde inte öppna porten { $port }: { $reason }

# Appinställningar — språk/tema/textskala (värdsidan, skilt från firmwarens Inställningar).
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
