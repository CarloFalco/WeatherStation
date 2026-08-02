# Step 12 – Umidità del terreno, ripristino di fabbrica, rail sensori

**Data:** 2026-07-16
**Versione:** v3.0.0-alpha.3

Tre componenti hardware aggiunti da Carlo al `pinout.md`: sonda di umidità
del terreno, tasto di ripristino, comando di spegnimento alimentazione.
Implementati i primi due; il terzo richiede una decisione hardware.

## Verifica del pinout (nessun conflitto)

| GPIO | Uso | Verifica |
|------|-----|----------|
| 1 | Umidità terreno (analogico) | ✔ **ADC1**_CH0 — scelta corretta: ADC2 è inutilizzabile con il WiFi attivo. RTC-capable. Libero. |
| 17 | Tasto reset | ✔ RTC-capable (0–21) ⇒ utilizzabile come wake source. Libero. |
| 18 | Alimentazione sensori | ✔ RTC-capable ⇒ lo stato può essere congelato in deep sleep. Libero. |

Nessuno dei tre è pin di strapping (0, 3, 45, 46), PSRAM octal (33–37),
USB (19/20) o flash (26–32). Occupati in precedenza: 4, 5, 6, 7, 8, 9, 10,
11, 12, 13, 48.

**Unico rilievo elettrico**: la sonda capacitiva va alimentata a **3.3 V**.
Alimentata a 5 V la sua uscita arriva a ~4.2 V e supera il limite d'ingresso
dell'ADC dell'ESP32-S3 (servirebbe un partitore).

## Attività svolte

- `logic::soilRawToPercent` (`src/logic/`): conversione **inversa** tipica
  delle sonde capacitive (più secco ⇒ tensione più alta), con clamp e
  rifiuto della taratura incoerente. 7 test nativi.
- `SoilMoistureSensor` (`src/sensors/`): media di N letture, attenuazione
  12 dB, controllo di plausibilità (ingresso flottante ⇒ campo assente).
  Campo `soil` [%] aggiunto al protocollo.
- `FactoryResetButton` (`src/core/`) + `AppConfig::factoryReset()`:
  conferma a pressione prolungata con LED come countdown; il reset
  ripristina i default, riscrive `/config.ini` e azzera lo stato RTC.
- `PowerManager`: wake EXT0 → **EXT1** con due pin (pluviometro + tasto),
  distinti via bitmask; `setSensorRail()` con hold in deep sleep.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Wake unificato su EXT1 invece di EXT0 + EXT1 | EXT0 gestisce un solo pin; EXT1 è la modalità documentata per più pin e `esp_sleep_get_ext1_wakeup_status()` dice esattamente chi ha svegliato. Un'unica sorgente da ragionare. |
| Il tasto NON prende il quick path pioggia | Una pressione deve arrivare al boot completo, altrimenti il ripristino non sarebbe mai raggiungibile. |
| Conferma a pressione prolungata (3 s) | Il tasto è anche wake source: senza conferma, un contatto accidentale (o un falso trigger) cancellerebbe la configurazione. |
| `factoryReset()` riassegna struct intere (`= StationConfig{}`) | Impossibile dimenticare un campo quando si aggiungono sezioni nuove. |
| Taratura sonda in `config.ini`, non hardcoded | Ogni sonda capacitiva ha valori diversi; la taratura si fa sul campo con i raw stampati nel log di debug. |
| GPIO 18 pilotato ma rail mai spento | Serve uno stato definito (un gate flottante è peggio di nessun controllo), ma spegnere davvero richiede prima la decisione sulla topologia. |

## Feedback sul punto 2 (spegnimento alimentazione)

**La funzionalità non è superflua: è la raccomandazione n.1 del power
budget.** Ma il ramo proposto (INA3221 + BME280 + LoRa) è quello sbagliato:
quei tre in sleep assorbono ~2.3 µA in totale. I consumi veri sono
**AS5600 (~1.5 mA)** e la **sonda di umidità (~5 mA)**, che nel ramo
proposto non compaiono. Analisi completa e topologia consigliata in
`docs/power-budget.md`.

## TODO per lo step successivo

- [ ] **Validazione hardware step 12**: lettura sonda + taratura, tasto di
      reset (da sveglia e da deep sleep), wake pioggia ancora funzionante
      dopo il passaggio a EXT1.
- [ ] Decisione topologia ramo commutato ⇒ poi implementare lo spegnimento
      effettivo prima del deep sleep.
- [ ] OTA stage 2: scrittura in partizione, riavvio, rollback.
