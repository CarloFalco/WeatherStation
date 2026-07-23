# Changelog

Formato basato su [Keep a Changelog](https://keepachangelog.com/it/1.1.0/);
il progetto segue il [Semantic Versioning](https://semver.org/).

## [3.0.0-alpha.2] – 2026-07-16 — OTA stage 1: fix dal primo test su firmware reale

Primo trasferimento di un firmware vero (425 kB, 2365 chunk): falliva
sistematicamente. Tre bug distinti, tutti corretti.

### Fixed
- **Timeout di sessione**: `kSessionTimeoutMs` era 120 s, dimensionato sul
  file di test da 4 kB. Un'immagine reale richiede ~20 minuti a 0.5 s/chunk
  e il trasferimento moriva sempre a ~240 chunk (120.3 s misurati). Ora è
  `session_timeout_s` in `config.ini`, default 1800 s.
- **Buchi in ricezione**: `receiveRaw()` usava il `receive()` bloccante di
  RadioLib, che ascolta in finestre RX-single da ~102 ms; i chunk il cui
  preambolo cadeva tra due finestre venivano persi (causa dei "chunk failed
  after 8 attempts" su link a -35 dBm). Ora usa **RX continuo** con polling
  di DIO0. Lato base: `LoRa.receive()` subito dopo `endPacket()`, altrimenti
  la radio restava in standby e perdeva la richiesta successiva.
- **CRC letto come 0**: `doc["crc"] | 0` fa convertire ArduinoJson attraverso
  `int`, quindi ogni CRC sopra `0x7FFFFFFF` (metà dei casi) veniva letto 0 —
  sia lato stazione (`offer.crc`, avrebbe fatto fallire ~metà degli
  aggiornamenti) sia nei due sketch di debug. Ora default `| 0UL` ovunque.

### Added
- **Ripresa del trasferimento**: progresso (chunk successivo + CRC parziale)
  in RTC RAM, quindi sopravvive al deep sleep. Se la base rioffre la stessa
  immagine e il file su LittleFS è coerente, la stazione riprende dal punto
  raggiunto invece di ricominciare da capo; `ota_done` porta il campo `next`
  perché la base sappia dove si è fermata.
- Sezione `[ota]` in `config.ini`: `chunk_timeout_ms`, `max_retries`,
  `session_timeout_s`. Backoff casuale 20–80 ms tra i tentativi.
- Log di avanzamento con percentuale ed **ETA** su entrambi i lati.

## [3.0.0-alpha.1] – 2026-07-15 — OTA stage 1: trasferimento a chunk via LoRa

La serie 3.0 introduce gli aggiornamenti OTA via LoRa; le pre-release
`-alpha.N` sono gli increment di sviluppo (3.0.0 finale = OTA completo).

### Added
- Protocollo di trasferimento OTA (docs/lora-protocol.md v1.1): offerta
  piggyback nell'ACK (`ota{size,crc,chunks,ver}`), modello **pull** —
  la stazione richiede ogni chunk (`ota_req`) ⇒ ACK/ritrasmissioni
  implicite — chunk **binari** da 180 byte (`0xA5 + idx LE + payload`),
  esito finale `ota_done`.
- `OtaReceiver` (`src/ota/`): sessione di ricezione con retry per chunk
  (8), timeout (1.5 s/chunk, 120 s/sessione), streaming su LittleFS e
  verifica CRC-32 dell'immagine — stage 1: file di test.
- `logic::crc32` (IEEE 802.3, incrementale, senza tabella) con 5 unit
  test nativi (vettori noti, streaming a chunk = one-shot, corruzione).
- `LoRaLink::receiveRaw()`: ricezione binaria (base64-in-JSON avrebbe
  sprecato ~33% del payload LoRa).
- DebugLora.ino: sender OTA di test — 'u' da seriale arma un'offerta di
  4 KB (pattern noto), serve le richieste e stampa l'esito.
- DebugOta.ino: prototipo del gateway (ESP32-S3 + WiFi + SX1276) — legge
  l'ultima release da GitHub, scarica il `.bin` su LittleFS calcolando il
  CRC-32 e lo trasferisce alla stazione col protocollo OTA (offerta
  nell'ACK, pull dei chunk). Base per la futura integrazione nella
  dashboard. Comandi seriali: 'u' controlla GitHub, 'f' forza l'offerta,
  'i' stato.

### Roadmap
- Stage 2: scrittura nella partizione OTA inattiva + riavvio + rollback.
- Stage 3: SHA-256 + firma del firmware. Dashboard MQTT: lato gateway.

## [2.9.0] – 2026-07-15 — Increment 9: ottimizzazione consumi e stima autonomia

### Added
- Env `station-release` (produzione): `CORE_DEBUG_LEVEL=2` (log_d/log_i
  eliminati a compile-time) e niente attesa USB di 2 s al boot. La release
  CI ora pubblica il binario di produzione; `check.sh` compila entrambi.
- `docs/power-budget.md`: duty cycle, budget di sleep, tre scenari di
  autonomia, raccomandazioni hardware e procedura di misura.
- Post-build script PlatformIO (`scripts/post_build_docs.py`): dopo ogni
  build dell'env `station` la documentazione Doxygen viene rigenerata
  automaticamente in `docs/html/` (doxygen assente o in errore = warning,
  mai build fallita).

### Changed
- CPU a 80 MHz durante la veglia (`CPU_FREQ_MHZ` in config.h): ~metà
  della corrente attiva rispetto ai 240 MHz di default.
- AS5600 forzato in **LPM3** al `begin()` (~1.5 mA invece di ~6.5 mA
  continui): il sensore resta alimentato anche nel deep sleep ed era la
  voce dominante del budget — da ~18 a ~66 giorni di autonomia teorica a
  batteria; > 12 mesi restano subordinati al pannello o al load switch
  sul rail sensori (vedi power-budget.md).

## [2.8.0] – 2026-07-09 — Increment 8: protocollo ACK e consegna affidabile

### Tooling (step 09, in release con questa versione)
- Env `native` + `src/logic/` (prima estrazione: `logic::vbatToSoc`) con
  unit test host (`pio test -e native`, 6 test).
- `scripts/check.sh`: gate qualità unico (test native + build).
- `docs/troubleshooting.md` append-only con gli 11 problemi già risolti.
- Slash command `.claude/commands/{new-sensor,fix-crash}.md`;
  CLAUDE.MD riallineato al progetto; `docs/pinout.md` corretto
  (reed su GPIO 19/20 come da config.h, vincolo USB nativo documentato).

### Added
- Finestra RX dopo ogni trasmissione (`LoRaLink::receive`, RX-single
  pollato fino a scadenza) in attesa dell'ACK della base:
  `{"type":"ack","id":...,"seq":...}` con `id`/`seq` combacianti; i
  pacchetti estranei nella finestra vengono ignorati senza chiuderla.
- Ritrasmissione dello stesso messaggio (stesso `seq`) in assenza di ACK,
  fino a `tx_retries` volte.
- Nuove chiavi `[lora]`: `ack_enabled` (default true), `ack_timeout_ms`
  (default 600), `tx_retries` (default 1); con `ack_enabled = false` la
  stazione resta fire-and-forget come in 2.7.0.
- DebugLora.ino: risponde con l'ACK ai messaggi `data` ricevuti (~50 ms
  di ritardo per la commutazione TX→RX della stazione).
- docs/lora-protocol.md consolidato da bozza a v1.0 (sequenza ACK,
  timing, deduplicazione lato base).

### Changed
- Reed switch spostati su **GPIO 6 (pluviometro, wake-on-rain) e GPIO 7
  (anemometro)** — ricablaggio hardware: libera i GPIO 19/20 dell'USB
  nativo mantenendo la capability RTC per il wake da pioggia.
- L'accumulatore pioggia viene consumato **solo a consegna confermata**
  e per sottrazione dello snapshot riportato (le basculate arrivate
  durante TX/finestra ACK non vanno perse); senza ACK il dato viaggia
  riaggregato nel ciclo successivo.

## [2.7.0] – 2026-07-09 — Increment 7: trasmissione LoRa

### Added
- Interfaccia `ITelemetryLink` (`src/comm/`): astrazione del trasporto di
  telemetria — predispone il requisito bonus (futuro `MqttLink` WiFi
  diretto selezionabile da config, senza toccare `main.cpp`).
- `LoRaLink`: SX1276 via RadioLib con i parametri `[lora]` di config.ini
  (frequenza, BW, SF, CR, sync word, potenza; CRC attivo, preambolo 8).
  Invio bloccante fire-and-forget; time-on-air stimato nel log di debug;
  radio messa in sleep (~0.2 µA) prima del deep sleep dell'MCU.
- `main.cpp`: il JSON di telemetria ora viaggia via radio a ogni ciclo
  (oltre alla stampa su seriale); esito TX sul monitor.
- Dipendenza: RadioLib ^7.2.1.

### Fixed (durante la validazione dell'Increment 6)
- Mappatura canali INA3221 verificata su hardware: CH1 = carico,
  CH2 = batteria, CH3 = pannello.
- Correnti INA3221 erano 1000× troppo piccole (µV/mΩ dà già mA).
- Shunt reale da 100 mΩ (era 10 nel codice del vecchio prototipo),
  verificato contro una misura di riferimento.

## [2.6.0] – 2026-07-09 — Increment 6: monitor energia INA3221

### Added
- `Ina3221Driver` (`src/sensors/`): driver minimale a registri, derivato
  dalla libreria Beast Devices (MIT) usata nel vecchio prototipo,
  rielaborato per il progetto: conversioni **single-shot** (il chip resta
  in power-down tra i cicli, ~0.35 mA risparmiati) e lettura shunt
  **con segno** (ADC bidirezionale a 13 bit in complemento a due).
- `PowerMonitor` (`src/sensors/`): ISensor per i tre canali (1 = pannello,
  2 = batteria, 3 = carico) → campi `vbat` [V], `soc` [%], `ibat`/`ipan`/
  `iload` [mA]; `ibat` positiva = carica.
- `vbToSoc()`: stato di carica dalla curva di scarica LiPo del vecchio
  prototipo (interpolazione lineare, 21 punti, clamp 0–100); il campo
  `soc` è aggiunto al protocollo LoRa (docs/lora-protocol.md).
- Sezione `[power]` in `config.ini`: `shunt_mohm` (default 10).

### Changed
- Rimossi `include/INA3221.{h,cpp}` (vecchia base, rielaborata nei nuovi
  moduli). Eliminati i workaround sul segno della corrente batteria del
  vecchio `getAll()`, resi superflui dalla lettura signed.

## [2.5.0] – 2026-07-09 — Increment 5: anemometro e banderuola

### Added
- `Anemometer` (`src/sensors/`): conteggio impulsi su GPIO 20 in una
  finestra di campionamento a bin da 1 s → `ws` (media) e `wg` (raffica,
  bin più alto) in m/s. Debounce 5 ms.
- `WindVane` (`src/sensors/`): lettura AS5600 via I2C diretta (registri
  raw angle + verifica presenza magnete al `begin()`) → `wd` [°, 0–359,
  0 = Nord]. Nessuna libreria esterna.
- Sezione `[wind]` in `config.ini`: `mps_per_hz` (default 0.667 =
  "1 Hz = 2.4 km/h", da sostituire col dato di targa reale),
  `sample_window_s` (default 5), `vane_offset_deg` (offset di montaggio,
  taratura sul campo puntando la banderuola a Nord).

## [2.4.0] – 2026-07-09 — Increment 4: pluviometro e wake-on-rain

### Added
- `RainGauge` (`src/sensors/`): conteggio impulsi del pluviometro a
  vaschetta su GPIO 19 con debounce software (150 ms), accumulo in RTC RAM
  (`RtcState::rainPulses`) → campo `rain` [mm] nel JSON di telemetria.
- Wake-on-rain: una basculata durante il deep sleep sveglia la stazione
  (EXT0, pull-up nel dominio RTC); il "quick path" in `main.cpp` conta
  l'impulso e torna subito a dormire per il tempo residuo fino al prossimo
  ciclo schedulato (`RtcState::nextWakeEpochS`) — niente seriale, sensori
  né radio: un evento pioggia costa millisecondi di CPU.
- Sezione `[rain]` in `config.ini`: `mm_per_pulse` (default 0.4743 mm =
  3 cm³ di vaschetta / 63.25 cm² di bocca di raccolta), ricalibrabile
  senza ricompilare.

### Fixed
- Corretto il fattore di conversione del pluviometro rispetto al vecchio
  prototipo: 0.4743 mm/impulso, non 47.43 (errore di unità cm²→m²).

## [2.3.0] – 2026-07-08 — Increment 3: interfaccia sensori e BME280

### Added
- Interfaccia `ISensor` (`src/sensors/`): punto di estensione del firmware —
  un nuovo sensore = una classe + una riga di registrazione, senza toccare
  `main.cpp` né il futuro modulo LoRa.
- `SensorManager`: registro dei sensori; un sensore guasto viene marcato
  unhealthy e saltato (i suoi campi JSON restano assenti, come da protocollo)
  senza bloccare gli altri.
- `Bme280Sensor`: primo sensore reale (I2C 0x76) con profilo Bosch "weather
  monitoring" (forced mode, oversampling 1x, filtro off) → campi `t`, `rh`, `p`.
- `main.cpp`: finestra di misura reale — compone il JSON di telemetria
  (`type`, `id`, `fw`, `seq` da RTC RAM + campi sensori) e lo stampa su
  seriale in forma compatta, identica a quella che viaggerà via LoRa.
- Dipendenze: ArduinoJson 7, Adafruit BME280 Library.

## [2.2.0] – 2026-07-08 — Increment 2: PowerManager e deep sleep

### Added
- Modulo `PowerManager` (`src/core/`): classificazione della causa di
  wake-up, deep sleep con timer armato su `wake_interval_s`.
- `include/rtc_state.h`: struttura `RtcState` in RTC RAM che sopravvive ai
  deep sleep (boot count, contatore `seq` dei messaggi, impulsi pioggia —
  questi ultimi usati dall'Increment 4).
- `main.cpp` ristrutturato nel ciclo reale: wake → banner con boot count →
  config → finestra di misura (placeholder) → deep sleep. `loop()` non
  viene mai raggiunto.

## [2.1.0] – 2026-07-08 — Increment 1: configurazione runtime e protocollo LoRa

### Added
- Modulo `AppConfig` (`src/core/`): parser/serializer INI su LittleFS per
  `/config.ini` con sezioni `[station]` (id, wake_interval_s) e `[lora]`
  (freq, bw, sf, cr, tx_power, sync_word). Se il file manca viene
  rigenerato con i default, quindi `uploadfs` è opzionale.
- `data/config.ini` di default per l'upload del filesystem.
- `docs/lora-protocol.md`: bozza del protocollo — parametri radio SX1276
  e formato JSON di telemetria con tabella campi/unità, come riferimento
  per lo sviluppo del ricevitore.
- `main.cpp`: carica e stampa la configurazione attiva al boot.

### Changed
- Partizioni portate a 16MB (modulo N16R8): slot OTA 4MB + LittleFS ~7.9MB.
- Pin I2C confermati (SDA=4, SCL=5); LED di stato = `LED_BUILTIN`.

## [2.0.0] – 2026-07-08 — Increment 0: scaffold del progetto

### Added
- Progetto PlatformIO per ESP32-S3 (framework Arduino, LittleFS,
  partizioni 4MB OTA-ready con doppio slot applicativo).
- `include/config.h`: pinout completo (LoRa SX1276, sensori I2C, reed switch).
- `include/version.h`: versionamento SemVer del firmware.
- `src/main.cpp`: boot banner con versione e causa di wake-up, heartbeat LED.
- Architettura documentata in `docs/architecture.md` (moduli, interfacce
  `ISensor`/`ITelemetryLink`, roadmap increment).
- CI GitHub Actions: build e release automatica con binario su push di tag.
- README, CHANGELOG, `.gitignore`, log di avanzamento `progress_step_00.md`.

### Notes
- La serie V2 riparte da `v2.0.0`; i tag `v0.x` appartengono al vecchio
  prototipo, rimosso con il commit di fresh start.
