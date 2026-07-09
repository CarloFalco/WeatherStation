# Changelog

Formato basato su [Keep a Changelog](https://keepachangelog.com/it/1.1.0/);
il progetto segue il [Semantic Versioning](https://semver.org/).

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
