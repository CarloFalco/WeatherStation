# CLAUDE.md — WeatherStation V2 (stazione meteo LoRa ESP32-S3)

## Cos'è
Stazione meteo a batteria (ESP32-S3 + SX1276 868 MHz + BME280/AS5600/INA3221 +
pluviometro/anemometro reed): sveglia ogni 10 min → misura → JSON → LoRa con ACK
→ deep sleep. La base/gateway (LoRa↔MQTT) sarà un progetto separato; qui c'è solo
il nodo stazione + lo sketch di debug `DebugLora/` per il secondo ESP32.
Sviluppo a increment piccoli validati su hardware dall'utente prima di procedere.

## Comandi essenziali
- Gate qualità:        `./scripts/check.sh`   ← USA QUESTO dopo ogni modifica
- Solo build:          `pio run -e station`   (o `./scripts/check.sh --fast`)
- Test logica (no HW): `pio test -e native`
- Flash:               `pio run -t upload` · Filesystem: `pio run -t uploadfs`
- Monitor:             `pio device monitor` (115200, exception decoder già attivo)
- Su questa macchina `pio` non è nel PATH: usare `~/.platformio/penv/Scripts/pio.exe`
  (check.sh lo trova da solo).

## Regole non negoziabili
1. Dopo OGNI modifica esegui `./scripts/check.sh` e correggi prima di proseguire.
2. Pin e indirizzi I2C SOLO in `include/config.h`; parametri runtime (calibrazioni,
   intervalli, radio) SOLO in `config.ini` via `AppConfig` — mai hardcoded nei .cpp.
3. `src/logic/` non deve MAI includere Arduino.h o header ESP-IDF (compila native).
4. Nuovo sensore = classe `ISensor` in `src/sensors/` + registrazione in main.cpp;
   in caso di guasto il sensore NON scrive i suoi campi JSON (mai valori inventati).
5. ISR: `IRAM_ATTR`, corpo minimo, debounce a timestamp; stato che deve
   sopravvivere al deep sleep → RTC RAM (`include/rtc_state.h`), mai flash.
6. I campi JSON e i parametri radio sono definiti in `docs/lora-protocol.md`:
   non inventarne di nuovi senza aggiornare quel file (il ricevitore deve sapere).
7. Log: macro `log_e/log_w/log_i/log_d` del core ESP32 (gated da CORE_DEBUG_LEVEL,
   4 in sviluppo, 2 in produzione). `Serial.print` solo per l'output "di prodotto"
   del ciclo (banner, telemetria, esito TX).
8. Commenti Doxygen in inglese; progress file e CHANGELOG in italiano.
9. Commit frequenti (Conventional Commits); tag `vX.Y.Z` e release SOLO dopo la
   validazione su hardware dell'utente. Una release per increment.

## Architettura in 5 righe
- Tutto il ciclo vive in `setup()` (dopo il deep sleep si riparte da lì); `loop()` vuoto.
- `core/` = AppConfig (INI su LittleFS) + PowerManager (sleep, wake cause, RTC state).
- `sensors/` = ISensor + SensorManager (registro; guasto ⇒ campi assenti, ciclo continua).
- `comm/` = ITelemetryLink → LoRaLink (RadioLib, TX + finestra ACK + retry).
- Wake pioggia (EXT0) = quick path: conta impulso e ridorme per il tempo residuo.
- Dettagli: `docs/architecture.md` · protocollo: `docs/lora-protocol.md`.

## Hardware (riassunto — dettagli in docs/pinout.md)
- ESP32-S3 Dev Module **N16R8** (16MB flash, PSRAM disattivata di proposito).
- SX1276 su SPI: SCK=12, MISO=13, MOSI=11, NSS=10, RST=9, DIO0=8.
- I2C SDA=4/SCL=5: BME280 @0x76, AS5600 @0x36, INA3221 @0x40
  (canali INA: CH1=carico, CH2=batteria, CH3=pannello — verificati su HW).
- Reed: pluviometro GPIO 19 (wake-on-rain), anemometro GPIO 20.
  Vincolo: 19/20 = USB nativo ⇒ flash/monitor via connettore UART.
- LED stato = LED_BUILTIN.

## Problemi noti / gotcha
- Consultare SEMPRE `docs/troubleshooting.md` prima di diagnosticare (11 voci reali).
- Dopo aver cambiato `data/config.ini` serve `pio run -t uploadfs` (il file sul
  device vince sui default nel codice).
- `wake_interval_s = 30` in locale è un valore di test: nel repo va committato 600.
- Su N16R8 i GPIO 33-37 sono della PSRAM octal interna: NON usabili.
- PowerShell 5.1: niente `"` nei messaggi di commit (troubleshooting §11).

## Prima di committare
- `./scripts/check.sh` verde.
- Pin/protocolli toccati ⇒ aggiorna `docs/pinout.md` / `docs/lora-protocol.md`.
- Bug non ovvio risolto ⇒ voce append-only in `docs/troubleshooting.md`.
- Increment completato ⇒ `docs/progress/progress_step_XX.md`, CHANGELOG,
  bump `include/version.h`; tag solo dopo validazione hardware.
