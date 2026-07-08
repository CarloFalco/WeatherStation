# Step 00 – Fresh start, architettura e scaffold del progetto

**Data:** 2026-07-08
**Versione:** v2.0.0

## Attività svolte

- Rimosso il codice del prototipo v0.2.0 (commit di fresh start; storia e
  tag `v0.x` conservati nel repository per riferimento).
- Raccolti i chiarimenti iniziali richiesti dai requisiti (vedi Decisioni).
- Definita l'architettura software modulare → `docs/architecture.md`.
- Creato lo scaffold PlatformIO:
  - `platformio.ini` (ESP32-S3, Arduino, LittleFS, monitor 115200);
  - `partitions.csv` 4MB con doppio slot OTA + partizione LittleFS;
  - `include/config.h` con il pinout completo;
  - `include/version.h` (SemVer);
  - `src/main.cpp` minimale: banner di boot, causa di wake-up, heartbeat.
- Ripristinati README, CHANGELOG, `.gitignore` e workflow CI di release.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Fresh start su `main`, versioni da `v2.0.0` | Riscrittura completa; i tag `v0.1.x` esistenti impediscono di riusare la serie 0.x senza ambiguità. |
| Solo firmware stazione in questo repo | Il gateway sarà un progetto separato; il protocollo LoRa/JSON verrà documentato in `docs/lora-protocol.md` perché il gateway possa implementarlo. |
| Wake ogni 10 min (configurabile via `config.ini`) | Compromesso granularità/consumi compatibile con autonomia > 12 mesi. |
| Interfacce `ISensor` e `ITelemetryLink`, nient'altro | Estensibilità richiesta (aggiungere/cambiare sensori, LoRa vs MQTT diretto) senza layer superflui. |
| Partizioni su base 4MB | Funziona su qualunque modulo S3 (N4/N8/N16); ampliabili quando sarà noto il taglio di flash esatto. |
| RadioLib per SX1276 (da Increment 7) | Libreria mantenuta, supporto interrupt e ricezione, adatta al futuro OTA a pacchetti. |
| Commenti Doxygen in inglese | Scelta dell'utente; progress file in italiano. |

## TODO per lo step successivo (Increment 1 – AppConfig)

- [ ] **Validazione hardware step 0**: flash, verifica banner su seriale e LED.
- [ ] Confermare il GPIO del LED di stato (requisiti indicavano "97", non
      valido su ESP32-S3; provvisoriamente GPIO 48).
- [ ] Confermare i pin I2C reali (provvisori SDA=4, SCL=5: i default S3
      SDA=8/SCL=9 collidono con LORA_DIO0/LORA_RST).
- [ ] Comunicare il taglio di flash del modulo (N8? N16?) per eventuale
      ampliamento delle partizioni.
- [ ] Implementare `AppConfig` (`/config.ini` su LittleFS) con i parametri:
      `station_id`, `wake_interval_s`, sezione `[lora]`.
