# Step 01 – Configurazione runtime (AppConfig) e bozza protocollo LoRa

**Data:** 2026-07-08
**Versione:** v2.1.0

## Attività svolte

- Chiusa la validazione hardware dello Step 00: LED = `LED_BUILTIN`,
  I2C su SDA=4/SCL=5, modulo N16R8 → partizioni portate a 16MB
  (slot OTA 4MB + LittleFS ~7.9MB). Tag `v2.0.0` pubblicato con release CI.
- Implementato il modulo `AppConfig` (`src/core/AppConfig.{h,cpp}`):
  - parser INI minimale (sezioni, `chiave = valore`, commenti `;`/`#`),
    nessuna dipendenza esterna;
  - sezioni `[station]` (id, wake_interval_s) e `[lora]` (freq_mhz,
    bw_khz, sf, cr, tx_power_dbm, sync_word);
  - default nel codice + rigenerazione automatica di `/config.ini` se
    assente → `pio run -t uploadfs` è opzionale;
  - validazione dei range (sf 6–12, cr 5–8, tx 2–17 dBm, intervallo ≥ 10 s).
- Creato `data/config.ini` con i valori di default.
- Scritta la bozza del protocollo in `docs/lora-protocol.md`:
  parametri radio (868.1 MHz, BW 125 kHz, SF7, CR 4/5, sync 0x12) e
  formato JSON di telemetria con tabella campi/unità — riferimento per
  la preparazione del ricevitore di debug.
- `main.cpp` aggiornato: carica e stampa la configurazione al boot.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Parser INI fatto in casa (~80 righe) | Il file è minuscolo; una libreria esterna sarebbe complessità inutile. |
| Default nel codice, file rigenerato se assente | La stazione funziona anche con filesystem vergine; nessun passo di provisioning obbligatorio. |
| SF7 come default LoRa | Airtime minimo (~120 ms per ~150 byte) → minor consumo e duty cycle; SF configurabile fino a 12 se servirà portata. |
| JSON con campi opzionali | Il ricevitore non deve rompersi mentre i sensori vengono aggiunti un increment alla volta. |
| Campo `seq` + semantica `rain` "dall'ultimo invio riuscito" | Prepara ACK e ritrasmissioni senza perdere pioggia in caso di pacchetti persi. |

## TODO per lo step successivo (Increment 2 – PowerManager)

- [ ] **Validazione hardware step 1**: flash, verificare la stampa della
      configurazione; modificare un valore in `data/config.ini`,
      `pio run -t uploadfs`, verificare che il nuovo valore sia attivo.
- [ ] `PowerManager` (`src/core/`): deep sleep con wake da timer
      (`wake_interval_s` da AppConfig), lettura causa di wake-up,
      struttura `RtcState` in RTC RAM (contatore boot, seq messaggi).
- [ ] Predisporre il wake-on-rain (EXT wake-up su GPIO 19) — implementazione
      completa nell'increment del pluviometro.
