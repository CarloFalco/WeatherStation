# Step 11 – OTA stage 1: trasferimento file a chunk via LoRa

**Data:** 2026-07-15
**Versione:** v3.0.0-alpha.1

Inizio della serie 3.0 (OTA via LoRa). Piano concordato sui 10 punti
dell'utente: **stage 1** = protocollo + trasferimento file di test + ACK/
ritrasmissioni + CRC (punti 2,3,4,5,7); **stage 2** = partizioni OTA,
scrittura, riavvio e rollback (1,6,8); **stage 3** = firma firmware (10);
dashboard MQTT (9) = responsabilità del futuro gateway.

## Attività svolte

- Validato l'Increment 9 su hardware; pubblicata release `v2.9.0`.
- Protocollo OTA documentato (`docs/lora-protocol.md` v1.1):
  - offerta piggyback nel campo `ota` dell'ACK (size, crc, chunks, ver);
  - modello **pull**: la stazione richiede ogni chunk con `ota_req` —
    ordinamento e ritrasmissione sono impliciti, un chunk perso viene
    semplicemente richiesto di nuovo;
  - chunk **binari** (magic 0xA5 + idx u16 LE + ≤180 byte): niente
    base64-in-JSON, che avrebbe sprecato ~33% del payload;
  - esito `ota_done` con CRC calcolato.
- `logic::crc32` in `src/logic/` (puro C++, API incrementale per lo
  streaming) con 5 test nativi: vettore canonico `123456789`→0xCBF43926,
  input vuoto, frase nota, chunked==one-shot, rilevamento corruzione.
- `LoRaLink::receiveRaw()` per i frame binari.
- `OtaReceiver` (`src/ota/`): sessione bloccante con retry per chunk (8),
  timeout per chunk (1.5 s) e di sessione (120 s), scrittura streaming su
  LittleFS (`/ota_test.bin`) e verifica CRC-32 finale.
- `main.cpp`: `waitForAck` ora restituisce l'intero documento ACK; se
  contiene `ota` la stazione entra in sessione prima di dormire.
- DebugLora.ino: sender di test — `u` da seriale arma un'offerta di 4 KB
  (pattern deterministico `(i*31+7)&0xFF`), serve le `ota_req` e stampa
  l'esito riportato dalla stazione.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Modello pull (stazione chiede ogni chunk) | Stop-and-wait guidato dal ricevitore: ACK e ritrasmissione vengono gratis, nessuno stato di finestra da gestire su entrambi i lati. |
| Offerta dentro l'ACK | La stazione dorme: l'unica finestra utile è quella già aperta dopo la telemetria; zero round-trip aggiuntivi. |
| Chunk binari, controllo in JSON | Il controllo è raro e leggibile; i dati sono tanti e contati: 180 B utili su ~183 B trasmessi (98%). |
| Stage 1 su LittleFS, non sulla partizione | Increment piccoli: prima si valida il trasporto con file innocui, poi si tocca `esp_ota_*` (un bug lì può brickare lo slot). |
| CRC-32 ora, SHA-256+firma allo stage 3 | Il CRC valida il trasporto; l'autenticità (firma) è un requisito dello stage sicurezza, con chiavi da generare e custodire. |
| Versioni `3.0.0-alpha.N` | Richiesta "versione 3.0 con OTA": le alpha marcano gli stage di sviluppo, `v3.0.0` sarà l'OTA completo end-to-end. |

## Note per la validazione

1. Riflashare stazione (`pio run -t upload`, `uploadfs` non necessario) e
   DebugLora (Arduino IDE).
2. Sul monitor del DebugLora premere **`u`** → `[OTA] armato: 4096 byte,
   23 chunk, CRC 0x...`.
3. Al ciclo successivo della stazione:
   - stazione: `OTA transfer: 4096 bytes in 23 chunks (version test-4k)`,
     progresso ogni 8 chunk, poi `OTA transfer SUCCESS ... in ~10-15 s`;
   - DebugLora: chunk inviati e `[OTA] esito stazione: SUCCESS`.
4. Prova di robustezza: durante il trasferimento allontana/schermare
   un'antenna per qualche secondo → si devono vedere retry
   (`OTA: chunk N timeout`) e la sessione deve comunque chiudersi bene.
5. Il file ricevuto resta su LittleFS (`/ota_test.bin`, 4096 byte).

## TODO per lo step successivo (OTA stage 2)

- [ ] **Validazione hardware stage 1** come da note sopra.
- [ ] `OtaReceiver` → scrittura streaming su partizione OTA inattiva
      (`esp_ota_begin/write/end` + `esp_ota_set_boot_partition`).
- [ ] Auto-validazione al primo boot del nuovo firmware
      (`esp_ota_mark_app_valid_cancel_rollback` dopo un ciclo completo
      riuscito) e rollback automatico in caso contrario.
- [ ] Gestione ripresa: trasferimento firmware reale (~490 KB ≈ 2800
      chunk, ~20 min a SF7) da spezzare su più finestre o sessione lunga
      dedicata — da decidere con Carlo.
