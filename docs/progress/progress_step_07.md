# Step 07 – Trasmissione LoRa (ITelemetryLink + LoRaLink)

**Data:** 2026-07-09
**Versione:** v2.7.0

## Attività svolte

- Validato l'Increment 6 su hardware (dopo tre fix emersi dalla
  validazione: mappatura canali CH1 = carico / CH2 = batteria /
  CH3 = pannello, scala correnti ×1000, shunt reale 100 mΩ);
  pubblicata release `v2.6.0`.
- Definita l'interfaccia `ITelemetryLink` (`src/comm/ITelemetryLink.h`):
  `begin()` / `send(payload)` / `sleep()` — il trasporto è intercambiabile
  (LoRa oggi, MQTT diretto in futuro come da requisito bonus).
- Implementato `LoRaLink` (`src/comm/LoRaLink.{h,cpp}`) con RadioLib:
  - init SX1276 sul bus SPI dedicato (SCK 12, MISO 13, MOSI 11, CS 10,
    RST 9, DIO0 8) con i parametri `[lora]` di config.ini;
  - CRC attivo, preambolo 8 simboli (come da docs/lora-protocol.md);
  - `send()` bloccante fire-and-forget con esito e time-on-air nel log;
  - `sleep()`: SX1276 in sleep mode (~0.2 µA) prima del deep sleep —
    senza, la radio resterebbe in standby (~1.6 mA) vanificando tutto.
- `main.cpp`: trasmissione dopo la composizione del JSON, esito su
  seriale, radio a riposo in ogni caso prima di dormire.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| RadioLib invece di driver SX1276 fatto in casa | A differenza di AS5600/INA3221 la radio è complessa (modem, IRQ, FIFO, modi); RadioLib è mantenuta, supporta la futura ricezione (ACK/OTA) e altri chip se l'hardware cambiasse. |
| TX bloccante | ~120 ms a SF7: nel ciclo sequenziale non c'è nulla da fare nel frattempo; l'interrupt-driven arriverà con la finestra RX dell'increment protocollo. |
| Parametri radio dalla config, non hardcoded | Cambiare SF/frequenza per prove di portata senza ricompilare (basta `uploadfs`); il ricevitore deve solo restare allineato. |
| Fire-and-forget in questo increment | Increment piccoli: prima si valida il collegamento fisico, poi (Increment 8) ACK, retry e azzeramento pioggia condizionato. |
| `LoRaLink` tiene un riferimento alla config, letto in `begin()` | L'oggetto è statico ma i valori vengono letti solo dopo il caricamento di config.ini. |

## Note per la validazione

- Ricevitore: parametri in docs/lora-protocol.md — 868.1 MHz, BW 125 kHz,
  SF7, CR 4/5, sync word 0x12, CRC on. Deve stampare il JSON ricevuto.
- Sul monitor della stazione: `SX1276 ready: ...` al boot, poi
  `LoRa TX: ok` e nel log debug il time-on-air stimato.
- Verifiche: JSON identico tra stazione e ricevitore; `seq` che avanza
  senza buchi (a stazione ferma vicino al ricevitore); RSSI lato
  ricevitore ragionevole (< -30 dBm se molto vicini è normale che saturi).
- Se `SX1276 init failed error -2`: cablaggio SPI/CS; `-707`/`-706`:
  sync word/parametri disallineati (non impedisce il TX, solo la RX).

## TODO per lo step successivo (Increment 8 – protocollo)

- [ ] **Validazione hardware step 7**: pacchetti ricevuti dalla base di
      debug, `seq` continuo, portata di massima.
- [ ] Finestra RX dopo il TX + messaggio ACK (`{"type":"ack","id":...,"seq":...}`).
- [ ] Azzeramento accumulatore pioggia solo su ACK ricevuto; retry/backoff.
- [ ] Consolidare docs/lora-protocol.md da bozza a v1.0.
