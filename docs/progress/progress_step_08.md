# Step 08 – Protocollo ACK e consegna affidabile

**Data:** 2026-07-09
**Versione:** v2.8.0

## Attività svolte

- Validato l'Increment 7 su hardware con il nodo DebugLora (dopo il fix
  della frequenza del ricevitore: 868.0 → 868.1 MHz); release `v2.7.0`.
- `ITelemetryLink`/`LoRaLink`: aggiunto `receive(payload, timeoutMs)` —
  RX-single di RadioLib pollato fino alla scadenza; pacchetti con CRC
  errato loggati e ignorati senza chiudere la finestra.
- `main.cpp`: ciclo di invio affidabile —
  TX → finestra RX `ack_timeout_ms` → match `type/id/seq` → altrimenti
  ritrasmissione dello stesso messaggio fino a `tx_retries` volte.
- `RainGauge`: azzeramento per **sottrazione dello snapshot** riportato
  (sotto lock interrupt) e chiamato solo a consegna confermata: nessuna
  basculata persa né per pacchetti droppati né durante la finestra ACK.
- Nuove chiavi `[lora]`: `ack_enabled`, `ack_timeout_ms`, `tx_retries`.
- `DebugLora.ino`: ACK responder (delay ~50 ms + eco di `id`/`seq`).
- `docs/lora-protocol.md`: da bozza a **v1.0** (sequenza, timing,
  ritrasmissioni, nota di deduplicazione lato base).

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Stop-and-wait con 1 retry (default) | Il payload è autoconsistente e il ciclo si ripete ogni 10 min: schemi più complessi (sliding window, backoff) non ripagano la complessità su questo duty cycle. |
| Stesso `seq` in ritrasmissione | Permette alla base di deduplicare; un `seq` ripetuto = ACK precedente perso. |
| Pioggia consumata per sottrazione, non azzerata | Tra `read()` e ACK passano ~1–2 s: sotto pioggia intensa una basculata in quella finestra andrebbe persa con l'azzeramento secco. |
| `ack_enabled` disattivabile da config | Bring-up e debug del ricevitore restano possibili in fire-and-forget senza ricompilare. |
| Finestra RX 600 ms | ACK a SF7 ≈ 45 ms di airtime + ~50 ms di ritardo lato base + margine per il logging del ricevitore. |
| Costo energetico accettato: ~600 ms di RX (~12 mA) per ciclo | ~2 mWs per ciclo, trascurabile rispetto alla veglia totale (~8 s); in cambio nessun dato pioggia perso. |

## Note per la validazione

1. Riflashare **entrambi**: stazione (`upload` + `uploadfs`, nuove chiavi
   config) e nodo DebugLora (ACK responder).
2. Caso nominale: stazione stampa `LoRa TX: delivered (ACK)`; DebugLora
   stampa il pacchetto e `[TX] ACK inviato: {...}`.
3. Caso perdita: spegnere il DebugLora → la stazione fa il retry e chiude
   con `LoRa TX: NOT delivered`; la pioggia accumulata (simulare con
   GPIO 19 a GND) deve comparire ancora nel ciclo successivo e sparire
   solo quando torna l'ACK.
4. Verificare che pacchetti estranei (es. secondo nodo che trasmette
   altro) non vengano scambiati per ACK.

## TODO per lo step successivo (Increment 9 – consumi)

- [ ] **Validazione hardware step 8** come da note sopra.
- [ ] Misura della corrente di deep sleep e della durata/consumo della
      veglia (INA3221 canale carico + oscilloscopio/multimetro).
- [ ] Ottimizzazioni: `delay(2000)` USB condizionato, gating del debug,
      valutare riduzione frequenza CPU in veglia.
- [ ] Stima autonomia documentata (duty cycle, mAh/giorno, mesi attesi).
