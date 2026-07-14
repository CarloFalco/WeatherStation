# Step 10 – Increment 9: ottimizzazione consumi e stima autonomia

**Data:** 2026-07-15
**Versione:** v2.9.0

## Attività svolte

- Validato l'Increment 8 su hardware (ACK end-to-end con i reed ricablati
  su GPIO 6/7); pubblicata release `v2.8.0`.
- **Env di produzione `station-release`**: `CORE_DEBUG_LEVEL=2` →
  `log_d`/`log_i` eliminati dal binario e `delay(2000)` USB saltato
  (−2 s di veglia/ciclo). platformio.ini rifattorizzato con base comune
  `[esp32s3_base]`; la release CI pubblica il binario di produzione;
  `check.sh` compila entrambi gli env.
- **CPU a 80 MHz** in veglia (prima riga di `setup()`): ~metà corrente
  attiva, nessun impatto su I2C/SPI/UART/LoRa.
- **AS5600 → LPM3** in `WindVane::begin()`: il sensore non ha enable pin
  e resta alimentato nel deep sleep; in default assorbe ~6.5 mA continui
  (= batteria da 3 Ah in ~18 giorni!). In LPM3 scende a ~1.5 mA con
  campionamento interno a 10 ms, più che sufficiente per il vento.
- **`docs/power-budget.md`**: duty cycle per fase, budget di sleep per
  componente, tre scenari di autonomia, raccomandazioni hardware e
  procedura di misura per validare le stime.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Due env invece di flag manuali | La differenza dev/produzione è un solo numero (`CORE_DEBUG_LEVEL`) e viaggia con la CI: impossibile rilasciare per sbaglio una build con log e attese di debug. |
| 80 MHz fissi, non dynamic frequency scaling | Il ciclo dura secondi ed è sequenziale: il DFS aggiungerebbe complessità per un guadagno nullo rispetto al deep sleep. |
| AS5600 LPM3 via firmware, load switch rimandato all'hardware | LPM3 è gratis e recupera il 77% del problema; il load switch (P-MOSFET sul rail sensori) resta la soluzione definitiva ed è documentato come raccomandazione n.1. |
| Stime dichiarate, non spacciate per misure | Il documento distingue "stima analitica" da "misurato" e include la procedura per chiudere il cerchio con multimetro/INA3221. |

## Numeri chiave (stima, batteria 3000 mAh senza sole)

| Scenario | Autonomia |
|----------|-----------|
| fw 2.8.0 (AS5600 default) | ~18 giorni |
| fw 2.9.0 (LPM3) | ~66 giorni (ok col pannello) |
| + load switch hardware | ~310 giorni (> 12 mesi col pannello, con margine) |

## Note per la validazione

1. Flash della build **dev** per il test funzionale: banner con
   `CPU frequency : 80 MHz`, log `AS5600 switched to LPM3` al primo boot
   (non ai wake successivi: il registro resta impostato), ciclo completo
   con ACK invariato.
2. Se hai un multimetro: corrente di sleep dalla batteria (USB staccato) —
   atteso ~1.5 mA dominato dall'AS5600 (era ~6.5+).
3. Prova anche la build di produzione (`pio run -e station-release -t
   upload`): boot immediato senza i 2 s di attesa, niente dump di debug,
   telemetria e ACK regolari.

## TODO per lo step successivo (Increment 10 – OTA via LoRa)

- [ ] **Validazione hardware step 10** come da note sopra.
- [ ] Valutare load switch sul rail sensori (hardware, quando possibile).
- [ ] Iniziare l'OTA via LoRa: progettazione del protocollo a pacchetti
      (messaggi `ota_*` in lora-protocol.md, numerazione, ACK,
      ritrasmissione, verifica integrità, `Update` + rollback).
