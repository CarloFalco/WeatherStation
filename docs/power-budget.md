# Power budget e stima autonomia

Data: 2026-07-15 · fw 2.9.0 · Stato: **stima analitica**, da verificare con
misure reali (procedura in fondo). Requisito: autonomia > 12 mesi.

## Duty cycle (build di produzione, `wake_interval_s = 600`)

| Fase | Durata | Corrente media stimata | Carica |
|------|--------|------------------------|--------|
| Boot + init (80 MHz) | ~0.5 s | ~35 mA | 18 mAs |
| Sensori (BME forced + INA single-shot + AS5600) | ~0.4 s | ~35 mA | 14 mAs |
| Campionamento vento | 5 s | ~30 mA | 150 mAs |
| TX LoRa (SF7, ~160 B, 14 dBm) | ~0.13 s | ~120 mA | 16 mAs |
| Finestra ACK (RX) | ≤ 0.6 s | ~45 mA | 27 mAs |
| **Totale veglia** | **~6.6 s** | — | **~225 mAs ≈ 0.063 mAh/ciclo** |

144 cicli/giorno → **~9 mAh/giorno di veglia**. (La build dev con delay USB
2 s e log attivi sale a ~13 mAh/giorno: usare `station-release` sul campo.)

## Sleep (la parte che decide l'autonomia)

| Componente | Corrente in deep sleep | mAh/giorno |
|------------|------------------------|------------|
| ESP32-S3 deep sleep + RTC | ~10–15 µA | ~0.3 |
| SX1276 in sleep mode | ~0.2 µA | ~0 |
| BME280 (auto-sleep dopo forced) | ~0.1 µA | ~0 |
| INA3221 (power-down dopo single-shot) | ~2 µA | ~0.05 |
| **AS5600 in LPM3** (fw ≥ 2.9.0) | **~1500 µA** | **~36** |
| *(AS5600 in modalità default, fw < 2.9.0)* | *~6500 µA* | *~156* |

## Scenari di autonomia (batteria 3000 mAh, senza sole)

| Scenario | Consumo/giorno | Autonomia |
|----------|----------------|-----------|
| fw < 2.9.0 (AS5600 default) | ~165 mAh | **~18 giorni** ✗ |
| fw 2.9.0 (AS5600 LPM3) | ~45 mAh | **~66 giorni** — ok solo col pannello |
| Con load switch sul rail sensori (TODO hardware) | ~9.5 mAh | **~310 giorni** ✓ |

Col **pannello solare** (capacità osservata ~660 mA in pieno sole) bastano
in media ~2 mA di harvesting per coprire lo scenario LPM3: il requisito
> 12 mesi è raggiungibile già con fw 2.9.0, con margine ridotto nelle
settimane invernali più buie.

## Raccomandazioni hardware (in ordine di impatto)

1. **Load switch (P-MOSFET high-side) sul rail dei sensori I2C**, comandato
   da un GPIO: azzera il contributo dell'AS5600 in sleep (36 → ~0.5
   mAh/giorno). È il singolo intervento che porta l'autonomia da "dipende
   dal sole" a "> 12 mesi anche al buio". Predisposizione firmware banale
   (accendere il rail a inizio ciclo, spegnerlo prima del deep sleep).
2. **DevKit vs deploy**: la devkit ha bridge USB-seriale e LED di potenza
   sempre alimentati (mA persi non conteggiati sopra, dipendono dalla
   revisione). Per l'installazione definitiva: alimentare il rail 3V3
   direttamente da un buck/LDO a bassa quiescente (< 5 µA) o rimuovere i
   componenti superflui.
3. Se serve ridurre ancora la veglia: `sample_window_s = 3` (−90 mAs/ciclo)
   e/o `wake_interval_s = 900`.

## Ottimizzazioni firmware attive (fw 2.9.0)

- CPU a **80 MHz** in veglia (`CPU_FREQ_MHZ`, ~metà della corrente vs 240).
- Build `station-release`: niente `delay(2000)` USB (−2 s/ciclo) e log
  `log_d/log_i` eliminati a compile-time (`CORE_DEBUG_LEVEL=2`).
- AS5600 forzato in **LPM3** a ogni boot (registro CONF volatile).
- SX1276 in sleep mode (~0.2 µA) prima di ogni deep sleep.
- INA3221 in single-shot: power-down automatico tra i cicli.
- Quick path pioggia: wake EXT0 → conteggio → sleep in ~0.5 s senza radio
  né sensori.

## Procedura di misura (per validare le stime)

1. **Sleep**: multimetro in serie alla batteria (portata µA/mA), stazione in
   deep sleep, USB staccato. Atteso ~1.5 mA (LPM3); se molto di più,
   cercare il colpevole staccando un sensore alla volta.
2. **Veglia**: canale carico dell'INA3221 (`iload` nel JSON/debug) durante
   il ciclo, oppure multimetro in modalità max-hold.
3. **Bilancio reale**: `ibat` medio su 24 h (il gateway futuro potrà
   integrarlo dai dati di telemetria: positivo di giorno, negativo di notte).
