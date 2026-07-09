# Step 05 – Vento: anemometro e banderuola AS5600

**Data:** 2026-07-09
**Versione:** v2.5.0

## Attività svolte

- Validato l'Increment 4 su hardware; pubblicata release `v2.4.0`.
- Implementato `Anemometer` (`src/sensors/Anemometer.{h,cpp}`):
  - interrupt su GPIO 20 (pull-up, fronte di discesa), debounce 5 ms;
  - campionamento bloccante nella finestra di misura, a bin da 1 s:
    `ws` = frequenza media × fattore, `wg` = bin massimo × fattore;
  - fattore di conversione configurabile (`mps_per_hz`).
- Implementato `WindVane` (`src/sensors/WindVane.{h,cpp}`):
  - AS5600 pilotato direttamente via `Wire` (raw angle 12 bit, registro
    0x0C), senza libreria esterna;
  - `begin()` verifica la presenza del magnete (bit MD del registro
    status 0x0B): magnete assente ⇒ sensore unhealthy, campo `wd` assente;
  - `wd = (raw + vane_offset_deg) mod 360`, con l'angolo raw loggato a
    livello debug per la taratura.
- Sezione `[wind]` in `config.ini` (`mps_per_hz`, `sample_window_s`,
  `vane_offset_deg` normalizzato a 0–359).

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Vento campionato solo durante la veglia (nessun conteggio in sleep) | Contare i giri 24/7 richiederebbe di restare svegli o ULP dedicato: costo/complessità ingiustificati; la media su 5 s a ogni ciclo è la prassi per stazioni a batteria. |
| Raffica = bin da 1 s più alto | Con bin da 1 s la risoluzione è ~0.7 m/s: sufficiente e semplicissima; la finestra è allargabile da config. |
| AS5600 senza libreria | Servono due letture di registro: una dipendenza esterna sarebbe più codice di quello che evita. |
| `mps_per_hz = 0.667` come default dichiarato provvisorio | Convenzione dei cup anemometer ("1 Hz = 2.4 km/h"); da sostituire con il dato di targa reale dell'anemometro installato. |
| Offset banderuola in config, non hardcoded | Il sensore può essere montato in qualsiasi orientamento; la taratura si fa sul campo senza ricompilare. |

## Note per la validazione

- **Anemometro**: la finestra di misura ora dura ~5 s in più (si vede la
  pausa tra `Sensors ready` e `Telemetry`). Simulare i giri
  cortocircuitando GPIO 20 a GND ripetutamente: `ws` > 0 e `wg` ≥ `ws`.
  Es. 10 contatti in 5 s → 2 Hz medi → `"ws":1.3`.
- **Banderuola**: ruotare il magnete dell'AS5600 e verificare che `wd`
  cambi (0–359). Senza magnete: warning al boot e JSON senza `wd`.
- **Taratura offset**: puntare la banderuola a Nord, leggere l'angolo raw
  (log debug), impostare `vane_offset_deg = 360 - raw`.

## TODO per lo step successivo (Increment 6 – INA3221)

- [ ] **Validazione hardware step 5** come da note sopra.
- [ ] Chiedere il dato di targa reale dell'anemometro (impulsi/giro e
      conversione in m/s) e aggiornare `mps_per_hz`.
- [ ] `PowerMonitor` (INA3221, I2C 0x40): tensioni/correnti dei 3 canali
      (1 = pannello, 2 = batteria, 3 = carico) → `vbat`, `ibat`, `ipan`,
      `iload`; valutare la resistenza di shunt installata.
