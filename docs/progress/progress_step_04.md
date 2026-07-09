# Step 04 – Pluviometro: conteggio impulsi e wake-on-rain

**Data:** 2026-07-09
**Versione:** v2.4.0

## Attività svolte

- Validato l'Increment 3 su hardware; pubblicata release `v2.3.0`.
- Implementato `RainGauge` (`src/sensors/RainGauge.{h,cpp}`):
  - interrupt su GPIO 19 (input pull-up, fronte di discesa) con debounce
    software di 150 ms contro i rimbalzi del reed;
  - accumulo impulsi in `RtcState::rainPulses` (RTC RAM): nessuna pioggia
    persa tra i cicli di deep sleep;
  - campo `rain` [mm] nel JSON (2 decimali); azzeramento dell'accumulatore
    dopo ogni invio (dopo ogni *invio confermato* quando arriverà l'ACK).
- Wake-on-rain in `PowerManager::deepSleep()`: pull-up abilitato nel
  dominio RTC (quello digitale è spento in deep sleep) + EXT0 su livello
  basso.
- "Quick path" in `main.cpp`: al risveglio per pioggia la stazione conta
  l'impulso e ridorme per il tempo residuo fino al ciclo schedulato,
  calcolato con `RtcState::nextWakeEpochS` e l'orologio RTC (che continua
  a correre in deep sleep). Niente attesa USB, sensori né radio.
- Sezione `[rain]` in `config.ini` con `mm_per_pulse` (validazione 0–10).

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| `mm_per_pulse = 0.4743`, non 47.43 come nel vecchio prototipo | 3 ml / 63.25 cm² = 0.04743 cm = **0.4743 mm** per basculata (1 L/m² = 1 mm). Il 47.43 derivava da cm² trattati come 10⁻⁶ m² anziché 10⁻⁴. Corretto e reso configurabile. |
| Wake da pioggia = solo conteggio, nessuna trasmissione | Con pioggia intensa (50 mm/h ≈ 100 basculate/h) trasmettere a ogni tick distruggerebbe batteria e duty cycle LoRa; il dato viaggia col ciclo periodico. |
| Schedulazione con `nextWakeEpochS` in RTC RAM | Il timer di sleep non "ricorda" il tempo già dormito: senza questo, ogni basculata farebbe ripartire l'intervallo da capo ritardando la telemetria indefinitamente sotto pioggia continua. |
| Debounce 150 ms nell'ISR | I rimbalzi del reed durano pochi ms; due basculate reali non possono essere più vicine di così nemmeno in un nubifragio. |
| Il quick path salta anche `Serial.begin` + delay 2 s | È il grosso del tempo di veglia: il costo energetico di un evento pioggia si riduce di ~10×. |

## Note per la validazione

- Simulare le basculate cortocircuitando GPIO 19 a GND (filo o pulsante):
  - **a stazione sveglia**: ogni contatto = +1 impulso (rimbalzi filtrati);
  - **in deep sleep**: il contatto sveglia la scheda che ridorme subito
    (sul monitor non appare quasi nulla: è il quick path — normale);
  - al ciclo successivo il JSON deve riportare `rain` > 0, es. 5 impulsi
    → `"rain":2.37`.
- Verificare che il ciclo periodico resti puntuale anche con impulsi in
  mezzo (l'intervallo non deve ripartire da capo a ogni contatto).
- Il conteggio si azzera dopo ogni invio: due cicli senza pioggia
  consecutivi → `"rain":0`.

## TODO per lo step successivo (Increment 5 – vento)

- [ ] **Validazione hardware step 4** come da note sopra.
- [ ] `Anemometer`: conteggio impulsi su GPIO 20 in una finestra di
      campionamento durante la veglia → `ws` (media) e `wg` (raffica).
      Servirà il fattore di conversione impulsi→m/s dell'anemometro.
- [ ] `WindVane`: lettura angolo AS5600 via I2C (0x36) → `wd` [°],
      con offset di montaggio configurabile in `config.ini`.
