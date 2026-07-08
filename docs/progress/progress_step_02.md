# Step 02 – PowerManager: deep sleep e stato in RTC RAM

**Data:** 2026-07-08
**Versione:** v2.2.0

## Attività svolte

- Validato l'Increment 1 su hardware; pubblicata release `v2.1.0`.
- Implementato `PowerManager` (`src/core/PowerManager.{h,cpp}`):
  - `begin()`: classifica la causa di wake-up e incrementa il boot count;
  - `deepSleep(seconds)`: arma il timer (`esp_sleep_enable_timer_wakeup`),
    svuota la seriale ed entra in deep sleep (non ritorna);
  - predisposto il punto di aggancio per il wake-on-rain (EXT0, Increment 4).
- Creato `include/rtc_state.h` con la struttura `RtcState` in RTC slow
  memory: `bootCount`, `msgSeq` (futuro campo `seq` del JSON), `rainPulses`.
  Sopravvive al deep sleep, azzerata dall'hardware al power-on.
- `main.cpp` ristrutturato nel ciclo definitivo: tutto avviene in `setup()`
  (wake → log → config → misura → sleep); `loop()` non è mai raggiunto.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Stato tra i cicli in RTC RAM, non su flash | Scritture flash = lente, energivore e con usura; la RTC RAM è gratuita e sufficiente (contatori e accumulatori). |
| Tutto il ciclo in `setup()`, `loop()` vuoto | Dopo il deep sleep l'esecuzione riparte comunque da `setup()`: un ciclo lineare è più leggibile e i consumi sono prevedibili. |
| `deepSleep()` marcata `[[noreturn]]` | Rende esplicito il contratto: dopo la chiamata non esiste "dopo". |
| Il `delay(2000)` per l'enumerazione USB resta | Serve per il debug seriale durante lo sviluppo; sarà condizionato/rimosso nell'increment di ottimizzazione consumi. |

## Note per la validazione

- Con la seriale sul **connettore USB nativo** dell'S3 la porta COM cade a
  ogni deep sleep e si ri-enumera al wake-up: è normale; se disponibile,
  il connettore UART (bridge USB-seriale) mantiene la porta stabile.
- Per non aspettare 10 minuti tra i cicli: impostare temporaneamente
  `wake_interval_s = 20` in `data/config.ini` + `pio run -t uploadfs`
  (valida anche l'Increment 1), ricordandosi di ripristinare 600 dopo.

## TODO per lo step successivo (Increment 3 – primo sensore)

- [ ] **Validazione hardware step 2**: boot count che cresce a ogni ciclo,
      causa di wake-up = "timer", intervallo di sleep rispettato.
- [ ] Interfaccia `ISensor` + `SensorManager` (registro sensori, raccolta
      letture in un documento JSON via ArduinoJson).
- [ ] Primo sensore concreto: `Bme280Sensor` su I2C (SDA=4, SCL=5, 0x76)
      con letture di temperatura, umidità e pressione nel payload.
