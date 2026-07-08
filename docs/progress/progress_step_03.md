# Step 03 – Interfaccia sensori, SensorManager e BME280

**Data:** 2026-07-08
**Versione:** v2.3.0

## Attività svolte

- Validato l'Increment 2 su hardware; pubblicata release `v2.2.0`.
- Definita l'interfaccia `ISensor` (`src/sensors/ISensor.h`):
  `name()` / `begin()` / `read(JsonObject&)` — ogni sensore scrive da sé i
  propri campi nel messaggio di telemetria.
- Implementato `SensorManager`: registro a capacità fissa (8 sensori,
  niente heap), `beginAll()` marca unhealthy i sensori che non rispondono
  e `readAll()` li salta — un guasto non blocca il ciclo né gli altri
  sensori, i campi mancano e basta (come previsto dal protocollo).
- Implementato `Bme280Sensor` (I2C `0x76`, bus su SDA=4/SCL=5):
  profilo Bosch "weather monitoring" (forced mode, oversampling 1x,
  filtro IIR off) → una conversione per ciclo, poi il sensore torna in
  sleep (~0.1 µA). Campi prodotti: `t` [°C], `rh` [%], `p` [hPa].
- `main.cpp`: la finestra di misura ora è reale — `Wire.begin`,
  registrazione sensori, composizione del JSON (`type`, `id`, `fw`,
  `seq` incrementato in RTC RAM) e stampa compatta su seriale, nello
  stesso formato che dall'Increment 7 partirà via LoRa.
- Aggiunte dipendenze: ArduinoJson `^7.4.1`, Adafruit BME280 `^2.2.4`.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| I sensori scrivono direttamente nel JSON (`read(JsonObject&)`) | Evita una struct intermedia da tenere sincronizzata col protocollo; il formato è definito in un posto solo (docs/lora-protocol.md). |
| Registro a capacità fissa, sensori come istanze statiche | Niente allocazioni dinamiche: footprint prevedibile su un firmware che riparte da zero a ogni wake-up. |
| Sensore guasto ⇒ campi assenti, ciclo continua | La stazione resta utile anche degradata; il ricevitore già tollera campi mancanti. |
| BME280 in forced mode | Profilo raccomandato Bosch per meteo: minima corrente, nessun campionamento continuo inutile tra i wake-up. |
| Arrotondamento a 1 decimale in aritmetica double | ArduinoJson serializzerebbe i float con artefatti ("21.39999962"); così il payload resta compatto e leggibile. |

## Note per la validazione

- Output atteso sul monitor seriale a ogni ciclo, ad esempio:
  `Telemetry (89 bytes): {"type":"data","id":"ws-01","fw":"2.3.0","seq":4,"t":24.3,"rh":51.2,"p":1008.7}`
- `seq` cresce a ogni ciclo (si azzera solo togliendo alimentazione).
- Se il BME280 non è collegato/raggiungibile: warning
  `Sensor BME280 failed to initialize` e JSON senza `t/rh/p` — anche
  questo è un comportamento da verificare (basta staccare SDA).

## TODO per lo step successivo (Increment 4 – pluviometro)

- [ ] **Validazione hardware step 3**: JSON con `t/rh/p` plausibili;
      prova anche il caso "sensore staccato".
- [ ] `RainGauge`: conteggio impulsi su GPIO 19 con debounce, accumulo in
      `RtcState::rainPulses`, campo `rain` [mm] nel JSON.
- [ ] Wake-on-rain: EXT0 su GPIO 19 in `PowerManager::deepSleep()` e
      gestione della causa di wake-up "pioggia" (conteggio senza invio o
      invio anticipato, da decidere).
- [ ] Definire i mm per impulso del pluviometro (dato di targa da chiedere).
