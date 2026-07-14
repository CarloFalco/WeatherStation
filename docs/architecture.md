# WeatherStation V2 – Architettura software

Data: 2026-07-08 · Stato: approvata per Increment 0

## Visione d'insieme

Stazione meteo a batteria basata su ESP32-S3. Ciclo operativo:

```
wake-up (timer 10 min oppure evento pioggia)
   └─> lettura sensori ──> payload JSON ──> invio LoRa (SX1276, 868 MHz)
          └─> finestra di ricezione (ACK / comandi / OTA)
                 └─> deep sleep
```

Possono esistere più stazioni che parlano con un'unica base (gateway
ESP32 sempre alimentato, fuori dallo scope di questo repo): ogni messaggio
include uno `station_id`.

## Principi guida

1. **Increment piccoli e validabili**: ogni modulo arriva in un increment
   dedicato, compilato e testato su hardware prima del successivo.
2. **Estensibilità dei sensori**: aggiungere o sostituire un sensore non
   deve toccare `main.cpp` né il modulo LoRa — solo una nuova classe che
   implementa `ISensor` e una riga di registrazione.
3. **Semplicità prima di tutto**: niente RTOS task espliciti né layer
   superflui finché non servono; il ciclo è sequenziale (wake → misura →
   invia → dormi), il che rende i consumi prevedibili.
4. **Separazione configurazione**:
   - `include/config.h` → *hardware* (pin, indirizzi I2C), fisso a build-time
   - `/config.ini` su LittleFS → *parametri runtime* (intervallo, station_id,
     parametri LoRa), modificabile senza ricompilare

## Struttura dei moduli

```
src/
  main.cpp              orchestrazione del ciclo (unico punto che "sa tutto")
  core/
    AppConfig.*         parser/serializer di /config.ini su LittleFS
    PowerManager.*      deep sleep, cause di wake-up, stato persistente in RTC RAM
  sensors/
    ISensor.h           interfaccia: begin() / read(JsonObject&) / name()
    SensorManager.*     registro dei sensori, raccolta letture nel JSON
    Bme280Sensor.*      temperatura / umidità / pressione (I2C 0x76)
    RainGauge.*         pluviometro a impulsi (GPIO 6, contatore in RTC RAM,
                        wake-on-rain via EXT wake-up)
    Anemometer.*        velocità vento, conteggio impulsi su finestra (GPIO 7)
    WindVane.*          direzione vento via AS5600 (I2C 0x36)
    PowerMonitor.*      INA3221: pannello / batteria / carico (I2C 0x40)
  comm/
    ITelemetryLink.h    interfaccia: begin() / send() / receive()
    LoRaLink.*          implementazione SX1276 via RadioLib
    (futuro) MqttLink.* variante bonus: WiFi + MQTT diretto, stessa interfaccia
  ota/
    (futuro) LoRaOta.*  ricezione firmware a pacchetti: numerazione, ACK,
                        ritrasmissione, verifica integrità, Update + reboot
include/
  config.h              pinout e indirizzi I2C
  version.h             versione firmware (SemVer)
  rtc_state.h           struttura dati persistente in RTC RAM tra i deep sleep
data/
  config.ini            configurazione di default caricata su LittleFS
docs/
  architecture.md       questo file
  lora-protocol.md      (futuro) formato messaggi JSON + protocollo OTA
  progress/             progress_step_XX.md per ogni increment
```

### Perché le interfacce `ISensor` e `ITelemetryLink`

- `ISensor` disaccoppia il ciclo di misura dai sensori concreti: il
  `SensorManager` itera sui sensori registrati e ognuno scrive i propri
  campi nel documento JSON. Sensori futuri = nuova classe + registrazione.
- `ITelemetryLink` copre il requisito bonus: la stazione può inviare via
  LoRa (default) o direttamente via WiFi/MQTT, scelta da `config.ini`,
  senza toccare il resto del codice.

Sono le **uniche due astrazioni** previste: il resto è composizione diretta,
per tenere il codice leggibile.

## Gestione energia

- Deep sleep tra i cicli (target < 20 µA di sleep current lato MCU).
- Wake-up da timer (`esp_sleep_enable_timer_wakeup`, default 10 min).
- Wake-up da evento pioggia: GPIO 6 è RTC-capable → EXT wake-up; il
  conteggio delle vaschette è mantenuto in RTC RAM e azzerato dopo l'invio.
- L'anemometro NON sveglia il nodo: la velocità è misurata contando gli
  impulsi in una finestra di campionamento (es. 3–5 s) durante il wake.
- Radio, I2C e sensori alimentati/attivi solo durante la finestra di misura.
- Stima consumi e duty cycle: increment dedicato dopo l'integrazione LoRa.

## Formato dati (bozza, da consolidare nell'increment LoRa)

```json
{
  "id": "ws-01",
  "fw": "2.0.0",
  "seq": 123,
  "t": 21.4,   "rh": 63.0,  "p": 1013.2,
  "rain": 0.6, "ws": 3.2,   "wd": 270,
  "vbat": 3.98, "ipan": 120, "iload": 35
}
```

Vincolo LoRa: payload ≤ 222 byte (SF7..SF9, 868 MHz); il JSON compatto sopra
è ~150 byte. Se in futuro servisse più spazio si valuterà un formato binario,
ma il requisito attuale è JSON.

## Roadmap degli increment

| # | Increment | Versione | Validazione |
|---|-----------|----------|-------------|
| 0 | Scaffold: PlatformIO, pinout, boot, versioning, CI release | v2.0.0 | build + flash, banner su seriale |
| 1 | AppConfig: `/config.ini` su LittleFS | v2.1.0 | lettura/scrittura config da seriale |
| 2 | PowerManager: deep sleep + wake timer + stato RTC | v2.2.0 | ciclo sleep/wake osservato |
| 3 | ISensor + SensorManager + BME280 | v2.3.0 | letture T/RH/P plausibili |
| 4 | RainGauge: impulsi + wake-on-rain | v2.4.0 | conteggio vaschette anche in sleep |
| 5 | Anemometer + WindVane (AS5600) | v2.5.0 | velocità/direzione plausibili |
| 6 | PowerMonitor (INA3221) | v2.6.0 | correnti/tensioni sui 3 canali |
| 7 | LoRaLink: invio JSON alla base | v2.7.0 | ricezione su nodo di test |
| 8 | Protocollo: station_id, seq, ACK + doc `lora-protocol.md` | v2.8.0 | round-trip con ACK |
| 9 | Ottimizzazione consumi + stima autonomia | v2.9.0 | misura corrente in sleep |
| 10+ | OTA via LoRa (più increment: trasporto, integrità, apply) | v2.10+ | update end-to-end |
| — | Doxygen + rifinitura doc → release stabile | v3.0.0 | doc generata |

Ogni increment si chiude con: build pulita → flash e test su hardware →
`progress_step_XX.md` → commit → tag → GitHub Release con changelog.
