# WeatherStation V2 – Protocollo LoRa (v1.0)

Data: 2026-07-09 · Stato: **implementato** (fw ≥ 2.8.0) per telemetria e
ACK; i messaggi OTA verranno aggiunti dagli increment dedicati.

## Parametri radio

Raw LoRa punto-punto (non LoRaWAN), banda EU 868 MHz. I valori sono i
default di `config.ini` della stazione: **il ricevitore deve usare gli
stessi parametri**, altrimenti non si sente nulla.

| Parametro | Default | Note |
|-----------|---------|------|
| Frequenza | **868.1 MHz** | banda EU868, duty cycle ≤ 1% |
| Bandwidth | **125 kHz** | |
| Spreading factor | **SF7** | airtime minimo → minor consumo; alzare a SF9 se serve più portata |
| Coding rate | **4/5** | |
| Sync word | **0x12** | rete privata (0x34 è riservato a LoRaWAN) |
| Preambolo | 8 simboli | default SX1276 |
| CRC | attivo | header esplicito |
| Potenza TX | 14 dBm | limite ERP banda 868 |

Con questi parametri il payload di telemetria (~150 byte) ha un airtime di
~120 ms. Limite payload LoRa a SF7/125kHz: 222 byte.

## Messaggio di telemetria (stazione → base)

Un oggetto JSON compatto (senza spazi né newline) per ogni ciclo di misura:

```json
{"type":"data","id":"ws-01","fw":"2.6.0","seq":123,"t":21.4,"rh":63.0,"p":1013.2,"rain":0.6,"ws":3.2,"wg":5.1,"wd":270,"vbat":3.98,"soc":87,"ibat":-52,"ipan":120,"iload":35}
```

| Campo | Tipo | Unità | Sorgente | Descrizione |
|-------|------|-------|----------|-------------|
| `type` | string | — | — | `"data"` = telemetria (futuri: `"ack"`, `"ota"`, ...) |
| `id` | string | — | config.ini | Identificativo stazione (`ws-01`, `ws-02`, ...) |
| `fw` | string | — | version.h | Versione firmware SemVer |
| `seq` | uint | — | RTC RAM | Contatore progressivo messaggi (rollover a 65535); per rilevare messaggi persi e correlare gli ACK |
| `t` | float | °C | BME280 | Temperatura, 1 decimale |
| `rh` | float | % | BME280 | Umidità relativa, 1 decimale |
| `p` | float | hPa | BME280 | Pressione (a livello stazione), 1 decimale |
| `rain` | float | mm | Pluviometro | Pioggia accumulata dall'ultimo invio riuscito |
| `ws` | float | m/s | Anemometro | Velocità vento media nella finestra di campionamento |
| `wg` | float | m/s | Anemometro | Raffica (massimo nella finestra) |
| `wd` | uint | ° | AS5600 | Direzione vento, 0–359 (0 = Nord, senso orario) |
| `vbat` | float | V | INA3221 ch2 | Tensione batteria |
| `soc` | uint | % | INA3221 ch2 | Stato di carica batteria (0–100), da curva di scarica LiPo |
| `ibat` | int | mA | INA3221 ch2 | Corrente batteria (positiva = carica, negativa = scarica) |
| `ipan` | int | mA | INA3221 ch1 | Corrente pannello solare |
| `iload` | int | mA | INA3221 ch3 | Corrente assorbita dal carico |

Note per il ricevitore:

- Campi **assenti** = sensore guasto o non ancora implementato: il parser
  non deve assumere che ci siano tutti (durante lo sviluppo arriveranno
  prima `t/rh/p`, poi via via gli altri).
- `id` va sempre controllato: più stazioni condividono lo stesso canale.

## ACK (base → stazione)

```json
{"type":"ack","id":"ws-01","seq":123}
```

Sequenza (fw ≥ 2.8.0, parametri in `config.ini` sezione `[lora]`):

1. La stazione trasmette il messaggio `data` con `seq` corrente.
2. Apre una finestra RX di `ack_timeout_ms` (default **600 ms**).
3. La base, ricevuto il pacchetto, risponde con l'ACK dopo un breve
   ritardo (~50 ms, per lasciare alla stazione la commutazione TX→RX):
   `id` e `seq` devono essere **copiati dal messaggio ricevuto**.
4. La stazione accetta solo l'ACK con `type/id/seq` combacianti; i
   pacchetti estranei nella finestra vengono ignorati senza chiuderla.
5. Se la finestra scade, la stazione ritrasmette lo **stesso** messaggio
   (stesso `seq`) fino a `tx_retries` volte (default 1 retry).
6. Solo a consegna confermata l'accumulatore pioggia viene consumato
   (sottraendo lo snapshot riportato, così le basculate arrivate durante
   la finestra non vanno perse); senza ACK, `rain` viaggia riaggregato
   nel ciclo successivo.

Con `ack_enabled = false` la stazione è fire-and-forget: nessuna finestra
RX e accumulatori azzerati a ogni tentativo (modalità di debug/bring-up).

Nota per la base: un `seq` già visto con lo stesso `id` indica una
ritrasmissione (il precedente ACK si è perso): va comunque ri-ACKato e
il dato va deduplicato a valle.
