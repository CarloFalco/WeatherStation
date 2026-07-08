# WeatherStation V2 – Protocollo LoRa (bozza v0.1)

Data: 2026-07-08 · Stato: **bozza di riferimento** per lo sviluppo del
ricevitore/gateway; verrà consolidato nell'increment del protocollo (ACK,
ritrasmissioni) e in quello OTA.

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
{"type":"data","id":"ws-01","fw":"2.1.0","seq":123,"t":21.4,"rh":63.0,"p":1013.2,"rain":0.6,"ws":3.2,"wg":5.1,"wd":270,"vbat":3.98,"ibat":-52,"ipan":120,"iload":35}
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
| `ibat` | int | mA | INA3221 ch2 | Corrente batteria (positiva = carica, negativa = scarica) |
| `ipan` | int | mA | INA3221 ch1 | Corrente pannello solare |
| `iload` | int | mA | INA3221 ch3 | Corrente assorbita dal carico |

Note per il ricevitore:

- Campi **assenti** = sensore guasto o non ancora implementato: il parser
  non deve assumere che ci siano tutti (durante lo sviluppo arriveranno
  prima `t/rh/p`, poi via via gli altri).
- `id` va sempre controllato: più stazioni condividono lo stesso canale.
- Fino all'increment "protocollo" la stazione trasmette senza attendere
  ACK (fire-and-forget): per il primo debug basta stampare i pacchetti.

## ACK (base → stazione) — bozza, non ancora implementato

```json
{"type":"ack","id":"ws-01","seq":123}
```

Inviato dalla base subito dopo la ricezione; la stazione resta in RX per
una breve finestra (~500 ms) dopo ogni trasmissione. Se l'ACK non arriva,
`rain` e gli altri accumulatori NON vengono azzerati e il dato viene
riaggregato nell'invio successivo. I dettagli (timeout, retry) verranno
definiti nell'increment dedicato, insieme ai messaggi OTA.
