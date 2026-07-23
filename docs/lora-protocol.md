# WeatherStation V2 – Protocollo LoRa (v1.1)

Data: 2026-07-15 · Stato: telemetria e ACK **implementati** (fw ≥ 2.8.0);
trasferimento OTA a chunk **implementato** (fw ≥ 3.0.0-alpha.1, stage 1:
file di test su LittleFS); scrittura partizione/rollback/firma negli
increment successivi.

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

## Trasferimento OTA (base → stazione) — stage 1 in fw 3.0.0-alpha.1

La stazione dorme: ogni scambio parte da lei. La base può **piggybackare
un'offerta OTA dentro l'ACK** di telemetria; la stazione resta sveglia ed
entra in sessione di trasferimento con modello **pull** (richiede lei ogni
chunk ⇒ ACK, ordinamento e ritrasmissioni sono impliciti).

### 1. Offerta (campo `ota` nell'ACK)

```json
{"type":"ack","id":"ws-01","seq":123,
 "ota":{"size":4096,"crc":305419896,"chunks":23,"ver":"3.1.0"}}
```

| Campo | Descrizione |
|-------|-------------|
| `size` | Dimensione totale dell'immagine [byte] |
| `crc` | CRC-32 (IEEE 802.3, poly 0xEDB88320) dell'intera immagine, come uint |
| `chunks` | Numero di chunk = ceil(size / 180) |
| `ver` | Versione offerta (informativa; la stazione la logga) |

### 2. Richiesta chunk (stazione → base, JSON)

```json
{"type":"ota_req","id":"ws-01","idx":0}
```

### 3. Chunk (base → stazione, **binario**, non JSON)

| Offset | Campo | Dimensione | Note |
|--------|-------|-----------|------|
| 0 | magic | 1 B | `0xA5` |
| 1 | idx | 2 B | little-endian, deve combaciare con la richiesta |
| 3 | payload | ≤ 180 B | ultimo chunk = size − idx·180 byte |

Binario puro: base64 dentro JSON sprecherebbe ~33% dei 255 byte LoRa.
Integrità del singolo pacchetto già garantita dal CRC LoRa hardware.

### 4. Esito (stazione → base)

```json
{"type":"ota_done","id":"ws-01","ok":true,"crc":305419896,"next":2365}
```

`next` = primo chunk non ancora ricevuto (punto di ripresa). Con `ok:false`
la base **deve mantenere l'offerta armata**: non è un fallimento definitivo,
la stazione riprenderà da `next` al ciclo successivo.

> ⚠️ Nota per chi implementa la base in ArduinoJson: leggere `crc` con un
> default **unsigned** (`doc["crc"] | 0UL`). Con `| 0` la conversione passa
> per `int` e ogni CRC sopra `0x7FFFFFFF` — metà dei casi — viene letto 0.

### 5. Ripresa del trasferimento

Un'immagine reale (~425 kB) sono ~2400 chunk e **~20 minuti** a SF7: una
sessione persa non deve ripartire da capo. La stazione conserva in RTC RAM
(quindi anche attraverso il deep sleep) `size`, `crc`, `chunks`, il prossimo
chunk e il CRC parziale. Alla successiva offerta:

- se `size`/`crc`/`chunks` **coincidono** e il file su LittleFS è lungo
  esattamente `next × 180` byte ⇒ riprende da `next` (append);
- altrimenti riparte da 0 (immagine diversa, oppure file incoerente).

La base non deve fare nulla di speciale: continua a servire i chunk
richiesti, semplicemente il primo `ota_req` della nuova sessione avrà un
indice > 0.

### Timing e ritrasmissioni (lato stazione, fw ≥ 3.0.0-alpha.2)

Configurabili in `config.ini`, sezione `[ota]`:

| Parametro | Default | Significato |
|-----------|---------|-------------|
| `chunk_timeout_ms` | 1500 | attesa del chunk dopo ogni richiesta |
| `max_retries` | 8 | tentativi per chunk (ritrasmettendo `ota_req`) |
| `session_timeout_s` | 1800 | tetto al tempo di veglia di UNA sessione |

- Fra un tentativo e l'altro la stazione attende 20–80 ms casuali, per non
  restare agganciata a un pattern di collisione con la base.
- La stazione ascolta in **RX continuo** durante il trasferimento (non a
  finestre RX-single come per l'ACK): non ci sono buchi d'ascolto in cui
  perdere il preambolo di un chunk.
- La base deve attendere ~25 ms prima di rispondere a una `ota_req`
  (commutazione TX→RX della stazione), **rimettersi subito in ascolto dopo
  aver trasmesso** il chunk, e considerare la sessione morta dopo ~60 s
  senza richieste.
- Throughput misurato su hardware a SF7: **~0.5 s/chunk** (~113 ms la
  richiesta + ~25 ms turnaround + ~300 ms il chunk) ⇒ ~21 kB/min, cioè
  ~20 minuti per un firmware completo.

### Roadmap stage successivi

| Stage | Contenuto | Stato |
|-------|-----------|-------|
| 1 | Trasporto a chunk + CRC-32, immagine di test su LittleFS | fw 3.0.0-alpha.1 |
| 2 | Scrittura streaming nella partizione OTA inattiva (`esp_ota_begin/write/end`), riavvio, rollback automatico se il nuovo firmware non si auto-valida | pianificato |
| 3 | Sicurezza: digest SHA-256 nell'offerta + firma del binario verificata prima del boot definitivo | pianificato |
| — | Dashboard/notifiche MQTT (Home Assistant), download da GitHub Release: responsabilità del **gateway** (repo separato) — il gateway usa questo protocollo verso la stazione | fuori scope stazione |
