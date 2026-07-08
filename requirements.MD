

# Obiettivo generale

## Progettare e sviluppare una stazione meteorologica basata su ESP32-S3, alimentata a batteria:

- Legge vari sensori (pluviometro, temperatura, umidità, pressione, direzione e velocità del vento, ecc.).
- Invia tramite lora in formato json ad una base le informazioni raccolte
- Supporta aggiornamenti firmware (OTA) con una struttura di questo tipo (GitHub Release -> Internet -> MQTT -> Gateway ESP32 -> LoRa -> Stazione meteo)
- Possono esserci piu stazioni meteo che comunicano ad un unica base. 
- È sviluppata con PlatformIO.
- È versionata su GitHub con gestione delle release.
- Ha documentazione generata con Doxygen.
- Tiene traccia degli avanzamenti in file Markdown.

# Informazioni aggiuntive
- Invio periodico + deep sleep
- configurabile su un file .ini
Autonomia attesa > 12 mesi
Stack software preferito
- Arduino framework

---

# Requisiti hardware e funzionali

## Microcontrollore
ESP32-S3 (specifica le funzionalità che intendi usare: LORA, timer, deep sleep, ecc.) e gestione accurata dei consumi (uso di deep sleep, riduzione periferiche attive, ecc.).

## Sensori
- Pluviometro (conta impulsi).
- Sensore temperatura/umidità (DHT22, BME280).
- Direzione del vento (AS5600).
- Velocità del vento (Cup anemometer, reed switch).
- Sensore corrente (INA3221) monitoraggio flussi (1 pannello, 2 batteria, 3 consumo)


## Gestione consumi
- Wake-up periodico per misure e invio dati.
- Possibile wake-up su evento (es. pioggia, vento forte) se sensato.
- Stima qualitativa dei consumi e del duty cycle.

## Pinout

- BME280 (BME280_ADDRESS 0x76)
- Pluviometro a reed switch (GPIO_NUM_19)
- Anemometro a reed switch (GPIO_NUM_20 )
- AS5600 (AS5600_ADDRESS 0x36)
- INA3221 (INA_ADDRESS 0x40)
- define LED_BUILTIN 97

## Comunicazione 
- Lora tramite modulo SX1276 868 Mhx

---

# Requisiti software e architetturali

## Ambiente di sviluppo
- Uso di PlatformIO (con file platformio.ini ben configurato).
- Struttura del progetto (cartelle src/, include/, lib/, ecc.).

## Architettura del codice
Progettazione modulare:

Modulo per ogni sensore (es. SensorRain, SensorTempHum, SensorWindDir, SensorWindSpeed).
Modulo per gestione energia/sleep.
Modulo per LORA.
Modulo per OTA/aggiornamenti.
Uso di classi C++ o strutture ben organizzate.

## Aggiornamenti OTA

1.Le stazioni meteo comunicano tramite LoRa con un gateway, sempre alimentato e connesso alla rete Wi-Fi. 
Il gateway riceve i dati dai nodi e li pubblica sul broker MQTT.

2.Periodicamente il gateway verifica la disponibilità di nuove versioni del firmware. 
Se è presente un aggiornamento, pubblica una notifica su MQTT, rendendola disponibile all'utente tramite Home Assistant o un'app dedicata.

3.Quando l'utente conferma l'aggiornamento, il gateway scarica il nuovo firmware (o utilizza una copia già memorizzata), 
lo suddivide in pacchetti e lo trasmette alla stazione meteo tramite LoRa. 
Al termine del trasferimento, il nodo verifica l'integrità del firmware e, se la verifica ha esito positivo, esegue l'aggiornamento OTA e si riavvia.

Per garantire l'affidabilità del processo, il protocollo di aggiornamento prevede la numerazione dei pacchetti, la conferma di ricezione (ACK), la ritrasmissione dei pacchetti persi e la verifica finale dell'integrità del firmware.


## Bonus
lasciamo la predisposizione per poter decidere se la stazione comunica via Lora alla base e poi sara quest'ultima a comunicare con MQTT, oppure è direttamente connessa al wifi e comunica via MQTT al broker
Nel caso in cui venga utilizzato il secondo modo andranno create delle funzioni per la creazione di un access point al quale connettersi per dare le informazioni del server MQTT e del wifi. 



---

# Requisiti di progetto, GitHub e release

Repository GitHub
Proponi una struttura di repository chiara (cartelle, file principali).
Suggerisci un file README.md iniziale con:
Descrizione progetto.
Requisiti hardware/software.
Istruzioni base per compilazione e flash con PlatformIO.
Suggerisci file .gitignore adatto a PlatformIO/ESP32.
Gestione rilasci software con changelog 

Gestione release
Proponi una strategia di versioning (es. SemVer: v1.0.0, v1.1.0, ecc.).
Per ogni milestone importante, prevedi una GitHub Release con:
Changelog sintetico.
Eventuale asset binario del firmware (file .bin).

---

# Documentazione con Doxygen

Configurazione Doxygen
Fornisci un file di configurazione base Doxyfile (o indicazioni precise per generarlo).

Struttura dei commenti Doxygen nel codice (per classi, funzioni, costanti, moduli).

Stile di documentazione
Commenti Doxygen in italiano o inglese (scegli uno e mantienilo coerente).

Descrizione di:
Scopo di ogni modulo.
Parametri e ritorni delle funzioni principali.
Esempi d’uso se opportuno.

---

# Log di avanzamento in Markdown

Ad ogni avanzamento logico del progetto (es. definizione architettura, implementazione sensori, implementazione MQTT, implementazione OTA, ottimizzazione consumi, ecc.), voglio che tu produca un file Markdown (o il suo contenuto) che contenga:
Titolo dello step (es. Step 01 - Definizione architettura sensori).
Data (puoi usare una data fittizia o generica).
Elenco puntato delle attività svolte.
Eventuali decisioni progettuali prese e motivazioni.
Eventuali TODO per lo step successivo.

Il file potrà chiamarsi ad esempio progress_step_XX.md.

---

# Modalità di lavoro

## Chiarimenti iniziali
Prima di iniziare, fammi qualche domanda mirata per chiarire:
Elenco sensori definitivi (se voglio fissarli).
Frequenza di campionamento e invio dati.
Vincoli di autonomia (es. giorni di funzionamento attesi).
Eventuali preferenze su librerie (es. Arduino framework vs ESP-IDF in PlatformIO).

## Approccio iterativo/ agile Increment 0. Gestione credenziali

utilizza un approccio a piccoli incrementi e ben documentati aggiungendo poche funzion alla volta e versionando il codice con git 

Riassumi cosa è stato fatto.
Fornisci il contenuto del file Markdown di avanzamento relativo allo step.
Rilascio della versione su github, con changelog 

Stile del codice
Usa C++ (Arduino framework su ESP32-S3, salvo diversa indicazione).

Mantieni il codice leggibile, con nomi chiari e coerenti.
Inserisci commenti Doxygen man mano che definisci classi e funzioni.



