/*
 * LoRa Transceiver - ESP32-S3 + SX1276 (868 MHz)
 *
 * Firmware IDENTICO per entrambe le schede: ognuna trasmette
 * periodicamente un pacchetto JSON e resta in ascolto per ricevere
 * quello dell'altro nodo (comunicazione bidirezionale, half-duplex).
 *
 * Il LED RGB integrato mostra la potenza del segnale ricevuto (RSSI)
 * con una sfumatura verde -> rosso tramite rgbLedWrite().
 *
 * Librerie richieste (Library Manager):
 *  - LoRa by Sandeep Mistry
 *  - ArduinoJson by Benoit Blanchon (v6.x)
 *
 * IMPORTANTE: assegna un NODE_ID diverso su ciascuna scheda prima del flash.
 * Richiede ESP32 Arduino Core >= 3.0 (per rgbLedWrite).


 {"type":"data","id":"ws-01","fw":"2.7.0","seq":10,"t":28.1,"rh":51.7,"p":1005.9,"rain":0,"ws":0,"wg":0,"wd":296,"vbat":4.06,"soc":86,"ibat":503,"ipan":-6,"iload":1}


| Campo | Tipo | Unità | Sorgente | Descrizione |
|-------|------|-------|----------|-------------|
| type | string | — | — | "data" = telemetria (futuri: "ack", "ota", ...) |
| id | string | — | config.ini | Identificativo stazione (ws-01, ws-02, ...) |
| fw | string | — | version.h | Versione firmware SemVer |
| seq | uint | — | RTC RAM | Contatore progressivo messaggi (rollover a 65535); per rilevare messaggi persi e correlare gli ACK |
| t | float | °C | BME280 | Temperatura, 1 decimale |
| rh | float | % | BME280 | Umidità relativa, 1 decimale |
| p | float | hPa | BME280 | Pressione (a livello stazione), 1 decimale |
| rain | float | mm | Pluviometro | Pioggia accumulata dall'ultimo invio riuscito |
| ws | float | m/s | Anemometro | Velocità vento media nella finestra di campionamento |
| wg | float | m/s | Anemometro | Raffica (massimo nella finestra) |
| wd | uint | ° | AS5600 | Direzione vento, 0–359 (0 = Nord, senso orario) |
| vbat | float | V | INA3221 ch2 | Tensione batteria |
| soc | int | % | INA3221 ch2 | SOC batteria |
| ibat | int | mA | INA3221 ch2 | Corrente batteria (positiva = carica, negativa = scarica) |
| ipan | int | mA | INA3221 ch1 | Corrente pannello solare |
| iload | int | mA | INA3221 ch3 | Corrente assorbita dal carico |


 */

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// --- Identificativo del nodo: CAMBIARE su ogni scheda ---
#define NODE_ID 3   // sulla seconda scheda impostare 2

// --- Pinout SPI custom per ESP32-S3 ---
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_RST  9
#define LORA_DIO0 8

// DEVE combaciare con [lora] freq_mhz della stazione (868.1 MHz):
// con BW 125 kHz un offset di 100 kHz basta a perdere quasi tutti i pacchetti.
#define LORA_FREQ 868.1E6

// --- LED RGB integrato ---
// GPIO48 sulla maggior parte delle ESP32-S3-DevKitC-1, GPIO38 su alcune varianti/revisioni.
#define RGB_LED_PIN 48

#define TX_INTERVAL_MS 3000

unsigned long packetCounter = 0;
unsigned long lastSendTime = 0;

// --- OTA test sender ----------------------------------------------------
// Premere 'u' sul monitor seriale per armare un'offerta OTA: al prossimo
// pacchetto data l'ACK conterra' i metadati e partira' la sessione di
// trasferimento (protocollo in docs/lora-protocol.md, modello pull).
#define OTA_TEST_SIZE   4096
#define OTA_CHUNK_DATA  180
#define OTA_CHUNK_MAGIC 0xA5

bool otaArmed = false;
uint8_t otaData[OTA_TEST_SIZE];
uint32_t otaCrc = 0;
uint16_t otaChunks = 0;

// CRC-32 IEEE, identico a src/logic/crc32.cpp della stazione.
uint32_t crc32Buf(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

void armOta() {
  // Pattern deterministico: verificabile anche lato stazione.
  for (size_t i = 0; i < OTA_TEST_SIZE; i++) {
    otaData[i] = (uint8_t)((i * 31 + 7) & 0xFF);
  }
  otaCrc = crc32Buf(otaData, OTA_TEST_SIZE);
  otaChunks = (OTA_TEST_SIZE + OTA_CHUNK_DATA - 1) / OTA_CHUNK_DATA;
  otaArmed = true;
  Serial.printf("[OTA] armato: %u byte, %u chunk, CRC 0x%08lX\n",
                OTA_TEST_SIZE, otaChunks, (unsigned long)otaCrc);
  Serial.println("[OTA] l'offerta parte col prossimo ACK alla stazione");
}

// Sessione lato sender: serve le richieste ota_req finche' la stazione
// non manda ota_done (o timeout).
void otaSenderSession() {
  Serial.println("[OTA] sessione avviata, attendo richieste dalla stazione...");
  unsigned long deadline = millis() + 60000UL;

  while ((long)(deadline - millis()) > 0) {
    int packetSize = LoRa.parsePacket();
    if (!packetSize) continue;

    String rx = "";
    while (LoRa.available()) rx += (char)LoRa.read();

    StaticJsonDocument<192> doc;
    if (deserializeJson(doc, rx)) continue;
    const char* type = doc["type"] | "";

    if (strcmp(type, "ota_req") == 0) {
      uint16_t idx = doc["idx"] | 0;
      size_t off = (size_t)idx * OTA_CHUNK_DATA;
      if (off >= OTA_TEST_SIZE) continue;
      size_t n = OTA_TEST_SIZE - off;
      if (n > OTA_CHUNK_DATA) n = OTA_CHUNK_DATA;

      delay(30);  // commutazione TX->RX della stazione
      LoRa.beginPacket();
      LoRa.write(OTA_CHUNK_MAGIC);
      LoRa.write(idx & 0xFF);
      LoRa.write((idx >> 8) & 0xFF);
      LoRa.write(otaData + off, n);
      LoRa.endPacket();
      deadline = millis() + 60000UL;  // sessione viva: rinnova il timeout
      if (idx % 8 == 0) Serial.printf("[OTA] chunk %u/%u inviato\n", idx, otaChunks);

    } else if (strcmp(type, "ota_done") == 0) {
      bool okFlag = doc["ok"] | false;
      Serial.printf("[OTA] esito stazione: %s (CRC 0x%08lX, atteso 0x%08lX)\n",
                    okFlag ? "SUCCESS" : "FAILED",
                    (unsigned long)(doc["crc"] | 0), (unsigned long)otaCrc);
      otaArmed = false;
      return;
    }
  }
  Serial.println("[OTA] timeout sessione (nessuna richiesta/done)");
  otaArmed = false;
}

// Colora il LED in base a un valore 0-100 (sfumatura verde -> rosso)
void setRgbFromSignal(int value, int pin) {
  value = constrain(value, 0, 100);

  int red = map(value, 50, 100, 0, 255);
  int green = map(value, 50, 100, 255, 0);
  int blue = 0;

  // Il map() estrapola fuori dal range 50-100, quindi clampiamo
  red = constrain(red, 0, 255);
  green = constrain(green, 0, 255);

  rgbLedWrite(pin, red, green, blue);
}

// Converte l'RSSI (dBm, tipicamente -120..-30) in percentuale 0-100
int rssiToPercent(int rssi) {
  int value = map(rssi, -120, -30, 0, 100);
  return constrain(value, 0, 100);
}


void receivePacket() {
  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Stampa sempre la stringa raw: utile per capire se arrivano pacchetti
  // corrotti/parziali anche quando il parsing fallisce.
  Serial.print("[RX raw] (");
  Serial.print(received.length());
  Serial.print(" B, RSSI ");
  Serial.print(rssi);
  Serial.print(" dBm): ");
  Serial.println(received);

  // ~160 B di JSON + overhead di ArduinoJson: 256 era al limite (NoMemory).
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, received);

  if (error) {
    Serial.print("[RX] Errore parsing JSON: ");
    Serial.println(error.c_str());
    return;
  }
 // {"type":"data","id":"ws-01","fw":"2.7.0","seq":10,"t":28.1,"rh":51.7,"p":1005.9,"rain":0,"ws":0,"wg":0,"wd":296,"vbat":4.06,"soc":86,"ibat":503,"ipan":-6,"iload":1}

  // --- ACK verso la stazione -------------------------------------------
  // La stazione dopo il TX apre una finestra RX di ack_timeout_ms (600 ms
  // default): rispondiamo subito con {"type":"ack","id":...,"seq":...}.
  // Il piccolo delay lascia alla stazione il tempo di commutare TX -> RX.
  const char* type = doc["type"] | "";
  if (strcmp(type, "data") == 0) {
    delay(50);
    StaticJsonDocument<256> ackDoc;
    ackDoc["type"] = "ack";
    ackDoc["id"] = doc["id"];
    ackDoc["seq"] = doc["seq"];
    if (otaArmed) {
      // Offerta OTA piggyback nell'ACK (docs/lora-protocol.md).
      JsonObject ota = ackDoc.createNestedObject("ota");
      ota["size"] = OTA_TEST_SIZE;
      ota["crc"] = otaCrc;
      ota["chunks"] = otaChunks;
      ota["ver"] = "test-4k";
    }
    String ack;
    serializeJson(ackDoc, ack);
    LoRa.beginPacket();
    LoRa.print(ack);
    LoRa.endPacket();
    Serial.println("[TX] ACK inviato: " + ack);

    if (otaArmed) {
      otaSenderSession();
    }
  }

  String id = doc["id"];
  float temp = doc["t"];
  float hum = doc["rh"];

  Serial.println("---- Pacchetto ricevuto ----");
  Serial.print("ID:        "); Serial.println(id);
  Serial.print("Temp:      "); Serial.print(temp); Serial.println(" C");
  Serial.print("Umidita':  "); Serial.print(hum); Serial.println(" %");
  Serial.print("RSSI:      "); Serial.print(rssi); Serial.println(" dBm");
  Serial.print("SNR:       "); Serial.println(snr);
  Serial.println("-----------------------------");

  int signalPercent = rssiToPercent(rssi);
  setRgbFromSignal(signalPercent, RGB_LED_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Errore: modulo LoRa non rilevato. Controlla i collegamenti.");
    while (1) delay(1000);
  }

  // Parametri radio: devono combaciare con la stazione (docs/lora-protocol.md)
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);   // rete privata (default del chip, ma esplicito e' meglio)
  LoRa.enableCrc();         // la stazione trasmette con CRC attivo
  LoRa.setTxPower(14);      // limite ERP banda EU868

  Serial.print("LoRa transceiver pronto - Nodo ");
  Serial.println(NODE_ID);

  rgbLedWrite(RGB_LED_PIN, 0, 0, 0); // LED spento all'avvio
}

void loop() {
  // 'u' da seriale = arma l'offerta OTA di test
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'u' || c == 'U') armOta();
  }

  // Invio periodico non bloccante
  if (millis() - lastSendTime >= TX_INTERVAL_MS) {
    lastSendTime = millis();
    // sendPacket();
  }

  // Ricezione non bloccante: parsePacket() rimette il radio in RX se necessario
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    receivePacket();
  }
}
