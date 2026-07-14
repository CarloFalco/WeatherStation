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
    StaticJsonDocument<128> ackDoc;
    ackDoc["type"] = "ack";
    ackDoc["id"] = doc["id"];
    ackDoc["seq"] = doc["seq"];
    String ack;
    serializeJson(ackDoc, ack);
    LoRa.beginPacket();
    LoRa.print(ack);
    LoRa.endPacket();
    Serial.println("[TX] ACK inviato: " + ack);
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
