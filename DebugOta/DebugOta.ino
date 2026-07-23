/*
 * DebugOta - Gateway di test OTA (ESP32-S3 + SX1276 + WiFi)
 * ==========================================================
 *
 * Prototipo del gateway: verifica su GitHub Releases se esiste un firmware
 * piu' recente di quello in esecuzione sulla stazione, lo scarica via WiFi
 * su LittleFS e lo trasferisce alla stazione via LoRa, usando il protocollo
 * OTA "stage 1" gia' implementato lato nodo (src/ota/OtaReceiver + la
 * sezione "Trasferimento OTA" di docs/lora-protocol.md):
 *
 *   GitHub Release  --HTTPS-->  gateway (LittleFS)  --LoRa-->  stazione
 *
 * Flusso:
 *   1. il gateway interroga GitHub e scarica l'ultima release (.bin);
 *   2. calcola il CRC-32 mentre scarica e ricava size/chunks;
 *   3. quando la stazione invia la telemetria ("data"), il gateway risponde
 *      con l'ACK; se c'e' un aggiornamento per quella stazione, nell'ACK
 *      inserisce l'offerta OTA {size,crc,chunks,ver};
 *   4. la stazione entra in sessione e richiede i chunk (ota_req): il
 *      gateway li legge dal file e li spedisce (modello pull);
 *   5. a fine trasferimento la stazione risponde con ota_done (esito+CRC).
 *
 * NOTA STAGE 1: la stazione salva il .bin su LittleFS e ne verifica il CRC,
 * NON esegue ancora il flash (arrivera' con l'OTA stage 2). Quindi la sua
 * versione "fw" non cambia dopo il trasferimento: per ripetere la prova usa
 * il comando 'f' (force) qui sotto.
 *
 * Comandi da monitor seriale:
 *   u  -> controlla GitHub adesso (e scarica se c'e' un tag nuovo)
 *   f  -> forza l'offerta al prossimo "data" anche se le versioni coincidono
 *   i  -> stampa lo stato corrente
 *
 * Librerie richieste (Library Manager):
 *   - LoRa by Sandeep Mistry
 *   - ArduinoJson by Benoit Blanchon (v6.x)
 *   (WiFi / HTTPClient / WiFiClientSecure / LittleFS sono nel core ESP32)
 *
 * Arduino IDE: selezionare uno schema di partizione con filesystem
 * SPIFFS/LittleFS >= 1 MB (es. "Default 4MB with spiffs") per contenere il
 * firmware scaricato.
 *
 * Richiede ESP32 Arduino Core >= 3.0.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// --- WiFi: COMPILARE con le proprie credenziali -------------------------
#define WIFI_SSID     "IL_TUO_SSID"
#define WIFI_PASSWORD "LA_TUA_PASSWORD"

// --- Repository GitHub da cui prelevare le release ----------------------
#define GH_OWNER "CarloFalco"
#define GH_REPO  "WeatherStation"

// --- Pinout SPI del modulo LoRa (identico a stazione e DebugLora) -------
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_RST  9
#define LORA_DIO0 8

// --- Parametri radio: DEVONO combaciare con [lora] di config.ini --------
#define LORA_FREQ   868.1E6
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_CR     5
#define LORA_SYNC   0x12
#define LORA_TXPWR  14

// --- Protocollo OTA (vedi docs/lora-protocol.md / OtaReceiver.h) --------
#define OTA_CHUNK_MAGIC 0xA5
#define OTA_CHUNK_DATA  180
#define FW_PATH         "/firmware.bin"

// --- Ogni quanto ricontrollare GitHub da solo ---------------------------
#define CHECK_INTERVAL_MS (5UL * 60UL * 1000UL)

// --- Stato dell'aggiornamento -------------------------------------------
String   latestTag    = "";     // tag piu' recente visto su GitHub
String   downloadedTag = "";    // tag del firmware attualmente su LittleFS
uint32_t otaSize      = 0;      // dimensione firmware [byte]
uint32_t otaCrc       = 0;      // CRC-32 del firmware
uint16_t otaChunks    = 0;      // numero di chunk
bool     otaReady     = false;  // firmware scaricato e pronto da offrire
bool     forceOffer   = false;  // offri anche se le versioni coincidono ('f')

unsigned long lastCheck = 0;

// ------------------------------------------------------------------------
// Utility
// ------------------------------------------------------------------------

// Rimuove la 'v' iniziale dal tag (v3.0.0 -> 3.0.0) per confrontarlo con
// il campo "fw" della telemetria.
String stripV(const String& t) {
  return t.startsWith("v") ? t.substring(1) : t;
}

// CRC-32 IEEE 802.3 incrementale, identico a src/logic/crc32.cpp.
uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return crc;
}

void printStatus() {
  Serial.println("---- stato gateway OTA ----");
  Serial.printf("WiFi        : %s\n", WiFi.status() == WL_CONNECTED
                ? WiFi.localIP().toString().c_str() : "disconnesso");
  Serial.printf("GitHub tag  : %s\n", latestTag.isEmpty() ? "(mai controllato)" : latestTag.c_str());
  Serial.printf("Scaricato   : %s\n", downloadedTag.isEmpty() ? "(nessuno)" : downloadedTag.c_str());
  if (otaReady) {
    Serial.printf("Firmware    : %u byte, %u chunk, CRC 0x%08lX\n",
                  otaSize, otaChunks, (unsigned long)otaCrc);
  }
  Serial.printf("Force offer : %s\n", forceOffer ? "ON" : "off");
  Serial.println("---------------------------");
}

// ------------------------------------------------------------------------
// Download del firmware da GitHub su LittleFS (calcolando il CRC-32)
// ------------------------------------------------------------------------
bool downloadFirmware(const String& url, uint32_t expectedSize) {
  if (expectedSize == 0) {
    Serial.println("[OTA] dimensione asset sconosciuta, annullo");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();  // demo: nessun pinning del certificato GitHub

  HTTPClient https;
  https.begin(client, url);
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);  // github.com -> objects.githubusercontent.com
  https.addHeader("User-Agent", "WeatherStation-Gateway");

  int code = https.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] download HTTP %d\n", code);
    https.end();
    return false;
  }

  File f = LittleFS.open(FW_PATH, "w");
  if (!f) {
    Serial.println("[OTA] impossibile aprire " FW_PATH " in scrittura");
    https.end();
    return false;
  }

  WiFiClient* stream = https.getStreamPtr();
  uint32_t crc = 0xFFFFFFFFu;
  uint32_t total = 0;
  uint8_t buf[1024];
  unsigned long lastLog = millis();

  Serial.printf("[OTA] download di %u byte in corso...\n", expectedSize);
  while (https.connected() && total < expectedSize) {
    size_t avail = stream->available();
    if (avail) {
      int c = stream->readBytes(buf, min(avail, sizeof(buf)));
      f.write(buf, c);
      crc = crc32Update(crc, buf, c);
      total += c;
      if (millis() - lastLog > 1000) {
        Serial.printf("[OTA]   %u / %u byte (%d%%)\n", total, expectedSize,
                      (int)(100UL * total / expectedSize));
        lastLog = millis();
      }
    } else {
      delay(1);
    }
  }
  f.close();
  https.end();

  if (total != expectedSize) {
    Serial.printf("[OTA] download incompleto: %u / %u byte\n", total, expectedSize);
    return false;
  }

  otaCrc = crc ^ 0xFFFFFFFFu;
  otaSize = total;
  otaChunks = (otaSize + OTA_CHUNK_DATA - 1) / OTA_CHUNK_DATA;
  return true;
}

// ------------------------------------------------------------------------
// Interroga GitHub Releases; se c'e' un tag nuovo, scarica e arma l'offerta
// ------------------------------------------------------------------------
void checkForUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] WiFi non connesso, controllo saltato");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  // per_page=1 -> solo la release piu' recente (incluse le pre-release alpha)
  String api = "https://api.github.com/repos/" GH_OWNER "/" GH_REPO "/releases?per_page=1";
  https.begin(client, api);
  https.addHeader("User-Agent", "WeatherStation-Gateway");
  https.addHeader("Accept", "application/vnd.github+json");

  int code = https.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] GitHub API HTTP %d\n", code);
    https.end();
    return;
  }

  // Filtro: dal JSON (array di release) tieni solo cio' che serve.
  StaticJsonDocument<256> filter;
  filter[0]["tag_name"] = true;
  filter[0]["assets"][0]["name"] = true;
  filter[0]["assets"][0]["browser_download_url"] = true;
  filter[0]["assets"][0]["size"] = true;

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(
      doc, https.getStream(), DeserializationOption::Filter(filter));
  https.end();
  if (err) {
    Serial.printf("[OTA] parsing JSON fallito: %s\n", err.c_str());
    return;
  }

  JsonArray rels = doc.as<JsonArray>();
  if (rels.isNull() || rels.size() == 0) {
    Serial.println("[OTA] nessuna release trovata");
    return;
  }
  JsonObject rel = rels[0];
  latestTag = String((const char*)(rel["tag_name"] | ""));

  // Cerca l'asset .bin (nome tipo WeatherStation_v3.0.0-alpha.1.bin)
  String url;
  uint32_t size = 0;
  for (JsonObject a : rel["assets"].as<JsonArray>()) {
    String name = String((const char*)(a["name"] | ""));
    if (name.endsWith(".bin")) {
      url = String((const char*)(a["browser_download_url"] | ""));
      size = a["size"] | 0;
      break;
    }
  }
  if (url.isEmpty()) {
    Serial.printf("[OTA] release %s senza asset .bin\n", latestTag.c_str());
    return;
  }

  Serial.printf("[OTA] release GitHub piu' recente: %s (%u byte)\n",
                latestTag.c_str(), size);

  if (latestTag == downloadedTag && otaReady) {
    Serial.println("[OTA] firmware gia' scaricato per questo tag");
    return;
  }

  if (downloadFirmware(url, size)) {
    downloadedTag = latestTag;
    otaReady = true;
    Serial.printf("[OTA] PRONTO: %u byte, %u chunk, CRC 0x%08lX, ver %s\n",
                  otaSize, otaChunks, (unsigned long)otaCrc,
                  stripV(downloadedTag).c_str());
    Serial.println("[OTA] l'offerta partira' col prossimo 'data' della stazione");
  }
}

// ------------------------------------------------------------------------
// Sessione di invio: serve i chunk richiesti leggendoli dal file
// ------------------------------------------------------------------------
void serveOtaSession() {
  File f = LittleFS.open(FW_PATH, "r");
  if (!f) {
    Serial.println("[OTA] impossibile riaprire " FW_PATH);
    return;
  }

  Serial.printf("[OTA] sessione avviata: servo %u chunk...\n", otaChunks);
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
      uint32_t off = (uint32_t)idx * OTA_CHUNK_DATA;
      if (off >= otaSize) continue;
      uint16_t n = min((uint32_t)OTA_CHUNK_DATA, otaSize - off);

      uint8_t payload[OTA_CHUNK_DATA];
      f.seek(off);
      f.read(payload, n);

      delay(30);  // lascia alla stazione il tempo di commutare TX -> RX
      LoRa.beginPacket();
      LoRa.write(OTA_CHUNK_MAGIC);
      LoRa.write(idx & 0xFF);
      LoRa.write((idx >> 8) & 0xFF);
      LoRa.write(payload, n);
      LoRa.endPacket();

      deadline = millis() + 60000UL;  // sessione viva: rinnova il timeout
      if (idx % 16 == 0) Serial.printf("[OTA] chunk %u/%u inviato\n", idx, otaChunks);

    } else if (strcmp(type, "ota_done") == 0) {
      bool ok = doc["ok"] | false;
      uint32_t crc = doc["crc"] | 0;
      Serial.printf("[OTA] esito stazione: %s (CRC 0x%08lX, atteso 0x%08lX)\n",
                    ok ? "SUCCESS" : "FAILED",
                    (unsigned long)crc, (unsigned long)otaCrc);
      f.close();
      if (ok) {
        // Trasferimento completato: non ri-offrire allo stesso ciclo.
        // (con l'OTA stage 2 la stazione si riavviera' con la nuova
        //  versione e non chiedera' piu' l'aggiornamento)
        forceOffer = false;
        Serial.println("[OTA] trasferimento completato");
      }
      return;
    }
  }
  f.close();
  Serial.println("[OTA] timeout sessione (nessun ota_req / ota_done)");
}

// ------------------------------------------------------------------------
// Gestione di un pacchetto ricevuto dalla stazione
// ------------------------------------------------------------------------
void handlePacket() {
  String rx = "";
  while (LoRa.available()) rx += (char)LoRa.read();
  int rssi = LoRa.packetRssi();

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, rx)) return;  // ignora pacchetti non-JSON
  const char* type = doc["type"] | "";

  if (strcmp(type, "data") != 0) return;  // qui interessa solo la telemetria

  String stationId = String((const char*)(doc["id"] | ""));
  String stationFw = String((const char*)(doc["fw"] | ""));
  Serial.printf("[RX] data da %s fw %s (RSSI %d dBm)\n",
                stationId.c_str(), stationFw.c_str(), rssi);

  // C'e' un aggiornamento per questa stazione?
  bool updateAvailable = otaReady &&
      (forceOffer || stripV(downloadedTag) != stationFw);

  delay(50);  // commutazione TX -> RX della stazione
  StaticJsonDocument<256> ack;
  ack["type"] = "ack";
  ack["id"] = doc["id"];
  ack["seq"] = doc["seq"];
  if (updateAvailable) {
    JsonObject ota = ack.createNestedObject("ota");
    ota["size"] = otaSize;
    ota["crc"] = otaCrc;
    ota["chunks"] = otaChunks;
    ota["ver"] = stripV(downloadedTag);
  }
  String out;
  serializeJson(ack, out);
  LoRa.beginPacket();
  LoRa.print(out);
  LoRa.endPacket();
  Serial.println("[TX] ACK: " + out);

  if (updateAvailable) {
    serveOtaSession();
  } else if (otaReady) {
    Serial.println("[OTA] stazione gia' aggiornata (usa 'f' per forzare)");
  }
}

// ------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== DebugOta - gateway di test OTA via LoRa ===");

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS non montato");
  }

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LoRa] modulo non rilevato, controlla i collegamenti");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TXPWR);
  Serial.println("[LoRa] pronto");

  // WiFi
  Serial.printf("[WiFi] connessione a %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] connesso: %s\n", WiFi.localIP().toString().c_str());
    checkForUpdate();  // primo controllo all'avvio
  } else {
    Serial.println("[WiFi] connessione fallita (riprovo ai controlli periodici)");
  }

  Serial.println("Comandi: 'u' controlla GitHub, 'f' forza offerta, 'i' stato");
  lastCheck = millis();
}

void loop() {
  // Comandi da seriale
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'u' || c == 'U') checkForUpdate();
    else if (c == 'f' || c == 'F') {
      forceOffer = !forceOffer;
      Serial.printf("[OTA] force offer %s\n", forceOffer ? "ON" : "off");
    } else if (c == 'i' || c == 'I') printStatus();
  }

  // Controllo periodico di GitHub
  if (millis() - lastCheck >= CHECK_INTERVAL_MS) {
    lastCheck = millis();
    checkForUpdate();
  }

  // Ricezione LoRa (telemetria dalla stazione)
  int packetSize = LoRa.parsePacket();
  if (packetSize) handlePacket();
}
