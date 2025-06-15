#include "ESP32Mqtt.h"
#include "../../src/secret.h"


/**
 * @brief Gestisce i messaggi ricevuti sul topic MQTT "Paperino".
 * 
 * Estrae e interpreta i campi "msg", "LedSts" e "prova" da un documento JSON.
 * - "msg" è un messaggio testuale.
 * - "LedSts" è una stringa numerica (es. "1" o "0") che rappresenta lo stato del LED.
 * - "prova" è una stringa di test o debug.
 * 
 * @param doc Oggetto JSON statico ricevuto, con dimensione massima di 200 byte.
 * 
 * @note Assicurarsi che i campi esistano nel JSON per evitare valori nulli.
 * 
 * @example
 * // Esempio di messaggio MQTT:
 * // Topic: "Paperino"
 * // Payload JSON:
 * // {
 * //   "msg": "accendi",
 * //   "LedSts": "1",
 * //   "prova": "1"
 * // }
 */

void handlePaperinoTopic(const StaticJsonDocument<200>& doc) {
  const char* msg = doc["msg"];
  String ledStatusString = doc["LedSts"];
  String provaString = doc["prova"];

  int ledStatus = ledStatusString.toInt();
  log_i("Message: %s | Led Status: %d | Prova: %s", msg, ledStatus, provaString.c_str());

  // Qui potresti aggiungere azioni in base al valore di msg o ledStatus
}

// Topic: reset_rqt
// msg: {"newMsg": "20", "value": 1}
void handleRstRqtTopic(const StaticJsonDocument<200>& doc) {
 
  int rstValue = doc["rstValue"];
  rqtReset = false;

  if (rstValue == 1){ 
    rqtReset = true;
  }
  log_i("Message Value: %d | Update Request: %d", rstValue, rqtReset);

}

// Topic: upd_rqt
// msg: {"updValue": "1"}
void handleUpdRqtTopic(const StaticJsonDocument<200>& doc) {

  int updValue = doc["updValue"];
  rqtUpdate = false;

  if (updValue == 1){ 
    rqtUpdate = true;
  }
  log_i("Message Value: %d | Update Request: %d", updValue, rqtUpdate);

}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  // log_i("[mqttCallback]: Topic: %s ", topic);
  // log_i("Payload: %s", messagePrint.c_str());
  // Serial.print("Payload: ");
  // Serial.println(message);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    log_e("[mqttCallback]: Errore nel parsing del JSON: %s", error.c_str());
    // Serial.print("Errore nel parsing del JSON: ");
    // Serial.println(error.c_str());
    return;
  }


  if (strcmp(topic, "paperino") == 0) {
    handlePaperinoTopic(doc);
  } else if (strcmp(topic, "reset_rqt") == 0) {
    handleRstRqtTopic(doc);
  } else if (strcmp(topic, "upd_rqt") == 0) {
    handleUpdRqtTopic(doc);
  }
}


void mqtt_init() {

  mqtt_client.setCallback(mqttCallback);
  espClient.setCACert(leCaCrtMqtt);
  mqtt_client.setServer(mqtt_server, 8883);

  if (mqtt_client.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
    log_d("MQTT connected");
    if (mqtt_client.subscribe("paperino"))  { log_d("Sottoscritto al topic 'paperino'");} 
    if (mqtt_client.subscribe("reset_rqt"))     { log_d("Sottoscritto al topic 'reset_rqt'");} 
    if (mqtt_client.subscribe("upd_rqt"))   { log_d("Sottoscritto al topic 'upd_rqt'");} 

  } else {
    log_e("MQTT connection failed, rc=%d", mqtt_client.state());
  }

}

void publishJsonMessage(const char* topic, const JsonDocument& doc) {
    char buffer[MAX_JSON_SIZE];
    size_t len = serializeJson(doc, buffer, MAX_JSON_SIZE);

    if (len > MQTT_MAX_PACKET_SIZE) {
        log_e("[ERRORE] Payload JSON (%d byte) supera limite MQTT (%d byte)", len, MQTT_MAX_PACKET_SIZE);
        return;
    }

    if (!mqtt_client.publish(topic, buffer, true)) {
        log_e("[ERRORE] Fallita pubblicazione su topic %s", topic);
    } else {
        log_i("Pubblicato su %s: %s", topic, buffer);
    }
}





