#include "ESP32Mqtt.h"
#include "../../src/secret.h"

void handlePaperinoTopic(const StaticJsonDocument<200>& doc) {
  const char* msg = doc["msg"];
  String ledStatusString = doc["LedSts"];
  String provaString = doc["prova"];

  int ledStatus = ledStatusString.toInt();
  Serial.print("Messaggio: ");
  Serial.println(msg);

  Serial.print("Stato LED: ");
  Serial.println(ledStatus);

  Serial.print("String: ");
  Serial.print(provaString);
  Serial.print("\t ");
  Serial.println(provaString.toInt());

  if (ledStatus == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void handlePlutoTopic(const StaticJsonDocument<200>& doc) {
  const char* newMsg = doc["newMsg"];
  int newValue = doc["newValue"];

  Serial.print("Nuovo Messaggio: ");
  Serial.println(newMsg);

  Serial.print("Nuovo Valore: ");
  Serial.println(newValue);
}

void handlePippoTopic(const StaticJsonDocument<200>& doc) {
  const char* newMsg = doc["newMsg"];
  int newValue = doc["newValue"];

  Serial.print("Nuovo Messaggio: ");
  Serial.println(newMsg);

  Serial.print("Nuovo Valore: ");
  Serial.println(newValue);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Messaggio ricevuto su topic: ");
  Serial.println(topic);

  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.print("Payload: ");
  Serial.println(message);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("Errore nel parsing del JSON: ");
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, "paperino") == 0) {
    handlePaperinoTopic(doc);
  } else if (strcmp(topic, "pluto") == 0) {
    handlePlutoTopic(doc);
  } else if (strcmp(topic, "pippo") == 0) {
    handlePippoTopic(doc);
  }
}

void mqtt_init() {
  mqtt_client.setCallback(mqttCallback);

  Serial.println(leCaCrtMqtt);
  espClient.setCACert(leCaCrtMqtt);
  mqtt_client.setServer(mqtt_server, 8883);

  if (mqtt_client.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
    Serial.println("MQTT connected");

    if (mqtt_client.subscribe("paperino")) {
      Serial.println("Sottoscritto al topic 'paperino'");
    } else {
      Serial.println("Errore nella sottoscrizione al topic 'paperino'");
    }

    if (mqtt_client.subscribe("pluto")) {
      Serial.println("Sottoscritto al topic 'pluto'");
    } else {
      Serial.println("Errore nella sottoscrizione al topic 'pluto'");
    }
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqtt_client.state());
  }
}