#ifndef ___MQTT_H___
#define ___MQTT_H___

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include "ESP32GithubOtaUpdate.h"
#include "secret.h"

extern WiFiClientSecure espClient;
extern PubSubClient mqtt_client; 
// extern ESP32GithubOtaUpdate otaUpdate;

// Funzione di callback per la ricezione dei messaggi MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Messaggio ricevuto su topic: ");
  Serial.println(topic);

  // Convertire il payload in una stringa JSON
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0'; // Aggiungi il terminatore di stringa

  Serial.print("Payload: ");
  Serial.println(message);

  // Parsing del JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("Errore nel parsing del JSON: ");
    Serial.println(error.c_str());
    return;
  }

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

  // Controllo del LED in base al messaggio ricevuto
  if (ledStatus == 1) {
    digitalWrite(LED_BUILTIN, HIGH); // Accendi LED
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // Spegni LED
  }
}


void mqtt_init() {

    // Imposta la funzione di callback per i messaggi ricevuti
  mqtt_client.setCallback(mqttCallback);

  Serial.println(leCaCrtMqtt);
  espClient.setCACert(leCaCrtMqtt);
  mqtt_client.setServer(mqtt_server,8883);

  if (mqtt_client.connect(mqtt_client_id,mqtt_user,mqtt_pass)){
    Serial.println("MQTT connected");
    
    if (mqtt_client.subscribe("paperino")) {
      Serial.println("Sottoscritto al topic 'paperino'");
    } else {
      Serial.println("Errore nella sottoscrizione al topic 'paperino'");
    }

  }else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqtt_client.state());
  }


}



#endif