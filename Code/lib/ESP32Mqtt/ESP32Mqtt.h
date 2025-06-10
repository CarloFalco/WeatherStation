#ifndef ___MQTT_H___
#define ___MQTT_H___

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

//#include "ESP32GithubOtaUpdate.h"

// JSON
#define MAX_JSON_SIZE 512 // 512B
// attenzione: il buffer JSON deve essere modificato in base al valore di MQTT_MAX_PACKET_SIZE
// #define MQTT_MAX_PACKET_SIZE 1024 nella libreria PubSubClient.h va modificato in base alla grandezza del buffer JSON
// #define MQTT_MAX_PACKET_SIZE 1024

extern WiFiClientSecure espClient;
extern PubSubClient mqtt_client;
extern bool rqtUpdate;
extern bool rqtReset;

void handlePaperinoTopic(const StaticJsonDocument<200>& doc);
void handleRstRqtTopic(const StaticJsonDocument<200>& doc);
void handleUpdRqtTopic(const StaticJsonDocument<200>& doc);

void publishJsonMessage(const char* topic, const JsonDocument& doc);
void mqttCallback(char* topic, byte* payload, unsigned int length);

void mqtt_init();




#endif