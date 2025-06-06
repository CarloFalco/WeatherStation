#ifndef ___MQTT_H___
#define ___MQTT_H___

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
//#include "ESP32GithubOtaUpdate.h"


extern WiFiClientSecure espClient;
extern PubSubClient mqtt_client;
extern bool rqtUpdate;
extern bool rqtReset;


void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqtt_init();

#endif