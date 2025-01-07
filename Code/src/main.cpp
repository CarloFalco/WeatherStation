#define ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG Uncomment to enable logs.

#include <Arduino.h>
#include <WiFiClientSecure.h>

#include "HttpReleaseUpdate.h"
#include "ESP32GithubOtaUpdate.h"
#include "secret.h"

#define S_TO_uS_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define S_TO_mS_FACTOR 1000ULL  /* Conversion factor for milli seconds to seconds */
#define mS_TO_uS_FACTOR 1000ULL  /* Conversion factor for milli seconds to micro seconds */
#define LED_BUILTIN 97


const char* OTA_FILE_LOCATION = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/firmware.bin";
const char* VERSION_URL = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/version.txt";

const int current_fw_version = 2025010101;  // YYYYMMDDRR where R = release of the day



ESP32GithubOtaUpdate otaUpdate;
bool needToStayAlive = 0;
bool rqtUpdate = false;
int avblUpdate = 0;




bool ledState = false; // Variabile per lo stato del LED

void setupWiFi() {
  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
    Serial.print("ESP8266");
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
    Serial.print("ESP32");
  #endif

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

void setupOtaUpdate() {
  otaUpdate.setOTADownloadUrl(OTA_FILE_LOCATION);
  otaUpdate.setVersionCheckUrl(VERSION_URL);
  otaUpdate.setCurrentFirmwareVersion(current_fw_version);
  otaUpdate.setUpdateCheckInterval(60); // Check every 60 seconds.
  otaUpdate.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.print("WelcomeToNewBuild");
  setupWiFi();   
  pinMode(LED_BUILTIN, OUTPUT);
  setupOtaUpdate();
}



void loop(){

  ledState = !ledState; // Inverti lo stato
  digitalWrite(LED_BUILTIN, ledState); // Imposta il nuovo stato
  Serial.println("needToStayAlive ST: ");
  delay(1000);
}
