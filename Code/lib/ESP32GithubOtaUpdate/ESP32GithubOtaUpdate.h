#pragma once

#include "HttpReleaseUpdate.h"

#ifdef ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG
  #ifdef DEBUG_ESP_PORT
    #define DEBUG_ESP32GOA(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
  #else
    #define DEBUG_ESP32GOA(...) if(Serial) Serial.printf( __VA_ARGS__ )
  #endif
#else
  #define DEBUG_ESP32GOA(x...) if (false) do { (void)0; } while (0)
#endif

typedef enum {
  NO_UPDATE = 0,
  UPDATE_AVAILABLE = 1,
  UPDATE_GOING = 2,
  UPDATE_COMPLETE = 3,
  ERROR = 4
} update_status_t;




extern bool needToStayAlive;
extern bool rqtUpdate;
extern update_status_t avblUpdate;





class ESP32GithubOtaUpdate {
  public:
    void setOTADownloadUrl(const char* otaDownloadUrl);
    void setVersionCheckUrl(const char* versionCheckUrl);
    void setCurrentFirmwareVersion(int currentFirmwareVersion);
    void setUpdateCheckInterval(int updateCheckInterval);
    void begin();    
    void checkForOTA();
    void checkOTAOnce();
    void fetchVersionFile();
    int getCurrentFirmwareVersion();
          
  private:
    void setClock();    
    bool doVersionCheck();
    void doFirmwareUpdate();


/**
 * @brief Gestisce la procedura di riattivazione del dispositivo dopo un'interruzione di corrente.
 * 
 * Questa funzione viene chiamata quando il dispositivo si risveglia a seguito di una perdita di alimentazione.
 * Esegue le seguenti operazioni:
 * - Stampa un messaggio di debug per indicare la causa del risveglio.
 * - Stabilisce la connessione WiFi tramite `setupWiFi()`.
 * - Inizializza l'orologio in tempo reale (RTC) con `setup_rtc_time(&rtc)`.
 * - Recupera e stampa la data/ora attuale formattata in log.
 * 
 * @note Assicura che la variabile globale `rtc` sia inizializzata correttamente prima della chiamata.
 */

    const char * leCaCrt = \
      "-----BEGIN CERTIFICATE-----\n" \
      "MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB\n" \
      "iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl\n" \
      "cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV\n" \
      "BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw\n" \
      "MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV\n" \
      "BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU\n" \
      "aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy\n" \
      "dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK\n" \
      "AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B\n" \
      "3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY\n" \
      "tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/\n" \
      "Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2\n" \
      "VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT\n" \
      "79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6\n" \
      "c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT\n" \
      "Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l\n" \
      "c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee\n" \
      "UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE\n" \
      "Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd\n" \
      "BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G\n" \
      "A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF\n" \
      "Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO\n" \
      "VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3\n" \
      "ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs\n" \
      "8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR\n" \
      "iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze\n" \
      "Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ\n" \
      "XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/\n" \
      "qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB\n" \
      "VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB\n" \
      "L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG\n" \
      "jjxDah2nGN59PRbxYvnKkKj9\n" \
      "-----END CERTIFICATE-----";

  const char* _otaDownloadUrl = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/firmware.bin"; 
  const char* _versionCheckUrl = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/version.txt";
  int _currentFirmwareVersion = 2025060601; // YYYYMMDDRR where R = release of the day
  int _updateCheckInterval = 60; // 60 seconds.   
};



void ESP32GithubOtaUpdate::fetchVersionFile(void){
  WiFiClientSecure client;
  client.setCACert(leCaCrt);  

  if (!client.connect("raw.githubusercontent.com", 443)) {
    Serial.println("Connessione fallita");
    return;
  }

  client.print(String("GET ") +
               "/CarloFalco/WeatherStation/refs/heads/main/Code/version.txt HTTP/1.1\r\n" +
               "Host: raw.githubusercontent.com\r\n" +
               "Connection: close\r\n\r\n");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  // Leggi la risposta
  while (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
  }
}


void ESP32GithubOtaUpdate::setUpdateCheckInterval(int updateCheckInterval) {
    _updateCheckInterval = updateCheckInterval;
    DEBUG_ESP32GOA("[setUpdateCheckInterval()]: updateCheckInterval: %d\r DONE\n", _updateCheckInterval);
}

void ESP32GithubOtaUpdate::setOTADownloadUrl(const char* otaDownloadUrl) {
    _otaDownloadUrl = otaDownloadUrl;
    DEBUG_ESP32GOA("[setOTADownloadUrl()]: otaDownloadUrl: %s\r DONE\n", _otaDownloadUrl);
}

void ESP32GithubOtaUpdate::setVersionCheckUrl(const char* versionCheckUrl) {
    _versionCheckUrl = versionCheckUrl;
    DEBUG_ESP32GOA("[setVersionCheckUrl()]: versionCheckUrl: %s\r DONE\n", _versionCheckUrl);
}

void ESP32GithubOtaUpdate::setCurrentFirmwareVersion(int currentFirmwareVersion) {
    _currentFirmwareVersion = currentFirmwareVersion;
    DEBUG_ESP32GOA("[setCurrentFirmwareVersion()]: currentFirmwareVersion: %d\r DONE\n", _currentFirmwareVersion);
}

int ESP32GithubOtaUpdate::getCurrentFirmwareVersion() {
    return _currentFirmwareVersion;
}


void ESP32GithubOtaUpdate::doFirmwareUpdate() {
  DEBUG_ESP32GOA("[doFirmwareUpdate()]: Downloading new firmware update ..\r\n");

  WiFiClientSecure client;
  client.setCACert(leCaCrt);
  client.setTimeout(12);
  client.setHandshakeTimeout(8);
   
  {  // WiFiClientSecure needs to be destroyed after HttpReleaseUpdate
    HttpReleaseUpdate httpUpdate;
    httpUpdate.rebootOnUpdate(true);

    httpUpdate.onStart([](void) {
      avblUpdate = UPDATE_GOING;
      DEBUG_ESP32GOA("[doFirmwareUpdate()]: Start downloading..\r\n");
    });
    httpUpdate.onEnd([](bool success) {
      DEBUG_ESP32GOA("[doFirmwareUpdate()]: Downloading ended.\r\n");
      avblUpdate = UPDATE_COMPLETE;
      //if (success) SPIFFS.remove("/update");
    });
    httpUpdate.onProgress([](int progress, int total) {
      DEBUG_ESP32GOA("[doFirmwareUpdate()]: Downloading: %d\r\n", (progress / (total / 100)));
    });
    auto ret = httpUpdate.update(client, _otaDownloadUrl);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        DEBUG_ESP32GOA("[doFirmwareUpdate()]: Http Update Failed (Error=%d): %s\r\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        DEBUG_ESP32GOA("[doFirmwareUpdate()]: No Update!\r\n");
        break;
      case HTTP_UPDATE_OK:
        DEBUG_ESP32GOA("[doFirmwareUpdate()]: Update OK!\r\n");
        break;
    }
  }
}

bool ESP32GithubOtaUpdate::doVersionCheck() { 
  DEBUG_ESP32GOA("[doVersionCheck()]: Checking for new firmware update ..\r\n");

  Serial.println("UpdateCheck-> checking");
  bool isUpdateAvailable = false;
  WiFiClientSecure *client = new WiFiClientSecure;

  if(client) {
    client->setCACert(leCaCrt);
    client->setTimeout(12);
    {
       // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
      HTTPClient https;
      if (https.begin(*client, _versionCheckUrl)) {
        int httpCode = https.GET();  

        if (httpCode > 0) {
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String server_fw_version = https.getString();     
            int new_fw_version_int = server_fw_version.toInt();            
            DEBUG_ESP32GOA("[doVersionCheck()]: Current FW version: %d. Server FW version: %d \r\n", _currentFirmwareVersion, new_fw_version_int);
            
            if (new_fw_version_int <= _currentFirmwareVersion) {
              DEBUG_ESP32GOA("[doVersionCheck()]: Device is already on latest firmware version: %d\r\n", _currentFirmwareVersion);                            
            } else {
              DEBUG_ESP32GOA("[doVersionCheck()]: New firmware version available version!\r\n");
              isUpdateAvailable = true;
            }
          }
        } else {
          DEBUG_ESP32GOA("[doVersionCheck()]: GET... failed, error: %s\r\n", https.errorToString(httpCode));
        }  
        https.end();
      } else {
        DEBUG_ESP32GOA("[doVersionCheck()]: Unable to connect\r\n");
      }      
    } // End extra scoping block  
    delete client;
  } else {
    DEBUG_ESP32GOA("[doVersionCheck()]: Unable to create WiFiClientSecure client!!\r\n");
  } 

  return isUpdateAvailable;
}

void ESP32GithubOtaUpdate::checkOTAOnce(){
  Serial.println("checkForOTAONCE");
  if(doVersionCheck()) { // aggiornamento disponibile 
    avblUpdate = UPDATE_AVAILABLE;
    DEBUG_ESP32GOA("[checkOTAOnce]: UpdateAvailable %d\n", avblUpdate);
    DEBUG_ESP32GOA("[checkOTAOnce]: UserUpdateRequest %d\n", rqtUpdate);

    if (rqtUpdate){
      DEBUG_ESP32GOA("[checkOTAOnce]: UserRequestUpdate\n");
      doFirmwareUpdate();
    }
  }

}

void ESP32GithubOtaUpdate::setClock() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC
  DEBUG_ESP32GOA("Waiting for NTP time sync: ");

  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    DEBUG_ESP32GOA(".");
    now = time(nullptr);
  }
  DEBUG_ESP32GOA("\r\n");

  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  DEBUG_ESP32GOA("Current time: %s\r\n", asctime(&timeinfo));
}

void ESP32GithubOtaUpdate::checkForOTA(){
    for (;;){
      
      Serial.println("checkForOTA");
      if(doVersionCheck()) { // aggiornamento disponibile 
        avblUpdate = UPDATE_AVAILABLE;
        Serial.println("UpdateAvailable");
        if (rqtUpdate){
          Serial.println("UserRequestUpdate");
          // doFirmwareUpdate();
        }
      }
      vTaskDelay((_updateCheckInterval * 1000) / portTICK_PERIOD_MS);
    }
}

void ESP32GithubOtaUpdate::begin() {
  // setClock();

  xTaskCreate([](void* o){ static_cast<ESP32GithubOtaUpdate*>(o)->checkForOTA(); },
          "ESP32OTAUpdateTask",    // Name of the task (for debugging)
          1024 * 8,                // Stack size (bytes)
          this,                   // Parameter to pass
          5,                      // Task priority
          NULL);                    // Task handle
}
