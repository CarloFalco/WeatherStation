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
extern bool needToStayAlive;
extern bool rqtUpdate;
extern int avblUpdate;

class ESP32GithubOtaUpdate {
  public:
    void setOTADownloadUrl(const char* otaDownloadUrl);
    void setVersionCheckUrl(const char* versionCheckUrl);
    void setCurrentFirmwareVersion(int currentFirmwareVersion);
    void setUpdateCheckInterval(int updateCheckInterval);
    void begin();    
    void checkForOTA();
          
  private:
    void setClock();    
    bool doVersionCheck();
    void doFirmwareUpdate();

    const char * leCaCrt = \
      "-----BEGIN CERTIFICATE-----\n" \
      "MIIEyDCCA7CgAwIBAgIQDPW9BitWAvR6uFAsI8zwZjANBgkqhkiG9w0BAQsFADBh\n" \
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
      "MjAeFw0yMTAzMzAwMDAwMDBaFw0zMTAzMjkyMzU5NTlaMFkxCzAJBgNVBAYTAlVT\n" \
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxMzAxBgNVBAMTKkRpZ2lDZXJ0IEdsb2Jh\n" \
      "bCBHMiBUTFMgUlNBIFNIQTI1NiAyMDIwIENBMTCCASIwDQYJKoZIhvcNAQEBBQAD\n" \
      "ggEPADCCAQoCggEBAMz3EGJPprtjb+2QUlbFbSd7ehJWivH0+dbn4Y+9lavyYEEV\n" \
      "cNsSAPonCrVXOFt9slGTcZUOakGUWzUb+nv6u8W+JDD+Vu/E832X4xT1FE3LpxDy\n" \
      "FuqrIvAxIhFhaZAmunjZlx/jfWardUSVc8is/+9dCopZQ+GssjoP80j812s3wWPc\n" \
      "3kbW20X+fSP9kOhRBx5Ro1/tSUZUfyyIxfQTnJcVPAPooTncaQwywa8WV0yUR0J8\n" \
      "osicfebUTVSvQpmowQTCd5zWSOTOEeAqgJnwQ3DPP3Zr0UxJqyRewg2C/Uaoq2yT\n" \
      "zGJSQnWS+Jr6Xl6ysGHlHx+5fwmY6D36g39HaaECAwEAAaOCAYIwggF+MBIGA1Ud\n" \
      "EwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFHSFgMBmx9833s+9KTeqAx2+7c0XMB8G\n" \
      "A1UdIwQYMBaAFE4iVCAYlebjbuYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAd\n" \
      "BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQG\n" \
      "CCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKG\n" \
      "NGh0dHA6Ly9jYWNlcnRzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RH\n" \
      "Mi5jcnQwQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29t\n" \
      "L0RpZ2lDZXJ0R2xvYmFsUm9vdEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwC\n" \
      "ATAHBgVngQwBATAIBgZngQwBAgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG\n" \
      "9w0BAQsFAAOCAQEAkPFwyyiXaZd8dP3A+iZ7U6utzWX9upwGnIrXWkOH7U1MVl+t\n" \
      "wcW1BSAuWdH/SvWgKtiwla3JLko716f2b4gp/DA/JIS7w7d7kwcsr4drdjPtAFVS\n" \
      "slme5LnQ89/nD/7d+MS5EHKBCQRfz5eeLjJ1js+aWNJXMX43AYGyZm0pGrFmCW3R\n" \
      "bpD0ufovARTFXFZkAdl9h6g4U5+LXUZtXMYnhIHUfoyMo5tS58aI7Dd8KvvwVVo4\n" \
      "chDYABPPTHPbqjc1qCmBaZx2vN4Ye5DUys/vZwP9BFohFrH/6j/f3IL16/RZkiMN\n" \
      "JCqVJUzKoZHm1Lesh3Sz8W2jmdv51b2EQJ8HmA==\n" \
      "-----END CERTIFICATE-----";


  const char* _otaDownloadUrl; 
  const char* _versionCheckUrl;
  int _currentFirmwareVersion;
  int _updateCheckInterval = 60; // 60 seconds.   
};




void ESP32GithubOtaUpdate::setUpdateCheckInterval(int updateCheckInterval) {
    _updateCheckInterval = updateCheckInterval;
    Serial.print("updateCheckInterval: ");
    Serial.println("Done");
}

void ESP32GithubOtaUpdate::setOTADownloadUrl(const char* otaDownloadUrl) {
    _otaDownloadUrl = otaDownloadUrl;
    Serial.print("otaDownloadUrl: ");
    Serial.println("Done");
}

void ESP32GithubOtaUpdate::setVersionCheckUrl(const char* versionCheckUrl) {
    _versionCheckUrl = versionCheckUrl;
    Serial.print("versionCheckUrl: ");
    Serial.print(versionCheckUrl);
    Serial.println("\t Done");
}

void ESP32GithubOtaUpdate::setCurrentFirmwareVersion(int currentFirmwareVersion) {
    _currentFirmwareVersion = currentFirmwareVersion;
    Serial.print("currentFirmwareVersion: ");
    Serial.println("Done");
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
      needToStayAlive = 1;
      avblUpdate = 2;
      DEBUG_ESP32GOA("[doFirmwareUpdate()]: Start downloading..\r\n");
    });
    httpUpdate.onEnd([](bool success) {
      DEBUG_ESP32GOA("[doFirmwareUpdate()]: Downloading ended.\r\n");
      needToStayAlive = 0;
      avblUpdate = 3;
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
            
            if (_currentFirmwareVersion == new_fw_version_int) {
              DEBUG_ESP32GOA("[doVersionCheck()]: Device is already on latest firmware version: %d\r\n", _currentFirmwareVersion);                            
            } else {
              DEBUG_ESP32GOA("[doVersionCheck()]: New firmware version available version!\r\n");
              isUpdateAvailable = true;
            }
          }
        } else {
          DEBUG_ESP32GOA("[doVersionCheck()]: GET... failed, error: %s\r\n", https.errorToString(httpCode).c_str());
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

void ESP32GithubOtaUpdate::checkForOTA()
{
    for (;;)
    {
      if(doVersionCheck()) { // aggiornamento disponibile 
        avblUpdate = 1;
        Serial.println("UpdateAvailable");
        if (rqtUpdate){
          Serial.println("UserRequestUpdate");
          doFirmwareUpdate();
        }
      }
      vTaskDelay((_updateCheckInterval * 1000) / portTICK_PERIOD_MS);
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

void ESP32GithubOtaUpdate::begin() {
  // setClock();

  xTaskCreate([](void* o){ static_cast<ESP32GithubOtaUpdate*>(o)->checkForOTA(); },
          "ESP32OTAUpdateTask",    // Name of the task (for debugging)
          1024 * 8,                // Stack size (bytes)
          this,                   // Parameter to pass
          5,                      // Task priority
          NULL);                    // Task handle
}
