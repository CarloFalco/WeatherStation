/**
 * @file AppConfig.cpp
 * @brief Implementation of the AppConfig singleton.
 */

#include "AppConfig.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

AppConfig& AppConfig::getInstance() {
    static AppConfig instance;
    return instance;
}

// ---------------------------------------------------------------------------
// load()
// ---------------------------------------------------------------------------

bool AppConfig::load(const char* path) {
    File f = LittleFS.open(path, "r");
    if (!f) {
        Serial.printf("[AppConfig] Cannot open '%s'\n", path);
        return false;
    }

    String currentSection;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        trim(line);

        // Skip empty lines and comments
        if (line.isEmpty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // Section header  [section_name]
        if (line[0] == '[') {
            int close = line.indexOf(']');
            if (close > 1) {
                currentSection = line.substring(1, close);
                currentSection.toLowerCase();
            }
            continue;
        }

        // Key = value pair
        int sep = line.indexOf('=');
        if (sep <= 0) continue;

        String key   = line.substring(0, sep);
        String value = line.substring(sep + 1);
        trim(key);
        trim(value);
        key.toLowerCase();

        applyKeyValue(currentSection, key, value);
    }

    f.close();
    Serial.printf("[AppConfig] Loaded '%s'\n", path);
    return isValid();
}

// ---------------------------------------------------------------------------
// save()
// ---------------------------------------------------------------------------

bool AppConfig::save(const char* path) {
    File f = LittleFS.open(path, "w");
    if (!f) {
        Serial.printf("[AppConfig] Cannot write '%s'\n", path);
        return false;
    }

    f.println("; ESP32-S3 Weather Station – runtime configuration");
    f.println("; Generated automatically by the provisioning wizard.");
    f.println("; Lines starting with ; or # are comments.");
    f.println();

    f.println("[wifi]");
    f.printf("ssid     = %s\n", wifiSsid.c_str());
    f.printf("password = %s\n", wifiPassword.c_str());
    f.println();

    f.println("[mqtt]");
    f.printf("host      = %s\n", mqttHost.c_str());
    f.printf("port      = %u\n", mqttPort);
    f.printf("user      = %s\n", mqttUser.c_str());
    f.printf("password  = %s\n", mqttPassword.c_str());
    f.printf("client_id = %s\n", mqttClientId.c_str());
    f.printf("base_topic = %s\n", mqttBaseTopic.c_str());
    f.println();

    f.println("[ota]");
    f.printf("github_repo  = %s\n", otaGithubRepo.c_str());
    f.printf("github_token = %s\n", otaGithubToken.c_str());
    f.println();

    f.println("[sampling]");
    f.printf("bme280_s  = %lu\n", (unsigned long)intervalBme280S);
    f.printf("rain_s    = %lu\n", (unsigned long)intervalRainS);
    f.printf("wind_s    = %lu\n", (unsigned long)intervalWindS);
    f.printf("pm25_s    = %lu\n", (unsigned long)intervalPm25S);
    f.printf("ccs811_s  = %lu\n", (unsigned long)intervalCcs811S);
    f.printf("mics_s    = %lu\n", (unsigned long)intervalMics6814S);
    f.printf("ina_s     = %lu\n", (unsigned long)intervalIna3221S);
    f.printf("publish_s = %lu\n", (unsigned long)mqttPublishIntervalS);
    f.printf("sleep_s   = %lu\n", (unsigned long)deepSleepDurationS);
    f.println();

    f.close();
    Serial.printf("[AppConfig] Saved '%s'\n", path);
    return true;
}

// ---------------------------------------------------------------------------
// isValid()
// ---------------------------------------------------------------------------

bool AppConfig::isValid() const {
    return !wifiSsid.isEmpty() && !mqttHost.isEmpty();
}

// ---------------------------------------------------------------------------
// dump()
// ---------------------------------------------------------------------------

void AppConfig::dump() const {
    Serial.println("--- AppConfig dump ---");
    Serial.printf("  wifi.ssid         = %s\n",  wifiSsid.c_str());
    Serial.printf("  wifi.password     = %s\n",  wifiPassword.isEmpty() ? "(empty)" : "***");
    Serial.printf("  mqtt.host         = %s\n",  mqttHost.c_str());
    Serial.printf("  mqtt.port         = %u\n",  mqttPort);
    Serial.printf("  mqtt.user         = %s\n",  mqttUser.c_str());
    Serial.printf("  mqtt.password     = %s\n",  mqttPassword.isEmpty() ? "(empty)" : "***");
    Serial.printf("  mqtt.client_id    = %s\n",  mqttClientId.c_str());
    Serial.printf("  mqtt.base_topic   = %s\n",  mqttBaseTopic.c_str());
    Serial.printf("  ota.github_repo   = %s\n",  otaGithubRepo.c_str());
    Serial.printf("  ota.github_token  = %s\n",  otaGithubToken.isEmpty() ? "(empty)" : "***");
    Serial.printf("  sampling.bme280_s = %lu\n", (unsigned long)intervalBme280S);
    Serial.printf("  sampling.rain_s   = %lu\n", (unsigned long)intervalRainS);
    Serial.printf("  sampling.wind_s   = %lu\n", (unsigned long)intervalWindS);
    Serial.printf("  sampling.pm25_s   = %lu\n", (unsigned long)intervalPm25S);
    Serial.printf("  sampling.ccs811_s = %lu\n", (unsigned long)intervalCcs811S);
    Serial.printf("  sampling.mics_s   = %lu\n", (unsigned long)intervalMics6814S);
    Serial.printf("  sampling.ina_s    = %lu\n", (unsigned long)intervalIna3221S);
    Serial.printf("  sampling.publish_s= %lu\n", (unsigned long)mqttPublishIntervalS);
    Serial.printf("  sampling.sleep_s  = %lu\n", (unsigned long)deepSleepDurationS);
    Serial.println("---------------------");
}

// ---------------------------------------------------------------------------
// applyKeyValue()  (private)
// ---------------------------------------------------------------------------

void AppConfig::applyKeyValue(const String& section,
                              const String& key,
                              const String& value) {
    if (section == "wifi") {
        if (key == "ssid")     wifiSsid     = value;
        if (key == "password") wifiPassword = value;

    } else if (section == "mqtt") {
        if (key == "host")       mqttHost       = value;
        if (key == "port")       mqttPort       = (uint16_t)value.toInt();
        if (key == "user")       mqttUser       = value;
        if (key == "password")   mqttPassword   = value;
        if (key == "client_id")  mqttClientId   = value;
        if (key == "base_topic") mqttBaseTopic  = value;

    } else if (section == "ota") {
        if (key == "github_repo")  otaGithubRepo  = value;
        if (key == "github_token") otaGithubToken = value;

    } else if (section == "sampling") {
        if (key == "bme280_s")  intervalBme280S      = (uint32_t)value.toInt();
        if (key == "rain_s")    intervalRainS        = (uint32_t)value.toInt();
        if (key == "wind_s")    intervalWindS        = (uint32_t)value.toInt();
        if (key == "pm25_s")    intervalPm25S        = (uint32_t)value.toInt();
        if (key == "ccs811_s")  intervalCcs811S      = (uint32_t)value.toInt();
        if (key == "mics_s")    intervalMics6814S    = (uint32_t)value.toInt();
        if (key == "ina_s")     intervalIna3221S     = (uint32_t)value.toInt();
        if (key == "publish_s") mqttPublishIntervalS = (uint32_t)value.toInt();
        if (key == "sleep_s")   deepSleepDurationS   = (uint32_t)value.toInt();
    }
}

// ---------------------------------------------------------------------------
// trim()  (private, static)
// ---------------------------------------------------------------------------

void AppConfig::trim(String& s) {
    // Leading whitespace
    while (s.length() > 0 && (s[0] == ' ' || s[0] == '\t' || s[0] == '\r')) {
        s.remove(0, 1);
    }
    // Trailing whitespace
    while (s.length() > 0) {
        char last = s[s.length() - 1];
        if (last == ' ' || last == '\t' || last == '\r' || last == '\n') {
            s.remove(s.length() - 1);
        } else {
            break;
        }
    }
}
