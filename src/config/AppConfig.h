#pragma once

/**
 * @file AppConfig.h
 * @brief Runtime configuration loaded from an INI file stored in LittleFS.
 *
 * AppConfig is a singleton that reads a simple `key = value` INI file
 * (sections delimited by `[section]` headers) and exposes every
 * user-configurable parameter to the rest of the firmware.
 *
 * The configuration file is located at `/config.ini` on the LittleFS
 * partition and is written by the provisioning captive-portal during the
 * first-time setup (or after a factory reset).
 *
 * @par Supported sections
 * | Section     | Keys                                                          |
 * |-------------|---------------------------------------------------------------|
 * | [wifi]      | ssid, password                                                |
 * | [mqtt]      | host, port, user, password, client_id, base_topic             |
 * | [ota]       | github_repo, github_token                                     |
 * | [sampling]  | bme280_s, rain_s, wind_s, pm25_s, ccs811_s, mics_s, ina_s,   |
 * |             | publish_s, sleep_s                                            |
 *
 * @par Usage
 * @code
 * AppConfig& cfg = AppConfig::getInstance();
 * if (!cfg.load("/config.ini")) {
 *     // handle error – enter provisioning
 * }
 * Serial.println(cfg.mqttHost);
 * @endcode
 */

#include <Arduino.h>
#include <LittleFS.h>

class AppConfig {
public:
    // -------------------------------------------------------------------------
    // WiFi
    // -------------------------------------------------------------------------
    String wifiSsid;       ///< WiFi network SSID.
    String wifiPassword;   ///< WiFi network password (empty = open network).

    // -------------------------------------------------------------------------
    // MQTT
    // -------------------------------------------------------------------------
    String   mqttHost;                            ///< Broker hostname or IP.
    uint16_t mqttPort      = 1883;                ///< Broker port.
    String   mqttUser;                            ///< Broker username (may be empty).
    String   mqttPassword;                        ///< Broker password (may be empty).
    String   mqttClientId  = "weather-station-01";///< Unique MQTT client ID.
    String   mqttBaseTopic = "weather/station01"; ///< Root topic prefix.

    // -------------------------------------------------------------------------
    // OTA / GitHub
    // -------------------------------------------------------------------------
    String otaGithubRepo;  ///< GitHub repository, e.g. "user/esp32-weather-station".
    String otaGithubToken; ///< Personal Access Token (empty for public repos).

    // -------------------------------------------------------------------------
    // Sampling intervals  (all values in seconds)
    // -------------------------------------------------------------------------
    uint32_t intervalBme280S    = 60;  ///< BME280 read interval.
    uint32_t intervalRainS      = 60;  ///< Rain-gauge accumulation window.
    uint32_t intervalWindS      = 10;  ///< Anemometer / direction sample interval.
    uint32_t intervalPm25S      = 120; ///< PM2.5 read interval.
    uint32_t intervalCcs811S    = 120; ///< CCS811 read interval.
    uint32_t intervalMics6814S  = 120; ///< MICS6814 read interval.
    uint32_t intervalIna3221S   = 60;  ///< INA3221 read interval.
    uint32_t mqttPublishIntervalS = 300; ///< Aggregated publish interval.
    uint32_t deepSleepDurationS   = 300; ///< Deep-sleep duration between wake cycles.

    // =========================================================================
    // Singleton access
    // =========================================================================

    /**
     * @brief Returns the singleton AppConfig instance.
     * @return Reference to the global AppConfig object.
     */
    static AppConfig& getInstance();

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * @brief Loads configuration from an INI file on LittleFS.
     *
     * Parses every `key = value` line and populates the public member fields.
     * Lines starting with `;` or `#` are treated as comments and ignored.
     *
     * @param path  LittleFS absolute path (e.g. "/config.ini").
     * @return      `true` if the file was opened and wifi.ssid is non-empty.
     */
    bool load(const char* path);

    /**
     * @brief Persists the current in-memory configuration to LittleFS.
     *
     * Overwrites the file at `path` with the serialised INI content.
     *
     * @param path  LittleFS absolute path (e.g. "/config.ini").
     * @return      `true` on success.
     */
    bool save(const char* path);

    /**
     * @brief Returns `true` when the minimum required fields are populated.
     *
     * Minimum requirement: `wifiSsid` and `mqttHost` must be non-empty.
     */
    bool isValid() const;

    /**
     * @brief Prints all configuration values to Serial (DEBUG only).
     *
     * MQTT/WiFi passwords are masked with `***`.
     */
    void dump() const;

private:
    AppConfig() = default;
    AppConfig(const AppConfig&) = delete;
    AppConfig& operator=(const AppConfig&) = delete;

    /**
     * @brief Applies a single parsed key/value pair from a given INI section.
     * @param section   Current INI section name (without brackets).
     * @param key       Key string (trimmed, lower-case).
     * @param value     Value string (trimmed).
     */
    void applyKeyValue(const String& section,
                       const String& key,
                       const String& value);

    /**
     * @brief Trims leading and trailing whitespace from a String in place.
     * @param s  String to trim.
     */
    static void trim(String& s);
};
