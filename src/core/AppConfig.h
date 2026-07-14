/**
 * @file AppConfig.h
 * @brief Runtime configuration stored as an INI file on LittleFS.
 *
 * Hardware constants (pins, I2C addresses) live in include/config.h and are
 * fixed at build time; everything the user may want to tune in the field
 * (station identity, wake interval, LoRa radio parameters) lives here and
 * is loaded from /config.ini at every boot.
 *
 * If the file is missing (first boot, erased flash) the defaults defined
 * in this class are used and a fresh /config.ini is written back, so the
 * filesystem image upload (`pio run -t uploadfs`) is optional.
 *
 * Usage example:
 * @code
 * AppConfig config;
 * config.begin();                       // mount FS + load (or create) file
 * uint64_t us = config.station.wakeIntervalS * 1000000ULL;
 * @endcode
 */

#ifndef WEATHERSTATION_APPCONFIG_H
#define WEATHERSTATION_APPCONFIG_H

#include <Arduino.h>

/**
 * @brief Loads, stores and persists the station runtime configuration.
 *
 * Minimal hand-rolled INI parser (sections, key=value, ';'/'#' comments):
 * no external library needed for a file this small.
 */
class AppConfig {
public:
    /** @brief `[station]` section: identity and duty cycle. */
    struct StationConfig {
        String   id             = "ws-01";  ///< Station identifier sent in every LoRa message.
        uint32_t wakeIntervalS  = 600;      ///< Seconds of deep sleep between measurement cycles.
    };

    /** @brief `[rain]` section: rain-gauge calibration. */
    struct RainConfig {
        float mmPerPulse = 0.4743f;  ///< Rainfall per bucket tip [mm]: bucket
                                     ///< volume / collector area = 3 cm3 / 63.25 cm2.
    };

    /** @brief `[wind]` section: anemometer calibration and vane mounting offset. */
    struct WindConfig {
        float   mpsPerHz      = 0.667f;  ///< Wind speed per pulse frequency [m/s per Hz]
                                         ///< (cup anemometer convention "1 Hz = 2.4 km/h").
        uint8_t sampleWindowS = 5;       ///< Wind sampling window per cycle [s].
        int16_t vaneOffsetDeg = 0;       ///< Added to the AS5600 raw angle to align 0 to North.
    };

    /** @brief `[power]` section: INA3221 energy monitor. */
    struct PowerConfig {
        uint32_t shuntMohm = 100;  ///< Shunt resistance [milliohm], all channels
                                   ///< (hardware-verified against a reference
                                   ///< current during Increment 6 validation).
    };

    /** @brief `[lora]` section: SX1276 radio parameters (see docs/lora-protocol.md). */
    struct LoraConfig {
        float    freqMhz     = 868.1f;  ///< Carrier frequency [MHz], EU868 band.
        float    bwKhz       = 125.0f;  ///< Bandwidth [kHz].
        uint8_t  sf          = 7;       ///< Spreading factor (7..12).
        uint8_t  cr          = 5;       ///< Coding rate denominator (5 -> 4/5).
        int8_t   txPowerDbm  = 14;      ///< TX power [dBm], EU868 ERP limit.
        uint8_t  syncWord    = 0x12;    ///< Private-network sync word.
        bool     ackEnabled  = true;    ///< Wait for the base ACK after each TX.
        uint16_t ackTimeoutMs = 600;    ///< RX window length after the TX [ms].
        uint8_t  txRetries   = 1;       ///< Retransmissions when the ACK is missing.
    };

    /**
     * @brief Mount LittleFS and load /config.ini.
     *
     * On a missing file the defaults are kept and persisted with save().
     *
     * @return true if the filesystem is available (config is usable),
     *         false on unrecoverable filesystem failure (defaults still set).
     */
    bool begin();

    /**
     * @brief Parse /config.ini and override the current values.
     * @return true if the file existed and was parsed.
     */
    bool load();

    /**
     * @brief Write the current configuration back to /config.ini.
     * @return true on success.
     */
    bool save() const;

    /**
     * @brief Pretty-print the active configuration (for the serial log).
     * @param out Destination stream, e.g. Serial.
     */
    void printTo(Stream &out) const;

    StationConfig station;  ///< Active `[station]` values.
    RainConfig    rain;     ///< Active `[rain]` values.
    WindConfig    wind;     ///< Active `[wind]` values.
    PowerConfig   power;    ///< Active `[power]` values.
    LoraConfig    lora;     ///< Active `[lora]` values.

private:
    /// Apply a single parsed `key = value` pair belonging to @p section.
    void applyKey(const String &section, const String &key, const String &value);

    static constexpr const char *kConfigPath = "/config.ini";  ///< File location on LittleFS.
};

#endif // WEATHERSTATION_APPCONFIG_H
