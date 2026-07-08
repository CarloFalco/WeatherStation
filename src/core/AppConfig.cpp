/**
 * @file AppConfig.cpp
 * @brief Implementation of the INI-based runtime configuration.
 */

#include "AppConfig.h"

#include <LittleFS.h>

bool AppConfig::begin() {
    if (!LittleFS.begin(true /* format on first mount failure */)) {
        log_e("LittleFS mount failed: running on default configuration");
        return false;
    }

    if (load()) {
        log_i("Configuration loaded from %s", kConfigPath);
    } else {
        log_w("%s not found: writing default configuration", kConfigPath);
        save();
    }
    return true;
}

bool AppConfig::load() {
    File file = LittleFS.open(kConfigPath, "r");
    if (!file) {
        return false;
    }

    String section;
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();

        // Skip blanks and full-line comments.
        if (line.isEmpty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // Section header: [name]
        if (line[0] == '[' && line.endsWith("]")) {
            section = line.substring(1, line.length() - 1);
            section.trim();
            section.toLowerCase();
            continue;
        }

        int eq = line.indexOf('=');
        if (eq < 0) {
            log_w("config.ini: ignoring malformed line '%s'", line.c_str());
            continue;
        }

        String key = line.substring(0, eq);
        String value = line.substring(eq + 1);
        key.trim();
        key.toLowerCase();
        value.trim();

        applyKey(section, key, value);
    }
    file.close();
    return true;
}

void AppConfig::applyKey(const String &section, const String &key, const String &value) {
    if (section == "station") {
        if (key == "id") {
            station.id = value;
        } else if (key == "wake_interval_s") {
            long v = value.toInt();
            if (v >= 10) {
                station.wakeIntervalS = (uint32_t)v;
            } else {
                log_w("config.ini: wake_interval_s=%ld too small (<10 s), keeping %lu",
                      v, (unsigned long)station.wakeIntervalS);
            }
        } else {
            log_w("config.ini: unknown key [station] %s", key.c_str());
        }
        return;
    }

    if (section == "lora") {
        if (key == "freq_mhz") {
            lora.freqMhz = value.toFloat();
        } else if (key == "bw_khz") {
            lora.bwKhz = value.toFloat();
        } else if (key == "sf") {
            long v = value.toInt();
            lora.sf = (uint8_t)constrain(v, 6L, 12L);
        } else if (key == "cr") {
            long v = value.toInt();
            lora.cr = (uint8_t)constrain(v, 5L, 8L);
        } else if (key == "tx_power_dbm") {
            long v = value.toInt();
            lora.txPowerDbm = (int8_t)constrain(v, 2L, 17L);
        } else if (key == "sync_word") {
            // Accepts both decimal and 0x-prefixed hex.
            lora.syncWord = (uint8_t)strtol(value.c_str(), nullptr, 0);
        } else {
            log_w("config.ini: unknown key [lora] %s", key.c_str());
        }
        return;
    }

    log_w("config.ini: unknown section [%s]", section.c_str());
}

bool AppConfig::save() const {
    File file = LittleFS.open(kConfigPath, "w");
    if (!file) {
        log_e("Cannot open %s for writing", kConfigPath);
        return false;
    }

    file.println("; WeatherStation V2 - runtime configuration");
    file.println("; Edit and re-upload with 'pio run -t uploadfs', or let the");
    file.println("; firmware regenerate this file with defaults if deleted.");
    file.println();
    file.println("[station]");
    file.printf("id = %s\n", station.id.c_str());
    file.printf("wake_interval_s = %lu\n", (unsigned long)station.wakeIntervalS);
    file.println();
    file.println("[lora]");
    file.printf("freq_mhz = %.1f\n", lora.freqMhz);
    file.printf("bw_khz = %.1f\n", lora.bwKhz);
    file.printf("sf = %u\n", lora.sf);
    file.printf("cr = %u\n", lora.cr);
    file.printf("tx_power_dbm = %d\n", lora.txPowerDbm);
    file.printf("sync_word = 0x%02X\n", lora.syncWord);

    file.close();
    return true;
}

void AppConfig::printTo(Stream &out) const {
    out.println("Active configuration:");
    out.printf("  [station] id              = %s\n", station.id.c_str());
    out.printf("  [station] wake_interval_s = %lu\n", (unsigned long)station.wakeIntervalS);
    out.printf("  [lora]    freq_mhz        = %.1f\n", lora.freqMhz);
    out.printf("  [lora]    bw_khz          = %.1f\n", lora.bwKhz);
    out.printf("  [lora]    sf              = %u\n", lora.sf);
    out.printf("  [lora]    cr              = 4/%u\n", lora.cr);
    out.printf("  [lora]    tx_power_dbm    = %d\n", lora.txPowerDbm);
    out.printf("  [lora]    sync_word       = 0x%02X\n", lora.syncWord);
}
