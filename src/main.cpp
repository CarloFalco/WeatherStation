/**
 * @file main.cpp
 * @brief WeatherStation V2 firmware entry point.
 *
 * Increment 1: boot banner + runtime configuration loaded from
 * /config.ini on LittleFS (AppConfig). The real wake -> measure ->
 * transmit -> deep-sleep cycle is introduced in later increments.
 */

#include <Arduino.h>

#include "config.h"
#include "version.h"
#include "core/AppConfig.h"

/// Station runtime configuration, loaded from LittleFS at boot.
static AppConfig appConfig;

/**
 * @brief Return a human-readable description of the ESP32 wake-up cause.
 *
 * Useful to distinguish a cold boot from a timer wake-up (periodic
 * measurement) or an external wake-up (rain gauge pulse).
 *
 * @return Static string describing the cause of the last wake-up.
 */
static const char *wakeupReasonToString() {
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER:    return "timer (periodic wake-up)";
        case ESP_SLEEP_WAKEUP_EXT0:     return "external signal (EXT0)";
        case ESP_SLEEP_WAKEUP_EXT1:     return "external signal (EXT1)";
        case ESP_SLEEP_WAKEUP_ULP:      return "ULP coprocessor";
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:                        return "power-on / reset (cold boot)";
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // give USB-CDC time to enumerate so the banner is visible

    Serial.println();
    Serial.println("============================================");
    Serial.printf("  WeatherStation V2 - fw v%s\n", FW_VERSION);
    Serial.println("============================================");
    Serial.printf("Wake-up cause : %s\n", wakeupReasonToString());
    Serial.printf("CPU frequency : %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
    Serial.printf("Free heap     : %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.println();

    appConfig.begin();
    appConfig.printTo(Serial);

    pinMode(STATUS_LED_PIN, OUTPUT);
}

void loop() {
    // Increment 0 heartbeat: 1 s blink + periodic log line.
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(900);

    static uint32_t seconds = 0;
    if (++seconds % 10 == 0) {
        Serial.printf("[heartbeat] alive for %lu s\n", (unsigned long)seconds);
    }
}
