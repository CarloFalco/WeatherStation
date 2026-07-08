/**
 * @file main.cpp
 * @brief WeatherStation V2 firmware entry point.
 *
 * Increment 2: real duty cycle skeleton. Every wake-up runs setup() from
 * scratch: banner -> load configuration -> (sensors and LoRa in the next
 * increments) -> deep sleep for station.wakeIntervalS seconds. loop() is
 * never reached.
 */

#include <Arduino.h>

#include "config.h"
#include "version.h"
#include "core/AppConfig.h"
#include "core/PowerManager.h"

/// Station runtime configuration, loaded from LittleFS at boot.
static AppConfig appConfig;
/// Deep-sleep and wake-up cause management.
static PowerManager power;

/**
 * @brief Blink the status LED a few times to signal activity.
 *
 * @param times Number of blinks.
 */
static void blink(uint8_t times) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(80);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(120);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // give USB-CDC time to enumerate so the log is visible

    power.begin();

    Serial.println();
    Serial.println("============================================");
    Serial.printf("  WeatherStation V2 - fw v%s\n", FW_VERSION);
    Serial.println("============================================");
    Serial.printf("Boot count    : %lu%s\n", (unsigned long)g_rtcState.bootCount,
                  power.isColdBoot() ? " (cold boot)" : "");
    Serial.printf("Wake-up cause : %s\n", power.wakeupCauseString());
    Serial.println();

    appConfig.begin();
    appConfig.printTo(Serial);

    // --- Measurement window -------------------------------------------------
    // Increment 3+: read sensors here and transmit the JSON over LoRa.
    blink(3);

    // --- Back to sleep ------------------------------------------------------
    Serial.printf("\nEntering deep sleep for %lu s\n",
                  (unsigned long)appConfig.station.wakeIntervalS);
    power.deepSleep(appConfig.station.wakeIntervalS);
}

void loop() {
    // Never reached: every cycle ends in deep sleep and restarts from setup().
}
