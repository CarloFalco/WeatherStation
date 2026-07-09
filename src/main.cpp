/**
 * @file main.cpp
 * @brief WeatherStation V2 firmware entry point.
 *
 * Increment 4: two wake paths. A timer wake-up runs the full cycle:
 * banner -> configuration -> sensor readings -> telemetry JSON (printed
 * on serial; sent over LoRa from Increment 7) -> deep sleep. A rain-pulse
 * wake-up (EXT0) takes the quick path instead: count the bucket tip in
 * RTC RAM and go straight back to sleep for the remaining time, keeping
 * the energy cost of rain events minimal. loop() is never reached.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "config.h"
#include "version.h"
#include "core/AppConfig.h"
#include "core/PowerManager.h"
#include "sensors/Bme280Sensor.h"
#include "sensors/RainGauge.h"
#include "sensors/SensorManager.h"

/// Station runtime configuration, loaded from LittleFS at boot.
static AppConfig appConfig;
/// Deep-sleep and wake-up cause management.
static PowerManager power;
/// Registry of all station sensors.
static SensorManager sensors;
/// Temperature / humidity / pressure sensor.
static Bme280Sensor bme280;
/// Tipping-bucket rain gauge.
static RainGauge rainGauge;

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
    power.begin();

    // --- Quick path: rain pulse during deep sleep ---------------------------
    // Count the tip and go back to sleep for the time left until the next
    // scheduled cycle. No serial wait, no sensors, no radio: a rain event
    // costs milliseconds of CPU, not a full measurement cycle. The RTC
    // clock keeps running in deep sleep, so time() is monotonic here.
    if (power.wakeupCause() == ESP_SLEEP_WAKEUP_EXT0) {
        RainGauge::countSleepPulse();
        int64_t remaining = (int64_t)g_rtcState.nextWakeEpochS - (int64_t)time(nullptr);
        if (remaining > 3) {
            power.deepSleep((uint32_t)remaining);
        }
        // Almost time for the scheduled cycle anyway: fall through and run it.
    }

    Serial.begin(115200);
    delay(2000);  // give USB-CDC time to enumerate so the log is visible

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
    Wire.begin(I2C_SDA, I2C_SCL);

    rainGauge.configure(appConfig.rain.mmPerPulse);
    sensors.add(&bme280);
    sensors.add(&rainGauge);
    size_t healthy = sensors.beginAll();
    Serial.printf("\nSensors ready : %u\n", (unsigned)healthy);

    JsonDocument doc;
    JsonObject msg = doc.to<JsonObject>();
    msg["type"] = "data";
    msg["id"] = appConfig.station.id;
    msg["fw"] = FW_VERSION;
    msg["seq"] = ++g_rtcState.msgSeq;
    sensors.readAll(msg);

    // From Increment 7 this payload goes to the LoRa link; for now it is
    // printed so the cycle can be validated end-to-end on serial.
    String payload;
    serializeJson(doc, payload);
    Serial.printf("Telemetry (%u bytes): %s\n",
                  (unsigned)payload.length(), payload.c_str());

    // Until the ACK protocol lands (Increment 8), the rain accumulator is
    // reset after every send attempt.
    rainGauge.resetAccumulator();

    blink(3);

    // --- Back to sleep ------------------------------------------------------
    g_rtcState.nextWakeEpochS = (uint64_t)time(nullptr) + appConfig.station.wakeIntervalS;
    Serial.printf("\nEntering deep sleep for %lu s\n",
                  (unsigned long)appConfig.station.wakeIntervalS);
    power.deepSleep(appConfig.station.wakeIntervalS);
}

void loop() {
    // Never reached: every cycle ends in deep sleep and restarts from setup().
}
