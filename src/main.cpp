/**
 * @file main.cpp
 * @brief WeatherStation V2 firmware entry point.
 *
 * Increment 8: reliable delivery. A timer wake-up runs the full cycle:
 * banner -> configuration -> sensor readings -> telemetry JSON -> LoRa
 * transmission with ACK window and retries -> radio to sleep -> deep
 * sleep. The rain accumulator is consumed only on confirmed delivery,
 * so no rainfall is lost to dropped packets. A rain-pulse wake-up (EXT0)
 * takes the quick path instead: count the bucket tip in RTC RAM and go
 * straight back to sleep for the remaining time. loop() is never reached.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "comm/LoRaLink.h"
#include "config.h"
#include "version.h"
#include "core/AppConfig.h"
#include "core/PowerManager.h"
#include "sensors/Anemometer.h"
#include "sensors/Bme280Sensor.h"
#include "sensors/PowerMonitor.h"
#include "sensors/RainGauge.h"
#include "sensors/SensorManager.h"
#include "sensors/WindVane.h"

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
/// Cup anemometer (wind speed and gust).
static Anemometer anemometer;
/// AS5600 wind vane (wind direction).
static WindVane windVane;
/// INA3221 energy monitor (panel / battery / load).
static PowerMonitor powerMonitor;
/// LoRa telemetry link (parameters read from appConfig at begin()).
static LoRaLink telemetryLink(appConfig.lora);

/**
 * @brief Listen for the base ACK matching the given sequence number.
 *
 * Keeps the receive window open for [lora] ack_timeout_ms, discarding
 * unrelated packets (other stations, foreign traffic) until the matching
 * `{"type":"ack","id":<ours>,"seq":<seq>}` arrives or the window closes.
 *
 * @param seq Sequence number of the telemetry message just sent.
 * @return true if the matching ACK was received.
 */
static bool waitForAck(uint16_t seq) {
    uint32_t deadline = millis() + appConfig.lora.ackTimeoutMs;

    while ((int32_t)(deadline - millis()) > 0) {
        String rx;
        if (!telemetryLink.receive(rx, deadline - millis())) {
            break;  // window expired with no packet
        }

        JsonDocument doc;
        if (deserializeJson(doc, rx) != DeserializationError::Ok) {
            log_w("RX window: non-JSON packet ignored");
            continue;
        }
        if (strcmp(doc["type"] | "", "ack") == 0 &&
            appConfig.station.id == (doc["id"] | "") &&
            (uint16_t)(doc["seq"] | 0) == seq) {
            return true;
        }
        log_d("RX window: unrelated packet ignored (%s)", rx.c_str());
    }
    return false;
}

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
    anemometer.configure(appConfig.wind.mpsPerHz, appConfig.wind.sampleWindowS);
    windVane.configure(appConfig.wind.vaneOffsetDeg);
    powerMonitor.configure(appConfig.power.shuntMohm);
    sensors.add(&bme280);
    sensors.add(&rainGauge);
    sensors.add(&anemometer);
    sensors.add(&windVane);
    sensors.add(&powerMonitor);
    size_t healthy = sensors.beginAll();
    Serial.printf("\nSensors ready : %u\n", (unsigned)healthy);

    JsonDocument doc;
    JsonObject msg = doc.to<JsonObject>();
    msg["type"] = "data";
    msg["id"] = appConfig.station.id;
    msg["fw"] = FW_VERSION;
    msg["seq"] = ++g_rtcState.msgSeq;
    sensors.readAll(msg);

    String payload;
    serializeJson(doc, payload);
    Serial.printf("Telemetry (%u bytes): %s\n",
                  (unsigned)payload.length(), payload.c_str());

    // --- Transmission with ACK and retries ----------------------------------
    uint16_t seq = doc["seq"];
    bool delivered = false;
    if (telemetryLink.begin()) {
        for (uint8_t attempt = 0; attempt <= appConfig.lora.txRetries; attempt++) {
            if (attempt > 0) {
                Serial.printf("LoRa TX: retry %u/%u\n", attempt, appConfig.lora.txRetries);
            }
            if (!telemetryLink.send(payload)) {
                continue;
            }
            if (!appConfig.lora.ackEnabled) {
                delivered = true;  // fire-and-forget mode: assume delivery
                break;
            }
            if (waitForAck(seq)) {
                delivered = true;
                break;
            }
        }
    }
    Serial.printf("LoRa TX: %s\n", delivered
                      ? (appConfig.lora.ackEnabled ? "delivered (ACK)" : "sent (no-ACK mode)")
                      : "NOT delivered");
    telemetryLink.sleep();  // SX1276 to sleep mode before the MCU powers down

    // Consume the reported rainfall only when delivery is confirmed:
    // otherwise it stays in the accumulator and rides the next message.
    if (delivered) {
        rainGauge.resetAccumulator();
    }

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
