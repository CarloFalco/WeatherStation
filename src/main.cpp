/**
 * @file main.cpp
 * @brief Application entry point – Increment 1: FreeRTOS, WiFi, deep-sleep, RTC/NTP.
 *
 * Boot sequence:
 *  1.  Initialise Serial and LittleFS.
 *  2.  Increment g_bootCount (RTC-persistent wake-cycle counter).
 *  3.  Check provisioning trigger (reset pin held ≥ 3 s, or missing config).
 *  4.  Load runtime configuration from `/config.ini`.
 *  5.  Initialise PowerManager – configure MOSFET GPIO pins, print wakeup cause.
 *  6.  Initialise TimeManager – restore approximate epoch from RTC memory.
 *  7.  Create global FreeRTOS EventGroup (g_sysEvents).
 *  8.  Start WiFiManager task (TaskWifi).
 *  9.  Start sleep-orchestration task (TaskSleep).
 * 10.  TaskSleep: connect WiFi → (optionally) sync NTP → sleep.
 *
 * @par Deep-sleep cycle
 * Each wake cycle is intentionally short (WiFi connect + NTP + future sensors).
 * After completing its work, TaskSleep calls PowerManager::enterDeepSleep()
 * which never returns.  The next boot restarts from setup().
 *
 * @par Deep-sleep disabled (debug)
 * If `sleep_s = 0` in config.ini **or** the build flag `-DDEBUG_SKIP_DEEP_SLEEP`
 * is set (env:esp32s3-weather-debug), the device stays awake and idles.
 *
 * @version 0.2.0
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "config.h"
#include "version.h"
#include "rtc_state.h"
#include "task_events.h"
#include "config/AppConfig.h"
#include "provisioning/Provisioning.h"
#include "power/PowerManager.h"
#include "connectivity/WiFiManager.h"
#include "rtc/TimeManager.h"

// ===========================================================================
// RTC-persistent state  (retained across deep-sleep; cleared on hard reset)
// ===========================================================================

RTC_DATA_ATTR uint32_t g_bootCount        = 0;     ///< Wake-cycle counter.
RTC_DATA_ATTR time_t   g_lastEpoch        = 0;     ///< Epoch before last sleep.
RTC_DATA_ATTR uint32_t g_lastSleepSeconds = 0;     ///< Sleep duration of last cycle.
RTC_DATA_ATTR bool     g_ntpEverSynced    = false; ///< Set after first NTP sync.

// ===========================================================================
// Global FreeRTOS synchronisation handle
// ===========================================================================

EventGroupHandle_t g_sysEvents = nullptr;

// ===========================================================================
// Task configuration
// ===========================================================================

/** @brief FreeRTOS stack depth (words) for the sleep-orchestration task. */
static constexpr uint32_t    SLEEP_TASK_STACK = 5120;

/** @brief FreeRTOS priority for TaskSleep. */
static constexpr UBaseType_t SLEEP_TASK_PRIO  = 2;

// ===========================================================================
// Forward declarations
// ===========================================================================

static bool shouldEnterProvisioning();
static void enterProvisioning();

/**
 * @brief FreeRTOS task that orchestrates each wake cycle.
 *
 * Per-cycle sequence:
 *  1. Enable sensor rails via PowerManager.
 *  2. Wait up to 30 s for WiFi (EVT_WIFI_CONNECTED or EVT_WIFI_FAILED).
 *  3. If connected and NTP re-sync is due, call TimeManager::syncNtp().
 *  4. (Increment 2) Read all sensors.
 *  5. (Increment 3) Publish JSON payload via MQTT.
 *  6. Disconnect WiFi, save epoch to RTC, enter deep sleep.
 *
 * @param pvParams  Unused.
 */
static void vTaskSleep(void* pvParams);

// ===========================================================================
// setup()
// ===========================================================================

void setup() {
    Serial.begin(115200);
    delay(400);   // Allow USB CDC to enumerate

    Serial.printf("\n\n");
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.printf( "║  ESP32-S3 Weather Station  v%-10s║\n", FW_VERSION);
    Serial.printf( "║  %-37s║\n", FW_VERSION_DESC);
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.println();

    // ---- Wake-cycle counter -------------------------------------------------
    g_bootCount++;
    Serial.printf("[Boot] Wake cycle #%lu\n", static_cast<unsigned long>(g_bootCount));

    // ---- Filesystem ---------------------------------------------------------
    if (!LittleFS.begin(true /* format on fail */)) {
        Serial.println("[FATAL] LittleFS mount failed – cannot continue");
        pinMode(LED_BUILTIN_PIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
            delay(200);
        }
    }
    Serial.println("[Boot] LittleFS OK");

    // ---- Provisioning check -------------------------------------------------
    if (shouldEnterProvisioning()) {
        enterProvisioning();
        // Never reached – ESP.restart() is called inside enterProvisioning()
    }

    // ---- Load configuration -------------------------------------------------
    AppConfig& cfg = AppConfig::getInstance();
    if (!cfg.load("/config.ini")) {
        Serial.println("[Boot] Config load failed – falling back to provisioning");
        enterProvisioning();
    }

#ifdef ENV_DEBUG
    cfg.dump();
#endif

    Serial.printf("[Boot] SSID: %-20s  MQTT: %s:%u\n",
                  cfg.wifiSsid.c_str(),
                  cfg.mqttHost.c_str(),
                  cfg.mqttPort);

    // ---- Power management ---------------------------------------------------
    PowerManager& pwr = PowerManager::getInstance();
    pwr.init();
    pwr.printWakeupReason();

    // ---- Restore time from RTC slow memory ----------------------------------
    TimeManager::getInstance().init();

    // ---- FreeRTOS event group -----------------------------------------------
    g_sysEvents = xEventGroupCreate();
    configASSERT(g_sysEvents != nullptr);

    // ---- Start WiFi management task -----------------------------------------
    WiFiManager::getInstance().init(cfg.wifiSsid, cfg.wifiPassword);

    // ---- Start sleep-orchestration task ------------------------------------
    BaseType_t rc = xTaskCreatePinnedToCore(
        vTaskSleep,
        "TaskSleep",
        SLEEP_TASK_STACK,
        nullptr,
        SLEEP_TASK_PRIO,
        nullptr,
        1   // Core 1
    );
    configASSERT(rc == pdPASS);

    Serial.println("[Boot] Increment 1 – FreeRTOS tasks running");
}

// ===========================================================================
// loop()
// ===========================================================================

void loop() {
    // All application logic runs in FreeRTOS tasks.
    // loop() yields indefinitely to avoid starving the scheduler.
    vTaskDelay(pdMS_TO_TICKS(10000));
}

// ===========================================================================
// vTaskSleep()  – wake-cycle orchestration
// ===========================================================================

static void vTaskSleep(void* /*pvParams*/) {
    AppConfig&    cfg  = AppConfig::getInstance();
    PowerManager& pwr  = PowerManager::getInstance();
    WiFiManager&  wifi = WiFiManager::getInstance();
    TimeManager&  rtc  = TimeManager::getInstance();

    Serial.println("[TaskSleep] Wake cycle started");

    // 1. Enable sensor supply rails (sensors will be read in Increment 2)
    pwr.enable5V(true);
    pwr.enable3V3(true);

    // 2. Wait for WiFi – up to 30 s
    const bool wifiOk = wifi.waitForConnection(30000);

    if (wifiOk) {
        Serial.println("[TaskSleep] WiFi connected OK");

        // 3. NTP sync if due (first boot, or every NTP_RESYNC_BOOTS cycles)
        if (rtc.isResyncDue() || !rtc.isTimeValid()) {
            if (!rtc.syncNtp(NTP_TIMEOUT_MS)) {
                Serial.println("[TaskSleep] NTP failed – using restored epoch");
            }
        } else {
            Serial.printf("[TaskSleep] NTP not due (boot #%lu, resync every %d cycles)\n",
                          static_cast<unsigned long>(g_bootCount),
                          NTP_RESYNC_BOOTS);
        }
    } else {
        Serial.println("[TaskSleep] WiFi unavailable – proceeding without network");
    }

    if (rtc.isTimeValid()) {
        rtc.printCurrentTime();
    } else {
        Serial.println("[TaskSleep] Time unknown – timestamp will be 0 in payload");
    }

    // =========================================================================
    // TODO – Increment 2: instantiate SensorManager and read all sensors
    // TODO – Increment 3: connect MqttManager, serialise JSON, publish
    // =========================================================================

    // 4. Disconnect WiFi to minimise current draw during sleep
    wifi.disconnect();

    // 5. Persist epoch so the next boot can restore approximate time
    rtc.saveEpochToRtc();
    g_lastSleepSeconds = cfg.deepSleepDurationS;

    // 6. Enter deep sleep (or idle if disabled)
#ifdef DEBUG_SKIP_DEEP_SLEEP
    Serial.println("[TaskSleep] DEBUG_SKIP_DEEP_SLEEP – skipping deep sleep");
    while (true) { vTaskDelay(pdMS_TO_TICKS(10000)); }
#endif

    if (cfg.deepSleepDurationS == 0) {
        Serial.println("[TaskSleep] sleep_s=0 – deep sleep disabled, idling");
        while (true) { vTaskDelay(pdMS_TO_TICKS(10000)); }
    }

    Serial.printf("[TaskSleep] Cycle complete – sleeping %lu s\n",
                  static_cast<unsigned long>(cfg.deepSleepDurationS));
    pwr.enterDeepSleep(cfg.deepSleepDurationS);
    // Never reached
    vTaskDelete(nullptr);
}

// ===========================================================================
// shouldEnterProvisioning()  (static helper)
// ===========================================================================

static bool shouldEnterProvisioning() {
    pinMode(PIN_RESET_CONFIG, INPUT_PULLUP);
    delay(50);   // Debounce

    if (digitalRead(PIN_RESET_CONFIG) == LOW) {
        Serial.printf("[Boot] Reset pin (GPIO%d) LOW – waiting %d ms to confirm…\n",
                      static_cast<int>(PIN_RESET_CONFIG), RESET_HOLD_MS);

        uint32_t held = 0;
        while (digitalRead(PIN_RESET_CONFIG) == LOW && held < RESET_HOLD_MS) {
            delay(50);
            held += 50;
        }

        if (held >= RESET_HOLD_MS) {
            Serial.println("[Boot] Config reset confirmed → provisioning mode");
            if (LittleFS.exists("/config.ini")) {
                LittleFS.remove("/config.ini");
                Serial.println("[Boot] /config.ini removed");
            }
            return true;
        }
    }

    if (!LittleFS.exists("/config.ini")) {
        Serial.println("[Boot] /config.ini not found → provisioning mode");
        return true;
    }

    return false;
}

// ===========================================================================
// enterProvisioning()  (static helper)
// ===========================================================================

static void enterProvisioning() {
    Serial.println("[Boot] Entering provisioning mode…");
    pinMode(LED_BUILTIN_PIN, OUTPUT);

    static Provisioning prov;   // static to outlive this call frame
    prov.begin();

    uint32_t lastBlink = 0;
    while (true) {
        prov.handle();

        const uint32_t now = millis();
        if (now - lastBlink >= 1000) {
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
            lastBlink = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Never reached – Provisioning::handleSave() calls ESP.restart()
}
