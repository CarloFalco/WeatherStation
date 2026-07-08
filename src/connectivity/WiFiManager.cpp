/**
 * @file WiFiManager.cpp
 * @brief Implementation of the WiFiManager singleton and its FreeRTOS task.
 */

#include "WiFiManager.h"
#include "task_events.h"
#include <freertos/event_groups.h>

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

WiFiManager& WiFiManager::getInstance() {
    static WiFiManager instance;
    return instance;
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

void WiFiManager::init(const String& ssid, const String& password) {
    _ssid     = ssid;
    _password = password;
    _retries  = 0;

    xTaskCreatePinnedToCore(
        taskFunction,
        "TaskWifi",
        WIFI_TASK_STACK,
        nullptr,
        WIFI_TASK_PRIO,
        &_handle,
        1   // Core 1 (same as Arduino loop)
    );

    Serial.println("[WiFi] TaskWifi created");
}

// ---------------------------------------------------------------------------
// waitForConnection()
// ---------------------------------------------------------------------------

bool WiFiManager::waitForConnection(uint32_t timeoutMs) {
    const EventBits_t bits = xEventGroupWaitBits(
        g_sysEvents,
        EVT_WIFI_CONNECTED | EVT_WIFI_FAILED,   // wait for either
        pdFALSE,                                 // do NOT clear bits
        pdFALSE,                                 // wait for ANY, not ALL
        pdMS_TO_TICKS(timeoutMs)
    );
    return (bits & EVT_WIFI_CONNECTED) != 0;
}

// ---------------------------------------------------------------------------
// isConnected()
// ---------------------------------------------------------------------------

bool WiFiManager::isConnected() const {
    return WiFi.isConnected();
}

// ---------------------------------------------------------------------------
// disconnect()
// ---------------------------------------------------------------------------

void WiFiManager::disconnect() {
    WiFi.disconnect(true /*wifioff*/);
    WiFi.mode(WIFI_OFF);

    // Clear connection bits so the next wake cycle starts clean
    xEventGroupClearBits(g_sysEvents, EVT_WIFI_CONNECTED | EVT_WIFI_FAILED);

    Serial.println("[WiFi] Disconnected – radio OFF");
}

// ---------------------------------------------------------------------------
// taskFunction()  (static)
// ---------------------------------------------------------------------------

void WiFiManager::taskFunction(void* /*pvParams*/) {
    WiFiManager& mgr = getInstance();

    // Register event callback before calling WiFi.begin() so we catch all
    // events from the first connection attempt onward.
    WiFi.onEvent(onWifiEvent);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);   // Manual reconnect via onWifiEvent

    Serial.printf("[WiFi] Connecting to SSID '%s'…\n", mgr._ssid.c_str());
    WiFi.begin(mgr._ssid.c_str(), mgr._password.c_str());

    // The task simply keeps the scheduler alive.
    // All state changes are driven by onWifiEvent().
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
// onWifiEvent()  (static)
// ---------------------------------------------------------------------------

void WiFiManager::onWifiEvent(WiFiEvent_t event) {
    WiFiManager& mgr = getInstance();

    switch (event) {

        // ---- Connected and IP assigned ------------------------------------
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            mgr._retries = 0;   // Reset retry counter on success
            xEventGroupSetBits(g_sysEvents, EVT_WIFI_CONNECTED);
            xEventGroupClearBits(g_sysEvents, EVT_WIFI_FAILED);
            Serial.printf("[WiFi] Connected  IP=%s  RSSI=%d dBm\n",
                          WiFi.localIP().toString().c_str(),
                          WiFi.RSSI());
            break;

        // ---- Disconnected -------------------------------------------------
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            xEventGroupClearBits(g_sysEvents, EVT_WIFI_CONNECTED);
            mgr._retries++;

            if (mgr._retries > MAX_RETRIES) {
                Serial.printf("[WiFi] Max retries (%d) reached – giving up\n",
                              MAX_RETRIES);
                xEventGroupSetBits(g_sysEvents, EVT_WIFI_FAILED);
                mgr._retries = 0;   // Reset so the next wake cycle can try again
            } else {
                Serial.printf("[WiFi] Disconnected – retry %d/%d in %lu ms\n",
                              mgr._retries, MAX_RETRIES,
                              (unsigned long)RETRY_DELAY_MS);
                // Trigger reconnect after a delay (runs in the calling context,
                // but it is safe to call WiFi.reconnect() from event callbacks).
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                WiFi.reconnect();
            }
            break;

        default:
            break;
    }
}
