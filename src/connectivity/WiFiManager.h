#pragma once

/**
 * @file WiFiManager.h
 * @brief Manages the WiFi STA lifecycle in a dedicated FreeRTOS task.
 *
 * @par Architecture
 * A background task (`TaskWifi`) owns the WiFi state machine.  On connection
 * success it sets `EVT_WIFI_CONNECTED` in `g_sysEvents`; on exhausted retries
 * it sets `EVT_WIFI_FAILED`.  Other tasks call `waitForConnection()` to block
 * on those bits with an optional timeout.
 *
 * @par Retry policy
 * After every `ARDUINO_EVENT_WIFI_STA_DISCONNECTED` event, the task waits
 * `RETRY_DELAY_MS` before calling `WiFi.reconnect()`.  After `MAX_RETRIES`
 * consecutive failures `EVT_WIFI_FAILED` is set and the task resets its
 * counter, leaving the door open for the next wake cycle.
 *
 * @par Usage
 * @code
 * WiFiManager& wifi = WiFiManager::getInstance();
 * wifi.init(cfg.wifiSsid, cfg.wifiPassword);   // spawns TaskWifi
 * bool ok = wifi.waitForConnection(30000);       // returns when connected or timeout
 * if (ok) { /* proceed *\/ }
 * wifi.disconnect();                             // before deep sleep
 * @endcode
 */

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/** @brief FreeRTOS stack depth (words) allocated for TaskWifi. */
#define WIFI_TASK_STACK   4096

/** @brief FreeRTOS priority for TaskWifi. */
#define WIFI_TASK_PRIO    3

class WiFiManager {
public:
    /** @brief Returns the singleton instance. */
    static WiFiManager& getInstance();

    /**
     * @brief Store credentials and spawn the WiFi management task.
     *
     * @param ssid      Target network SSID.
     * @param password  Network password; pass an empty string for open networks.
     */
    void init(const String& ssid, const String& password);

    /**
     * @brief Block the calling task until WiFi connects or the timeout expires.
     *
     * Waits for `EVT_WIFI_CONNECTED` **or** `EVT_WIFI_FAILED` in
     * `g_sysEvents` — whichever comes first.
     *
     * @param timeoutMs  Maximum wait in milliseconds.
     * @return `true`  if `EVT_WIFI_CONNECTED` was set within the timeout.
     * @return `false` if `EVT_WIFI_FAILED` or the timeout elapsed first.
     */
    bool waitForConnection(uint32_t timeoutMs = 30000);

    /** @brief `true` while the STA interface has a valid IP address. */
    bool isConnected() const;

    /**
     * @brief Disconnect STA and power off the WiFi radio.
     *
     * Should be called from vTaskSleep before entering deep sleep to
     * minimise current draw during the sleep period.
     */
    void disconnect();

private:
    WiFiManager() = default;
    WiFiManager(const WiFiManager&) = delete;
    WiFiManager& operator=(const WiFiManager&) = delete;

    /** @brief FreeRTOS task entry point. */
    static void taskFunction(void* pvParams);

    /** @brief Arduino WiFiEvent callback registered in taskFunction(). */
    static void onWifiEvent(WiFiEvent_t event);

    String       _ssid;
    String       _password;
    TaskHandle_t _handle  = nullptr;
    uint8_t      _retries = 0;     ///< Consecutive disconnection counter.

    /** @brief Maximum consecutive connection failures per wake cycle. */
    static constexpr uint8_t  MAX_RETRIES    = 10;

    /** @brief Delay between reconnection attempts (ms). */
    static constexpr uint32_t RETRY_DELAY_MS = 5000;
};
