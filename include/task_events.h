#pragma once

/**
 * @file task_events.h
 * @brief FreeRTOS EventGroup bit definitions and global handle declaration.
 *
 * A single EventGroupHandle_t `g_sysEvents` is used for all inter-task
 * synchronisation.  It is created in `setup()` (main.cpp) and must exist
 * before any task that uses it is started.
 *
 * Bit allocation
 * | Bit | Constant           | Set by          | Meaning                       |
 * |-----|--------------------|-----------------|-------------------------------|
 * |  0  | EVT_WIFI_CONNECTED | WiFiManager     | STA connected, IP assigned    |
 * |  1  | EVT_WIFI_FAILED    | WiFiManager     | All retries exhausted         |
 * |  2  | EVT_NTP_SYNCED     | TimeManager     | NTP sync succeeded            |
 * |  3  | EVT_MQTT_CONNECTED | MqttManager (I3)| Broker connected              |
 * |  4  | EVT_SENSORS_DONE   | SensorManager(I2)| Sensor reading complete      |
 * |  5  | EVT_MQTT_PUBLISHED | MqttManager (I3)| JSON payload published        |
 */

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

// ---------------------------------------------------------------------------
// Global handle  (defined in main.cpp)
// ---------------------------------------------------------------------------

/** @brief System-wide FreeRTOS event group created in setup(). */
extern EventGroupHandle_t g_sysEvents;

// ---------------------------------------------------------------------------
// Bit definitions
// ---------------------------------------------------------------------------

/** @brief WiFi STA is connected and has a valid IP address. */
#define EVT_WIFI_CONNECTED    BIT0

/** @brief WiFi connection failed after MAX_RETRIES attempts. */
#define EVT_WIFI_FAILED       BIT1

/** @brief NTP time synchronisation completed successfully. */
#define EVT_NTP_SYNCED        BIT2

/** @brief MQTT broker connected (Increment 3). */
#define EVT_MQTT_CONNECTED    BIT3

/** @brief All sensor readings collected (Increment 2). */
#define EVT_SENSORS_DONE      BIT4

/** @brief MQTT JSON payload published (Increment 3). */
#define EVT_MQTT_PUBLISHED    BIT5
