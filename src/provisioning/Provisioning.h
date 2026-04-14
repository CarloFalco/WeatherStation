#pragma once

/**
 * @file Provisioning.h
 * @brief First-time setup wizard via Wi-Fi Access Point and captive portal.
 *
 * When the firmware detects that no valid configuration exists (or the user
 * holds the reset pin), the Provisioning module:
 *
 *  1. Starts the ESP32 in **Soft-AP** mode (SSID: "WeatherStation-Setup").
 *  2. Launches a **DNS server** that resolves every hostname to the ESP's own
 *     IP address, triggering the OS captive-portal detection on phones/tablets.
 *  3. Serves a **single-page HTML form** (loaded from LittleFS `/index.html`)
 *     on port 80 where the user enters:
 *       - Wi-Fi credentials
 *       - MQTT broker settings
 *       - GitHub repository for OTA updates
 *       - Station identity / topic prefix
 *  4. On form submission (`POST /save`), the values are validated, written to
 *     `/config.ini` via AppConfig::save(), and the device **restarts**.
 *
 * @par Usage
 * @code
 * Provisioning prov;
 * prov.begin();
 * while (true) {
 *     prov.handle();       // process DNS and HTTP requests
 *     vTaskDelay(10);
 * }
 * @endcode
 *
 * @note begin() does not return until the AP is fully up.
 *       handle() must be called frequently (every ~10 ms) from the main loop
 *       or a dedicated FreeRTOS task.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "config/AppConfig.h"

/** @brief SSID broadcast by the provisioning Access Point. */
#define PROV_AP_SSID        "WeatherStation-Setup"

/** @brief Password for the provisioning AP (empty = open network). */
#define PROV_AP_PASSWORD    ""

/** @brief IP address of the provisioning AP (default gateway seen by clients). */
#define PROV_AP_IP          IPAddress(192, 168, 4, 1)

/** @brief DNS port – must be 53. */
#define PROV_DNS_PORT       53

/** @brief HTTP port for the configuration web server. */
#define PROV_HTTP_PORT      80

class Provisioning {
public:
    /**
     * @brief Constructs a Provisioning instance (does not start AP yet).
     */
    Provisioning();

    /**
     * @brief Initialises the AP, DNS server, and HTTP server.
     *
     * Must be called once before entering the handle() loop.
     */
    void begin();

    /**
     * @brief Processes pending DNS queries and HTTP requests.
     *
     * Call this as frequently as possible (ideally every 10 ms).
     */
    void handle();

    /**
     * @brief Returns true once the user has submitted valid credentials.
     *
     * After this returns true the device will restart automatically.
     */
    bool isConfigured() const { return _configured; }

private:
    DNSServer  _dns;
    WebServer  _http;
    bool       _configured = false;

    // ---- HTTP route handlers ------------------------------------------------

    /** @brief Serves the main configuration HTML page. */
    void handleRoot();

    /** @brief Handles `POST /save` – validates and persists the form data. */
    void handleSave();

    /** @brief Returns a minimal inline HTML error page. */
    void handleNotFound();

    /**
     * @brief Sends a styled inline HTML response (used for success/error pages).
     * @param title   Page title.
     * @param body    HTML body content.
     * @param isError If true the heading is styled in red.
     */
    void sendPage(const String& title,
                  const String& body,
                  bool isError = false);

    /**
     * @brief Extracts and URL-decodes a POST parameter by name.
     * @param name  Parameter key as sent by the HTML form.
     * @return Decoded value string, empty if not present.
     */
    String formParam(const String& name) ;
};
