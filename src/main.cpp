/**
 * @file main.cpp
 * @brief Application entry point for the ESP32-S3 Weather Station firmware.
 *
 * Boot sequence (Increment 0):
 *  1. Initialise Serial and LittleFS.
 *  2. Check whether the user is requesting provisioning mode
 *     (reset-config pin held LOW, or no config file found).
 *  3. If provisioning is needed, start the captive-portal wizard and block
 *     until the user saves credentials (device restarts automatically).
 *  4. Load the runtime configuration from `/config.ini`.
 *  5. (Placeholder) Hand off to FreeRTOS tasks – to be implemented in
 *     Increment 1 (sleep management, Wi-Fi, RTC).
 *
 * @version 0.1.0
 */

#include <Arduino.h>
#include <LittleFS.h>

#include "config.h"
#include "version.h"
#include "config/AppConfig.h"
#include "provisioning/Provisioning.h"

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

/**
 * @brief Determines whether the device should enter provisioning mode.
 *
 * Returns true when:
 *  - The reset-config GPIO (PIN_RESET_CONFIG) is held LOW for at least
 *    RESET_HOLD_MS milliseconds at boot, OR
 *  - The file `/config.ini` does not exist on LittleFS.
 *
 * @return true  if provisioning mode should be entered.
 * @return false otherwise.
 */
static bool shouldEnterProvisioning();

/**
 * @brief Enters provisioning mode (blocks until device restarts).
 *
 * Instantiates a Provisioning object, starts the AP + captive-portal, and
 * spins in a loop calling handle().  The loop never exits because the
 * Provisioning::handleSave() calls ESP.restart() after writing the config.
 */
static void enterProvisioning();

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(2000);   // Allow USB CDC to enumerate

    Serial.printf("\n\n");
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.printf( "║  ESP32-S3 Weather Station  v%-10s║\n", FW_VERSION);
    Serial.printf( "║  %-37s║\n", FW_VERSION_DESC);
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.println();

    // ---- Filesystem ---------------------------------------------------------
    if (!LittleFS.begin(true /* format on fail */)) {
        Serial.println("[FATAL] LittleFS mount failed – cannot continue.");
        // Blink LED rapidly to signal fatal error
        pinMode(LED_BUILTIN_PIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
            delay(1000);
        }
    }
    Serial.println("[Boot] LittleFS mounted OK");

    // ---- Provisioning check -------------------------------------------------
    if (shouldEnterProvisioning()) {
        enterProvisioning();
        // Never reached – ESP.restart() is called inside enterProvisioning()
    }

    // ---- Load configuration -------------------------------------------------
    AppConfig& cfg = AppConfig::getInstance();
    if (!cfg.load("/config.ini")) {
        Serial.println("[Boot] Config load failed or invalid – falling back to provisioning");
        enterProvisioning();
    }

#ifdef ENV_DEBUG
    cfg.dump();
#endif

    Serial.println("[Boot] Configuration OK");
    Serial.printf( "[Boot] Wi-Fi SSID  : %s\n", cfg.wifiSsid.c_str());
    Serial.printf( "[Boot] MQTT broker : %s:%u\n", cfg.mqttHost.c_str(), cfg.mqttPort);
    Serial.printf( "[Boot] GitHub repo : %s\n", cfg.otaGithubRepo.c_str());
    Serial.println();

    // =========================================================================
    // TODO – Increment 1:
    //   • Initialise PowerManager (MOSFET rails, deep-sleep wake stubs)
    //   • Sync RTC via NTP
    //   • Start FreeRTOS tasks:
    //       - WiFiTask   : connect / reconnect
    //       - SleepTask  : orchestrate wake/sleep cycle
    // =========================================================================

    Serial.println("[Boot] Increment 0 complete – awaiting next increment.");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------

void loop() {
    // In the final firmware all work is done in FreeRTOS tasks.
    // This loop intentionally yields to the RTOS scheduler.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------------------------------------------------------------------
// shouldEnterProvisioning()  (static)
// ---------------------------------------------------------------------------

static bool shouldEnterProvisioning() {
    // --- Check reset-config pin ---
    pinMode(PIN_RESET_CONFIG, INPUT_PULLUP);
    delay(50);   // Debounce

    if (digitalRead(PIN_RESET_CONFIG) == LOW) {
        Serial.printf("[Boot] Reset pin (GPIO%d) is LOW – waiting %d ms to confirm…\n",
                      (int)PIN_RESET_CONFIG, RESET_HOLD_MS);

        uint32_t held = 0;
        while (digitalRead(PIN_RESET_CONFIG) == LOW && held < RESET_HOLD_MS) {
            delay(50);
            held += 50;
        }

        if (held >= RESET_HOLD_MS) {
            Serial.println("[Boot] Config reset confirmed → Provisioning mode");

            // Erase stored config so the wizard shows a clean form
            if (LittleFS.exists("/config.ini")) {
                LittleFS.remove("/config.ini");
                Serial.println("[Boot] /config.ini removed");
            }
            return true;
        }
        // Button was released before threshold – normal boot
    }

    // --- Check config file ---
    if (!LittleFS.exists("/config.ini")) {
        Serial.println("[Boot] /config.ini not found → Provisioning mode");
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// enterProvisioning()  (static)
// ---------------------------------------------------------------------------

static void enterProvisioning() {
    Serial.println("[Boot] Entering provisioning mode…");
    pinMode(LED_BUILTIN_PIN, OUTPUT);

    static Provisioning prov;   // static so it outlives this call frame
    prov.begin();

    // Blink LED slowly while waiting for user to configure
    uint32_t lastBlink = 0;
    while (true) {
        prov.handle();

        uint32_t now = millis();
        if (now - lastBlink >= 1000) {
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
            lastBlink = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Never reached – Provisioning::handleSave() calls ESP.restart()
}
