/**
 * @file PowerManager.cpp
 * @brief Implementation of the PowerManager singleton.
 */

#include "PowerManager.h"
#include "config.h"
#include <esp_sleep.h>
#include <driver/rtc_io.h>

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

PowerManager& PowerManager::getInstance() {
    static PowerManager instance;
    return instance;
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

void PowerManager::init() {
    // Configure MOSFET gate pins
    pinMode(PIN_PWR_5V,  OUTPUT);
    pinMode(PIN_PWR_3V3, OUTPUT);

    // P-channel MOSFET: HIGH on gate = transistor OFF → rail disabled
    digitalWrite(PIN_PWR_5V,  HIGH);
    digitalWrite(PIN_PWR_3V3, HIGH);

    Serial.println("[Power] PowerManager initialised – all sensor rails OFF");
}

// ---------------------------------------------------------------------------
// enable5V() / enable3V3()
// ---------------------------------------------------------------------------

void PowerManager::enable5V(bool on) {
    digitalWrite(PIN_PWR_5V, on ? LOW : HIGH);
    Serial.printf("[Power] 5V rail %s\n", on ? "ON" : "OFF");
}

void PowerManager::enable3V3(bool on) {
    digitalWrite(PIN_PWR_3V3, on ? LOW : HIGH);
    Serial.printf("[Power] 3.3V rail %s\n", on ? "ON" : "OFF");
}

// ---------------------------------------------------------------------------
// enterDeepSleep()
// ---------------------------------------------------------------------------

void PowerManager::enterDeepSleep(uint32_t seconds) {
    Serial.printf("[Power] Preparing deep sleep for %u s…\n", seconds);

    // Disable both sensor rails
    enable5V(false);
    enable3V3(false);

    // ---- Timer wakeup -------------------------------------------------------
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);

    // ---- EXT1 GPIO wakeup (reed switches, active LOW) ----------------------
    // GPIO19 (rain gauge) and GPIO20 (anemometer) are in the ESP32-S3 RTC domain.
    // External pull-ups keep the lines HIGH at rest; switch closure pulls LOW.
    const uint64_t wakeupMask =
        (1ULL << static_cast<uint32_t>(PIN_RAIN_GAUGE)) |
        (1ULL << static_cast<uint32_t>(PIN_ANEMOMETER));

    // Enable RTC pull-ups so the lines are not floating during sleep
    rtc_gpio_init(PIN_RAIN_GAUGE);
    rtc_gpio_set_direction(PIN_RAIN_GAUGE, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(PIN_RAIN_GAUGE);
    rtc_gpio_pulldown_dis(PIN_RAIN_GAUGE);

    rtc_gpio_init(PIN_ANEMOMETER);
    rtc_gpio_set_direction(PIN_ANEMOMETER, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(PIN_ANEMOMETER);
    rtc_gpio_pulldown_dis(PIN_ANEMOMETER);

    esp_sleep_enable_ext1_wakeup(wakeupMask, ESP_EXT1_WAKEUP_ANY_LOW);

    Serial.flush();
    esp_deep_sleep_start();
    // Never returns
}

// ---------------------------------------------------------------------------
// getWakeupCause() / isTimerWakeup() / isGpioWakeup()
// ---------------------------------------------------------------------------

esp_sleep_wakeup_cause_t PowerManager::getWakeupCause() const {
    return esp_sleep_get_wakeup_cause();
}

bool PowerManager::isTimerWakeup() const {
    return getWakeupCause() == ESP_SLEEP_WAKEUP_TIMER;
}

bool PowerManager::isGpioWakeup() const {
    return getWakeupCause() == ESP_SLEEP_WAKEUP_EXT1;
}

// ---------------------------------------------------------------------------
// printWakeupReason()
// ---------------------------------------------------------------------------

void PowerManager::printWakeupReason() const {
    switch (getWakeupCause()) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("[Power] Wakeup cause: TIMER");
            break;

        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t status = esp_sleep_get_ext1_wakeup_status();
            if (status & (1ULL << static_cast<uint32_t>(PIN_RAIN_GAUGE))) {
                Serial.println("[Power] Wakeup cause: EXT1 – rain gauge (GPIO19)");
            } else if (status & (1ULL << static_cast<uint32_t>(PIN_ANEMOMETER))) {
                Serial.println("[Power] Wakeup cause: EXT1 – anemometer (GPIO20)");
            } else {
                Serial.printf("[Power] Wakeup cause: EXT1 (mask=0x%llX)\n", status);
            }
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:
            Serial.println("[Power] Wakeup cause: power-on / external reset");
            break;

        default:
            Serial.printf("[Power] Wakeup cause: other (%d)\n",
                          static_cast<int>(getWakeupCause()));
            break;
    }
}
