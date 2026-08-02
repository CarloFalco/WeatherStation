/**
 * @file PowerManager.cpp
 * @brief Implementation of deep-sleep orchestration.
 */

#include "PowerManager.h"

#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include "config.h"

/// RTC-resident state: survives deep sleep, zeroed on power-on reset.
RTC_DATA_ATTR RtcState g_rtcState = {};

void PowerManager::begin() {
    _cause = esp_sleep_get_wakeup_cause();
    _extPins = (_cause == ESP_SLEEP_WAKEUP_EXT1) ? esp_sleep_get_ext1_wakeup_status() : 0;
    g_rtcState.bootCount++;

    // Release the deep-sleep hold so the rail pin can be driven again.
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(SENSOR_POWER_PIN);

    log_i("Boot #%lu, wake-up cause: %s",
          (unsigned long)g_rtcState.bootCount, wakeupCauseString());
}

void PowerManager::setSensorRail(bool on) {
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, on ? SENSOR_POWER_ON_LEVEL : !SENSOR_POWER_ON_LEVEL);
}

const char *PowerManager::wakeupCauseString() const {
    switch (_cause) {
        case ESP_SLEEP_WAKEUP_TIMER:    return "timer (periodic wake-up)";
        case ESP_SLEEP_WAKEUP_EXT0:     return "external signal (EXT0)";
        case ESP_SLEEP_WAKEUP_EXT1:
            if (wokeFromRain() && wokeFromButton()) return "rain gauge + reset button";
            if (wokeFromRain())                     return "rain gauge pulse (EXT1)";
            if (wokeFromButton())                   return "factory reset button (EXT1)";
            return "external signal (EXT1)";
        case ESP_SLEEP_WAKEUP_ULP:      return "ULP coprocessor";
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:                        return "power-on / reset (cold boot)";
    }
}

void PowerManager::deepSleep(uint32_t seconds) {
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);

    // Two wake sources share EXT1 (EXT0 only supports a single pin): a
    // bucket tip on the rain gauge and the factory reset button, both
    // pulling their pin low. The RTC domain needs its own pull-ups because
    // the digital GPIO ones are powered down in deep sleep.
    rtc_gpio_pullup_en(RAIN_GAUGE_PIN);
    rtc_gpio_pulldown_dis(RAIN_GAUGE_PIN);
    rtc_gpio_pullup_en(FACTORY_RESET_PIN);
    rtc_gpio_pulldown_dis(FACTORY_RESET_PIN);
    esp_sleep_enable_ext1_wakeup((1ULL << RAIN_GAUGE_PIN) | (1ULL << FACTORY_RESET_PIN),
                                 ESP_EXT1_WAKEUP_ANY_LOW);

    // Freeze the sensor rail control so it keeps its level while the CPU
    // is powered down.
    gpio_hold_en(SENSOR_POWER_PIN);
    gpio_deep_sleep_hold_en();

    Serial.flush();
    esp_deep_sleep_start();

    // esp_deep_sleep_start() does not return; satisfy [[noreturn]] anyway.
    while (true) {}
}
