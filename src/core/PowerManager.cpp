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
    g_rtcState.bootCount++;

    log_i("Boot #%lu, wake-up cause: %s",
          (unsigned long)g_rtcState.bootCount, wakeupCauseString());
}

const char *PowerManager::wakeupCauseString() const {
    switch (_cause) {
        case ESP_SLEEP_WAKEUP_TIMER:    return "timer (periodic wake-up)";
        case ESP_SLEEP_WAKEUP_EXT0:     return "external signal (EXT0, rain gauge)";
        case ESP_SLEEP_WAKEUP_EXT1:     return "external signal (EXT1)";
        case ESP_SLEEP_WAKEUP_ULP:      return "ULP coprocessor";
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:                        return "power-on / reset (cold boot)";
    }
}

void PowerManager::deepSleep(uint32_t seconds) {
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);

    // Rain-gauge wake-up: a bucket tip pulls RAIN_GAUGE_PIN low. The RTC
    // domain needs its own pull-up because the digital GPIO one is powered
    // down in deep sleep.
    rtc_gpio_pullup_en(RAIN_GAUGE_PIN);
    rtc_gpio_pulldown_dis(RAIN_GAUGE_PIN);
    esp_sleep_enable_ext0_wakeup(RAIN_GAUGE_PIN, 0 /* wake on low */);

    Serial.flush();
    esp_deep_sleep_start();

    // esp_deep_sleep_start() does not return; satisfy [[noreturn]] anyway.
    while (true) {}
}
