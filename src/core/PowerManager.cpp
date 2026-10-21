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

/// Longest wait for an active-low wake input to be released before sleeping.
static constexpr uint32_t kWakePinReleaseTimeoutMs = 300;
/// The input must read high this long to count as released (contact bounce).
static constexpr uint32_t kWakePinStableMs = 20;

/**
 * @brief Check that an active-low wake input is idle (high) before sleeping.
 *
 * EXT1 in ANY_LOW mode is level-triggered: a pin still low when the chip
 * goes to sleep wakes it again immediately. That happens for real in two
 * ways: the rain gauge reed still closed at the end of a quick-path wake
 * (the same tip would be counted twice), and an input stuck low for good
 * (bucket resting on the reed, water in an outdoor connector, button
 * jammed) — which would turn deep sleep into a wake loop draining the
 * battery in hours.
 *
 * Returns immediately when the pin is already high (the normal case, no
 * cost); otherwise waits for a stable release up to a timeout.
 *
 * @param pin Active-low wake input.
 * @return true if the pin is idle and can be armed as a wake source.
 */
static bool wakePinReleased(gpio_num_t pin) {
    pinMode(pin, INPUT_PULLUP);
    if (digitalRead(pin) == HIGH) {
        return true;
    }

    uint32_t start = millis();
    uint32_t highSince = 0;
    bool high = false;
    while (millis() - start < kWakePinReleaseTimeoutMs) {
        if (digitalRead(pin) == HIGH) {
            if (!high) {
                high = true;
                highSince = millis();
            } else if (millis() - highSince >= kWakePinStableMs) {
                return true;
            }
        } else {
            high = false;
        }
        delay(1);
    }
    return false;
}

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
    if (on) {
        _railOnMs = millis();
    }
    log_d("Sensor rail %s", on ? "ON" : "off");
}

void PowerManager::waitSensorRailSettled(uint16_t settleMs) const {
    uint32_t elapsed = millis() - _railOnMs;
    if (elapsed < settleMs) {
        delay(settleMs - elapsed);
    }
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
    // pulling their pin low. An input that is not released is left out for
    // this cycle (the timer still wakes the station): see wakePinReleased().
    uint64_t wakeMask = 0;
    if (wakePinReleased(RAIN_GAUGE_PIN)) {
        wakeMask |= 1ULL << RAIN_GAUGE_PIN;
    } else {
        log_w("Rain gauge input stuck low: wake-on-rain disabled for this sleep");
    }
    if (wakePinReleased(FACTORY_RESET_PIN)) {
        wakeMask |= 1ULL << FACTORY_RESET_PIN;
    } else {
        log_w("Reset button input stuck low: button wake disabled for this sleep");
    }

    if (wakeMask != 0) {
        // The RTC domain needs its own pull-ups: the digital GPIO ones are
        // powered down in deep sleep (the board also has external ones).
        rtc_gpio_pullup_en(RAIN_GAUGE_PIN);
        rtc_gpio_pulldown_dis(RAIN_GAUGE_PIN);
        rtc_gpio_pullup_en(FACTORY_RESET_PIN);
        rtc_gpio_pulldown_dis(FACTORY_RESET_PIN);
        esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
    }

    // Freeze the sensor rail control so it keeps its level while the CPU
    // is powered down.
    gpio_hold_en(SENSOR_POWER_PIN);
    gpio_deep_sleep_hold_en();

    Serial.flush();
    esp_deep_sleep_start();

    // esp_deep_sleep_start() does not return; satisfy [[noreturn]] anyway.
    while (true) {}
}
