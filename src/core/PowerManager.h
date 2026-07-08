/**
 * @file PowerManager.h
 * @brief Deep-sleep orchestration and wake-up cause handling.
 *
 * Owns the station duty cycle: classifies why the MCU woke up, keeps the
 * boot counter in RTC RAM up to date and puts the system back into deep
 * sleep with the timer (and, from Increment 4, the rain-gauge EXT wake-up)
 * armed.
 *
 * Usage example:
 * @code
 * PowerManager power;
 * power.begin();                                   // classify wake-up, ++bootCount
 * Serial.println(power.wakeupCauseString());
 * power.deepSleep(config.station.wakeIntervalS);   // never returns
 * @endcode
 */

#ifndef WEATHERSTATION_POWERMANAGER_H
#define WEATHERSTATION_POWERMANAGER_H

#include <Arduino.h>

#include "rtc_state.h"

/**
 * @brief Manages deep sleep, wake-up sources and the RTC-resident state.
 */
class PowerManager {
public:
    /**
     * @brief Classify the wake-up cause and update the RTC state.
     *
     * Must be called once, early in setup(). Increments
     * RtcState::bootCount (RTC RAM is zeroed by hardware on power-on,
     * so the first boot reads 1).
     */
    void begin();

    /** @return Raw ESP-IDF wake-up cause of the current boot. */
    esp_sleep_wakeup_cause_t wakeupCause() const { return _cause; }

    /** @return Human-readable description of the wake-up cause. */
    const char *wakeupCauseString() const;

    /** @return true on power-on/reset, false when waking from deep sleep. */
    bool isColdBoot() const { return _cause == ESP_SLEEP_WAKEUP_UNDEFINED; }

    /**
     * @brief Enter deep sleep with the timer wake-up armed.
     *
     * Flushes the serial output before powering down. Execution restarts
     * from setup() at the next wake-up: this function never returns.
     *
     * @param seconds Sleep duration [s] (typically station.wakeIntervalS).
     */
    [[noreturn]] void deepSleep(uint32_t seconds);

private:
    esp_sleep_wakeup_cause_t _cause = ESP_SLEEP_WAKEUP_UNDEFINED;
};

#endif // WEATHERSTATION_POWERMANAGER_H
