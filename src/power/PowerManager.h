#pragma once

/**
 * @file PowerManager.h
 * @brief Controls sensor power rails and coordinates deep-sleep entry.
 *
 * Responsibilities:
 *  - Drive P-channel MOSFET gates for the **5 V** (PIN_PWR_5V) and
 *    **3.3 V** (PIN_PWR_3V3) sensor supply rails.
 *  - Configure ESP32-S3 wakeup sources before calling
 *    `esp_deep_sleep_start()`:
 *      - Timer wakeup (periodic measurement cycle).
 *      - EXT1 GPIO wakeup on reed-switch inputs (rain gauge / anemometer)
 *        so that pulse events are never missed while the MCU is sleeping.
 *  - Report the wakeup cause on the next boot.
 *
 * @note  Both MOSFET gates are driven HIGH (off) by init() so the sensor
 *        rails are **disabled by default** at every boot.  The sleep
 *        orchestration task (vTaskSleep) enables them explicitly before
 *        reading sensors.
 *
 * @par P-channel MOSFET convention
 * Gate LOW  → switch ON  (enable rail)\n
 * Gate HIGH → switch OFF (disable rail)
 */

#include <Arduino.h>
#include <esp_sleep.h>

class PowerManager {
public:
    /** @brief Returns the singleton instance. */
    static PowerManager& getInstance();

    /**
     * @brief Configures MOSFET GPIO pins and disables both sensor rails.
     *
     * Must be called once in setup() before any sensor-dependent code.
     */
    void init();

    /**
     * @brief Enable or disable the 5 V sensor power rail.
     * @param on  `true` = assert gate LOW (rail ON); `false` = gate HIGH (OFF).
     */
    void enable5V(bool on);

    /**
     * @brief Enable or disable the 3.3 V sensor power rail.
     * @param on  `true` = gate LOW (rail ON); `false` = gate HIGH (OFF).
     */
    void enable3V3(bool on);

    /**
     * @brief Disable all rails, configure wakeup sources, enter deep sleep.
     *
     * Wakeup sources configured:
     *  1. **Timer** – wakes after @p seconds.
     *  2. **EXT1** – wakes when PIN_RAIN_GAUGE or PIN_ANEMOMETER goes LOW
     *     (reed switch closes).  Requires external pull-ups on those lines.
     *
     * @param seconds  Deep-sleep duration in seconds.
     * @warning  This function **never returns**.  `Serial.flush()` is called
     *           before entering sleep to ensure the last log lines are sent.
     */
    void enterDeepSleep(uint32_t seconds);

    /** @brief Returns the hardware wakeup cause of the current boot. */
    esp_sleep_wakeup_cause_t getWakeupCause() const;

    /** @brief `true` when the current boot was triggered by the periodic timer. */
    bool isTimerWakeup()  const;

    /** @brief `true` when the current boot was triggered by a reed-switch GPIO. */
    bool isGpioWakeup()   const;

    /** @brief Logs a human-readable wakeup reason to Serial. */
    void printWakeupReason() const;

private:
    PowerManager() = default;
    PowerManager(const PowerManager&) = delete;
    PowerManager& operator=(const PowerManager&) = delete;
};
