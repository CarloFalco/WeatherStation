/**
 * @file RainGauge.h
 * @brief Tipping-bucket rain gauge (reed switch, pulse counting).
 *
 * Contributes the "rain" [mm] field: rainfall accumulated since the last
 * transmission. Pulses are counted in two ways:
 *  - while awake: GPIO interrupt with software debounce;
 *  - while in deep sleep: the pulse itself wakes the MCU (EXT0), main.cpp
 *    increments the counter through countSleepPulse() and goes straight
 *    back to sleep (quick path, no radio, no sensors).
 *
 * The counter lives in RTC RAM (RtcState::rainPulses) so no rain is lost
 * across deep-sleep cycles. Calibration (mm per bucket tip) comes from
 * config.ini: [rain] mm_per_pulse.
 */

#ifndef WEATHERSTATION_RAINGAUGE_H
#define WEATHERSTATION_RAINGAUGE_H

#include "ISensor.h"

/**
 * @brief ISensor implementation for the tipping-bucket rain gauge.
 */
class RainGauge : public ISensor {
public:
    const char *name() const override { return "RainGauge"; }

    /**
     * @brief Set the calibration factor before begin().
     * @param mmPerPulse Rainfall per bucket tip [mm] (from config.ini).
     */
    void configure(float mmPerPulse) { _mmPerPulse = mmPerPulse; }

    /**
     * @brief Arm the pulse interrupt (input with pull-up, falling edge).
     * @return Always true: a pulse counter has no probe to fail.
     */
    bool begin() override;

    /**
     * @brief Publish the accumulated rainfall as "rain" [mm].
     * @param out Root JSON object of the telemetry message.
     * @return Always true.
     */
    bool read(JsonObject &out) override;

    /**
     * @brief Consume the pulses that were reported by the last read().
     *
     * Call after a confirmed delivery (base ACK received). Subtracts the
     * snapshot taken by read() instead of zeroing the counter, so bucket
     * tips that occurred during the transmission/ACK window are carried
     * over to the next cycle instead of being lost.
     */
    void resetAccumulator();

    /**
     * @brief Account for the single pulse that woke the MCU from deep sleep.
     *
     * Called by main.cpp on an EXT0 wake-up, where no interrupt was armed.
     */
    static void countSleepPulse();

private:
    /// Awake-time ISR: debounced increment of the RTC pulse counter.
    static void IRAM_ATTR onPulseIsr();

    static constexpr uint32_t kDebounceMs = 150;  ///< Reed-bounce lockout.

    float _mmPerPulse = 0.4743f;  ///< Calibration [mm/tip], overridden by config.
    uint32_t _reportedPulses = 0; ///< Snapshot published by the last read().
};

#endif // WEATHERSTATION_RAINGAUGE_H
