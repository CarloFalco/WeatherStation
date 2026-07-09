/**
 * @file Anemometer.h
 * @brief Cup anemometer with reed switch (pulse counting over a window).
 *
 * Contributes the "ws" (average wind speed) and "wg" (gust) fields, both
 * in m/s. Unlike the rain gauge, wind pulses are NOT counted during deep
 * sleep (that would require staying awake and would kill the battery):
 * the wind is sampled during a short blocking window in the measurement
 * cycle, counting pulses in 1-second bins:
 *  - ws = average frequency over the whole window x calibration factor;
 *  - wg = highest 1-second bin x calibration factor.
 *
 * Calibration comes from config.ini: [wind] mps_per_hz (the common cup
 * anemometer datasheet convention "1 Hz = 2.4 km/h" gives 0.667 m/s/Hz)
 * and sample_window_s.
 */

#ifndef WEATHERSTATION_ANEMOMETER_H
#define WEATHERSTATION_ANEMOMETER_H

#include "ISensor.h"

/**
 * @brief ISensor implementation for the reed-switch cup anemometer.
 */
class Anemometer : public ISensor {
public:
    const char *name() const override { return "Anemometer"; }

    /**
     * @brief Set calibration and sampling window before begin().
     * @param mpsPerHz Wind speed per pulse frequency [m/s per Hz].
     * @param windowS  Sampling window length [s].
     */
    void configure(float mpsPerHz, uint8_t windowS) {
        _mpsPerHz = mpsPerHz;
        _windowS = windowS;
    }

    /**
     * @brief Arm the pulse interrupt (input with pull-up, falling edge).
     * @return Always true: a pulse counter has no probe to fail.
     */
    bool begin() override;

    /**
     * @brief Sample wind pulses for the configured window (blocking).
     *
     * Blocks for sample_window_s seconds, then publishes "ws" and "wg".
     *
     * @param out Root JSON object of the telemetry message.
     * @return Always true.
     */
    bool read(JsonObject &out) override;

private:
    /// Pulse ISR: debounced increment of the current-bin counter.
    static void IRAM_ATTR onPulseIsr();

    /// Reed-bounce lockout. At 5 ms the measurable ceiling is 100 Hz
    /// (~67 m/s with the default factor): far above any real storm.
    static constexpr uint32_t kDebounceMs = 5;

    float _mpsPerHz = 0.667f;  ///< Calibration [m/s per Hz], from config.
    uint8_t _windowS = 5;      ///< Sampling window [s], from config.
};

#endif // WEATHERSTATION_ANEMOMETER_H
