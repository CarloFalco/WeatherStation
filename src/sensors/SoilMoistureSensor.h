/**
 * @file SoilMoistureSensor.h
 * @brief Capacitive soil moisture probe on an analog input.
 *
 * Contributes the "soil" field: soil moisture in percent, 0 = bone dry,
 * 100 = saturated. The raw-to-percent conversion lives in
 * src/logic/soil_moisture.* (pure logic, native-tested); this class only
 * does the I/O and the plausibility check.
 *
 * Calibration comes from config.ini ([soil] dry_raw / wet_raw): measure
 * the probe in air and in a glass of water, the raw counts are printed at
 * debug level on every cycle.
 */

#ifndef WEATHERSTATION_SOILMOISTURESENSOR_H
#define WEATHERSTATION_SOILMOISTURESENSOR_H

#include "ISensor.h"

/**
 * @brief ISensor implementation for the capacitive soil moisture probe.
 */
class SoilMoistureSensor : public ISensor {
public:
    const char *name() const override { return "SoilMoisture"; }

    /**
     * @brief Apply the calibration before begin().
     * @param dryRaw ADC counts with the probe in air.
     * @param wetRaw ADC counts with the probe in water.
     * @param samples Readings averaged per measurement.
     */
    void configure(uint16_t dryRaw, uint16_t wetRaw, uint8_t samples) {
        _dryRaw = dryRaw;
        _wetRaw = wetRaw;
        _samples = samples;
    }

    /**
     * @brief Configure the ADC input (12 dB attenuation, full 0-3.1 V span).
     * @return Always true: a plain analog input cannot be probed.
     */
    bool begin() override;

    /**
     * @brief Average the ADC and publish "soil" [%].
     * @param out Root JSON object of the telemetry message.
     * @return false if the probe looks disconnected or the calibration is
     *         invalid — the field is then absent, never invented.
     */
    bool read(JsonObject &out) override;

private:
    /// A floating input rails to one extreme: readings outside this window
    /// mean no probe connected (or a wiring fault), not dry/wet soil.
    static constexpr uint16_t kPlausibleMin = 50;
    static constexpr uint16_t kPlausibleMax = 4045;

    uint16_t _dryRaw = 3000;  ///< Calibration in air, from config.
    uint16_t _wetRaw = 1300;  ///< Calibration in water, from config.
    uint8_t _samples = 8;     ///< Readings averaged per measurement.
};

#endif // WEATHERSTATION_SOILMOISTURESENSOR_H
