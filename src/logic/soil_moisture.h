/**
 * @file soil_moisture.h
 * @brief Capacitive soil moisture conversion — pure logic, native-tested.
 *
 * Capacitive probes output an INVERSE analog signal: the drier the soil,
 * the higher the voltage. The conversion therefore needs two calibration
 * points measured on the actual probe (see docs/pinout.md):
 *  - `dryRaw`: probe in air;
 *  - `wetRaw`: probe in a glass of water (must be lower than dryRaw).
 */

#ifndef WEATHERSTATION_LOGIC_SOIL_MOISTURE_H
#define WEATHERSTATION_LOGIC_SOIL_MOISTURE_H

#include <cstdint>

namespace logic {

/** @brief Returned when the calibration is inconsistent. */
constexpr int kSoilInvalid = -1;

/**
 * @brief Convert a raw ADC reading into a moisture percentage.
 *
 * @param raw Averaged ADC counts from the probe.
 * @param dryRaw Calibration point measured in air (higher value).
 * @param wetRaw Calibration point measured in water (lower value).
 * @return Moisture 0..100 % (0 = bone dry, 100 = saturated), or
 *         @ref kSoilInvalid if the calibration points are not usable.
 */
int soilRawToPercent(uint16_t raw, uint16_t dryRaw, uint16_t wetRaw);

}  // namespace logic

#endif // WEATHERSTATION_LOGIC_SOIL_MOISTURE_H
