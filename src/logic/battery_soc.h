/**
 * @file battery_soc.h
 * @brief Battery voltage to state-of-charge conversion (pure logic).
 *
 * Pure C++ with no Arduino/ESP-IDF dependencies: this module compiles on
 * the host and is covered by native unit tests (test/test_battery_soc,
 * run with `pio test -e native`).
 */

#ifndef WEATHERSTATION_LOGIC_BATTERY_SOC_H
#define WEATHERSTATION_LOGIC_BATTERY_SOC_H

namespace logic {

/**
 * @brief Battery voltage to state of charge, by linear interpolation of a
 *        single-cell LiPo discharge curve (21 measured points, 5% steps).
 * @param vbatMv Battery voltage [mV].
 * @return State of charge [%], clamped to 0..100.
 */
int vbatToSoc(float vbatMv);

}  // namespace logic

#endif // WEATHERSTATION_LOGIC_BATTERY_SOC_H
