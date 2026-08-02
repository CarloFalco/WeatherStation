/**
 * @file soil_moisture.cpp
 * @brief Implementation of the capacitive soil moisture conversion.
 */

#include "soil_moisture.h"

namespace logic {

int soilRawToPercent(uint16_t raw, uint16_t dryRaw, uint16_t wetRaw) {
    // A capacitive probe always reads higher in air than in water: an
    // inverted (or zero-span) calibration means the values were swapped or
    // never measured, and silently producing a number would be worse than
    // reporting nothing.
    if (dryRaw <= wetRaw) {
        return kSoilInvalid;
    }

    if (raw >= dryRaw) {
        return 0;
    }
    if (raw <= wetRaw) {
        return 100;
    }

    // Integer math with rounding: percent = (dry - raw) / (dry - wet) * 100
    uint32_t span = (uint32_t)(dryRaw - wetRaw);
    uint32_t above = (uint32_t)(dryRaw - raw);
    return (int)((above * 100u + span / 2u) / span);
}

}  // namespace logic
