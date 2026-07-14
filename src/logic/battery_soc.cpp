/**
 * @file battery_soc.cpp
 * @brief Implementation of the LiPo discharge-curve SoC estimation.
 */

#include "battery_soc.h"

#include <cmath>
#include <cstdint>

namespace logic {

/// Linear interpolation: y at x0 on the segment (x1,y1)-(x2,y2).
static float linInterp(float x1, float x2, float y1, float y2, float x0) {
    return y1 + (x0 - x1) * (y2 - y1) / (x2 - x1);
}

int vbatToSoc(float vbatMv) {
    // Single-cell LiPo discharge curve: voltage [mV] vs depth of discharge
    // [%] (SOC = 100 - DOD). Field-measured points, 5% steps.
    static const int16_t kVbat[] = {4187, 4111, 4080, 4047, 3996, 3940, 3891,
                                    3849, 3806, 3759, 3714, 3670, 3632, 3596,
                                    3554, 3501, 3456, 3411, 3309, 3150, 2925};
    static const int16_t kDod[] = {0,  5,  10, 15, 20, 25, 30, 35, 40, 45, 50,
                                   55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
    constexpr std::size_t kPoints = sizeof(kVbat) / sizeof(kVbat[0]);

    if (vbatMv >= kVbat[0]) {
        return 100;
    }
    if (vbatMv <= kVbat[kPoints - 1]) {
        return 0;
    }
    for (std::size_t i = 0; i < kPoints - 1; i++) {
        if (kVbat[i] >= vbatMv && vbatMv > kVbat[i + 1]) {
            return 100 - (int)std::lround(linInterp(kVbat[i], kVbat[i + 1],
                                                    kDod[i], kDod[i + 1], vbatMv));
        }
    }
    return 0;  // unreachable, defensive
}

}  // namespace logic
