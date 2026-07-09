/**
 * @file PowerMonitor.cpp
 * @brief Implementation of the INA3221 power monitoring module.
 */

#include "PowerMonitor.h"

/// Linear interpolation: y at x0 on the segment (x1,y1)-(x2,y2).
static float linInterp(float x1, float x2, float y1, float y2, float x0) {
    return y1 + (x0 - x1) * (y2 - y1) / (x2 - x1);
}

int PowerMonitor::vbToSoc(float vbatMv) {
    // Single-cell LiPo discharge curve: voltage [mV] vs depth of discharge
    // [%] (SOC = 100 - DOD). Measured points, 5% steps.
    static const int16_t kVbat[] = {4187, 4111, 4080, 4047, 3996, 3940, 3891,
                                    3849, 3806, 3759, 3714, 3670, 3632, 3596,
                                    3554, 3501, 3456, 3411, 3309, 3150, 2925};
    static const int16_t kDod[] = {0,  5,  10, 15, 20, 25, 30, 35, 40, 45, 50,
                                   55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
    constexpr size_t kPoints = sizeof(kVbat) / sizeof(kVbat[0]);

    if (vbatMv >= kVbat[0]) {
        return 100;
    }
    if (vbatMv <= kVbat[kPoints - 1]) {
        return 0;
    }
    for (size_t i = 0; i < kPoints - 1; i++) {
        if (kVbat[i] >= vbatMv && vbatMv > kVbat[i + 1]) {
            return 100 - (int)lround(linInterp(kVbat[i], kVbat[i + 1],
                                               kDod[i], kDod[i + 1], vbatMv));
        }
    }
    return 0;  // unreachable, defensive
}

bool PowerMonitor::begin() {
    return _driver.begin();
}

bool PowerMonitor::read(JsonObject &out) {
    if (!_driver.triggerConversion() || !_driver.waitConversion()) {
        return false;
    }

    float vbatMv = _driver.busVoltageMv(kChBattery);
    float ibatMa = _driver.currentMa(kChBattery);
    float ipanMa = _driver.currentMa(kChPanel);
    float iloadMa = _driver.currentMa(kChLoad);

    if (isnan(vbatMv) || isnan(ibatMa) || isnan(ipanMa) || isnan(iloadMa)) {
        return false;
    }
    if (vbatMv < kMinValidVbatMv) {
        log_w("PowerMonitor: battery rail at %.0f mV, readings discarded", vbatMv);
        return false;
    }

    // Sign convention (docs/lora-protocol.md): ibat positive = charging.
    // With this board's wiring the shunt reads discharge as positive,
    // hence the inversion.
    out["vbat"] = round((double)vbatMv / 10.0) / 100.0;  // V, 2 decimals
    out["soc"] = vbToSoc(vbatMv);
    out["ibat"] = (int)lround(-ibatMa);
    out["ipan"] = (int)lround(ipanMa);
    out["iload"] = (int)lround(iloadMa);
    return true;
}
