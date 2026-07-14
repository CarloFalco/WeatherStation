/**
 * @file PowerMonitor.cpp
 * @brief Implementation of the INA3221 power monitoring module.
 */

#include "PowerMonitor.h"

#include "logic/battery_soc.h"

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

    // Debug dump of the raw channel readings (serial monitor only, stripped
    // at compile time when CORE_DEBUG_LEVEL < 4). Panel and load voltages
    // are read inside the macro so they cost nothing in production builds.
    log_d("panel  : %6.3f V  %8.1f mA", _driver.busVoltageMv(kChPanel) / 1000.0f, ipanMa);
    log_d("battery: %6.3f V  %8.1f mA  (soc %d%%)", vbatMv / 1000.0f, ibatMa,
          logic::vbatToSoc(vbatMv));
    log_d("load   : %6.3f V  %8.1f mA", _driver.busVoltageMv(kChLoad) / 1000.0f, iloadMa);

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
    out["soc"] = logic::vbatToSoc(vbatMv);
    out["ibat"] = (int)lround(-ibatMa);
    out["ipan"] = (int)lround(ipanMa);
    out["iload"] = (int)lround(iloadMa);
    return true;
}
