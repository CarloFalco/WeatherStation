/**
 * @file RainGauge.cpp
 * @brief Implementation of the tipping-bucket rain gauge module.
 */

#include "RainGauge.h"

#include "config.h"
#include "rtc_state.h"

/// Timestamp of the last accepted pulse, for the debounce lockout.
static volatile uint32_t s_lastPulseMs = 0;

void IRAM_ATTR RainGauge::onPulseIsr() {
    uint32_t now = millis();
    if (now - s_lastPulseMs < kDebounceMs) {
        return;  // reed switch bounce, ignore
    }
    s_lastPulseMs = now;
    g_rtcState.rainPulses++;
}

bool RainGauge::begin() {
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), onPulseIsr, FALLING);
    return true;
}

bool RainGauge::read(JsonObject &out) {
    // Round to 2 decimals in double math to avoid float serialization
    // artifacts (see Bme280Sensor).
    double mm = (double)g_rtcState.rainPulses * (double)_mmPerPulse;
    out["rain"] = round(mm * 100.0) / 100.0;
    return true;
}

void RainGauge::resetAccumulator() {
    g_rtcState.rainPulses = 0;
}

void RainGauge::countSleepPulse() {
    // The wake-up pulse is a single physical tip: no debounce needed,
    // the boot itself (hundreds of ms) already acts as a lockout.
    g_rtcState.rainPulses++;
    s_lastPulseMs = millis();
}
