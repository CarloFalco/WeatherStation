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
    _reportedPulses = g_rtcState.rainPulses;

    // Round to 2 decimals in double math to avoid float serialization
    // artifacts (see Bme280Sensor).
    double mm = (double)_reportedPulses * (double)_mmPerPulse;

    log_d("rain: %lu pulses -> %.2f mm", (unsigned long)_reportedPulses, mm);

    out["rain"] = round(mm * 100.0) / 100.0;
    return true;
}

void RainGauge::resetAccumulator() {
    // Subtract the reported snapshot under a brief interrupt lock: the ISR
    // may increment the counter concurrently and read-modify-write is not
    // atomic. Tips landed after read() stay in the accumulator.
    noInterrupts();
    g_rtcState.rainPulses -= _reportedPulses;
    interrupts();
    _reportedPulses = 0;
}

void RainGauge::countSleepPulse() {
    // The wake-up pulse is a single physical tip: no debounce needed,
    // the boot itself (hundreds of ms) already acts as a lockout.
    g_rtcState.rainPulses++;
    s_lastPulseMs = millis();
}
