/**
 * @file Anemometer.cpp
 * @brief Implementation of the cup anemometer module.
 */

#include "Anemometer.h"

#include "config.h"

/// Pulses counted in the current 1-second bin (written by the ISR).
static volatile uint32_t s_binPulses = 0;
/// Timestamp of the last accepted pulse, for the debounce lockout.
static volatile uint32_t s_lastPulseMs = 0;

void IRAM_ATTR Anemometer::onPulseIsr() {
    uint32_t now = millis();
    if (now - s_lastPulseMs < kDebounceMs) {
        return;  // reed switch bounce, ignore
    }
    s_lastPulseMs = now;
    s_binPulses++;
}

bool Anemometer::begin() {
    pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), onPulseIsr, FALLING);
    return true;
}

bool Anemometer::read(JsonObject &out) {
    uint32_t total = 0;
    uint32_t maxBin = 0;

    // Count pulses in 1-second bins: the average gives "ws", the busiest
    // bin gives the gust "wg".
    for (uint8_t i = 0; i < _windowS; i++) {
        s_binPulses = 0;
        delay(1000);
        uint32_t bin = s_binPulses;
        total += bin;
        maxBin = max(maxBin, bin);
    }

    double avgHz = (double)total / (double)_windowS;
    double ws = avgHz * (double)_mpsPerHz;
    double wg = (double)maxBin * (double)_mpsPerHz;

    log_d("wind: %lu pulses in %u s -> avg %.1f m/s (%.1f km/h), gust %.1f m/s",
          (unsigned long)total, _windowS, ws, ws * 3.6, wg);

    out["ws"] = round(ws * 10.0) / 10.0;
    out["wg"] = round(wg * 10.0) / 10.0;
    return true;
}
