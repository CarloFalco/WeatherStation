/**
 * @file SoilMoistureSensor.cpp
 * @brief Implementation of the capacitive soil moisture probe module.
 */

#include "SoilMoistureSensor.h"

#include "config.h"
#include "logic/soil_moisture.h"

bool SoilMoistureSensor::begin() {
    // 12 dB attenuation = full 0-3.1 V input span (the probe swings close
    // to the rail when dry). 12-bit resolution is the Arduino default.
    analogSetPinAttenuation(SOIL_MOISTURE_PIN, ADC_11db);
    return true;
}

bool SoilMoistureSensor::read(JsonObject &out) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < _samples; i++) {
        sum += analogRead(SOIL_MOISTURE_PIN);
        delay(2);  // let the ADC sample-and-hold settle between readings
    }
    uint16_t raw = (uint16_t)(sum / _samples);

    int percent = logic::soilRawToPercent(raw, _dryRaw, _wetRaw);
    log_d("soil: raw %u (dry %u, wet %u) -> %d %%", raw, _dryRaw, _wetRaw, percent);

    if (raw < kPlausibleMin || raw > kPlausibleMax) {
        log_w("SoilMoisture: raw %u out of range, probe disconnected?", raw);
        return false;
    }
    if (percent == logic::kSoilInvalid) {
        log_w("SoilMoisture: invalid calibration (dry_raw must exceed wet_raw)");
        return false;
    }

    out["soil"] = percent;
    return true;
}
