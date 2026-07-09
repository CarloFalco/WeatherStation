/**
 * @file WindVane.cpp
 * @brief Implementation of the AS5600 wind vane module.
 */

#include "WindVane.h"

#include <Wire.h>

#include "config.h"

bool WindVane::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {  // repeated start
        return false;
    }
    if (Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)len) != len) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool WindVane::begin() {
    uint8_t status = 0;
    if (!readRegisters(kRegStatus, &status, 1)) {
        return false;  // chip not answering on the bus
    }
    if (!(status & kStatusMagnetDetected)) {
        log_w("AS5600: no magnet detected (status=0x%02X)", status);
        return false;
    }
    return true;
}

bool WindVane::read(JsonObject &out) {
    uint8_t raw[2];
    if (!readRegisters(kRegRawAngle, raw, 2)) {
        return false;
    }

    uint16_t counts = ((uint16_t)(raw[0] & 0x0F) << 8) | raw[1];  // 12 bit
    double rawDeg = (double)counts * (360.0 / 4096.0);

    int wd = ((int)lround(rawDeg) + _offsetDeg) % 360;
    if (wd < 0) {
        wd += 360;
    }

    // 16-point compass rose, 22.5 deg per sector, N centered on 0.
    static const char *kCompass[16] = {"N", "NNE", "NE", "ENE", "E", "ESE",
                                       "SE", "SSE", "S", "SSW", "SW", "WSW",
                                       "W", "WNW", "NW", "NNW"};
    log_d("AS5600 raw %.1f deg -> wd %d deg (%s)", rawDeg, wd,
          kCompass[((wd * 2 + 22) / 45) % 16]);

    out["wd"] = wd;
    return true;
}
