/**
 * @file Ina3221Driver.cpp
 * @brief Implementation of the minimal INA3221 driver.
 */

#include "Ina3221Driver.h"

constexpr uint8_t Ina3221Driver::kRegShuntV[3];
constexpr uint8_t Ina3221Driver::kRegBusV[3];

bool Ina3221Driver::readReg(uint8_t reg, uint16_t &val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {  // repeated start
        return false;
    }
    if (Wire.requestFrom(_addr, (uint8_t)2) != 2) {
        return false;
    }
    val = ((uint16_t)Wire.read() << 8) | Wire.read();
    return true;
}

bool Ina3221Driver::writeReg(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((val >> 8) & 0xFF);
    Wire.write(val & 0xFF);
    return Wire.endTransmission() == 0;
}

bool Ina3221Driver::begin() {
    uint16_t id = 0;
    if (!readReg(kRegManufId, id)) {
        return false;
    }
    if (id != kManufIdTi) {
        log_w("INA3221: unexpected manufacturer ID 0x%04X", id);
        return false;
    }
    return true;
}

bool Ina3221Driver::triggerConversion() {
    return writeReg(kRegConf, kConfSingleShot);
}

bool Ina3221Driver::waitConversion(uint32_t timeoutMs) {
    uint32_t start = millis();
    uint16_t mask = 0;
    while (millis() - start < timeoutMs) {
        if (readReg(kRegMaskEnable, mask) && (mask & kMaskConvReady)) {
            return true;
        }
        delay(5);
    }
    log_w("INA3221: conversion-ready timeout");
    return false;
}

float Ina3221Driver::busVoltageMv(uint8_t channel) {
    uint16_t raw = 0;
    if (channel >= 3 || !readReg(kRegBusV[channel], raw)) {
        return NAN;
    }
    // 13-bit value in bits 15..3, LSB = 8 mV: (raw >> 3) * 8 == raw with
    // the low bits masked off.
    return (float)((int16_t)raw / 8 * 8);
}

float Ina3221Driver::currentMa(uint8_t channel) {
    uint16_t raw = 0;
    if (channel >= 3 || !readReg(kRegShuntV[channel], raw)) {
        return NAN;
    }
    // Signed 13-bit value in bits 15..3, LSB = 40 uV. Dividing the int16_t
    // by 8 (not shifting the unsigned raw) preserves the sign.
    int32_t shuntUv = (int32_t)((int16_t)raw / 8) * 40;
    // I [mA] = V [uV] / R [mOhm]  (1e-6 / 1e-3 = 1e-3, i.e. mA directly)
    return (float)shuntUv / (float)_shuntMohm;
}
