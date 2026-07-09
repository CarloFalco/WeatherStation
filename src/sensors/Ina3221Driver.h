/**
 * @file Ina3221Driver.h
 * @brief Minimal register-level driver for the TI INA3221 current monitor.
 *
 * Derived from the Beast Devices INA3221 Arduino library
 * (MIT License, Copyright (c) 2020 Beast Devices, Andrejs Bondarevs),
 * stripped down and adapted for this project:
 *  - single-shot (triggered) conversions instead of continuous mode: the
 *    chip powers itself down after each conversion set (~0.35 mA saved
 *    between measurement cycles, which matters on a >12-month battery
 *    budget);
 *  - SIGNED shunt voltage reading: the INA3221 shunt ADC is bidirectional
 *    (13-bit two's complement) so charge/discharge current comes out with
 *    its real sign — the original code read it unsigned and needed
 *    workarounds downstream;
 *  - everything not used by the station (alerts, limits, sum channels,
 *    filter-resistor offset estimation) removed.
 */

#ifndef WEATHERSTATION_INA3221DRIVER_H
#define WEATHERSTATION_INA3221DRIVER_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Register-level access to the INA3221 (3-channel bus/shunt monitor).
 */
class Ina3221Driver {
public:
    /**
     * @param i2cAddr 7-bit I2C address (0x40 with A0 to GND).
     */
    explicit Ina3221Driver(uint8_t i2cAddr) : _addr(i2cAddr) {}

    /**
     * @brief Probe the chip on the bus.
     *
     * The global Wire bus must already be initialized.
     *
     * @return true if the manufacturer ID register reads 0x5449 ("TI").
     */
    bool begin();

    /**
     * @brief Set the shunt resistor value used for current conversion.
     * @param mohm Shunt resistance [milliohm], same on all three channels.
     */
    void setShuntResMohm(uint32_t mohm) { _shuntMohm = mohm; }

    /**
     * @brief Start one single-shot conversion of all three channels.
     *
     * Configuration: 16-sample averaging, 1.1 ms conversion time for both
     * bus and shunt (~106 ms total). The chip returns to power-down by
     * itself when the conversion set completes.
     */
    bool triggerConversion();

    /**
     * @brief Block until the conversion-ready flag is set.
     * @param timeoutMs Give-up timeout [ms].
     * @return true if the conversion completed within the timeout.
     */
    bool waitConversion(uint32_t timeoutMs = 500);

    /**
     * @brief Bus (load-side) voltage of a channel.
     * @param channel Channel index 0..2.
     * @return Voltage [mV] (8 mV LSB), or NAN on I2C error.
     */
    float busVoltageMv(uint8_t channel);

    /**
     * @brief Signed current through a channel shunt.
     *
     * Positive when conventional current flows from IN+ to IN-.
     *
     * @param channel Channel index 0..2.
     * @return Current [mA], or NAN on I2C error.
     */
    float currentMa(uint8_t channel);

private:
    bool readReg(uint8_t reg, uint16_t &val);
    bool writeReg(uint8_t reg, uint16_t val);

    // Register map (subset).
    static constexpr uint8_t kRegConf = 0x00;
    static constexpr uint8_t kRegShuntV[3] = {0x01, 0x03, 0x05};
    static constexpr uint8_t kRegBusV[3] = {0x02, 0x04, 0x06};
    static constexpr uint8_t kRegMaskEnable = 0x0F;
    static constexpr uint8_t kRegManufId = 0xFE;

    static constexpr uint16_t kManufIdTi = 0x5449;  ///< "TI" in ASCII.
    /// All channels enabled, AVG=16, CT bus/shunt=1.1 ms, mode=single-shot.
    static constexpr uint16_t kConfSingleShot = 0x7523;
    static constexpr uint16_t kMaskConvReady = 0x0001;  ///< CVRF flag.

    uint8_t _addr;              ///< 7-bit I2C address.
    uint32_t _shuntMohm = 10;   ///< Shunt resistance [milliohm].
};

#endif // WEATHERSTATION_INA3221DRIVER_H
