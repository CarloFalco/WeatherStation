/**
 * @file WindVane.h
 * @brief Wind direction vane based on the AS5600 magnetic encoder (I2C).
 *
 * Contributes the "wd" field: wind direction in degrees, 0-359, where
 * 0 = North and angles grow clockwise (90 = East). The AS5600 reads the
 * absolute angle of the magnet on the vane shaft; since the sensor can
 * be mounted in any orientation, a configurable offset ([wind]
 * vane_offset_deg in config.ini) maps the raw angle to true North:
 *
 *     wd = (raw_angle_deg + vane_offset_deg) mod 360
 *
 * Field calibration: point the vane North and set vane_offset_deg to
 * (360 - raw) using the raw angle logged on serial.
 *
 * The register protocol is trivial (two-byte reads), so the chip is
 * driven directly over Wire without an external library.
 */

#ifndef WEATHERSTATION_WINDVANE_H
#define WEATHERSTATION_WINDVANE_H

#include "ISensor.h"

/**
 * @brief ISensor implementation for the AS5600 wind vane.
 */
class WindVane : public ISensor {
public:
    const char *name() const override { return "WindVane"; }

    /**
     * @brief Set the mounting offset before begin().
     * @param offsetDeg Degrees added to the raw angle to align 0 to North.
     */
    void configure(int16_t offsetDeg) { _offsetDeg = offsetDeg; }

    /**
     * @brief Probe the AS5600, verify the magnet, enter low-power mode.
     *
     * The AS5600 has no enable pin and stays powered through deep sleep:
     * in its default mode it draws ~6.5 mA continuously, which alone
     * would drain the battery in weeks. begin() switches it to LPM3
     * (~1.5 mA, 10 ms internal sampling — more than enough for wind).
     * See docs/power-budget.md for why a hardware load switch on the
     * sensor rail is still the proper long-term fix.
     *
     * @return true if the chip answers and the MD status bit is set.
     */
    bool begin() override;

    /**
     * @brief Read the vane angle and publish "wd" [deg, 0-359].
     * @param out Root JSON object of the telemetry message.
     * @return true if the angle was read successfully.
     */
    bool read(JsonObject &out) override;

private:
    /**
     * @brief Read consecutive AS5600 registers.
     * @param reg First register address.
     * @param buf Destination buffer.
     * @param len Number of bytes to read.
     * @return true if the I2C transaction succeeded.
     */
    bool readRegisters(uint8_t reg, uint8_t *buf, size_t len);

    static constexpr uint8_t kRegConf = 0x07;      ///< CONF register, 2 bytes.
    static constexpr uint8_t kRegStatus = 0x0B;    ///< Magnet status register.
    static constexpr uint8_t kRegRawAngle = 0x0C;  ///< Raw angle, 12 bit, 2 bytes.
    static constexpr uint8_t kStatusMagnetDetected = 0x20;  ///< MD bit mask.
    static constexpr uint8_t kConfPowerModeMask = 0x03;     ///< PM bits (CONF low byte).
    static constexpr uint8_t kConfPowerModeLpm3 = 0x03;     ///< LPM3: ~1.5 mA, 10 ms polling.

    int16_t _offsetDeg = 0;  ///< Mounting offset [deg], from config.
};

#endif // WEATHERSTATION_WINDVANE_H
