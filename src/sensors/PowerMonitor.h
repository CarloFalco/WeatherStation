/**
 * @file PowerMonitor.h
 * @brief Energy monitoring via INA3221: panel, battery and load channels.
 *
 * Contributes the power-related telemetry fields:
 *  - "vbat"  battery voltage [V]
 *  - "soc"   battery state of charge [%], from the LiPo discharge curve
 *            (pure logic in src/logic/battery_soc.*, native-tested)
 *  - "ibat"  battery current [mA], positive = charging
 *  - "ipan"  solar panel current [mA]
 *  - "iload" load (ESP + sensors) current [mA]
 *
 * Channel mapping (fixed by the board wiring, verified on hardware during
 * the Increment 6 validation — the old prototype comment had panel and
 * load swapped):
 *  CH1 = load, CH2 = battery, CH3 = solar panel.
 *
 * Each cycle runs one single-shot conversion (the INA3221 sits in
 * power-down the rest of the time). The shunt resistance is configurable
 * in config.ini ([power] shunt_mohm).
 */

#ifndef WEATHERSTATION_POWERMONITOR_H
#define WEATHERSTATION_POWERMONITOR_H

#include "ISensor.h"
#include "Ina3221Driver.h"
#include "config.h"

/**
 * @brief ISensor implementation for the INA3221 power monitor.
 */
class PowerMonitor : public ISensor {
public:
    const char *name() const override { return "PowerMonitor"; }

    /**
     * @brief Set the shunt resistance before begin().
     * @param shuntMohm Shunt value [milliohm] (from config.ini).
     */
    void configure(uint32_t shuntMohm) { _driver.setShuntResMohm(shuntMohm); }

    /**
     * @brief Probe the INA3221 on the I2C bus.
     * @return true if the chip answered with the TI manufacturer ID.
     */
    bool begin() override;

    /**
     * @brief Run one conversion and publish the power fields.
     * @param out Root JSON object of the telemetry message.
     * @return true if the conversion completed and readings are valid.
     */
    bool read(JsonObject &out) override;

private:
    /// INA3221 channel roles (board wiring, hardware-verified).
    static constexpr uint8_t kChLoad = 0;
    static constexpr uint8_t kChBattery = 1;
    static constexpr uint8_t kChPanel = 2;

    /// Below this bus voltage [mV] the battery channel reading is treated
    /// as invalid (sensor disconnected or rail down).
    static constexpr float kMinValidVbatMv = 100.0f;

    Ina3221Driver _driver{INA3221_ADDRESS};  ///< Register-level driver.
};

#endif // WEATHERSTATION_POWERMONITOR_H
