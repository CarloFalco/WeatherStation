/**
 * @file Bme280Sensor.h
 * @brief Temperature / humidity / pressure sensor (Bosch BME280, I2C).
 *
 * Contributes the "t" [°C], "rh" [%] and "p" [hPa] fields to the
 * telemetry JSON. The sensor is driven in forced mode with 1x
 * oversampling and no IIR filter — the Bosch-recommended "weather
 * monitoring" profile: one conversion on demand, then the sensor goes
 * back to sleep (~0.1 µA), which fits the station duty cycle perfectly.
 */

#ifndef WEATHERSTATION_BME280SENSOR_H
#define WEATHERSTATION_BME280SENSOR_H

#include <Adafruit_BME280.h>

#include "ISensor.h"

/**
 * @brief ISensor implementation for the Bosch BME280 (I2C address 0x76).
 */
class Bme280Sensor : public ISensor {
public:
    const char *name() const override { return "BME280"; }

    /**
     * @brief Probe the sensor on the I2C bus and apply the low-power profile.
     *
     * The global Wire bus must already be initialized (done in main.cpp).
     *
     * @return true if the BME280 answered at BME280_I2C_ADDR.
     */
    bool begin() override;

    /**
     * @brief Trigger one forced measurement and publish t / rh / p.
     * @param out Root JSON object of the telemetry message.
     * @return true if the readings are valid (not NaN).
     */
    bool read(JsonObject &out) override;

private:
    Adafruit_BME280 _bme;  ///< Adafruit driver instance.
};

#endif // WEATHERSTATION_BME280SENSOR_H
