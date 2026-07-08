/**
 * @file ISensor.h
 * @brief Common interface implemented by every sensor of the station.
 *
 * This is the extension point of the firmware: adding a new sensor means
 * writing one class that implements ISensor and registering it in the
 * SensorManager — main.cpp, the JSON payload assembly and the LoRa link
 * are not touched.
 *
 * Contract:
 *  - begin() is called once per wake-up cycle, before any read();
 *  - read() writes the sensor's own fields into the telemetry JSON object
 *    (see docs/lora-protocol.md for field names and units);
 *  - on failure the sensor simply does not add its fields: per protocol,
 *    absent fields mean "sensor unavailable" and the receiver must cope.
 */

#ifndef WEATHERSTATION_ISENSOR_H
#define WEATHERSTATION_ISENSOR_H

#include <ArduinoJson.h>

/**
 * @brief Abstract base class for all station sensors.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    /** @return Short sensor name used in log messages, e.g. "BME280". */
    virtual const char *name() const = 0;

    /**
     * @brief Initialize the sensor hardware.
     * @return true if the sensor responded and is usable this cycle.
     */
    virtual bool begin() = 0;

    /**
     * @brief Perform a measurement and write the results into @p out.
     * @param out Root JSON object of the telemetry message.
     * @return true if valid values were produced and added.
     */
    virtual bool read(JsonObject &out) = 0;
};

#endif // WEATHERSTATION_ISENSOR_H
