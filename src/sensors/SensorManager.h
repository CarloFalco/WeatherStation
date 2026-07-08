/**
 * @file SensorManager.h
 * @brief Registry that initializes all sensors and collects their readings.
 */

#ifndef WEATHERSTATION_SENSORMANAGER_H
#define WEATHERSTATION_SENSORMANAGER_H

#include "ISensor.h"

/**
 * @brief Owns the list of registered sensors and drives their life cycle.
 *
 * Sensors are registered once with add(); every wake-up cycle main.cpp
 * calls beginAll() and then readAll(). A sensor whose begin() fails is
 * marked unhealthy and skipped for the rest of the cycle, so one broken
 * sensor never blocks the others: its JSON fields are simply absent.
 */
class SensorManager {
public:
    /**
     * @brief Register a sensor. Call once per sensor at startup.
     * @param sensor Sensor instance (not owned; must outlive the manager).
     * @return true on success, false if the registry is full.
     */
    bool add(ISensor *sensor);

    /**
     * @brief Initialize every registered sensor, marking failures unhealthy.
     * @return Number of sensors that initialized correctly.
     */
    size_t beginAll();

    /**
     * @brief Collect readings from all healthy sensors into @p out.
     * @param out Root JSON object of the telemetry message.
     * @return Number of sensors that produced valid readings.
     */
    size_t readAll(JsonObject &out);

private:
    static constexpr size_t kMaxSensors = 8;  ///< Registry capacity.

    ISensor *_sensors[kMaxSensors] = {};  ///< Registered sensors.
    bool _healthy[kMaxSensors] = {};      ///< begin() outcome per sensor.
    size_t _count = 0;                    ///< Number of registered sensors.
};

#endif // WEATHERSTATION_SENSORMANAGER_H
