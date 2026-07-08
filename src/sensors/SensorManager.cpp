/**
 * @file SensorManager.cpp
 * @brief Implementation of the sensor registry.
 */

#include "SensorManager.h"

bool SensorManager::add(ISensor *sensor) {
    if (_count >= kMaxSensors) {
        log_e("Sensor registry full, cannot add %s", sensor->name());
        return false;
    }
    _sensors[_count++] = sensor;
    return true;
}

size_t SensorManager::beginAll() {
    size_t ok = 0;
    for (size_t i = 0; i < _count; i++) {
        _healthy[i] = _sensors[i]->begin();
        if (_healthy[i]) {
            ok++;
        } else {
            log_w("Sensor %s failed to initialize, skipping this cycle",
                  _sensors[i]->name());
        }
    }
    return ok;
}

size_t SensorManager::readAll(JsonObject &out) {
    size_t ok = 0;
    for (size_t i = 0; i < _count; i++) {
        if (!_healthy[i]) {
            continue;
        }
        if (_sensors[i]->read(out)) {
            ok++;
        } else {
            log_w("Sensor %s returned no valid reading", _sensors[i]->name());
        }
    }
    return ok;
}
