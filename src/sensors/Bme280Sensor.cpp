/**
 * @file Bme280Sensor.cpp
 * @brief Implementation of the BME280 sensor module.
 */

#include "Bme280Sensor.h"

#include "config.h"

/// Round to one decimal in double math so ArduinoJson serializes "21.4",
/// not the float artifact "21.39999962".
static double round1(float value) {
    return round((double)value * 10.0) / 10.0;
}

bool Bme280Sensor::begin() {
    if (!_bme.begin(BME280_I2C_ADDR, &Wire)) {
        return false;
    }

    // Bosch "weather monitoring" profile: forced mode, 1x oversampling,
    // IIR filter off. One conversion per wake-up, minimal current.
    _bme.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1,   // temperature
                     Adafruit_BME280::SAMPLING_X1,   // pressure
                     Adafruit_BME280::SAMPLING_X1,   // humidity
                     Adafruit_BME280::FILTER_OFF);
    return true;
}

bool Bme280Sensor::read(JsonObject &out) {
    if (!_bme.takeForcedMeasurement()) {
        return false;
    }

    float t = _bme.readTemperature();      // °C
    float rh = _bme.readHumidity();        // %
    float p = _bme.readPressure() / 100.0f;  // Pa -> hPa

    if (isnan(t) || isnan(rh) || isnan(p)) {
        return false;
    }

    log_d("T = %.2f degC, RH = %.2f %%, P = %.2f hPa", t, rh, p);

    out["t"] = round1(t);
    out["rh"] = round1(rh);
    out["p"] = round1(p);
    return true;
}
