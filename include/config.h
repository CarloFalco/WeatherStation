/**
 * @file config.h
 * @brief Hardware configuration: GPIO pinout, I2C addresses, board constants.
 *
 * Single source of truth for the wiring of the station. Runtime-tunable
 * parameters (wake interval, station ID, LoRa settings, ...) do NOT belong
 * here: they live in /config.ini on LittleFS (see AppConfig, Increment 1).
 */

#ifndef WEATHERSTATION_CONFIG_H
#define WEATHERSTATION_CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// LoRa SX1276 868 MHz (SPI)
// ---------------------------------------------------------------------------
#define LORA_SCK   GPIO_NUM_12  ///< SPI clock
#define LORA_MISO  GPIO_NUM_13  ///< SPI MISO
#define LORA_MOSI  GPIO_NUM_11  ///< SPI MOSI
#define LORA_CS    GPIO_NUM_10  ///< SPI chip select (NSS)
#define LORA_RST   GPIO_NUM_9   ///< Radio reset
#define LORA_DIO0  GPIO_NUM_8   ///< TX/RX done interrupt

// ---------------------------------------------------------------------------
// I2C bus (BME280, AS5600, INA3221)
// The ESP32-S3 Arduino defaults (SDA=8, SCL=9) clash with LORA_DIO0/LORA_RST,
// so the bus is remapped to GPIO 4/5.
// ---------------------------------------------------------------------------
#define I2C_SDA    GPIO_NUM_4   ///< I2C data
#define I2C_SCL    GPIO_NUM_5   ///< I2C clock

/// BME280 temperature/humidity/pressure sensor I2C address.
/// (Named *_I2C_ADDR because the Adafruit driver already defines
/// BME280_ADDRESS as its 0x77 default.)
#define BME280_I2C_ADDR 0x76
/// AS5600 magnetic encoder (wind direction) I2C address.
#define AS5600_ADDRESS 0x36
/// INA3221 3-channel current monitor (panel/battery/load) I2C address.
#define INA3221_ADDRESS 0x40

// ---------------------------------------------------------------------------
// Pulse sensors (reed switches)
// ---------------------------------------------------------------------------
#define RAIN_GAUGE_PIN GPIO_NUM_6  ///< Tipping-bucket rain gauge (RTC-capable, used for wake-on-rain)
#define ANEMOMETER_PIN GPIO_NUM_7  ///< Cup anemometer reed switch

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN LED_BUILTIN  ///< On-board status LED (validated on hardware)

#endif // WEATHERSTATION_CONFIG_H
