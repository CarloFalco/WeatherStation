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
// TODO(increment 3): confirm the actual SDA/SCL wiring with the user.
// The ESP32-S3 Arduino defaults (SDA=8, SCL=9) clash with LORA_DIO0/LORA_RST,
// so the bus MUST be remapped. Provisional assignment below.
// ---------------------------------------------------------------------------
#define I2C_SDA    GPIO_NUM_4   ///< I2C data  (provisional, to be confirmed)
#define I2C_SCL    GPIO_NUM_5   ///< I2C clock (provisional, to be confirmed)

/// BME280 temperature/humidity/pressure sensor I2C address.
#define BME280_ADDRESS 0x76
/// AS5600 magnetic encoder (wind direction) I2C address.
#define AS5600_ADDRESS 0x36
/// INA3221 3-channel current monitor (panel/battery/load) I2C address.
#define INA3221_ADDRESS 0x40

// ---------------------------------------------------------------------------
// Pulse sensors (reed switches)
// ---------------------------------------------------------------------------
#define RAIN_GAUGE_PIN GPIO_NUM_19  ///< Tipping-bucket rain gauge (RTC-capable, used for wake-on-rain)
#define ANEMOMETER_PIN GPIO_NUM_20  ///< Cup anemometer reed switch

// ---------------------------------------------------------------------------
// Misc
// TODO(increment 0 validation): requirements said "LED_BUILTIN 97" which is
// not a valid ESP32-S3 GPIO (max 48). GPIO 48 drives the on-board RGB LED on
// most S3 devkits; adjust after the first flash test.
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN GPIO_NUM_48  ///< On-board status LED (to be confirmed)

#endif // WEATHERSTATION_CONFIG_H
