#pragma once

#include "driver/gpio.h"
#include "driver/adc.h"

/**
 * @file config.h
 * @brief Compile-time hardware pin and peripheral address definitions.
 *
 * This file is the single source of truth for all GPIO assignments and I²C/UART
 * addresses used by the ESP32-S3 Weather Station firmware.
 *
 * @note I²C bus pins (PIN_I2C_SDA / PIN_I2C_SCL) are set to GPIO8 / GPIO9 by
 *       default, which matches the ESP32-S3-DevKitC-1 reference design.
 *       Modify here if your board uses different pins.
 *
 * @note The reset-config pin (PIN_RESET_CONFIG) defaults to GPIO0 (the BOOT
 *       button present on most DevKit boards).  Hold LOW at power-on to force
 *       the provisioning captive-portal.
 */

// =============================================================================
// I²C bus  (shared by BME280, AS5600, CCS811, INA3221)
// =============================================================================
/** @brief I²C SDA pin. */
#define PIN_I2C_SDA         GPIO_NUM_8
/** @brief I²C SCL pin. */
#define PIN_I2C_SCL         GPIO_NUM_9

// =============================================================================
// BME280 – Temperature / Humidity / Pressure
// =============================================================================
/** @brief BME280 I²C address (SDO pulled to GND → 0x76). */
#define BME280_I2C_ADDRESS  0x76

// =============================================================================
// Pluviometer – tipping-bucket reed switch
// =============================================================================
/** @brief GPIO for the rain-gauge reed switch (interrupt-driven, active LOW). */
#define PIN_RAIN_GAUGE      GPIO_NUM_19

// =============================================================================
// Anemometer – cup-type reed switch
// =============================================================================
/** @brief GPIO for the anemometer reed switch (interrupt-driven, active LOW). */
#define PIN_ANEMOMETER      GPIO_NUM_20

// =============================================================================
// AS5600 – Magnetic wind-direction encoder
// =============================================================================
/** @brief AS5600 I²C address (fixed, 0x36). */
#define AS5600_I2C_ADDRESS  0x36

// =============================================================================
// PM25AQI – Particulate-matter UART sensor (e.g. PMSA003I / PMS5003)
// =============================================================================
/** @brief UART RX pin connected to sensor TX. */
#define PM25_RX_PIN         GPIO_NUM_17
/** @brief UART TX pin connected to sensor RX. */
#define PM25_TX_PIN         GPIO_NUM_18
/** @brief Hardware UART port used for the PM25 sensor. */
#define PM25_UART_NUM       1   // → Serial1

// =============================================================================
// CCS811 – eCO2 / TVOC sensor
// =============================================================================
/** @brief CCS811 I²C address (ADDR pin pulled to GND → 0x5A). */
#define CCS811_I2C_ADDRESS  0x5A
/** @brief Active-LOW WAKE pin of the CCS811 (must be driven LOW before I²C). */
#define PIN_CCS811_WAK      GPIO_NUM_41

// =============================================================================
// MICS6814 – Triple-channel analog gas sensor
// =============================================================================
/** @brief ADC1 channel for the CO channel of the MICS6814.  GPIO5. */
#define MICS6814_CO_ADC_CH  ADC1_CHANNEL_4   // GPIO5
/** @brief ADC1 channel for the NO2 channel of the MICS6814. GPIO6. */
#define MICS6814_NO2_ADC_CH ADC1_CHANNEL_5   // GPIO6
/** @brief ADC1 channel for the NH3 channel of the MICS6814. GPIO7. */
#define MICS6814_NH3_ADC_CH ADC1_CHANNEL_6   // GPIO7

// =============================================================================
// INA3221 – Triple-channel voltage/current monitor
// =============================================================================
/** @brief INA3221 I²C address (A0 pulled to GND → 0x40). */
#define INA3221_I2C_ADDRESS 0x40

// =============================================================================
// Power-switching MOSFETs
// =============================================================================
/**
 * @brief GPIO controlling the P-channel MOSFET that switches 5 V rail.
 * @note  Drive LOW to enable 5 V (active-LOW gate for P-channel FET).
 */
#define PIN_PWR_5V          GPIO_NUM_11

/**
 * @brief GPIO controlling the P-channel MOSFET that switches 3.3 V rail.
 * @note  Drive LOW to enable 3.3 V (active-LOW gate for P-channel FET).
 */
#define PIN_PWR_3V3         GPIO_NUM_4

// =============================================================================
// System / user-interface
// =============================================================================
/** @brief On-board LED (active HIGH). */
#define LED_BUILTIN_PIN     97

/**
 * @brief GPIO used to force provisioning mode at boot.
 *
 * Hold this pin LOW while powering on (or pressing reset) to erase the stored
 * configuration and restart the captive-portal setup wizard.
 * Defaults to GPIO0 (the BOOT / FLASH button on most DevKit boards).
 */
#define PIN_RESET_CONFIG    GPIO_NUM_0

// =============================================================================
// Misc timing constants
// =============================================================================
/** @brief Debounce window (ms) applied to reed-switch interrupts. */
#define REED_DEBOUNCE_MS    50

/** @brief Time (ms) the BOOT button must be held to trigger a config reset. */
#define RESET_HOLD_MS       3000
