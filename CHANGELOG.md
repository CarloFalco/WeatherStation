# Changelog

All notable changes to the ESP32-S3 Weather Station firmware are documented here.

This project adheres to [Semantic Versioning](https://semver.org/) and
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

### Planned
- FreeRTOS task architecture (Increment 1)
- Deep-sleep with periodic wake-up (Increment 1)
- RTC synchronisation via NTP (Increment 1)
- Sensor driver classes: BME280, AS5600, rain gauge, anemometer,
  PM25AQI, CCS811, MICS6814, INA3221 (Increment 2)
- MQTT JSON publishing with aggregation (Increment 3)
- OTA updates via GitHub Releases API (Increment 4)

---

## [0.1.0] – 2025-03-29

### Added
- **Project scaffold**: PlatformIO project structure with custom
  4 MB partition table (OTA + LittleFS).
- **`include/config.h`**: Single source of truth for all GPIO pin assignments
  and I²C addresses.
- **`include/version.h`**: SemVer firmware version constants.
- **`AppConfig`** singleton: INI file parser/serialiser stored on LittleFS at
  `/config.ini`.  Supports sections: `[wifi]`, `[mqtt]`, `[ota]`,
  `[sampling]`.
- **Provisioning captive-portal**: On first boot or after a 3-second hold of
  the BOOT button (GPIO0), the device starts a Wi-Fi AP
  (`WeatherStation-Setup`) and serves a mobile-friendly HTML configuration
  form.  Credentials are written to LittleFS and the device restarts.
- **`data/index.html`**: Styled single-page configuration wizard with sections
  for Wi-Fi, MQTT, OTA / GitHub, and sampling intervals.
- **`data/config.ini`**: Template configuration file with documented defaults.
- **`main.cpp`** skeleton: Boot sequence — LittleFS mount, provisioning
  check, config load; FreeRTOS placeholders for Increment 1.
- **`.gitignore`** for PlatformIO / ESP32 projects.
- **`README.md`** with hardware pinout table, build instructions, and project
  structure overview.
- **`docs/Doxyfile`**: Doxygen configuration for HTML API documentation.
- **`progress/progress_step_00.md`**: First progress log.

[0.1.0]: https://github.com/CarloFalco/WeatherStation/releases/tag/v0.1.0
[Unreleased]: https://github.com/CarloFalco/WeatherStation/compare/v0.1.0...HEAD
