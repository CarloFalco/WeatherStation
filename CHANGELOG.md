# Changelog

All notable changes to the ESP32-S3 Weather Station firmware are documented here.

This project adheres to [Semantic Versioning](https://semver.org/) and
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

### Planned
- Sensor driver classes: BME280, AS5600, rain gauge, anemometer,
  PM25AQI, CCS811, MICS6814, INA3221 (Increment 2)
- MQTT JSON publishing with aggregation (Increment 3)
- OTA updates via GitHub Releases API (Increment 4)

---

## [0.2.0] – 2025-04-14

### Added
- **`PowerManager`** singleton (`src/power/`):
  - Controls P-channel MOSFET gates for the 5 V (`GPIO11`) and
    3.3 V (`GPIO4`) sensor supply rails (active-LOW gate, both OFF at boot).
  - Configures deep-sleep wakeup sources before calling
    `esp_deep_sleep_start()`:
    - **Timer wakeup** – periodic measurement cycle (`sleep_s` seconds).
    - **EXT1 GPIO wakeup** – GPIO19 (rain gauge) and GPIO20 (anemometer)
      wake the device when a reed switch closes.
  - Explicitly sets RTC pull-ups on wakeup GPIOs so they remain stable
    during deep sleep (via `driver/rtc_io.h`).
  - `printWakeupReason()` logs the wakeup cause (timer / GPIO / power-on).

- **`WiFiManager`** singleton + `TaskWifi` FreeRTOS task (`src/connectivity/`):
  - Background task manages the full STA connection lifecycle.
  - Sets `EVT_WIFI_CONNECTED` / `EVT_WIFI_FAILED` in `g_sysEvents`
    (global FreeRTOS EventGroup) so other tasks can wait without polling.
  - Automatic retry on disconnect with `RETRY_DELAY_MS` back-off and
    `MAX_RETRIES` (10) limit per wake cycle.
  - `waitForConnection(timeoutMs)` blocks the caller on the EventGroup.
  - `disconnect()` powers off the WiFi radio before deep sleep.

- **`TimeManager`** singleton (`src/rtc/`):
  - `init()` – restores UNIX epoch from RTC slow memory on every deep-sleep
    wake (`g_lastEpoch + g_lastSleepSeconds`), making timestamps available
    immediately without waiting for NTP.
  - `syncNtp()` – calls `configTime()` with `pool.ntp.org` / `time.google.com`
    and waits up to 15 s for SNTP to synchronise.  Sets `EVT_NTP_SYNCED` on
    success.
  - `isResyncDue()` – NTP is re-queried every `NTP_RESYNC_BOOTS` (288) wake
    cycles (~24 h at `sleep_s = 300`) to compensate for RTC drift.
  - `getIsoTimestamp()` returns an ISO 8601 UTC string for JSON payloads.
  - `saveEpochToRtc()` persists the current epoch before every sleep cycle.
  - Timezone configured via POSIX string `CET-1CEST,M3.5.0,M10.5.0/3`
    (Central European Time with automatic DST).

- **`rtc_state.h`** (`include/`):
  Four `RTC_DATA_ATTR` variables defined in `main.cpp` and shared across
  all modules: `g_bootCount`, `g_lastEpoch`, `g_lastSleepSeconds`,
  `g_ntpEverSynced`.

- **`task_events.h`** (`include/`):
  FreeRTOS EventGroup bit definitions (`EVT_WIFI_CONNECTED`,
  `EVT_WIFI_FAILED`, `EVT_NTP_SYNCED`, and stubs for Increments 2–3).

- **`vTaskSleep`** in `main.cpp`:
  Main wake-cycle orchestration task.  Sequence: enable rails → wait for
  WiFi → (optional) NTP sync → (stubs for sensors/MQTT) → disconnect WiFi
  → save epoch → `enterDeepSleep()`.

- **`DEBUG_SKIP_DEEP_SLEEP`** build flag honoured in `vTaskSleep`: the device
  idles instead of sleeping when the flag is defined (already set in the
  `esp32s3-weather-debug` PlatformIO environment).

### Changed
- `main.cpp` refactored: FreeRTOS task creation replaces the Increment 0
  placeholder comment.  Provisioning and config-load logic unchanged.
- `version.h` bumped to `0.2.0`.

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

---

[0.2.0]: https://github.com/<YOUR_USERNAME>/esp32-weather-station/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/<YOUR_USERNAME>/esp32-weather-station/releases/tag/v0.1.0
[Unreleased]: https://github.com/<YOUR_USERNAME>/esp32-weather-station/compare/v0.2.0...HEAD
