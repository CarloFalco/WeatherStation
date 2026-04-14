# Progress Step 00 – Project Scaffold & Provisioning System

**Date:** 2025-03-29
**Increment:** 0
**Firmware tag:** v0.1.0
**Status:** ✅ Complete

---

## Activities Completed

- Created the full PlatformIO project structure:
  `src/`, `include/`, `data/`, `lib/`, `docs/`, `progress/`, `.github/workflows/`.
- Authored `platformio.ini` with two environments (`esp32s3-weather` for
  production, `esp32s3-weather-debug` for development with deep-sleep disabled).
- Defined a custom 4 MB partition table (`partitions.csv`) that provides two
  OTA application slots and a 192 KB LittleFS data partition.
- Centralised all GPIO pin assignments and I²C addresses in `include/config.h`.
- Created `include/version.h` with SemVer constants used by the OTA manager
  (Increment 4) to compare against GitHub Release tags.
- Implemented **`AppConfig`** – a singleton INI file parser/serialiser that
  reads `/config.ini` from LittleFS and exposes all runtime parameters
  (Wi-Fi, MQTT, OTA, per-sensor sampling intervals, deep-sleep duration).
- Implemented **`Provisioning`** – captive-portal setup wizard:
  - Starts a Soft-AP (`WeatherStation-Setup`, open network).
  - Runs a DNS server that resolves all hostnames to `192.168.4.1`,
    triggering OS-level captive-portal detection on iOS, Android, and Windows.
  - Serves `data/index.html` from LittleFS via an embedded HTTP server.
  - Handles `POST /save`: validates inputs, populates `AppConfig`,
    calls `AppConfig::save("/config.ini")`, and calls `ESP.restart()`.
- Designed a **mobile-friendly captive-portal HTML page** (`data/index.html`)
  with sections for Wi-Fi, MQTT, OTA/GitHub, and collapsible sampling settings.
  No external CSS/JS dependencies – fully self-contained.
- Created a template `data/config.ini` with commented defaults for all keys.
- Wrote `src/main.cpp` skeleton boot sequence:
  1. Mount LittleFS (format-on-fail).
  2. Detect provisioning trigger (reset pin held ≥ 3 s, or missing config).
  3. Load and validate `AppConfig`.
  4. Placeholder comments for Increment 1 FreeRTOS tasks.
- Added `.gitignore` tuned for PlatformIO, ESP32, Doxygen, and macOS/Windows.
- Wrote `README.md` with hardware pinout tables, build instructions, and
  project structure overview.
- Created `CHANGELOG.md` following Keep a Changelog conventions.
- Configured `docs/Doxyfile` for HTML output from `src/` and `include/`.
- Added `.github/workflows/release.yml`: GitHub Actions workflow that builds
  the firmware binary and creates a GitHub Release (with changelog body
  extracted from `CHANGELOG.md`) whenever a `v*.*.*` tag is pushed.

---

## Design Decisions

| Decision | Rationale |
|---|---|
| **INI format** for config (not JSON or NVS) | Human-editable via a text editor after `pio run -t uploadfs`; easy to diff in git; no extra library dependency for parsing. |
| **LittleFS** for the filesystem | More wear-levelling and corruption-resistance than SPIFFS; officially recommended for Arduino ESP32 core ≥ 2.x. |
| **Captive-portal HTML from LittleFS** | Decouples the UI from the firmware binary – the page can be updated without reflashing the firmware. |
| **GPIO0 (BOOT button) as reset-config pin** | Available on all ESP32-S3 DevKit boards; no extra hardware required for development. Can be changed in `config.h` for production PCBs. |
| **I²C on GPIO8/GPIO9** | Default I²C pins on ESP32-S3-DevKitC-1 reference board. Documented as an assumption; override in `config.h` if needed. |
| **3-second hold** to confirm factory reset | Prevents accidental reset during normal operation (e.g., momentary power glitch). |
| **Two PlatformIO environments** | `esp32s3-weather` (production) and `esp32s3-weather-debug` (verbose logging, deep-sleep disabled) allow quick iteration without modifying source code. |
| **GitHub Actions CI** with `softprops/action-gh-release` | Automates binary attachment to releases; OTA manager (Increment 4) will query the same release for the `.bin` URL. |

---

## Assumptions to Confirm with Stakeholder

- [ ] I²C pins: **SDA = GPIO8, SCL = GPIO9** — confirm or specify alternatives.
- [ ] Reset-config pin: **GPIO0 (BOOT button)** — confirm for production PCB.
- [ ] GitHub repository name: `<YOUR_USERNAME>/esp32-weather-station` — replace
      in `README.md` and `CHANGELOG.md` before first push.
- [ ] Flash size: **4 MB** — confirm with the exact ESP32-S3 module used.

---

## TODO for Increment 1

- [ ] Implement `PowerManager` class:
  - Control 5 V and 3.3 V MOSFET rails (`PIN_PWR_5V`, `PIN_PWR_3V3`).
  - Configure deep-sleep with `esp_sleep_enable_timer_wakeup()`.
  - Configure GPIO wakeup for pluviometer (GPIO19) and anemometer (GPIO20)
    using `esp_sleep_enable_ext1_wakeup()`.
- [ ] Implement `WiFiManager` class with robust reconnection logic.
- [ ] Implement RTC time sync via NTP at first boot after WiFi connection;
      persist epoch in RTC_DATA_ATTR for continuity across deep-sleep cycles.
- [ ] Design FreeRTOS task structure:
  - `TaskWifi` – connection lifecycle.
  - `TaskSleep` – orchestrates sensor reading → MQTT publish → deep-sleep.
- [ ] Decide on POSIX timezone string for correct DST handling
      (e.g., `CET-1CEST,M3.5.0,M10.5.0/3` for Central Europe).
- [ ] Create `progress/progress_step_01.md`.
