# Progress Step 01 – FreeRTOS Architecture, WiFi Manager, Deep-Sleep & RTC/NTP

**Date:** 2025-04-14
**Increment:** 1
**Firmware tag:** v0.2.0
**Status:** ✅ Complete

---

## Activities Completed

- Implemented **`PowerManager`** singleton (`src/power/PowerManager.h/.cpp`):
  - `init()` – configures `PIN_PWR_5V` (GPIO11) and `PIN_PWR_3V3` (GPIO4) as
    active-LOW P-channel MOSFET gate outputs; both rails start **OFF**.
  - `enable5V(bool)` / `enable3V3(bool)` – called by `vTaskSleep` before and
    after the sensor-reading window.
  - `enterDeepSleep(uint32_t seconds)`:
    - Disables both rails.
    - Configures **timer wakeup** (`esp_sleep_enable_timer_wakeup`).
    - Configures **EXT1 wakeup** on GPIO19 (rain gauge) and GPIO20 (anemometer)
      via `esp_sleep_enable_ext1_wakeup(ANY_LOW)`.
    - Sets explicit RTC pull-ups via `driver/rtc_io.h` so the reed-switch
      lines are not floating during sleep.
    - Calls `esp_deep_sleep_start()` (never returns).
  - `printWakeupReason()` – logs "TIMER", "EXT1 – rain gauge / anemometer",
    or "power-on / reset" to Serial.

- Implemented **`WiFiManager`** singleton + **`TaskWifi`** FreeRTOS task
  (`src/connectivity/WiFiManager.h/.cpp`):
  - `init(ssid, password)` – stores credentials and spawns `TaskWifi` on
    Core 1 at priority 3 with a 4 KB stack.
  - `taskFunction()` – starts WiFi STA mode, registers `onWifiEvent()` callback,
    calls `WiFi.begin()`, then enters a 1 s tick loop.
  - `onWifiEvent()` – handles `ARDUINO_EVENT_WIFI_STA_GOT_IP`
    (sets `EVT_WIFI_CONNECTED`) and `ARDUINO_EVENT_WIFI_STA_DISCONNECTED`
    (increments retry counter, sets `EVT_WIFI_FAILED` after `MAX_RETRIES`,
    triggers `WiFi.reconnect()` with `RETRY_DELAY_MS` back-off).
  - `waitForConnection(ms)` – blocks the caller on `g_sysEvents` using
    `xEventGroupWaitBits`; returns `true` if `EVT_WIFI_CONNECTED` fires
    within the timeout.
  - `disconnect()` – calls `WiFi.disconnect(true)` + `WiFi.mode(WIFI_OFF)`,
    clears the event bits.

- Implemented **`TimeManager`** singleton (`src/rtc/TimeManager.h/.cpp`):
  - `init()` – restores approximate epoch from `g_lastEpoch + g_lastSleepSeconds`
    (RTC slow memory) via `settimeofday()`; sets POSIX timezone
    `CET-1CEST,M3.5.0,M10.5.0/3`.
  - `syncNtp(ms)` – calls `configTime()` + `getLocalTime()` with the
    configured timeout; sets `EVT_NTP_SYNCED` and `g_ntpEverSynced` on success.
  - `isResyncDue()` – returns `true` on first boot (never synced) or every
    `NTP_RESYNC_BOOTS = 288` wake cycles (~24 h at `sleep_s = 300`).
  - `saveEpochToRtc()` – writes current epoch to `g_lastEpoch` before sleep.
  - `getIsoTimestamp()` – returns an ISO 8601 UTC string ("2025-04-14T09:00:00Z").
  - `isTimeValid()` – epoch > 2024-01-01 00:00:00 UTC.

- Added **`include/rtc_state.h`** – declares the four `RTC_DATA_ATTR` variables
  (`g_bootCount`, `g_lastEpoch`, `g_lastSleepSeconds`, `g_ntpEverSynced`)
  as `extern`; defined once in `main.cpp`.

- Added **`include/task_events.h`** – defines the global `g_sysEvents`
  `EventGroupHandle_t` and all EventGroup bit constants (Increments 1–3).

- Updated **`main.cpp`** (Increment 1):
  - Increments `g_bootCount` at every wake.
  - Calls `PowerManager::init()` and `pwr.printWakeupReason()`.
  - Calls `TimeManager::init()` to restore epoch immediately after boot.
  - Creates `g_sysEvents` EventGroup.
  - Starts `WiFiManager::init()` → spawns `TaskWifi`.
  - Starts `vTaskSleep` (Core 1, priority 2, 5 KB stack).

- Implemented **`vTaskSleep`** orchestration task:
  - Enables 5V / 3.3V rails.
  - Waits up to 30 s for WiFi (`waitForConnection`).
  - If connected, calls `TimeManager::syncNtp()` when due.
  - Disconnects WiFi, saves epoch, enters deep sleep.
  - Idles (no sleep) when `sleep_s = 0` or `DEBUG_SKIP_DEEP_SLEEP` is defined.

- Bumped **`version.h`** to `0.2.0` / `FW_VERSION_DESC = "Increment 1 – …"`.

- Updated **`CHANGELOG.md`** with `[0.2.0]` section.

---

## Design Decisions

| Decision | Rationale |
|---|---|
| **Single global EventGroup** (`g_sysEvents`) | One object shared across all modules is simpler to reason about and avoids handle-passing proliferation.  Bit allocation is documented in `task_events.h`. |
| **TaskWifi on Core 1** | Arduino `loop()` also runs on Core 1.  Keeping network tasks on Core 1 allows Core 0 to be dedicated to sensor ISRs and computation (Increment 2). |
| **EXT1 GPIO wakeup** with explicit `rtc_gpio_pullup_en()` | Without explicitly setting RTC pull-ups, the reed-switch pins might float during deep sleep and cause spurious wakeups.  The `driver/rtc_io.h` API is the correct way to configure RTC GPIO state independently of the normal GPIO matrix. |
| **Epoch restoration from RTC + sleep offset** | Avoids an NTP round-trip on every wake cycle (saves ~1–2 s of active time and reduces WiFi TX energy).  Drift is bounded by `NTP_RESYNC_BOOTS` guard. |
| **`NTP_RESYNC_BOOTS = 288`** | At `sleep_s = 300` this is ~24 h.  ESP32 RTC drift is ≤ 150 ppm; 24 h accumulates ≤ 13 s error, acceptable for timestamped sensor data. |
| **`WiFi.setAutoReconnect(false)`** | Manual retry gives us precise control over the retry count per wake cycle and prevents the Arduino core from issuing reconnect attempts that interfere with the deep-sleep shutdown sequence. |
| **`DEBUG_SKIP_DEEP_SLEEP` build flag** | Already defined in the `esp32s3-weather-debug` PlatformIO environment.  `vTaskSleep` checks this at compile time so the production binary is unaffected. |

---

## Assumptions Confirmed / Updated

- ✅ I²C pins: SDA = GPIO8, SCL = GPIO9 (unchanged).
- ✅ GPIO19 and GPIO20 are in the ESP32-S3 RTC domain → EXT1 wakeup confirmed.
- ✅ Timezone: Central European Time (`CET-1CEST,M3.5.0,M10.5.0/3`).
- ⚠️ Reed-switch wiring: external 10 kΩ pull-ups recommended on GPIO19 / GPIO20
  (the internal RTC pull-ups are enabled during sleep, but external ones are
  more robust for the high-speed pulse counting in Increment 2).

---

## TODO for Increment 2

- [ ] Design **`SensorManager`** aggregator class and per-sensor driver classes:
  - `SensorBME280` – temperature / humidity / pressure (I²C 0x76).
  - `SensorRainGauge` – interrupt-driven pulse count on GPIO19.
  - `SensorAnemometer` – interrupt-driven pulse count + RPM on GPIO20.
  - `SensorAS5600` – wind direction via I²C magnetic encoder (0x36).
  - `SensorPM25AQI` – particulate matter via UART (RX=GPIO17, TX=GPIO18).
  - `SensorCCS811` – eCO₂ / TVOC via I²C (0x5A, WAKE=GPIO41).
  - `SensorMICS6814` – CO / NO₂ / NH₃ via ADC1 channels 4/5/6.
  - `SensorINA3221` – voltage / current (3 channels) via I²C (0x40).
- [ ] Initialise the I²C bus (Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) in setup().
- [ ] Implement reed-switch ISRs with `REED_DEBOUNCE_MS` debounce for rain and
  wind sensors; use `RTC_DATA_ATTR` accumulators so counts survive deep sleep
  when woken by EXT1.
- [ ] Power sensor rails, wait for sensor warm-up (e.g., CCS811 needs ~20 min
  baseline), read all sensors, store readings in a shared `WeatherData` struct.
- [ ] Add `EVT_SENSORS_DONE` signalling to `vTaskSleep`.
- [ ] Create `progress/progress_step_02.md`.
