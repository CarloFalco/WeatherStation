#pragma once

/**
 * @file TimeManager.h
 * @brief Manages system time via NTP and persists the epoch across deep-sleep cycles.
 *
 * @par Time-continuity strategy
 *  1. **Power-on / hard reset** – g_lastEpoch is 0; time is unknown until
 *     the first NTP synchronisation.
 *  2. **Deep-sleep wake** – g_lastEpoch is non-zero; init() immediately
 *     restores the epoch (`g_lastEpoch + g_lastSleepSeconds`) so that
 *     the firmware has a valid timestamp before WiFi has even connected.
 *  3. **NTP re-synchronisation** – runs every NTP_RESYNC_BOOTS wake cycles
 *     to compensate for the ESP32-S3 RTC drift (~50–150 ppm).
 *
 * @par Timezone
 * `CET-1CEST,M3.5.0,M10.5.0/3` — Central European Time with automatic DST.
 * Change `TZ_POSIX` in this header (or supply it from AppConfig) for other regions.
 *
 * @par NTP re-sync frequency example
 * With `sleep_s = 300` (5 min) and `NTP_RESYNC_BOOTS = 288`, NTP is queried
 * approximately once every 24 hours.
 */

#include <Arduino.h>
#include <time.h>

/** @brief POSIX timezone string – Central European Time (CET/CEST). */
#define TZ_POSIX           "CET-1CEST,M3.5.0,M10.5.0/3"

/** @brief Primary NTP server hostname. */
#define NTP_SERVER_1       "pool.ntp.org"

/** @brief Secondary NTP server hostname. */
#define NTP_SERVER_2       "time.google.com"

/** @brief Default NTP synchronisation timeout (ms). */
#define NTP_TIMEOUT_MS     15000

/**
 * @brief Number of wake cycles between NTP re-synchronisations.
 *
 * At `sleep_s = 300` this equals ~24 hours.  Decrease for faster drift
 * correction; increase to reduce WiFi usage.
 */
#define NTP_RESYNC_BOOTS   288

class TimeManager {
public:
    /** @brief Returns the singleton instance. */
    static TimeManager& getInstance();

    /**
     * @brief Restores system time from RTC slow memory at boot.
     *
     * If g_lastEpoch > 0 the epoch is set to `g_lastEpoch + g_lastSleepSeconds`.
     * The POSIX timezone is configured in both cases.
     *
     * Should be called **before** any task that needs a timestamp.
     */
    void init();

    /**
     * @brief Synchronise system time with NTP.
     *
     * Calls `configTime()` and waits up to @p timeoutMs for a valid
     * timestamp to become available.  On success, sets `EVT_NTP_SYNCED`
     * in `g_sysEvents` and saves the flag to RTC memory.
     *
     * @param timeoutMs  Maximum wait in milliseconds (default 15 000).
     * @return `true` on success, `false` on timeout or no WiFi.
     */
    bool syncNtp(uint32_t timeoutMs = NTP_TIMEOUT_MS);

    /**
     * @brief Returns `true` when the system time is considered valid.
     *
     * "Valid" is defined as epoch > 2024-01-01 00:00:00 UTC.
     */
    bool isTimeValid() const;

    /** @brief Returns the current UNIX epoch (UTC). */
    time_t getEpoch() const;

    /**
     * @brief Returns an ISO 8601 UTC timestamp string.
     * Example: `"2025-06-15T14:32:00Z"`
     */
    String getIsoTimestamp() const;

    /**
     * @brief Saves the current epoch into `g_lastEpoch` (RTC slow memory).
     *
     * Must be called just before `PowerManager::enterDeepSleep()` so that
     * the next boot can restore an approximate timestamp.
     */
    void saveEpochToRtc() const;

    /** @brief Prints the current local time to Serial (debug helper). */
    void printCurrentTime() const;

    /**
     * @brief Returns `true` when an NTP re-sync should be performed this boot.
     *
     * True when g_ntpEverSynced is false, or when
     * `g_bootCount % NTP_RESYNC_BOOTS == 0`.
     */
    bool isResyncDue() const;

private:
    TimeManager() = default;
    TimeManager(const TimeManager&) = delete;
    TimeManager& operator=(const TimeManager&) = delete;
};
