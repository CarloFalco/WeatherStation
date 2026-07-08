/**
 * @file TimeManager.cpp
 * @brief Implementation of the TimeManager singleton.
 */

#include "TimeManager.h"
#include "rtc_state.h"
#include "task_events.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

TimeManager& TimeManager::getInstance() {
    static TimeManager instance;
    return instance;
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

void TimeManager::init() {
    // Always configure the timezone first
    setenv("TZ", TZ_POSIX, 1);
    tzset();

    if (g_lastEpoch > 0) {
        // Restore approximate time: last saved epoch + sleep duration
        const time_t restored = g_lastEpoch + static_cast<time_t>(g_lastSleepSeconds);
        const struct timeval tv = { .tv_sec = restored, .tv_usec = 0 };
        settimeofday(&tv, nullptr);

        Serial.printf("[RTC] Time restored from RTC memory  epoch=%lld  (+%lus sleep)\n",
                      static_cast<long long>(restored),
                      static_cast<unsigned long>(g_lastSleepSeconds));
    } else {
        Serial.println("[RTC] No saved epoch – time unknown until NTP sync");
    }
}

// ---------------------------------------------------------------------------
// syncNtp()
// ---------------------------------------------------------------------------

bool TimeManager::syncNtp(uint32_t timeoutMs) {
    Serial.printf("[RTC] NTP sync starting  servers=%s / %s  timeout=%ums\n",
                  NTP_SERVER_1, NTP_SERVER_2, timeoutMs);

    // configTime sets UTC offset = 0; TZ env variable handles DST
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    setenv("TZ", TZ_POSIX, 1);
    tzset();

    // Wait for SNTP to produce a valid timestamp
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, timeoutMs)) {
        Serial.println("[RTC] NTP sync TIMEOUT");
        return false;
    }

    g_ntpEverSynced = true;
    xEventGroupSetBits(g_sysEvents, EVT_NTP_SYNCED);

    Serial.printf("[RTC] NTP sync OK  epoch=%lld\n",
                  static_cast<long long>(getEpoch()));
    printCurrentTime();
    return true;
}

// ---------------------------------------------------------------------------
// isTimeValid()
// ---------------------------------------------------------------------------

bool TimeManager::isTimeValid() const {
    // Consider any timestamp after 2024-01-01 00:00:00 UTC as valid
    return getEpoch() > static_cast<time_t>(1704067200LL);
}

// ---------------------------------------------------------------------------
// getEpoch()
// ---------------------------------------------------------------------------

time_t TimeManager::getEpoch() const {
    time_t now;
    time(&now);
    return now;
}

// ---------------------------------------------------------------------------
// getIsoTimestamp()
// ---------------------------------------------------------------------------

String TimeManager::getIsoTimestamp() const {
    const time_t now = getEpoch();
    struct tm tmUtc;
    gmtime_r(&now, &tmUtc);

    char buf[25];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
    return String(buf);
}

// ---------------------------------------------------------------------------
// saveEpochToRtc()
// ---------------------------------------------------------------------------

void TimeManager::saveEpochToRtc() const {
    g_lastEpoch = getEpoch();
    Serial.printf("[RTC] Epoch saved to RTC: %lld\n",
                  static_cast<long long>(g_lastEpoch));
}

// ---------------------------------------------------------------------------
// printCurrentTime()
// ---------------------------------------------------------------------------

void TimeManager::printCurrentTime() const {
    const time_t now = getEpoch();
    struct tm tmLocal;
    localtime_r(&now, &tmLocal);

    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &tmLocal);
    Serial.printf("[RTC] Local time: %s\n", buf);
}

// ---------------------------------------------------------------------------
// isResyncDue()
// ---------------------------------------------------------------------------

bool TimeManager::isResyncDue() const {
    if (!g_ntpEverSynced) {
        return true;
    }
    // Resync every NTP_RESYNC_BOOTS wake cycles (boot counter starts at 1)
    return (g_bootCount % NTP_RESYNC_BOOTS == 1);
}
