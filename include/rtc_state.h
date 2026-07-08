#pragma once

/**
 * @file rtc_state.h
 * @brief Variables stored in RTC slow memory – retained across deep-sleep cycles.
 *
 * Declared `extern` here and **defined once** in `main.cpp`.
 * Any translation unit that includes this header can read / write these values.
 *
 * @note  RTC slow memory is preserved during deep sleep but cleared on a
 *        hard power-cycle or chip reset.  Always check for sensible initial
 *        values before using them (e.g. g_lastEpoch == 0 → time unknown).
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Declarations  (definitions live in main.cpp)
// ---------------------------------------------------------------------------

/** @brief Number of wake cycles since the last power-on / hard reset. */
extern RTC_DATA_ATTR uint32_t g_bootCount;

/**
 * @brief UNIX epoch (seconds since 1970-01-01 UTC) saved just before
 *        the previous deep-sleep entry.  0 = never set.
 */
extern RTC_DATA_ATTR time_t g_lastEpoch;

/**
 * @brief Deep-sleep duration used in the previous cycle (seconds).
 * Added to g_lastEpoch on the next boot to restore approximate time
 * without an NTP round-trip.
 */
extern RTC_DATA_ATTR uint32_t g_lastSleepSeconds;

/** @brief True once NTP has synchronised at least once since hard reset. */
extern RTC_DATA_ATTR bool g_ntpEverSynced;
