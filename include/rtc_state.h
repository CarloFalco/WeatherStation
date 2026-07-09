/**
 * @file rtc_state.h
 * @brief State preserved in RTC slow memory across deep-sleep cycles.
 *
 * Variables placed in RTC RAM survive deep sleep but are reset to their
 * initializers on a power-on reset (cold boot). Everything the station
 * must remember between wake-ups without touching flash lives here:
 * flash writes are slow, power-hungry and wear the chip out, RTC RAM
 * is free.
 */

#ifndef WEATHERSTATION_RTC_STATE_H
#define WEATHERSTATION_RTC_STATE_H

#include <Arduino.h>

/**
 * @brief Persistent station state, kept in RTC RAM between deep sleeps.
 */
struct RtcState {
    uint32_t bootCount;      ///< Wake-up cycles since power-on (1 = cold boot).
    uint16_t msgSeq;         ///< LoRa telemetry sequence counter ("seq" JSON field).
    uint32_t rainPulses;     ///< Tipping-bucket pulses accumulated since the last
                             ///< acknowledged transmission.
    uint64_t nextWakeEpochS; ///< Scheduled time of the next measurement cycle
                             ///< [s, RTC epoch]. The RTC clock keeps running in
                             ///< deep sleep, so after a rain wake-up the station
                             ///< can go back to sleep for the remaining time
                             ///< instead of restarting the full interval.
};

/// Global RTC-resident state instance (defined in PowerManager.cpp).
extern RtcState g_rtcState;

#endif // WEATHERSTATION_RTC_STATE_H
