/**
 * @file version.h
 * @brief Firmware version constants (Semantic Versioning).
 *
 * The V2 firmware series starts at 2.0.0 to avoid clashing with the
 * v0.x tags of the previous prototype kept in the git history.
 * Bump rules:
 *  - MAJOR: incompatible LoRa protocol / config format changes, or a
 *    milestone feature set (3.0.0 = OTA over LoRa complete)
 *  - MINOR: new features (new sensor, new module) - one per increment
 *  - PATCH: bug fixes
 * The 3.0.0 pre-releases (-alpha.N) are the OTA development increments.
 */

#ifndef WEATHERSTATION_VERSION_H
#define WEATHERSTATION_VERSION_H

#define FW_VERSION_MAJOR 3
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0

/** @brief Firmware version as a human-readable string, e.g. "2.0.0". */
#define FW_VERSION "3.0.0-alpha.1"

#endif // WEATHERSTATION_VERSION_H
