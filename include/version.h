#pragma once

/**
 * @file version.h
 * @brief Firmware version constants following Semantic Versioning (SemVer).
 *
 * Update these values before tagging a new GitHub release.
 * The OTA manager compares FW_VERSION against the latest GitHub Release tag
 * to decide whether an update is available.
 */

/** @brief Major version – incremented on breaking changes. */
#define FW_VERSION_MAJOR    0
/** @brief Minor version – incremented on backward-compatible new features. */
#define FW_VERSION_MINOR    1
/** @brief Patch version – incremented on backward-compatible bug fixes. */
#define FW_VERSION_PATCH    1

/** @brief Full version string in the format "MAJOR.MINOR.PATCH". */
#define FW_VERSION          "0.1.1"

/** @brief Full version string prefixed with 'v', matching GitHub tag format. */
#define FW_VERSION_TAG      "v0.1.1"

/** @brief Human-readable description of this release. */
#define FW_VERSION_DESC     "Increment 0 – Project scaffold & provisioning"
