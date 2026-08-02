/**
 * @file test_main.cpp
 * @brief Native unit tests for logic::soilRawToPercent.
 *
 * Run on the host with: pio test -e native
 */

#include <unity.h>

#include "logic/soil_moisture.h"

using logic::kSoilInvalid;
using logic::soilRawToPercent;

// Typical calibration of a capacitive probe at 3.3 V, 12-bit ADC.
static constexpr uint16_t kDry = 3000;
static constexpr uint16_t kWet = 1300;

void setUp() {}
void tearDown() {}

/// The calibration points map exactly to the ends of the scale.
static void test_calibration_endpoints() {
    TEST_ASSERT_EQUAL_INT(0, soilRawToPercent(kDry, kDry, kWet));
    TEST_ASSERT_EQUAL_INT(100, soilRawToPercent(kWet, kDry, kWet));
}

/// Readings beyond the calibration points clamp instead of overflowing.
static void test_clamps_outside_calibration() {
    TEST_ASSERT_EQUAL_INT(0, soilRawToPercent(4095, kDry, kWet));
    TEST_ASSERT_EQUAL_INT(100, soilRawToPercent(0, kDry, kWet));
}

/// Midpoint of the calibration span is 50 %.
static void test_midpoint() {
    TEST_ASSERT_EQUAL_INT(50, soilRawToPercent((kDry + kWet) / 2, kDry, kWet));
}

/// The scale is inverted: a higher raw reading means drier soil.
static void test_inverse_relationship() {
    int dryish = soilRawToPercent(2700, kDry, kWet);
    int wettish = soilRawToPercent(1600, kDry, kWet);
    TEST_ASSERT_TRUE(wettish > dryish);
}

/// Swapped or missing calibration must be reported, never guessed.
static void test_invalid_calibration() {
    TEST_ASSERT_EQUAL_INT(kSoilInvalid, soilRawToPercent(2000, kWet, kDry));
    TEST_ASSERT_EQUAL_INT(kSoilInvalid, soilRawToPercent(2000, 0, 0));
    TEST_ASSERT_EQUAL_INT(kSoilInvalid, soilRawToPercent(2000, 1500, 1500));
}

/// Output stays a valid percentage over the whole ADC range.
static void test_output_range() {
    for (uint32_t raw = 0; raw <= 4095; raw += 7) {
        int pct = soilRawToPercent((uint16_t)raw, kDry, kWet);
        TEST_ASSERT_TRUE(pct >= 0 && pct <= 100);
    }
}

/// Monotonic: more water can never report less moisture.
static void test_monotonic() {
    int prev = 101;
    for (uint32_t raw = 0; raw <= 4095; raw += 1) {
        int pct = soilRawToPercent((uint16_t)raw, kDry, kWet);
        TEST_ASSERT_TRUE_MESSAGE(pct <= prev, "moisture rose with a drier reading");
        prev = pct;
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_endpoints);
    RUN_TEST(test_clamps_outside_calibration);
    RUN_TEST(test_midpoint);
    RUN_TEST(test_inverse_relationship);
    RUN_TEST(test_invalid_calibration);
    RUN_TEST(test_output_range);
    RUN_TEST(test_monotonic);
    return UNITY_END();
}
