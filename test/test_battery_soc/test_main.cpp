/**
 * @file test_main.cpp
 * @brief Native unit tests for logic::vbatToSoc (LiPo discharge curve).
 *
 * Run on the host with: pio test -e native
 */

#include <unity.h>

#include "logic/battery_soc.h"

using logic::vbatToSoc;

void setUp() {}
void tearDown() {}

/// Above the top of the curve the SoC clamps to 100%.
static void test_clamps_to_100_above_curve() {
    TEST_ASSERT_EQUAL_INT(100, vbatToSoc(4187.0f));
    TEST_ASSERT_EQUAL_INT(100, vbatToSoc(4300.0f));
    TEST_ASSERT_EQUAL_INT(100, vbatToSoc(5000.0f));
}

/// Below the bottom of the curve the SoC clamps to 0%.
static void test_clamps_to_0_below_curve() {
    TEST_ASSERT_EQUAL_INT(0, vbatToSoc(2925.0f));
    TEST_ASSERT_EQUAL_INT(0, vbatToSoc(2500.0f));
    TEST_ASSERT_EQUAL_INT(0, vbatToSoc(0.0f));
}

/// Table anchor points map exactly to 100 - DOD.
static void test_known_curve_points() {
    TEST_ASSERT_EQUAL_INT(95, vbatToSoc(4111.0f));
    TEST_ASSERT_EQUAL_INT(75, vbatToSoc(3940.0f));
    TEST_ASSERT_EQUAL_INT(50, vbatToSoc(3714.0f));
    TEST_ASSERT_EQUAL_INT(25, vbatToSoc(3501.0f));
    TEST_ASSERT_EQUAL_INT(5, vbatToSoc(3150.0f));
}

/// Between anchors the interpolation stays between the neighbours.
static void test_interpolation_between_points() {
    // Midpoint of the 3714 mV (SoC 50) - 3670 mV (SoC 45) segment:
    // DOD 52.5 -> lround 53 -> SoC 47.
    int soc = vbatToSoc(3692.0f);
    TEST_ASSERT_TRUE(soc >= 45 && soc <= 50);
    TEST_ASSERT_EQUAL_INT(47, soc);
}

/// The curve must be monotonic: more volts, never less charge.
static void test_monotonic_non_decreasing() {
    int prev = vbatToSoc(2500.0f);
    for (float mv = 2500.0f; mv <= 4400.0f; mv += 1.0f) {
        int soc = vbatToSoc(mv);
        TEST_ASSERT_TRUE_MESSAGE(soc >= prev, "SoC decreased while voltage increased");
        prev = soc;
    }
}

/// Output is always a valid percentage.
static void test_output_range() {
    for (float mv = 0.0f; mv <= 6000.0f; mv += 7.0f) {
        int soc = vbatToSoc(mv);
        TEST_ASSERT_TRUE(soc >= 0 && soc <= 100);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clamps_to_100_above_curve);
    RUN_TEST(test_clamps_to_0_below_curve);
    RUN_TEST(test_known_curve_points);
    RUN_TEST(test_interpolation_between_points);
    RUN_TEST(test_monotonic_non_decreasing);
    RUN_TEST(test_output_range);
    return UNITY_END();
}
