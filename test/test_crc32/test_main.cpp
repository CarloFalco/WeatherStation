/**
 * @file test_main.cpp
 * @brief Native unit tests for logic::crc32 (known vectors + streaming).
 *
 * Run on the host with: pio test -e native
 */

#include <cstring>
#include <unity.h>

#include "logic/crc32.h"

using namespace logic;

void setUp() {}
void tearDown() {}

/// Canonical CRC-32 check value ("123456789" -> 0xCBF43926).
static void test_canonical_vector() {
    const char *s = "123456789";
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32((const uint8_t *)s, strlen(s)));
}

/// Empty input yields 0.
static void test_empty_input() {
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, crc32(nullptr, 0));
}

/// Second known vector.
static void test_known_sentence() {
    const char *s = "The quick brown fox jumps over the lazy dog";
    TEST_ASSERT_EQUAL_HEX32(0x414FA339u, crc32((const uint8_t *)s, strlen(s)));
}

/// Incremental (chunked) updates must match the one-shot result — this is
/// exactly how the OTA receiver consumes the stream.
static void test_incremental_equals_oneshot() {
    uint8_t data[1024];
    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)((i * 31 + 7) & 0xFF);  // same pattern as DebugLora
    }
    uint32_t oneshot = crc32(data, sizeof(data));

    uint32_t crc = crc32Init();
    for (size_t off = 0; off < sizeof(data); off += 180) {  // OTA chunk size
        size_t n = sizeof(data) - off;
        if (n > 180) n = 180;
        crc = crc32Update(crc, data + off, n);
    }
    TEST_ASSERT_EQUAL_HEX32(oneshot, crc32Final(crc));
}

/// A single corrupted byte must change the checksum.
static void test_detects_corruption() {
    uint8_t data[256];
    for (size_t i = 0; i < sizeof(data); i++) data[i] = (uint8_t)i;
    uint32_t good = crc32(data, sizeof(data));
    data[100] ^= 0x01;
    TEST_ASSERT_NOT_EQUAL(good, crc32(data, sizeof(data)));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_canonical_vector);
    RUN_TEST(test_empty_input);
    RUN_TEST(test_known_sentence);
    RUN_TEST(test_incremental_equals_oneshot);
    RUN_TEST(test_detects_corruption);
    return UNITY_END();
}
