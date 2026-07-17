/**
 * @file crc32.cpp
 * @brief Bitwise CRC-32 implementation (no lookup table).
 *
 * Table-less on purpose: 1 KB of table buys little on payloads limited by
 * LoRa airtime, and the bitwise loop at 80 MHz processes a full firmware
 * image in a few milliseconds per hundred KB.
 */

#include "crc32.h"

namespace logic {

uint32_t crc32Update(uint32_t crc, const uint8_t *data, std::size_t len) {
    for (std::size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, std::size_t len) {
    return crc32Final(crc32Update(crc32Init(), data, len));
}

}  // namespace logic
