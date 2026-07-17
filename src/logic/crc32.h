/**
 * @file crc32.h
 * @brief CRC-32 (IEEE 802.3, poly 0xEDB88320) — pure logic, native-tested.
 *
 * Used by the OTA transfer to verify the integrity of the received image
 * (docs/lora-protocol.md). Incremental API so large files can be checked
 * while streaming chunk by chunk:
 *
 * @code
 * uint32_t crc = logic::crc32Init();
 * crc = logic::crc32Update(crc, chunk, len);   // repeat per chunk
 * uint32_t final = logic::crc32Final(crc);
 * @endcode
 */

#ifndef WEATHERSTATION_LOGIC_CRC32_H
#define WEATHERSTATION_LOGIC_CRC32_H

#include <cstddef>
#include <cstdint>

namespace logic {

/** @return Initial CRC-32 accumulator value. */
constexpr uint32_t crc32Init() { return 0xFFFFFFFFu; }

/**
 * @brief Feed a buffer into the CRC-32 accumulator.
 * @param crc Current accumulator (from crc32Init() or a previous update).
 * @param data Bytes to process.
 * @param len Number of bytes.
 * @return Updated accumulator.
 */
uint32_t crc32Update(uint32_t crc, const uint8_t *data, std::size_t len);

/** @return Final CRC-32 value from the accumulator. */
constexpr uint32_t crc32Final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

/**
 * @brief One-shot CRC-32 of a buffer.
 * @param data Bytes to process.
 * @param len Number of bytes.
 * @return CRC-32 checksum.
 */
uint32_t crc32(const uint8_t *data, std::size_t len);

}  // namespace logic

#endif // WEATHERSTATION_LOGIC_CRC32_H
