/**
 * @file OtaReceiver.h
 * @brief Chunked file transfer over LoRa — stage 1 of the OTA pipeline.
 *
 * Implements the receiver side of the OTA transfer protocol described in
 * docs/lora-protocol.md. Pull model: the station requests every chunk
 * explicitly (`ota_req`), so acknowledgement, ordering and retransmission
 * are implicit — a lost or corrupted chunk is simply requested again.
 *
 * This stage (fw 3.0.0-alpha.1) stores the received image on LittleFS and
 * verifies its CRC-32: it validates the transport with small test files.
 * The next stages will stream into the inactive OTA app partition
 * (esp_ota_*), reboot with rollback, and verify a firmware signature.
 */

#ifndef WEATHERSTATION_OTARECEIVER_H
#define WEATHERSTATION_OTARECEIVER_H

#include <Arduino.h>

#include "comm/LoRaLink.h"

/**
 * @brief Receives a chunked binary image over LoRa and verifies it.
 */
class OtaReceiver {
public:
    /** @brief Transfer metadata announced by the base inside the ACK. */
    struct Offer {
        uint32_t size = 0;    ///< Total image size [bytes].
        uint32_t crc = 0;     ///< Expected CRC-32 of the whole image.
        uint16_t chunks = 0;  ///< Number of chunks (ceil(size / 180)).
        String version;       ///< Version advertised by the base (informative).
    };

    /**
     * @param link Radio link (LoRa-specific: raw chunks bypass JSON).
     * @param stationId Station identifier put into every request.
     */
    OtaReceiver(LoRaLink &link, const String &stationId)
        : _link(link), _id(stationId) {}

    /**
     * @brief Run the whole transfer session (blocking).
     *
     * Requests every chunk, streams it to LittleFS, verifies the final
     * CRC-32 and reports the outcome to the base with `ota_done`.
     *
     * @param offer Transfer metadata from the base ACK.
     * @return true if the image was fully received and the CRC matches.
     */
    bool runSession(const Offer &offer);

    /** @return Path of the received test image on LittleFS. */
    static const char *testFilePath() { return kTestFilePath; }

private:
    static constexpr uint8_t kChunkMagic = 0xA5;   ///< First byte of every chunk.
    static constexpr size_t kChunkHeader = 3;      ///< magic + index (u16 LE).
    static constexpr size_t kChunkData = 180;      ///< Payload bytes per chunk.
    static constexpr uint8_t kMaxRetries = 8;      ///< Attempts per chunk.
    static constexpr uint32_t kChunkTimeoutMs = 1500;    ///< RX wait per request.
    static constexpr uint32_t kSessionTimeoutMs = 120000; ///< Whole-session cap.
    static constexpr const char *kTestFilePath = "/ota_test.bin";

    /** @brief Send the `ota_req` JSON for chunk @p idx. */
    bool sendChunkRequest(uint16_t idx);

    /** @brief Send the final `ota_done` report. */
    void sendReport(bool ok, uint32_t crc);

    LoRaLink &_link;    ///< Radio link.
    const String &_id;  ///< Station identifier.
};

#endif // WEATHERSTATION_OTARECEIVER_H
