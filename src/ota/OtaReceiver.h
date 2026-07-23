/**
 * @file OtaReceiver.h
 * @brief Chunked file transfer over LoRa — stage 1 of the OTA pipeline.
 *
 * Implements the receiver side of the OTA transfer protocol described in
 * docs/lora-protocol.md. Pull model: the station requests every chunk
 * explicitly (`ota_req`), so acknowledgement, ordering and retransmission
 * are implicit — a lost or corrupted chunk is simply requested again.
 *
 * A full firmware image (~425 kB) is ~2400 chunks and takes ~20 minutes
 * at SF7, so an interrupted session must not start over: progress
 * (next chunk + running CRC) lives in RTC RAM and the transfer resumes
 * on a later wake-up as soon as the base offers the same image again.
 *
 * This stage (fw 3.0.0-alpha.2) stores the received image on LittleFS and
 * verifies its CRC-32. The next stages will stream into the inactive OTA
 * app partition (esp_ota_*), reboot with rollback, and verify a firmware
 * signature.
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
     * @brief Apply the runtime tuning from config.ini ([ota] section).
     * @param chunkTimeoutMs Wait for a chunk after each request [ms].
     * @param maxRetries Attempts per chunk before giving up.
     * @param sessionTimeoutS Cap on the awake time of one session [s].
     */
    void configure(uint16_t chunkTimeoutMs, uint8_t maxRetries, uint16_t sessionTimeoutS) {
        _chunkTimeoutMs = chunkTimeoutMs;
        _maxRetries = maxRetries;
        _sessionTimeoutS = sessionTimeoutS;
    }

    /**
     * @brief Run a transfer session (blocking), resuming if possible.
     *
     * Requests every missing chunk, streams it to LittleFS, verifies the
     * final CRC-32 and reports the outcome to the base with `ota_done`.
     * On failure the progress is kept in RTC RAM so the next session
     * continues from where this one stopped.
     *
     * @param offer Transfer metadata from the base ACK.
     * @return true if the image was fully received and the CRC matches.
     */
    bool runSession(const Offer &offer);

    /** @return Path of the received image on LittleFS. */
    static const char *imagePath() { return kImagePath; }

private:
    static constexpr uint8_t kChunkMagic = 0xA5;   ///< First byte of every chunk.
    static constexpr size_t kChunkHeader = 3;      ///< magic + index (u16 LE).
    static constexpr size_t kChunkData = 180;      ///< Payload bytes per chunk.
    static constexpr const char *kImagePath = "/ota_image.bin";

    /** @brief Send the `ota_req` JSON for chunk @p idx. */
    bool sendChunkRequest(uint16_t idx);

    /** @brief Send the final `ota_done` report. */
    void sendReport(bool ok, uint32_t crc);

    /** @brief Persist the resume point in RTC RAM. */
    void saveProgress(uint16_t nextChunk, uint32_t runningCrc);

    LoRaLink &_link;    ///< Radio link.
    const String &_id;  ///< Station identifier.

    uint16_t _chunkTimeoutMs = 1500;  ///< Wait per chunk [ms], from config.
    uint8_t _maxRetries = 8;          ///< Attempts per chunk, from config.
    uint16_t _sessionTimeoutS = 1800; ///< Session cap [s], from config.
};

#endif // WEATHERSTATION_OTARECEIVER_H
