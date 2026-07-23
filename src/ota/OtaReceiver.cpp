/**
 * @file OtaReceiver.cpp
 * @brief Implementation of the chunked OTA transfer receiver.
 */

#include "OtaReceiver.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "logic/crc32.h"
#include "rtc_state.h"

bool OtaReceiver::sendChunkRequest(uint16_t idx) {
    JsonDocument doc;
    doc["type"] = "ota_req";
    doc["id"] = _id;
    doc["idx"] = idx;
    String out;
    serializeJson(doc, out);
    return _link.send(out);
}

void OtaReceiver::sendReport(bool ok, uint32_t crc) {
    JsonDocument doc;
    doc["type"] = "ota_done";
    doc["id"] = _id;
    doc["ok"] = ok;
    doc["crc"] = crc;
    doc["next"] = g_rtcState.ota.nextChunk;  // resume point for the base
    String out;
    serializeJson(doc, out);
    _link.send(out);
}

void OtaReceiver::saveProgress(uint16_t nextChunk, uint32_t runningCrc) {
    g_rtcState.ota.nextChunk = nextChunk;
    g_rtcState.ota.runningCrc = runningCrc;
}

bool OtaReceiver::runSession(const Offer &offer) {
    // Sanity checks on the offer before spending airtime.
    if (offer.size == 0 || offer.chunks == 0 ||
        (uint32_t)offer.chunks * kChunkData < offer.size ||
        (uint32_t)(offer.chunks - 1) * kChunkData >= offer.size) {
        log_e("OTA: inconsistent offer (size=%lu, chunks=%u)",
              (unsigned long)offer.size, offer.chunks);
        return false;
    }

    // Resume only if the base is offering exactly the image we already
    // started, and the file on disk is as long as our progress claims.
    bool resuming = g_rtcState.ota.size == offer.size &&
                    g_rtcState.ota.crc == offer.crc &&
                    g_rtcState.ota.chunks == offer.chunks &&
                    g_rtcState.ota.nextChunk > 0 &&
                    g_rtcState.ota.nextChunk < offer.chunks;

    uint16_t startChunk = 0;
    uint32_t crc = logic::crc32Init();

    if (resuming) {
        File probe = LittleFS.open(kImagePath, "r");
        uint32_t onDisk = probe ? probe.size() : 0;
        if (probe) {
            probe.close();
        }
        if (onDisk == (uint32_t)g_rtcState.ota.nextChunk * kChunkData) {
            startChunk = g_rtcState.ota.nextChunk;
            crc = g_rtcState.ota.runningCrc;
        } else {
            log_w("OTA: resume point %u does not match file size %lu, restarting",
                  g_rtcState.ota.nextChunk, (unsigned long)onDisk);
            resuming = false;
        }
    }

    if (!resuming) {
        g_rtcState.ota.size = offer.size;
        g_rtcState.ota.crc = offer.crc;
        g_rtcState.ota.chunks = offer.chunks;
        saveProgress(0, logic::crc32Init());
    }

    File file = LittleFS.open(kImagePath, resuming ? "a" : "w");
    if (!file) {
        log_e("OTA: cannot open %s for writing", kImagePath);
        return false;
    }

    if (resuming) {
        Serial.printf("OTA transfer: RESUME from chunk %u/%u (%lu bytes, version %s)\n",
                      startChunk, offer.chunks, (unsigned long)offer.size,
                      offer.version.c_str());
    } else {
        Serial.printf("OTA transfer: %lu bytes in %u chunks (version %s)\n",
                      (unsigned long)offer.size, offer.chunks, offer.version.c_str());
    }

    uint32_t sessionStart = millis();
    uint32_t sessionCapMs = (uint32_t)_sessionTimeoutS * 1000UL;
    uint8_t buf[RADIOLIB_SX127X_MAX_PACKET_LENGTH + 1];
    uint16_t idx = startChunk;
    bool ok = true;

    for (; idx < offer.chunks && ok; idx++) {
        size_t expected = offer.size - (uint32_t)idx * kChunkData;
        if (expected > kChunkData) {
            expected = kChunkData;
        }

        for (uint8_t attempt = 1;; attempt++) {
            if (millis() - sessionStart > sessionCapMs) {
                log_e("OTA: session time cap (%u s) reached at chunk %u",
                      _sessionTimeoutS, idx);
                ok = false;
                break;
            }
            if (attempt > _maxRetries) {
                log_e("OTA: chunk %u failed after %u attempts", idx, _maxRetries);
                ok = false;
                break;
            }
            if (attempt > 1) {
                // Short randomized backoff: avoids locking into a
                // request/chunk collision pattern with the base.
                delay(20 + random(0, 60));
            }

            if (!sendChunkRequest(idx)) {
                continue;
            }
            size_t len = 0;
            if (!_link.receiveRaw(buf, sizeof(buf), len, _chunkTimeoutMs)) {
                log_w("OTA: chunk %u timeout (attempt %u)", idx, attempt);
                continue;
            }

            uint16_t rxIdx = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
            if (len != expected + kChunkHeader || buf[0] != kChunkMagic || rxIdx != idx) {
                log_w("OTA: bad chunk (len=%u, idx=%u, expected %u/%u)",
                      (unsigned)len, rxIdx, (unsigned)(expected + kChunkHeader), idx);
                continue;
            }

            file.write(buf + kChunkHeader, expected);
            crc = logic::crc32Update(crc, buf + kChunkHeader, expected);
            break;
        }

        if (ok && ((idx + 1) % 32 == 0 || idx + 1 == offer.chunks)) {
            uint32_t elapsedS = (millis() - sessionStart) / 1000;
            uint32_t done = idx + 1 - startChunk;
            uint32_t etaS = done ? (uint32_t)((uint64_t)elapsedS *
                                              (offer.chunks - idx - 1) / done)
                                 : 0;
            Serial.printf("OTA: %u/%u chunks (%u%%), elapsed %lu s, ETA %lu s\n",
                          idx + 1, offer.chunks,
                          (unsigned)(100UL * (idx + 1) / offer.chunks),
                          (unsigned long)elapsedS, (unsigned long)etaS);
        }
    }
    file.flush();
    file.close();

    // idx is the first chunk NOT stored: that is exactly the resume point.
    saveProgress(idx, crc);

    uint32_t finalCrc = logic::crc32Final(crc);
    bool success = ok && idx == offer.chunks && finalCrc == offer.crc;
    if (ok && !success) {
        log_e("OTA: CRC mismatch (got %08lX, expected %08lX)",
              (unsigned long)finalCrc, (unsigned long)offer.crc);
    }

    sendReport(success, finalCrc);

    if (success) {
        g_rtcState.ota = {};  // transfer complete: nothing to resume
        Serial.printf("OTA transfer SUCCESS: %lu bytes in %lu s (CRC %08lX)\n",
                      (unsigned long)offer.size,
                      (unsigned long)((millis() - sessionStart) / 1000),
                      (unsigned long)finalCrc);
    } else {
        Serial.printf("OTA transfer INCOMPLETE: %u/%u chunks stored, "
                      "will resume next cycle\n",
                      idx, offer.chunks);
    }
    return success;
}
