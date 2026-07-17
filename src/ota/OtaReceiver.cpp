/**
 * @file OtaReceiver.cpp
 * @brief Implementation of the chunked OTA transfer receiver.
 */

#include "OtaReceiver.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "logic/crc32.h"

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
    String out;
    serializeJson(doc, out);
    _link.send(out);
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

    Serial.printf("OTA transfer: %lu bytes in %u chunks (version %s)\n",
                  (unsigned long)offer.size, offer.chunks, offer.version.c_str());

    File file = LittleFS.open(kTestFilePath, "w");
    if (!file) {
        log_e("OTA: cannot open %s for writing", kTestFilePath);
        return false;
    }

    uint32_t crc = logic::crc32Init();
    uint32_t sessionStart = millis();
    uint8_t buf[RADIOLIB_SX127X_MAX_PACKET_LENGTH + 1];
    bool ok = true;

    for (uint16_t idx = 0; idx < offer.chunks && ok; idx++) {
        size_t expected = offer.size - (uint32_t)idx * kChunkData;
        if (expected > kChunkData) {
            expected = kChunkData;
        }

        for (uint8_t attempt = 1;; attempt++) {
            if (millis() - sessionStart > kSessionTimeoutMs) {
                log_e("OTA: session timeout at chunk %u", idx);
                ok = false;
                break;
            }
            if (attempt > kMaxRetries) {
                log_e("OTA: chunk %u failed after %u attempts", idx, kMaxRetries);
                ok = false;
                break;
            }

            if (!sendChunkRequest(idx)) {
                continue;
            }
            size_t len = 0;
            if (!_link.receiveRaw(buf, sizeof(buf), len, kChunkTimeoutMs)) {
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

        if (ok && (idx + 1) % 8 == 0) {
            Serial.printf("OTA: %u/%u chunks\n", idx + 1, offer.chunks);
        }
    }
    file.close();

    uint32_t finalCrc = logic::crc32Final(crc);
    bool success = ok && finalCrc == offer.crc;
    if (ok && !success) {
        log_e("OTA: CRC mismatch (got %08lX, expected %08lX)",
              (unsigned long)finalCrc, (unsigned long)offer.crc);
    }

    sendReport(success, finalCrc);

    Serial.printf("OTA transfer %s: %lu bytes in %.1f s (CRC %08lX)\n",
                  success ? "SUCCESS" : "FAILED", (unsigned long)offer.size,
                  (millis() - sessionStart) / 1000.0, (unsigned long)finalCrc);
    return success;
}
