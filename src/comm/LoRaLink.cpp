/**
 * @file LoRaLink.cpp
 * @brief Implementation of the SX1276 telemetry link.
 */

#include "LoRaLink.h"

#include <SPI.h>

#include "config.h"

bool LoRaLink::begin() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    int16_t state = _radio.begin(_cfg.freqMhz, _cfg.bwKhz, _cfg.sf, _cfg.cr,
                                 _cfg.syncWord, _cfg.txPowerDbm,
                                 8 /* preamble symbols */);
    if (state != RADIOLIB_ERR_NONE) {
        log_e("SX1276 init failed, RadioLib error %d", state);
        _ready = false;
        return false;
    }

    // CRC on, as specified in docs/lora-protocol.md.
    _radio.setCRC(true);

    log_i("SX1276 ready: %.1f MHz, BW %.0f kHz, SF%u, CR 4/%u, sync 0x%02X, %d dBm",
          _cfg.freqMhz, _cfg.bwKhz, _cfg.sf, _cfg.cr, _cfg.syncWord, _cfg.txPowerDbm);
    _ready = true;
    return true;
}

bool LoRaLink::send(const String &payload) {
    if (!_ready) {
        return false;
    }

    uint32_t toaMs =
        _radio.getTimeOnAir(payload.length()) / 1000;  // us -> ms
    uint32_t start = millis();
    int16_t state = _radio.transmit(payload.c_str());

    if (state != RADIOLIB_ERR_NONE) {
        log_e("LoRa TX failed, RadioLib error %d", state);
        return false;
    }

    log_d("LoRa TX ok: %u bytes, time-on-air %lu ms (est.), %lu ms elapsed",
          (unsigned)payload.length(), (unsigned long)toaMs,
          (unsigned long)(millis() - start));
    return true;
}

void LoRaLink::sleep() {
    // The SX1276 keeps its last mode across MCU deep sleep: without this
    // it would sit in standby (~1.6 mA) forever.
    _radio.sleep();
}
