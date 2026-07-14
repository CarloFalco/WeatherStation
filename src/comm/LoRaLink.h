/**
 * @file LoRaLink.h
 * @brief ITelemetryLink implementation for the SX1276 LoRa radio (RadioLib).
 *
 * Raw LoRa point-to-point (not LoRaWAN) on the EU 868 MHz band. All radio
 * parameters come from config.ini ([lora] section) and must match the
 * receiver configuration: see docs/lora-protocol.md.
 *
 * Transmission is blocking; after each telemetry TX main.cpp opens a
 * short receive window waiting for the base ACK (see the "protocollo"
 * section of docs/lora-protocol.md). Both behaviours are driven from
 * config.ini ([lora] ack_enabled / ack_timeout_ms / tx_retries).
 */

#ifndef WEATHERSTATION_LORALINK_H
#define WEATHERSTATION_LORALINK_H

#include <RadioLib.h>

#include "ITelemetryLink.h"
#include "config.h"
#include "core/AppConfig.h"

/**
 * @brief SX1276 telemetry link driven through RadioLib.
 */
class LoRaLink : public ITelemetryLink {
public:
    /**
     * @param cfg Radio parameters (kept by reference: read at begin(),
     *            after config.ini has been loaded).
     */
    explicit LoRaLink(const AppConfig::LoraConfig &cfg)
        : _cfg(cfg), _module(LORA_CS, LORA_DIO0, LORA_RST, RADIOLIB_NC),
          _radio(&_module) {}

    /**
     * @brief Initialize SPI and the SX1276 with the configured parameters.
     * @return true if the radio answered and accepted the configuration.
     */
    bool begin() override;

    /**
     * @brief Transmit the payload (blocking, ~120 ms at SF7 for ~150 B).
     * @param payload Compact JSON telemetry message.
     * @return true on successful transmission.
     */
    bool send(const String &payload) override;

    /**
     * @brief Listen for one packet (RX single mode, polled until timeout).
     * @param payload Filled with the received message on success.
     * @param timeoutMs Maximum listening time [ms].
     * @return true if a packet passed CRC and was read within the timeout.
     */
    bool receive(String &payload, uint32_t timeoutMs) override;

    /**
     * @brief Put the SX1276 into sleep mode (~0.2 uA).
     */
    void sleep() override;

private:
    const AppConfig::LoraConfig &_cfg;  ///< Radio parameters from config.ini.
    Module _module;                     ///< RadioLib pin binding.
    SX1276 _radio;                      ///< RadioLib radio driver.
    bool _ready = false;                ///< begin() outcome.
};

#endif // WEATHERSTATION_LORALINK_H
