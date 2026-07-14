/**
 * @file ITelemetryLink.h
 * @brief Interface for the telemetry transport of the station.
 *
 * Decouples the measurement cycle from the physical link. The default
 * implementation is LoRaLink (SX1276 point-to-point to the gateway);
 * the requirements also foresee a future MqttLink (direct WiFi + MQTT)
 * selectable from config.ini — both plug in behind this interface
 * without touching main.cpp.
 */

#ifndef WEATHERSTATION_ITELEMETRYLINK_H
#define WEATHERSTATION_ITELEMETRYLINK_H

#include <Arduino.h>

/**
 * @brief Abstract telemetry transport.
 */
class ITelemetryLink {
public:
    virtual ~ITelemetryLink() = default;

    /**
     * @brief Initialize the transport hardware.
     * @return true if the link is ready to transmit.
     */
    virtual bool begin() = 0;

    /**
     * @brief Transmit one telemetry payload (blocking).
     * @param payload Compact JSON message (see docs/lora-protocol.md).
     * @return true if the payload was sent successfully.
     */
    virtual bool send(const String &payload) = 0;

    /**
     * @brief Listen for one incoming packet (blocking, with timeout).
     * @param payload Filled with the received message on success.
     * @param timeoutMs Maximum listening time [ms].
     * @return true if a packet was received within the timeout.
     */
    virtual bool receive(String &payload, uint32_t timeoutMs) = 0;

    /**
     * @brief Put the transport hardware into its lowest-power state.
     *
     * Called before entering deep sleep: the MCU power-down does not
     * automatically power down external radio chips.
     */
    virtual void sleep() = 0;
};

#endif // WEATHERSTATION_ITELEMETRYLINK_H
