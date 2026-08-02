/**
 * @file FactoryResetButton.h
 * @brief Physical push button that restores the factory settings.
 *
 * The station sleeps almost all the time, so the button is wired to an
 * RTC-capable pin and is a deep-sleep wake source (EXT1, together with the
 * rain gauge — see PowerManager): pressing it wakes the node, which then
 * asks the user to confirm by holding the button.
 *
 * Confirmation flow (hold-to-confirm, so a stray press cannot wipe the
 * configuration): the status LED blinks while the button is held; if it is
 * still held after the configured time the LED stays lit and the reset is
 * carried out, otherwise the boot continues normally.
 */

#ifndef WEATHERSTATION_FACTORYRESETBUTTON_H
#define WEATHERSTATION_FACTORYRESETBUTTON_H

#include <Arduino.h>

/**
 * @brief Debounced push button with hold-to-confirm feedback.
 */
class FactoryResetButton {
public:
    /** @brief Configure the input (pull-up, active low). */
    void begin();

    /** @return true while the button is held down. */
    bool isPressed() const;

    /**
     * @brief Wait for the button to be held for @p holdMs (blocking).
     *
     * Blinks the status LED while waiting and returns as soon as the
     * button is released, so an accidental press costs nothing.
     *
     * @param holdMs Hold time required to confirm [ms].
     * @return true if the button stayed pressed for the whole time.
     */
    bool confirmHold(uint32_t holdMs) const;

private:
    static constexpr uint32_t kDebounceMs = 50;  ///< Contact-bounce settle time.
};

#endif // WEATHERSTATION_FACTORYRESETBUTTON_H
