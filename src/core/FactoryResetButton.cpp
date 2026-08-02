/**
 * @file FactoryResetButton.cpp
 * @brief Implementation of the factory-reset push button.
 */

#include "FactoryResetButton.h"

#include "config.h"

void FactoryResetButton::begin() {
    pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
}

bool FactoryResetButton::isPressed() const {
    if (digitalRead(FACTORY_RESET_PIN) != LOW) {
        return false;
    }
    delay(kDebounceMs);
    return digitalRead(FACTORY_RESET_PIN) == LOW;
}

bool FactoryResetButton::confirmHold(uint32_t holdMs) const {
    uint32_t start = millis();
    while (millis() - start < holdMs) {
        if (digitalRead(FACTORY_RESET_PIN) != LOW) {
            digitalWrite(STATUS_LED_PIN, LOW);
            return false;  // released too early: treat as an accidental press
        }
        // Blink to show the countdown is running.
        digitalWrite(STATUS_LED_PIN, ((millis() - start) / 150) % 2);
        delay(10);
    }
    digitalWrite(STATUS_LED_PIN, HIGH);  // solid: confirmed
    return true;
}
