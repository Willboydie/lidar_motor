#pragma once

#include <Arduino.h>
#include "../../utils/constants.h"

inline const char* wiring_target_name() {
#if defined(STM32G4xx)
    return "g4";
#elif defined(ARDUINO_TEENSY41)
    return "teensy";
#else
    return "unknown";
#endif
}

inline void wiring_serial_begin() {
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    Serial.println();
}

// Drive n1/n2/n3/en as plain GPIO outputs, all LOW.
inline void wiring_pins_all_low(int n1, int n2, int n3, int en) {
    pinMode(n1, OUTPUT);
    pinMode(n2, OUTPUT);
    pinMode(n3, OUTPUT);
    pinMode(en, OUTPUT);
    digitalWrite(n1, LOW);
    digitalWrite(n2, LOW);
    digitalWrite(n3, LOW);
    digitalWrite(en, LOW);
}

// Hold each named pin HIGH for dwell_ms while the others stay LOW.
inline void wiring_gpio_map_sequence(
    const char* axis,
    int n1, int n2, int n3, int en,
    uint32_t dwell_ms = 2000) {

    struct PinStep {
        const char* name;
        int pin;
    };

    const PinStep steps[] = {
        {"N1", n1},
        {"N2", n2},
        {"N3", n3},
        {"EN", en},
    };

    wiring_pins_all_low(n1, n2, n3, en);

    for (const PinStep& step : steps) {
        wiring_pins_all_low(n1, n2, n3, en);
        digitalWrite(step.pin, HIGH);
        Serial.print("PIN ");
        Serial.print(step.name);
        Serial.print(" axis=");
        Serial.print(axis);
        Serial.print(" pin=");
        Serial.print(step.pin);
        Serial.println(" HIGH");
        delay(dwell_ms);
    }

    wiring_pins_all_low(n1, n2, n3, en);
}

// Operator confirms a manual bench step (USER_BTN on Nucleo, or 'y' on serial).
inline void wiring_wait_for_confirm(const char* step) {
    Serial.println();
    Serial.print("CONFIRM ");
    Serial.println(step);
#if defined(USER_BTN)
    pinMode(USER_BTN, INPUT_PULLUP);
    Serial.println("Press USER_BTN or send 'y' on serial.");
#else
    Serial.println("Send 'y' on serial to confirm.");
#endif

    while (true) {
#if defined(USER_BTN)
        if (digitalRead(USER_BTN) == LOW) {
            delay(30);
            while (digitalRead(USER_BTN) == LOW) {
                delay(10);
            }
            Serial.println("CONFIRMED");
            return;
        }
#endif
        if (Serial.available()) {
            const char c = static_cast<char>(Serial.read());
            if (c == 'y' || c == 'Y') {
                Serial.println("CONFIRMED");
                return;
            }
        }
        delay(10);
    }
}
