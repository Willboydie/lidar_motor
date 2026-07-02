#pragma once

#include <Arduino.h>
#include "mcu_alive_test.hpp"
#include "loop_timing_test.hpp"
#include "../../utils/constants.h"

// L0 — MCU & toolchain gate (alive + loop timing + ongoing heartbeat).
// Invoked from setup(); main loop() stays empty.
inline void mcu_l0_test() {
#if defined(STM32G4xx)
    constexpr const char* kTarget = "g4";
#elif defined(ARDUINO_TEENSY41)
    constexpr const char* kTarget = "teensy";
#else
    constexpr const char* kTarget = "unknown";
#endif

    const bool micros_ok = mcu_alive_test();
    const bool timing_ok = loop_timing_test();

    Serial.println();
    Serial.print("RESULT mcu_l0 ");
    Serial.print((micros_ok && timing_ok) ? "PASS" : "FAIL");
    Serial.print(" target=");
    Serial.println(kTarget);
    Serial.println("L0 complete — entering 1 Hz heartbeat (LED + serial). Ctrl+C monitor to stop.");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

#if defined(USER_BTN)
    pinMode(USER_BTN, INPUT_PULLUP);
    bool last_btn = digitalRead(USER_BTN);
    Serial.println("Press USER_BTN to log transitions.");
#else
    Serial.println("USER_BTN not defined on this board.");
#endif

    unsigned long last_hb_ms = 0;
    bool led_on = false;

    while (true) {
        const unsigned long now = millis();

        if (now - last_hb_ms >= 1000UL) {
            last_hb_ms = now;
            led_on = !led_on;
            digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
            Serial.print("hb ");
            Serial.println(now);
        }

#if defined(USER_BTN)
        const bool btn = digitalRead(USER_BTN);
        if (btn != last_btn) {
            last_btn = btn;
            Serial.print("btn ");
            Serial.println(btn == LOW ? "pressed" : "released");
        }
#endif
    }
}
