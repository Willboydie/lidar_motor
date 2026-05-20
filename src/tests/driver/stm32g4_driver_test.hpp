#pragma once

#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "../../pwm_driver/stm32g4/stm32_mcu.h"
#include "../../utils/constants.h"
#include "Arduino.h"

// Stage 3 bench test: edge-aligned 3PWM on motor X only (no coils connected).
// Serial commands drive PWM on/off for scope checks.
inline void stm32g4_driver_test() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);
    Serial.println("stm32g4 driver test starting...");

    PWMDriverSTM32G4 driver(N1_X, N2_X, N3_X, EN_X);
    if (!driver.init()) {
        Serial.println("Failed to initialize PWM driver");
        return;
    }

    driver.enable();
    Serial.println("Driver init OK. EN_X should be HIGH.");
    Serial.print("motorsUsed=");
    Serial.println(stm32_getNumMotorsUsed());
    Serial.println("Commands: o/on = PWM on (3.0, 1.5, 0.0 V), f/off = disable, e = enable + PWM on");

    bool pwm_active = false;

    while (true) {
        if (Serial.available() > 0) {
            const char cmd = static_cast<char>(Serial.read());
            switch (cmd) {
                case 'o':
                case 'O':
                case '1':
                    driver.setPwm(6.0f, 5.0f, 4.0f);
                    driver.enable();
                    pwm_active = true;
                    Serial.println("PWM ON: Va=6.0 V, Vb=5.0 V, Vc=4.0 V");
                    break;
                case 'f':
                case 'F':
                case '0':
                    driver.disable();
                    pwm_active = false;
                    Serial.println("PWM OFF: driver disabled");
                    break;
                case 'e':
                case 'E':
                    driver.enable();
                    driver.setPwm(6.0f, 5.0f, 4.0f);
                    pwm_active = true;
                    Serial.println("PWM ON: re-enabled with Va=3.0 V, Vb=1.5 V, Vc=0.0 V");
                    break;
                case '\r':
                case '\n':
                    break;
                default:
                    Serial.println("Unknown command. Use o/on, f/off, or e.");
                    break;
            }
        }

        static unsigned long last_status_ms = 0;
        const unsigned long now = millis();
        if (now - last_status_ms >= 2000UL) {
            last_status_ms = now;
            Serial.print("status: pwm_active=");
            Serial.println(pwm_active ? "true" : "false");
        }
    }
}
