#pragma once

#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "../../pwm_driver/stm32g4/stm32_mcu.h"
#include "../../utils/constants.h"
#include "Arduino.h"

// Stage 5 bench test: zero-output stopPwmHardware with motor coils connected.
// Low duty on phase A — verify safe disable/enable without timer teardown or motorsUsed leak.

#if defined(STM32G4xx)

inline void stm32g4_safe_stop_test() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);
    Serial.println("stm32g4 safe-stop test starting...");

    PWMDriverSTM32G4 driver(N1_X, N2_X, N3_X, EN_X);
    if (!driver.init()) {
        Serial.println("Failed to initialize PWM driver");
        return;
    }

    driver.enable();
    Serial.print("Driver init OK. motorsUsed=");
    Serial.println(stm32_getNumMotorsUsed());
    Serial.println("Bench checks (motor connected, low duty):");
    Serial.println("  1. Command 'p' — setPwm(3.0, 0, 0), coil should cog slightly");
    Serial.println("  2. Command 'd' — disable(); outputs zero, EN_X low");
    Serial.println("  3. Command 'e' — enable + restore same duty (no re-init)");
    Serial.println("  4. Command 'c' — 10x disable/enable cycle, log motorsUsed");
    Serial.println("  5. Teensy: run driver_test.hpp unchanged (manual)");

    constexpr float kVa = 3.0f;
    constexpr float kVb = 0.0f;
    constexpr float kVc = 0.0f;
    bool pwm_active = false;

    auto apply_pwm = [&]() {
        driver.setPwm(kVa, kVb, kVc);
        pwm_active = true;
        Serial.println("PWM ON: Va=3.0 V, Vb=0.0 V, Vc=0.0 V");
    };

    auto run_stress_cycles = [&](int cycles) {
        Serial.print("Running ");
        Serial.print(cycles);
        Serial.println(" disable/enable cycles...");
        const int motors_before = stm32_getNumMotorsUsed();
        for (int i = 0; i < cycles; ++i) {
            apply_pwm();
            delay(20);
            driver.disable();
            pwm_active = false;
            delay(5);
            driver.enable();
            apply_pwm();
            delay(5);
            const int motors_now = stm32_getNumMotorsUsed();
            Serial.print("  cycle ");
            Serial.print(i + 1);
            Serial.print("/");
            Serial.print(cycles);
            Serial.print(" motorsUsed=");
            Serial.println(motors_now);
            if (motors_now != motors_before) {
                Serial.println("FAIL: motorsUsed changed — leak detected");
                return;
            }
        }
        Serial.println("Stress cycles complete — motorsUsed stable");
    };

    while (true) {
        if (Serial.available() > 0) {
            const char cmd = static_cast<char>(Serial.read());
            switch (cmd) {
                case 'p':
                case 'P':
                    driver.enable();
                    apply_pwm();
                    break;
                case 'd':
                case 'D':
                    driver.disable();
                    pwm_active = false;
                    Serial.println("disabled — outputs zero, EN_X low");
                    break;
                case 'e':
                case 'E':
                    driver.enable();
                    apply_pwm();
                    Serial.println("re-enabled with same duty (no init)");
                    break;
                case 'c':
                case 'C':
                    run_stress_cycles(10);
                    break;
                case '\r':
                case '\n':
                    break;
                default:
                    Serial.println("Unknown command. Use p, d, e, or c.");
                    break;
            }
        }

        static unsigned long last_status_ms = 0;
        const unsigned long now = millis();
        if (now - last_status_ms >= 2000UL) {
            last_status_ms = now;
            Serial.print("status: pwm_active=");
            Serial.print(pwm_active ? "true" : "false");
            Serial.print(" motorsUsed=");
            Serial.println(stm32_getNumMotorsUsed());
        }
    }
}

#endif
