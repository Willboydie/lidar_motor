#pragma once

#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "../../pwm_driver/stm32g4/stm32_mcu.h"
#include "../../utils/constants.h"
#include "Arduino.h"

// Bench test: energize one BLDC phase at a time on motor X and/or Y.
// Feel for cogging (detent steps) as the rotor settles in the stator field.
// Use low voltage only — coils are connected.

#if defined(STM32G4xx)

namespace {

constexpr float kPhaseVolts = ALIGNMENT_VOLTAGE;  // 3.0 V — same as sensor alignment

enum class PhaseSel : uint8_t { A = 0, B = 1, C = 2 };

void print_help() {
    Serial.println();
    Serial.println("Coil cogging test — one phase energized at a time");
    Serial.print("  Phase voltage = ");
    Serial.print(kPhaseVolts, 1);
    Serial.println(" V (ALIGNMENT_VOLTAGE)");
    Serial.println();
    Serial.println("Motor X:");
    Serial.println("  1 = phase A (Va only)   2 = phase B   3 = phase C");
    Serial.println("Motor Y:");
    Serial.println("  4 = phase A             5 = phase B   6 = phase C");
    Serial.println();
    Serial.println("  0 or f = all off (both drivers disabled)");
    Serial.println("  S = auto-scan all 6 coils (3 s each, X then Y)");
    Serial.println("  h = this help");
    Serial.println();
    Serial.println("Bench: for each command, note whether the rotor snaps to");
    Serial.println("detents (cogging). Uneven or missing steps may indicate wiring");
    Serial.println("or pole-pair mismatch — compare X vs Y and A/B/C.");
}

void set_phase_pwm(PWMDriverSTM32G4& driver, PhaseSel phase) {
    float va = 0.0f;
    float vb = 0.0f;
    float vc = 0.0f;
    switch (phase) {
        case PhaseSel::A:
            va = kPhaseVolts;
            break;
        case PhaseSel::B:
            vb = kPhaseVolts;
            break;
        case PhaseSel::C:
            vc = kPhaseVolts;
            break;
    }
    driver.setPwm(va, vb, vc);
}

const char* phase_name(PhaseSel p) {
    switch (p) {
        case PhaseSel::A:
            return "A";
        case PhaseSel::B:
            return "B";
        case PhaseSel::C:
            return "C";
    }
    return "?";
}

void energize_coil(PWMDriverSTM32G4& active,
                   PWMDriverSTM32G4& other,
                   char motor_label,
                   PhaseSel phase) {
    other.enable();
    other.setPwm(0.0f, 0.0f, 0.0f);
    active.enable();
    set_phase_pwm(active, phase);
    Serial.print("ON  motor ");
    Serial.print(motor_label);
    Serial.print(" phase ");
    Serial.print(phase_name(phase));
    Serial.print("  (");
    Serial.print(kPhaseVolts, 1);
    Serial.println(" V on one phase, other phases 0)");
}

void all_off(PWMDriverSTM32G4& driver_x, PWMDriverSTM32G4& driver_y) {
    driver_x.disable();
    driver_y.disable();
    Serial.println("OFF both motors (disabled, EN low)");
}

}  // namespace

inline void stm32g4_coil_cogging_test() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);
    Serial.println("stm32g4 coil cogging test starting...");

    PWMDriverSTM32G4 driver_x(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 driver_y(N1_Y, N2_Y, N3_Y, EN_Y);

    if (!driver_x.init()) {
        Serial.println("FAIL: motor X PWM init");
        return;
    }
    if (!driver_y.init()) {
        Serial.println("FAIL: motor Y PWM init");
        return;
    }

    driver_x.enable();
    driver_y.enable();
    all_off(driver_x, driver_y);

    Serial.print("Both drivers init OK. motorsUsed=");
    Serial.println(stm32_getNumMotorsUsed());
    print_help();

    auto run_auto_scan = [&]() {
        Serial.println("AUTO-SCAN: 3 s per coil. Rotate shaft by hand between");
        Serial.println("steps if needed; note cogging at each stop.");
        constexpr unsigned long k_dwell_ms = 3000UL;
        const struct {
            PWMDriverSTM32G4& active;
            PWMDriverSTM32G4& other;
            char label;
            PhaseSel phase;
        } steps[] = {
            {driver_x, driver_y, 'X', PhaseSel::A},
            {driver_x, driver_y, 'X', PhaseSel::B},
            {driver_x, driver_y, 'X', PhaseSel::C},
            {driver_y, driver_x, 'Y', PhaseSel::A},
            {driver_y, driver_x, 'Y', PhaseSel::B},
            {driver_y, driver_x, 'Y', PhaseSel::C},
        };
        for (const auto& step : steps) {
            energize_coil(step.active, step.other, step.label, step.phase);
            delay(k_dwell_ms);
        }
        all_off(driver_x, driver_y);
        Serial.println("AUTO-SCAN done.");
    };

    while (true) {
        if (Serial.available() > 0) {
            const char cmd = static_cast<char>(Serial.read());
            switch (cmd) {
                case '1':
                    energize_coil(driver_x, driver_y, 'X', PhaseSel::A);
                    break;
                case '2':
                    energize_coil(driver_x, driver_y, 'X', PhaseSel::B);
                    break;
                case '3':
                    energize_coil(driver_x, driver_y, 'X', PhaseSel::C);
                    break;
                case '4':
                    energize_coil(driver_y, driver_x, 'Y', PhaseSel::A);
                    break;
                case '5':
                    energize_coil(driver_y, driver_x, 'Y', PhaseSel::B);
                    break;
                case '6':
                    energize_coil(driver_y, driver_x, 'Y', PhaseSel::C);
                    break;
                case '0':
                case 'f':
                case 'F':
                    all_off(driver_x, driver_y);
                    break;
                case 's':
                case 'S':
                    run_auto_scan();
                    break;
                case 'h':
                case 'H':
                case '?':
                    print_help();
                    break;
                case '\r':
                case '\n':
                    break;
                default:
                    Serial.print("Unknown '");
                    Serial.print(cmd);
                    Serial.println("'. Press h for help.");
                    break;
            }
        }
    }
}

#endif
