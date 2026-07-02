#pragma once

#include <Arduino.h>
#include "pwm_channel_x_test.hpp"
#include "pwm_channel_y_test.hpp"
#include "pwm_center_aligned_x_test.hpp"
#include "pwm_center_aligned_y_test.hpp"
#include "pwm_common.hpp"

// L5 — PWM signal integrity (motor X then motor Y).
// Invoked from setup(); main loop() stays empty.
inline void pwm_l5_test() {
    pwm_serial_begin();

    Serial.print("TEST pwm_l5 target=");
    Serial.println(wiring_target_name());
    Serial.println("L5 — PWM channel duty + center-aligned waveform. Scope required.");
    pwm_bench_prompt();

    Serial.println();
    Serial.println("=== motor X: per-channel duty sweep ===");
    pwm_channel_x_test();

    Serial.println();
    Serial.println("=== motor X: center-aligned hold + disable ===");
    pwm_center_aligned_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: per-channel duty sweep ===");
    pwm_channel_y_test();

    Serial.println();
    Serial.println("=== motor Y: center-aligned hold + disable ===");
    pwm_center_aligned_y_test();

    Serial.println();
    Serial.print("RESULT pwm_l5 PASS target=");
    Serial.println(wiring_target_name());
    Serial.println("L5 complete — all drivers disabled. Safe to disconnect.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
