#pragma once

#include <Arduino.h>
#include "pwm_sync_x_test.hpp"
#include "pwm_sync_y_test.hpp"
#include "pwm_sync_dual_test.hpp"
#include "pwm_sync_common.hpp"

// L6 — PWM synchronisation (motor X, motor Y, then both together).
// Invoked from setup(); main loop() stays empty.
inline void pwm_l6_test() {
    pwm_serial_begin();

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    Serial.print("TEST pwm_l6 target=");
    Serial.println(wiring_target_name());
    Serial.println("L6 — PWM synchronisation: per-motor timer/edge alignment + dual independence.");
    pwm_sync_bench_prompt();

    Serial.println();
    Serial.println("=== motor X: timer resolution + edge alignment ===");
    pwm_sync_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: timer resolution + edge alignment ===");
    pwm_sync_y_test();

    Serial.println();
    Serial.println("--- both axes at once ---");
    Serial.println();

    Serial.println("=== dual: distinct timers + no cross-talk ===");
    pwm_sync_dual_test();

    Serial.println();
    Serial.print("RESULT pwm_l6 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L6 complete — all drivers disabled. Fill the evidence log from SYNC/ASSESS/RESULT lines.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
