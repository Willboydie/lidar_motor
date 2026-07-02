#pragma once

#include <Arduino.h>
#include "velocity_estimate_x_test.hpp"
#include "velocity_estimate_y_test.hpp"
#include "../filter/filter_char_x_test.hpp"
#include "../filter/filter_char_y_test.hpp"
#include "../wiring/wiring_common.hpp"

// L4 — Velocity estimate, LPF, and notch characterization (X then Y).
// Invoked from setup(); main loop() stays empty.
inline void sensor_l4_test() {
    wiring_serial_begin();

    Serial.print("TEST sensor_l4 target=");
    Serial.println(wiring_target_name());
    Serial.println("L4 — velocity estimate + filter characterization. Follow serial prompts.");

    Serial.println();
    Serial.println("=== motor X: filter characterization (synthetic) ===");
    filter_char_x_test();

    Serial.println();
    Serial.println("=== motor X: velocity estimate (hand rotation) ===");
    velocity_estimate_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: filter characterization (synthetic) ===");
    filter_char_y_test();

    Serial.println();
    Serial.println("=== motor Y: velocity estimate (hand rotation) ===");
    velocity_estimate_y_test();

    Serial.println();
    Serial.print("RESULT sensor_l4 PASS target=");
    Serial.println(wiring_target_name());
    Serial.println("L4 complete — sensors deinitialised.");

    while (true) {
        delay(1000);
    }
}
