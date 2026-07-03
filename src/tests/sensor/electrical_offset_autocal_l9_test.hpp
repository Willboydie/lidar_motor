#pragma once

#include <Arduino.h>
#include "electrical_offset_autocal_x_test.hpp"
#include "electrical_offset_autocal_y_test.hpp"

// L9 (part 2) — Electrical-offset autocal (motor X then Y).
// Invoked from setup(); main loop() stays empty.
//
// Workflow: run this, read the two `RESULT ... offset_rad=` values, write them
// into SENSOR_OFFSET_X / SENSOR_OFFSET_Y in src/utils/constants.h, then rebuild
// and re-run to verify `change_vs_stored_rad ~= 0` for both axes.
inline void electrical_offset_autocal_l9_test() {
    wiring_serial_begin();

    Serial.print("TEST electrical_offset_autocal_l9 target=");
    Serial.println(wiring_target_name());
    Serial.println("L9.2 — stiction-bracket + circular-mean electrical-offset autocal.");
    Serial.println("Measures SENSOR_OFFSET for each axis; write results to constants.h and re-run.");

    Serial.println();
    Serial.println("=== motor X: electrical-offset autocal ===");
    electrical_offset_autocal_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: electrical-offset autocal ===");
    electrical_offset_autocal_y_test();

    Serial.println();
    Serial.print("RESULT electrical_offset_autocal_l9 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L9.2 complete — all drivers disabled. Store offsets in constants.h, then re-run to verify.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
