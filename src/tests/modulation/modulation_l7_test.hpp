#pragma once

#include <Arduino.h>
#include "modulation_x_test.hpp"
#include "modulation_y_test.hpp"
#include "modulation_common.hpp"

// L7 — Modulation math: Clarke/Park + midpoint-clamp SVPWM (motor X then Y).
// Invoked from setup(); main loop() stays empty.
inline void modulation_l7_test() {
    wiring_serial_begin();

    Serial.print("TEST modulation_l7 target=");
    Serial.println(wiring_target_name());
    Serial.println("L7 — setPhaseVoltage(): numeric assert + swept waveform (per axis).");
    modulation_bench_prompt();

    Serial.println();
    Serial.println("=== motor X: numeric check + swept waveform ===");
    modulation_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: numeric check + swept waveform ===");
    modulation_y_test();

    Serial.println();
    Serial.print("RESULT modulation_l7 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L7 complete — all drivers disabled. Fill the evidence log from MOD/DBG/RESULT lines.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
