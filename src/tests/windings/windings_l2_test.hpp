#pragma once

#include <Arduino.h>
#include "phase_energize_x_test.hpp"
#include "phase_energize_y_test.hpp"
#include "windings_common.hpp"

// L2 — Motor phase windings (motor X then motor Y).
// Invoked from setup(); main loop() stays empty.
inline void windings_l2_test() {
    windings_serial_begin();

    Serial.print("TEST windings_l2 target=");
    Serial.println(wiring_target_name());
    Serial.println("L2 — ohmmeter + phase-pair detent energise. Follow serial prompts.");

    phase_energize_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    phase_energize_y_test();

    Serial.println();
    Serial.print("RESULT windings_l2 PASS target=");
    Serial.println(wiring_target_name());
    Serial.println("L2 complete — all drivers disabled. Safe to disconnect.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
