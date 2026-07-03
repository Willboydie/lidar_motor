#pragma once

#include <Arduino.h>
#include "pole_pair_dir_x_test.hpp"
#include "pole_pair_dir_y_test.hpp"

// L9 (part 1) — Pole pairs & sensor direction (motor X then Y).
// Invoked from setup(); main loop() stays empty.
inline void pole_pair_dir_l9_test() {
    wiring_serial_begin();

    Serial.print("TEST pole_pair_dir_l9 target=");
    Serial.println(wiring_target_name());
    Serial.println("L9.1 — quasi-static field sweep over one mech rev; fit elec-vs-mech slope.");
    Serial.println("Proves pole_pairs (|slope|) and sensor_direction (sign of slope).");

    Serial.println();
    Serial.println("=== motor X: pole-pair / direction ===");
    pole_pair_dir_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: pole-pair / direction ===");
    pole_pair_dir_y_test();

    Serial.println();
    Serial.print("RESULT pole_pair_dir_l9 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L9.1 complete — all drivers disabled. Confirm pole_pairs=7 and dir before autocal.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
