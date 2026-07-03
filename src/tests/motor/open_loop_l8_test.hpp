#pragma once

#include <Arduino.h>
#include "open_loop_x_test.hpp"
#include "open_loop_y_test.hpp"

// L8 — Open-loop forced commutation (motor X then Y).
// Invoked from setup(); main loop() stays empty.
inline void open_loop_l8_test() {
    wiring_serial_begin();

    Serial.print("TEST open_loop_l8 target=");
    Serial.println(wiring_target_name());
    Serial.println("L8 — synthesized rotating field (Vd=0, fixed Vq); encoder reference-only.");
    Serial.println("Prove phase ORDER + modulation on the real motor: smooth, reversible motion.");
    Serial.println("Send 'r' to reverse the ramp; send 'q' to finish each axis.");

    Serial.println();
    Serial.println("=== motor X: open-loop forced commutation ===");
    open_loop_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: open-loop forced commutation ===");
    open_loop_y_test();

    Serial.println();
    Serial.print("RESULT open_loop_l8 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L8 complete — all drivers disabled. Fill the evidence log from DBG/RESULT lines.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
