#pragma once

#include <Arduino.h>
#include "gpio_map_x_test.hpp"
#include "gpio_map_y_test.hpp"
#include "wiring_common.hpp"

// L1 — Wiring & power integrity (motor X then motor Y).
// Invoked from setup(); main loop() stays empty.
inline void wiring_l1_test() {
    wiring_serial_begin();

    Serial.print("TEST wiring_l1 target=");
    Serial.println(wiring_target_name());
    Serial.println("L1 — GPIO pin map + EN de-energise. Follow serial prompts.");

    gpio_map_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    gpio_map_y_test();

    Serial.println();
    Serial.print("RESULT wiring_l1 PASS target=");
    Serial.println(wiring_target_name());
    Serial.println("L1 complete — all pins held LOW. Safe to disconnect.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
