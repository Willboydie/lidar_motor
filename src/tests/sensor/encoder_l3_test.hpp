#pragma once

#include <Arduino.h>
#include "encoder_integrity_x_test.hpp"
#include "encoder_integrity_y_test.hpp"
#include "sensor_common.hpp"

// L3 — Encoder integrity (motor X then motor Y).
// Invoked from setup(); main loop() stays empty.
inline void encoder_l3_test() {
    wiring_serial_begin();

    Serial.print("TEST encoder_l3 target=");
    Serial.println(wiring_target_name());
    Serial.println("L3 — AS5048 integrity, diagnostics, rest noise, hand rotation. Follow serial prompts.");

    encoder_integrity_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    encoder_integrity_y_test();

    Serial.println();
    Serial.print("RESULT encoder_l3 PASS target=");
    Serial.println(wiring_target_name());
    Serial.println("L3 complete — sensors deinitialised.");

    while (true) {
        delay(1000);
    }
}
