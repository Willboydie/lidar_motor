#pragma once

#include <Arduino.h>
#include "velocity_loop_x_test.hpp"
#include "velocity_loop_y_test.hpp"

// L13 — Velocity loop PID (motor X then Y). Both stm32g4 and teensy41.
inline void velocity_loop_l13_test() {
    wiring_serial_begin();

    Serial.print("TEST velocity_loop_l13 target=");
    Serial.println(wiring_target_name());
    Serial.println("L13 — velocity loop PID (VELOCITY mode, cascaded inner current loop).");
#if defined(STM32G4xx)
    Serial.println("STM32G4: depends on L12 current PI. Inner loop = measured-current PI.");
#elif defined(ARDUINO_TEENSY41)
    Serial.println("Teensy: depends on L10 commutation (skips L11/L12). Inner loop = estimate_Vq.");
#endif

    Serial.println();
    Serial.println("=== motor X: velocity loop ===");
    velocity_loop_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: velocity loop ===");
    velocity_loop_y_test();

    Serial.println();
    Serial.print("RESULT velocity_loop_l13 DONE target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) { delay(1000); }
}
