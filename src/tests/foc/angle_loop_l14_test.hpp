#pragma once

#include <Arduino.h>
#include "angle_loop_x_test.hpp"
#include "angle_loop_y_test.hpp"

// L14 — Angle (position) loop PID (motor X then Y). Both stm32g4 and teensy41.
inline void angle_loop_l14_test() {
    wiring_serial_begin();

    Serial.print("TEST angle_loop_l14 target=");
    Serial.println(wiring_target_name());
    Serial.println("L14 — angle loop PID (ANGLE mode, full cascade, shortest-path wrap).");
#if defined(STM32G4xx)
    Serial.println("STM32G4: depends on L13 velocity loop. Inner loop = measured-current PI.");
#elif defined(ARDUINO_TEENSY41)
    Serial.println("Teensy: depends on L13 velocity loop (skips L11/L12). Inner loop = estimate_Vq.");
#endif

    Serial.println();
    Serial.println("=== motor X: angle loop ===");
    angle_loop_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: angle loop ===");
    angle_loop_y_test();

    Serial.println();
    Serial.print("RESULT angle_loop_l14 DONE target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) { delay(1000); }
}
