#pragma once

#include <Arduino.h>
#include "current_loop_pi_x_test.hpp"
#include "current_loop_pi_y_test.hpp"

#if defined(STM32G4xx)

inline void current_loop_pi_l12_test() {
    wiring_serial_begin();

    Serial.print("TEST current_loop_pi_l12 target=");
    Serial.println(wiring_target_name());
    Serial.println("L12 — measured-current PI tracks Iq commands (commutation + PI).");
    Serial.println("Depends on L11 current-sense capture validated.");

    Serial.println();
    Serial.println("=== motor X: current-target PI ===");
    current_loop_pi_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: current-target PI ===");
    current_loop_pi_y_test();

    Serial.println();
    Serial.print("RESULT current_loop_pi_l12 DONE target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) { delay(1000); }
}

#endif
