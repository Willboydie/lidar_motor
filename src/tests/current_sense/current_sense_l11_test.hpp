#pragma once

#include <Arduino.h>
#include "current_sense_x_test.hpp"
#include "current_sense_y_test.hpp"

#if defined(STM32G4xx)

// L11 — Inline current-sense system validation (motor X then Y). STM32G4 only.
// Invoked from setup(); main loop() stays empty.
//
// Proves the ACS712 + ADC/TRGO/DMA capture path in isolation (offsets, baseline,
// fixed vector, sinusoid sweep) BEFORE the L12 current-target PI trusts it. The
// Teensy 4.1 build has no inline current sense and SKIPS L11/L12 (L10 -> L13).
inline void current_sense_l11_test() {
    wiring_serial_begin();

    Serial.print("TEST current_sense_l11 target=");
    Serial.println(wiring_target_name());
    Serial.println("L11 — validate inline current sensing (baseline + fixed + sinusoid capture).");
    Serial.println("Depends on L10 (commutation) proven; feeds L12 (current-target PI).");

    Serial.println();
    Serial.println("=== motor X: current-sense validation ===");
    current_sense_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: current-sense validation ===");
    current_sense_y_test();

    Serial.println();
    Serial.print("RESULT current_sense_l11 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L11 complete — all drivers disabled. Confirm capture per axis before L12.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}

#endif // STM32G4xx
