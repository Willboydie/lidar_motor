// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
// L10 — closed-loop commutation sign / torque test (real encoder commutation).
#include "tests/foc/torque_sign_l10_test.hpp"

void setup() {
    torque_sign_l10_test();
}

void loop() {}

#endif
