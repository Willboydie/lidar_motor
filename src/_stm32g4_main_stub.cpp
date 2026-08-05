// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
// L2 — Motor phase windings (ohmmeter + phase-pair detent, X then Y).
#include "tests/windings/windings_l2_test.hpp"

void setup() {
    windings_l2_test();
}

void loop() {}

#endif
