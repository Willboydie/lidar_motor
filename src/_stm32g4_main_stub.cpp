// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "tests/modulation/modulation_l7_test.hpp"

void setup() {
    modulation_l7_test();
}

void loop() {}

#endif
