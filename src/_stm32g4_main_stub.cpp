// Entry point for [env:stm32g4] until the full firmware main.cpp is ported.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "tests/ldm/stm32g4_ldm_test.hpp"

void setup() {
  stm32g4_ldm_test();
}

void loop() {}

#endif
