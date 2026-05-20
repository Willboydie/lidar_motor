// Entry point for [env:stm32g4] until the full firmware main.cpp is ported.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "tests/driver/stm32g4_center_aligned_test.hpp"

void setup() {
  stm32g4_center_aligned_test();
}

void loop() {}

#endif
