// Entry point for [env:stm32g4] until the full firmware main.cpp is ported.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "tests/sensor/electrical_offset_test.hpp"

void setup() {
  electrical_offset_test();
}

void loop() {}

#endif
