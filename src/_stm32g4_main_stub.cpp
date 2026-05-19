// Entry point for [env:stm32g4] until the full firmware main.cpp is ported.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "tests/system/stm32g4_heartbeat_test.hpp"

void setup() {
  stm32g4_heartbeat_test();
}

void loop() {}

#endif
