// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
// L0 — MCU & toolchain (alive + loop timing + heartbeat).
#include "tests/mcu/mcu_l0_test.hpp"

void setup() {
    mcu_l0_test();
}

void loop() {}

#endif
