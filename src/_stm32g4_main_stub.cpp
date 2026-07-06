// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
// L15 — dual-axis simultaneous FOC (StateMachine + Interface + SinusoidalScanPath).
#include "tests/foc/dual_foc_test.hpp"

void setup() {
    dual_foc_test();
}

void loop() {}

#endif
