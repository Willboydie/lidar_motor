// Entry point for [env:stm32g4] — switch the active bring-up test here.
#if defined(STM32G4xx)

#include <Arduino.h>
// L14 — angle (position) loop PID (ANGLE mode, full cascade, shortest-path wrap).
#include "tests/foc/angle_loop_l14_test.hpp"

void setup() {
    angle_loop_l14_test();
}

void loop() {}

#endif
