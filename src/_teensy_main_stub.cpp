// Entry point for [env:teensy41] — switch the active bring-up test here.
#if defined(ARDUINO_TEENSY41)

#include <Arduino.h>
// L10 — closed-loop commutation sign / torque test (real encoder commutation).
#include "tests/foc/torque_sign_l10_test.hpp"

void setup() {
    torque_sign_l10_test();
}

void loop() {}

#endif
