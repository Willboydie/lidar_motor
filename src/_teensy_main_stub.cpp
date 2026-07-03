// Entry point for [env:teensy41] — switch the active bring-up test here.
#if defined(ARDUINO_TEENSY41)

#include <Arduino.h>
// L14 — angle (position) loop PID (Teensy skips L11/L12; inner loop = estimate_Vq voltage model).
#include "tests/foc/angle_loop_l14_test.hpp"

void setup() {
    angle_loop_l14_test();
}

void loop() {}

#endif
