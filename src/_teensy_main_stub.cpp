// Entry point for [env:teensy41] — switch the active bring-up test here.
#if defined(ARDUINO_TEENSY41)

#include <Arduino.h>
#include "tests/modulation/modulation_l7_test.hpp"

void setup() {
    modulation_l7_test();
}

void loop() {}

#endif
