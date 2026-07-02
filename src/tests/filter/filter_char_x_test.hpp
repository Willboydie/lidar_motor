#pragma once

#include <Arduino.h>
#include "../wiring/wiring_common.hpp"
#include "filter_common.hpp"

// L4 — Motor X: synthetic LPF / notch characterization (no hardware motion).
inline void filter_char_x_test() {
    Serial.print("TEST filter_char axis=X target=");
    Serial.println(wiring_target_name());
    Serial.println("Synthetic step + sine through _LowPassFilter; notch at NOTCH_FILTER_FREQ.");

    filter_run_characterization();

    Serial.println("Check: vel t63 ~ VELOCITY_LOW_PASS_FILTER_TAU; notch_center_db << notch_pass_db.");
    wiring_wait_for_confirm("filter plots: step lag and notch depth look correct");

    Serial.println("RESULT filter_char_x PASS");
}
