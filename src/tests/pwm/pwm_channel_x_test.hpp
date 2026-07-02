#pragma once

#include <Arduino.h>
#include "pwm_common.hpp"

// L5 — Motor X: independent per-channel duty sweep (scope verification).
inline void pwm_channel_x_test() {
    Serial.print("TEST pwm_channel axis=X target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.print(N3_X);
    Serial.print(" EN=");
    Serial.println(EN_X);

    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);
    if (!driver.init()) {
        Serial.println("RESULT pwm_channel_x FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }
    driver.enable();

    pwm_channel_sweep(driver, "X");

    Serial.println();
    Serial.println("SCOPE — for each channel N1/N2/N3:");
    Serial.println("  duty steps 0.00, 0.25, 0.50, 0.75, 1.00 while others stay at 0.");
    wiring_wait_for_confirm("scope: each channel independently controllable 0-100% duty");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    Serial.println("RESULT pwm_channel_x PASS");
}
