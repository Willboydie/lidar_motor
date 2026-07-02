#pragma once

#include <Arduino.h>
#include "pwm_common.hpp"

// L5 — Motor Y: independent per-channel duty sweep (scope verification).
inline void pwm_channel_y_test() {
    Serial.print("TEST pwm_channel axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.print(N3_Y);
    Serial.print(" EN=");
    Serial.println(EN_Y);

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!driver.init()) {
        Serial.println("RESULT pwm_channel_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }
    driver.enable();

    pwm_channel_sweep(driver, "Y");

    Serial.println();
    Serial.println("SCOPE — for each channel N1/N2/N3:");
    Serial.println("  duty steps 0.00, 0.25, 0.50, 0.75, 1.00 while others stay at 0.");
    wiring_wait_for_confirm("scope: each channel independently controllable 0-100% duty");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("RESULT pwm_channel_y PASS");
}
