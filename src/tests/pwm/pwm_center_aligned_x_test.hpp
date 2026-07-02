#pragma once

#include <Arduino.h>
#include "pwm_common.hpp"

// L5 — Motor X: center-aligned mid-duty hold + disable cut verification.
inline void pwm_center_aligned_x_test() {
    Serial.print("TEST pwm_center_aligned axis=X target=");
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
        Serial.println("RESULT pwm_center_aligned_x FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }
    driver.enable();

    pwm_center_aligned_hold(driver, "X");
    pwm_disable_verify(driver);

    driver.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    Serial.println("RESULT pwm_center_aligned_x PASS");
}
