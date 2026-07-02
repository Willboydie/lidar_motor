#pragma once

#include <Arduino.h>
#include "pwm_common.hpp"

// L5 — Motor Y: center-aligned mid-duty hold + disable cut verification.
inline void pwm_center_aligned_y_test() {
    Serial.print("TEST pwm_center_aligned axis=Y target=");
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
        Serial.println("RESULT pwm_center_aligned_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }
    driver.enable();

    pwm_center_aligned_hold(driver, "Y");
    pwm_disable_verify(driver);

    driver.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("RESULT pwm_center_aligned_y PASS");
}
