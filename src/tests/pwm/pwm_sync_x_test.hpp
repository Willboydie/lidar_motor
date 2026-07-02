#pragma once

#include <Arduino.h>
#include "pwm_sync_common.hpp"

// L6 — Motor X: resolve the three phase timers, confirm frequency/edge alignment.
inline void pwm_sync_x_test() {
    Serial.print("TEST pwm_sync axis=X target=");
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

    const int pins[3] = {N1_X, N2_X, N3_X};
    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);
    if (!driver.init()) {
        Serial.println("RESULT pwm_sync_x FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }

    PwmSyncInfo info;
    pwm_sync_resolve(driver, pins, info);
    pwm_sync_print_axis("X", pins, info);

    driver.enable();
    pwm_sync_hold(driver, "X");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    Serial.println("RESULT pwm_sync_x PASS");
}
