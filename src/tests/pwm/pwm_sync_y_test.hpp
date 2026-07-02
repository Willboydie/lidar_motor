#pragma once

#include <Arduino.h>
#include "pwm_sync_common.hpp"

// L6 — Motor Y: resolve the three phase timers, confirm frequency/edge alignment.
inline void pwm_sync_y_test() {
    Serial.print("TEST pwm_sync axis=Y target=");
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

    const int pins[3] = {N1_Y, N2_Y, N3_Y};
    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!driver.init()) {
        Serial.println("RESULT pwm_sync_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }

    PwmSyncInfo info;
    pwm_sync_resolve(driver, pins, info);
    pwm_sync_print_axis("Y", pins, info);

    driver.enable();
    pwm_sync_hold(driver, "Y");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("RESULT pwm_sync_y PASS");
}
