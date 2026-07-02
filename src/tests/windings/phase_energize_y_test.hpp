#pragma once

#include <Arduino.h>
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "windings_common.hpp"

// L2 — Motor Y: DC energise each phase pair for detent check.
inline void phase_energize_y_test() {
    Serial.print("TEST phase_energize axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.print(N3_Y);
    Serial.print(" EN=");
    Serial.println(EN_Y);

    windings_ohmmeter_prompt("Y");
    windings_supply_prompt();

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!driver.init()) {
        Serial.println("RESULT phase_energize_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }
    driver.enable();

    const PhasePairStep pairs[] = {
        {"AB", L2_ENERGIZE_VOLTAGE, 0.0f, 0.0f},
        {"BC", 0.0f, L2_ENERGIZE_VOLTAGE, 0.0f},
        {"CA", 0.0f, 0.0f, L2_ENERGIZE_VOLTAGE},
    };

    for (const PhasePairStep& pair : pairs) {
        windings_energize_pair(driver, "Y", pair);
    }

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("RESULT phase_energize_y PASS");
}
