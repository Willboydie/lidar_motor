#pragma once

#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include <Arduino.h>
#include "../../utils/constants.h"


void driver_test() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Starting PWM Driver Test...");
    delay(1000);

    PWMDriverTeensy41 driver1(N1_1, N2_1, N3_1, EN_1);
    PWMDriverTeensy41 driver2(N1_2, N2_2, N3_2, EN_2);
    if (!driver1.init()) {
        Serial.println("Failed to initialize PWM Driver");
        return;
    }
    if (!driver2.init()) {
        Serial.println("Failed to initialize PWM Driver");
        return;
    }
    driver1.enable();
    driver2.enable();

    delay(1000);

    driver1.setPwm(3.0, 0.0, 0.0);
    driver2.setPwm(3.0, 0.0, 0.0);

    return;
}