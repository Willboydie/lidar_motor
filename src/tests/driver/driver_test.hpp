#pragma once

#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include <Arduino.h>
#include "../../utils/constants.h"


void driver_test() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Starting PWM Driver Test...");
    delay(1000);

    PWMDriverTeensy41 driver(N1, N2, N3, EN);
    if (!driver.init()) {
        Serial.println("Failed to initialize PWM Driver");
        return;
    }
    driver.enable();

    delay(1000);

    driver.setPwm(3.0, 0.0, 0.0);

    return;
}