#pragma once

#include <Arduino.h>
#include "torque_sign_common.hpp"

// L10 — Motor X: closed-loop commutation sign / torque test.
inline void torque_sign_x_test() {
    Serial.print("TEST torque_sign axis=X target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.print(N3_X);
    Serial.print(" EN=");
    Serial.print(EN_X);
    Serial.print(" offset=");
    Serial.println(SENSOR_OFFSET_X, 4);

    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);
    if (!torque_sign_init_driver(driver, "torque_sign_x")) {
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(1, SENSOR_OFFSET_X);
    AS5048 sensor(SENSOR_SPI_CS_X);
    FOCController foc(sensor, driver, motor);

    if (!sensor.init()) {
        Serial.println("RESULT torque_sign_x FAIL encoder_init=0 (angle required to commutate)");
        driver.deinit();
        wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();

    torque_sign_bench_prompt("X", SENSOR_OFFSET_X);
    torque_sign_run(foc, driver, sensor, SENSOR_OFFSET_X, "X");

    driver.disable();
    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
}
