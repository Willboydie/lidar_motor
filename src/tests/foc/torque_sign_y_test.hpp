#pragma once

#include <Arduino.h>
#include "torque_sign_common.hpp"

// L10 — Motor Y: closed-loop commutation sign / torque test.
inline void torque_sign_y_test() {
    Serial.print("TEST torque_sign axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.print(N3_Y);
    Serial.print(" EN=");
    Serial.print(EN_Y);
    Serial.print(" offset=");
    Serial.println(SENSOR_OFFSET_Y, 4);

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!torque_sign_init_driver(driver, "torque_sign_y")) {
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

    if (!sensor.init()) {
        Serial.println("RESULT torque_sign_y FAIL encoder_init=0 (angle required to commutate)");
        driver.deinit();
        wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();

    torque_sign_bench_prompt("Y", SENSOR_OFFSET_Y);
    torque_sign_run(foc, driver, sensor, SENSOR_OFFSET_Y, "Y");

    driver.disable();
    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
}
