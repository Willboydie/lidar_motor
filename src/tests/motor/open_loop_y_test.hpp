#pragma once

#include <Arduino.h>
#include "open_loop_common.hpp"

// L8 — Motor Y: open-loop forced commutation (encoder ignored for drive).
inline void open_loop_y_test() {
    Serial.print("TEST open_loop axis=Y target=");
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
    if (!driver.init()) {
        Serial.println("RESULT open_loop_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

    // Encoder is reference-only for L8; if it fails to init we still run
    // forced commutation (it drives the field open-loop regardless).
    const bool sensor_ok = sensor.init();
    if (sensor_ok) {
        sensor.enable();
    } else {
        Serial.println("WARN open_loop_y encoder init failed — running blind (enc=NA).");
    }

    open_loop_bench_prompt("Y");
    open_loop_run(foc, driver, sensor, sensor_ok, SENSOR_OFFSET_Y, "Y");

    driver.disable();
    driver.deinit();
    if (sensor_ok) sensor.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("RESULT open_loop_y DONE — operator fills PASS|FAIL from rotor motion + current.");
}
