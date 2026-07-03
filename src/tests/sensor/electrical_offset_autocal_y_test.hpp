#pragma once

#include <Arduino.h>
#include "electrical_offset_autocal_common.hpp"

// L9 — Motor Y: electrical-offset autocal (stiction-bracket + circular mean).
inline void electrical_offset_autocal_y_test() {
    Serial.print("TEST electrical_offset_autocal axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.print(N3_Y);
    Serial.print(" EN=");
    Serial.print(EN_Y);
    Serial.print(" stored_offset=");
    Serial.println(SENSOR_OFFSET_Y, 4);

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!l9_init_driver(driver, "electrical_offset_autocal_y")) {
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

    if (!sensor.init()) {
        Serial.println("RESULT electrical_offset_autocal_y FAIL encoder_init=0 (angle required)");
        driver.deinit();
        wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();

    l9_autocal_bench_prompt("Y", SENSOR_OFFSET_Y);
    l9_autocal_run(foc, driver, sensor, SENSOR_OFFSET_Y, "Y");

    driver.disable();
    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
}
