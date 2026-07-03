#pragma once

#include <Arduino.h>
#include "pole_pair_dir_common.hpp"

// L9 — Motor X: pole pairs & sensor direction from a quasi-static field sweep.
inline void pole_pair_dir_x_test() {
    Serial.print("TEST pole_pair_dir axis=X target=");
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
    if (!l9_init_driver(driver, "pole_pair_dir_x")) {
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(1, SENSOR_OFFSET_X);
    AS5048 sensor(SENSOR_SPI_CS_X);
    FOCController foc(sensor, driver, motor);

    if (!sensor.init()) {
        Serial.println("RESULT pole_pair_dir_x FAIL encoder_init=0 (angle required for slope fit)");
        driver.deinit();
        wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();

    l9_pp_bench_prompt("X");
    l9_pp_run(foc, driver, sensor, "X");

    driver.disable();
    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
}
